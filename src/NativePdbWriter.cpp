// Convert DMD CodeView debug information to PDB files
// Copyright (c) 2026 by Johannes Schindelin, All Rights Reserved
//
// License for redistribution is given by the Artistic License 2.0
// see file LICENSE for further details

// NativePdbWriter is the in-house cv2pdb::PdbWriter implementation that
// avoids any runtime dependency on mspdb*.dll.  At this stage it is a
// scaffold: every add* call is accepted and discarded, and commit()
// produces a structurally valid but otherwise empty PDB consisting of
// just the MSF container plus a populated PDB Info Stream (stream 1).
// That is enough to be accepted by "llvm-pdbutil dump --summary".
// Subsequent commits will add real payload (TPI, DBI, /names, modules,
// GSI/PSI, etc.).
//
// Format references:
//   - microsoft/microsoft-pdb (PDB/include/pdb.h, PDB/dbi.cpp):
//     https://github.com/microsoft/microsoft-pdb
//   - LLVM PDB documentation:
//     https://llvm.org/docs/PDB/PdbStream.html
//     https://llvm.org/docs/PDB/HashTable.html

#include "CV2PdbWriter.h"
#include "MsfBuilder.h"

#include <windows.h>
#include <rpc.h>
#pragma comment(lib, "rpcrt4.lib")

#include <cstdint>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

namespace cv2pdb {

namespace {

void appendU16LE(std::vector<uint8_t>& v, uint16_t x)
{
	v.push_back(static_cast<uint8_t>(x & 0xff));
	v.push_back(static_cast<uint8_t>((x >> 8) & 0xff));
}

void appendU32LE(std::vector<uint8_t>& v, uint32_t x)
{
	for (int i = 0; i < 4; i++)
		v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xff));
}

void padToAlign4(std::vector<uint8_t>& v)
{
	while (v.size() % 4 != 0)
		v.push_back(0);
}

// Lower-cased word-XOR hash used in PDB name hash tables.  Mirrors
// llvm::pdb::hashStringV1 (llvm/lib/DebugInfo/PDB/Native/Hash.cpp), which
// in turn mirrors microsoft-pdb's Hasher::lhashPbCb in PDB/include/misc.h.
uint32_t hashStringV1(const char* data, size_t size)
{
	uint32_t result = 0;
	size_t longs = size / 4;
	for (size_t i = 0; i < longs; i++)
	{
		uint32_t v;
		memcpy(&v, data + i * 4, 4);
		result ^= v;
	}
	const uint8_t* rest = reinterpret_cast<const uint8_t*>(data) + longs * 4;
	uint32_t restSize = static_cast<uint32_t>(size % 4);
	if (restSize >= 2)
	{
		uint16_t v;
		memcpy(&v, rest, 2);
		result ^= static_cast<uint32_t>(v);
		rest += 2;
		restSize -= 2;
	}
	if (restSize == 1)
		result ^= *rest;

	const uint32_t toLowerMask = 0x20202020;
	result |= toLowerMask;
	result ^= (result >> 11);
	return result ^ (result >> 16);
}

uint32_t nextPowerOfTwo(uint32_t n)
{
	uint32_t p = 1;
	while (p < n)
		p <<= 1;
	return p;
}

// Builds the NameMap blob embedded in the PDB Info stream.  Layout:
//   uint32_t StringBufferSize
//   char     StringBuffer[StringBufferSize]
//   uint32_t Size
//   uint32_t Capacity
//   uint32_t PresentBitmapWordCount
//   uint32_t PresentBitmap[PresentBitmapWordCount]
//   uint32_t DeletedBitmapWordCount  (always 0; cv2pdb never tombstones)
//   { uint32_t Key, uint32_t Value } Buckets[Size]   (in bucket-index order)
// Bucket placement uses linear probing on hashStringV1(name) % Capacity.
std::vector<uint8_t> buildNameMap(
    const std::vector<std::pair<std::string, uint32_t>>& entries)
{
	std::vector<uint8_t> stringBuffer;
	std::vector<uint32_t> keyOffsets;
	keyOffsets.reserve(entries.size());
	for (const auto& e : entries)
	{
		keyOffsets.push_back(static_cast<uint32_t>(stringBuffer.size()));
		stringBuffer.insert(stringBuffer.end(), e.first.begin(), e.first.end());
		stringBuffer.push_back(0);
	}

	uint32_t size = static_cast<uint32_t>(entries.size());
	uint32_t capacity = nextPowerOfTwo(size * 3 / 2 + 1);
	if (capacity < 2)
		capacity = 2;

	// -1 sentinel = empty bucket.
	std::vector<int32_t> bucketOf(capacity, -1);
	for (size_t i = 0; i < entries.size(); i++)
	{
		const std::string& name = entries[i].first;
		uint32_t h = hashStringV1(name.data(), name.size());
		uint32_t b = h % capacity;
		while (bucketOf[b] != -1)
			b = (b + 1) % capacity;
		bucketOf[b] = static_cast<int32_t>(i);
	}

	uint32_t bitmapWords = (capacity + 31) / 32;
	std::vector<uint32_t> presentBitmap(bitmapWords, 0);
	for (uint32_t b = 0; b < capacity; b++)
		if (bucketOf[b] != -1)
			presentBitmap[b / 32] |= (1u << (b % 32));

	std::vector<uint8_t> blob;
	appendU32LE(blob, static_cast<uint32_t>(stringBuffer.size()));
	blob.insert(blob.end(), stringBuffer.begin(), stringBuffer.end());
	appendU32LE(blob, size);
	appendU32LE(blob, capacity);
	appendU32LE(blob, bitmapWords);
	for (uint32_t w : presentBitmap)
		appendU32LE(blob, w);
	appendU32LE(blob, 0);  // DeletedBitmapWordCount
	for (uint32_t b = 0; b < capacity; b++)
	{
		int32_t entryIdx = bucketOf[b];
		if (entryIdx < 0)
			continue;
		appendU32LE(blob, keyOffsets[entryIdx]);
		appendU32LE(blob, entries[entryIdx].second);
	}
	return blob;
}

// Builds the /names stream content.  Layout:
//   uint32_t Magic        (0xEFFEEFFE)
//   uint32_t HashVersion  (1)
//   uint32_t ByteSize     (length of NameBuffer)
//   char     NameBuffer[ByteSize]   (offset 0 reserved as the "no name" slot)
//   uint32_t HashSize
//   uint32_t HashEntries[HashSize]  (NameBuffer offsets; 0 means empty slot)
//   uint32_t NumNames
class NamesStreamBuilder
{
public:
	NamesStreamBuilder()
	{
		// Reserve offset 0 for the conventional "no name" slot.
		buffer_.push_back(0);
	}

	// Returns the offset of the (deduplicated) string in NameBuffer.
	uint32_t addName(const std::string& s)
	{
		auto it = lookup_.find(s);
		if (it != lookup_.end())
			return it->second;
		uint32_t off = static_cast<uint32_t>(buffer_.size());
		buffer_.insert(buffer_.end(), s.begin(), s.end());
		buffer_.push_back(0);
		lookup_[s] = off;
		numNames_++;
		return off;
	}

	std::vector<uint8_t> buildStream() const
	{
		std::vector<uint8_t> blob;
		appendU32LE(blob, 0xEFFEEFFE);
		appendU32LE(blob, 1);
		appendU32LE(blob, static_cast<uint32_t>(buffer_.size()));
		blob.insert(blob.end(), buffer_.begin(), buffer_.end());

		// Hash table sizing matches LLVM's PDBStringTableBuilder:
		//   max(8, NextPowerOfTwo(NumStrings * 3 / 2)).
		uint32_t hashSize = nextPowerOfTwo(numNames_ * 3 / 2);
		if (hashSize < 8)
			hashSize = 8;

		std::vector<uint32_t> table(hashSize, 0);
		for (const auto& kv : lookup_)
		{
			uint32_t h = hashStringV1(kv.first.data(), kv.first.size());
			uint32_t b = h % hashSize;
			while (table[b] != 0)
				b = (b + 1) % hashSize;
			table[b] = kv.second;
		}

		appendU32LE(blob, hashSize);
		for (uint32_t e : table)
			appendU32LE(blob, e);
		appendU32LE(blob, numNames_);
		return blob;
	}

private:
	std::vector<uint8_t> buffer_;
	std::map<std::string, uint32_t> lookup_;
	uint32_t numNames_ = 0;
};

#pragma pack(push, 1)
struct TpiStreamHeader
{
	uint32_t Version;
	uint32_t HeaderSize;
	uint32_t TypeIndexBegin;
	uint32_t TypeIndexEnd;
	uint32_t TypeRecordBytes;

	uint16_t HashStreamIndex;
	uint16_t HashAuxStreamIndex;
	uint32_t HashKeySize;
	uint32_t NumHashBuckets;

	int32_t  HashValueBufferOffset;
	uint32_t HashValueBufferLength;

	int32_t  IndexOffsetBufferOffset;
	uint32_t IndexOffsetBufferLength;

	int32_t  HashAdjBufferOffset;
	uint32_t HashAdjBufferLength;
};
#pragma pack(pop)
static_assert(sizeof(TpiStreamHeader) == 56, "TpiStreamHeader must be 56 bytes");

// TPI and IPI streams share an identical on-disk layout; one builder serves
// both.  At this stage no records are accumulated, so the builder emits a
// header with TypeIndexBegin == TypeIndexEnd == 0x1000 and an empty hash
// stream.  The HashStreamIndex is supplied by the caller so the right index
// is written into the header even though the stream order is decided by
// MsfBuilder.
class TpiStreamBuilder
{
public:
	std::vector<uint8_t> buildStream(uint16_t hashStreamIndex) const
	{
		TpiStreamHeader hdr = {};
		hdr.Version           = 20040203;          // V80
		hdr.HeaderSize        = sizeof(hdr);
		hdr.TypeIndexBegin    = 0x1000;
		hdr.TypeIndexEnd      = 0x1000;            // == Begin: no records yet
		hdr.TypeRecordBytes   = 0;
		hdr.HashStreamIndex   = hashStreamIndex;
		hdr.HashAuxStreamIndex = 0xFFFF;
		hdr.HashKeySize       = 4;
		hdr.NumHashBuckets    = 0x40000 - 1;       // 262143
		// All EmbeddedBuf offsets/lengths stay zero; with no records there
		// is nothing for the hash stream to point at.
		std::vector<uint8_t> blob(sizeof(hdr));
		memcpy(blob.data(), &hdr, sizeof(hdr));
		return blob;
	}

	std::vector<uint8_t> buildHashStream() const
	{
		return {};
	}
};

#pragma pack(push, 1)
struct DbiStreamHeader
{
	int32_t  VersionSignature;
	uint32_t VersionHeader;
	uint32_t Age;
	uint16_t GlobalSymbolStreamIndex;
	uint16_t BuildNumber;
	uint16_t PublicSymbolStreamIndex;
	uint16_t PdbDllVersion;
	uint16_t SymRecordStreamIndex;
	uint16_t PdbDllRbld;
	int32_t  ModInfoSize;
	int32_t  SectionContributionSize;
	int32_t  SectionMapSize;
	int32_t  SourceInfoSize;
	int32_t  TypeServerMapSize;
	uint32_t MfcTypeServerIndex;
	int32_t  OptionalDbgHeaderSize;
	int32_t  ECSubstreamSize;
	uint16_t Flags;
	uint16_t Machine;
	uint32_t Padding;
};
#pragma pack(pop)
static_assert(sizeof(DbiStreamHeader) == 64, "DbiStreamHeader must be 64 bytes");

// Emits a DBI stream with the requested substream payloads.  Sub-streams that
// the LLVM reader requires to be sized non-zero get just enough header bytes
// to make sense; ModInfo and SourceInfo come from the caller as it has the
// per-module knowledge.
//
//   - ModInfo:       supplied via setModInfoSubstream (one ModuleInfoHeader +
//                    null-terminated obj/lib names + 4-byte padding per
//                    module).  Empty when no modules.
//   - SectionContrib: 4 bytes for the V60 version magic.
//   - SectionMap:    4 bytes for SectionMapHeader{Count=0, LogCount=0}.
//   - SourceInfo:    supplied via setSourceInfoSubstream so it can grow with
//                    NumModules and per-module file counts; defaults to the
//                    minimum {NumModules=0, NumSourceFiles=0}.
//   - OptionalDbgHeader: 22 bytes, eleven kInvalidStreamIndex entries.
// TypeServerMap and ECSubstream stay 0-sized; LLVM tolerates that.
class DbiStreamBuilder
{
public:
	void setModInfoSubstream(std::vector<uint8_t> bytes)
	{
		modInfo_ = std::move(bytes);
	}

	void setSourceInfoSubstream(std::vector<uint8_t> bytes)
	{
		sourceInfo_ = std::move(bytes);
	}

	std::vector<uint8_t> buildStream(uint16_t machine) const
	{
		// Substream payloads, in the order the DBI stream layout requires:
		// ModInfo, SecContrib, SecMap, SourceInfo, TypeServerMap, EC,
		// OptionalDbgHeader.
		std::vector<uint8_t> secContrib;
		appendU32LE(secContrib, 0xF12EBA2D); // DbiSecContribVer60

		std::vector<uint8_t> secMap;
		appendU16LE(secMap, 0);              // Count
		appendU16LE(secMap, 0);              // LogCount

		std::vector<uint8_t> defaultSourceInfo;
		appendU16LE(defaultSourceInfo, 0);   // NumModules
		appendU16LE(defaultSourceInfo, 0);   // NumSourceFiles
		const std::vector<uint8_t>& sourceInfo =
		    sourceInfo_.empty() ? defaultSourceInfo : sourceInfo_;

		std::vector<uint8_t> typeServerMap;  // empty

		// EC (Edit and Continue) substream carries a PDB string table that
		// dump --modules dereferences via DbiStream::getECName for the
		// ModuleInfoHeader's PdbFilePathNI / SrcFileNameNI fields, even
		// when those fields are 0 and the per-module HasECInfo flag is
		// off.  A 0-byte substream leaves ECNames default-constructed and
		// makes getECName(0) fail with "stream is too short" the moment
		// the dump tool iterates modules.  Emit a stub PDB string table
		// (one empty string at offset 0) so the lookup of a 0 NI returns
		// the empty string.  NamesStreamBuilder produces exactly the
		// right bytes for this stub when no names are added.
		NamesStreamBuilder ecStub;
		std::vector<uint8_t> ecSubstream = ecStub.buildStream();

		std::vector<uint8_t> optDbgHdr;
		for (int i = 0; i < 11; i++)
		{
			optDbgHdr.push_back(0xFF);
			optDbgHdr.push_back(0xFF);
		}

		DbiStreamHeader hdr = {};
		hdr.VersionSignature        = -1;
		hdr.VersionHeader           = 19990903;     // V70
		hdr.Age                     = 1;
		hdr.GlobalSymbolStreamIndex = 0xFFFF;
		hdr.BuildNumber             = 0x8E0B;       // 36363, lld-link's value
		hdr.PublicSymbolStreamIndex = 0xFFFF;
		hdr.PdbDllVersion           = 0;
		hdr.SymRecordStreamIndex    = 0xFFFF;
		hdr.PdbDllRbld              = 0;
		hdr.ModInfoSize             = static_cast<int32_t>(modInfo_.size());
		hdr.SectionContributionSize = static_cast<int32_t>(secContrib.size());
		hdr.SectionMapSize          = static_cast<int32_t>(secMap.size());
		hdr.SourceInfoSize          = static_cast<int32_t>(sourceInfo.size());
		hdr.TypeServerMapSize       = static_cast<int32_t>(typeServerMap.size());
		hdr.MfcTypeServerIndex      = 0xFFFFFFFF;
		hdr.OptionalDbgHeaderSize   = static_cast<int32_t>(optDbgHdr.size());
		hdr.ECSubstreamSize         = static_cast<int32_t>(ecSubstream.size());
		hdr.Flags                   = 0;
		hdr.Machine                 = machine;
		hdr.Padding                 = 0;

		std::vector<uint8_t> blob;
		blob.reserve(sizeof(hdr) + modInfo_.size() + secContrib.size()
		             + secMap.size() + sourceInfo.size()
		             + typeServerMap.size() + ecSubstream.size()
		             + optDbgHdr.size());
		const uint8_t* hdrBytes = reinterpret_cast<const uint8_t*>(&hdr);
		blob.insert(blob.end(), hdrBytes, hdrBytes + sizeof(hdr));
		blob.insert(blob.end(), modInfo_.begin(), modInfo_.end());
		blob.insert(blob.end(), secContrib.begin(), secContrib.end());
		blob.insert(blob.end(), secMap.begin(), secMap.end());
		blob.insert(blob.end(), sourceInfo.begin(), sourceInfo.end());
		blob.insert(blob.end(), typeServerMap.begin(), typeServerMap.end());
		blob.insert(blob.end(), ecSubstream.begin(), ecSubstream.end());
		blob.insert(blob.end(), optDbgHdr.begin(), optDbgHdr.end());
		return blob;
	}

private:
	std::vector<uint8_t> modInfo_;
	std::vector<uint8_t> sourceInfo_;
};

// Per-module symbol stream and DBI ModInfo entry.  At this stage no records
// or C13 subsections are accumulated; the per-module stream is just the C13
// signature followed by an empty global-refs trailer (8 bytes total).  Later
// commits will turn the stub add* methods into real accumulators, growing
// SymByteSize / C13ByteSize / SourceFileCount accordingly.
//
// Format references for the per-module symbol stream and ModuleInfoHeader:
//   - microsoft/microsoft-pdb (PDB/include/dbi.h)
//   - LLVM PDB documentation:
//     https://llvm.org/docs/PDB/ModiStream.html
//     https://llvm.org/docs/PDB/DbiStream.html#dbi-mod-info-substream
class ModuleStreamBuilder : public ModWriter
{
public:
	ModuleStreamBuilder(std::string objName, std::string libName)
	    : objName_(std::move(objName)), libName_(std::move(libName)) {}

	int addSecContrib(unsigned short, long, long, unsigned long) override { return 1; }
	int addTypes(unsigned char*, long) override { return 1; }
	int addSymbols(unsigned char*, long) override { return 1; }
	int addPublic(const char*, unsigned short, long, unsigned long) override { return 1; }
	int addLines(const char*, unsigned short, long, long, long, unsigned short,
	             unsigned char*, long) override { return 1; }
	int close() override { return 1; }

	std::vector<uint8_t> buildStream() const
	{
		std::vector<uint8_t> blob;
		appendU32LE(blob, 4);   // CV_SIGNATURE_C13
		// No symbol records, no C13 subsections.
		appendU32LE(blob, 0);   // GlobalRefCount
		return blob;
	}

	uint32_t symByteSize() const { return 4; }     // signature only
	uint32_t c13ByteSize() const { return 0; }
	uint16_t sourceFileCount() const { return 0; }

	std::vector<uint8_t> buildModInfoEntry(uint16_t streamIndex) const
	{
		std::vector<uint8_t> blob;
		appendU32LE(blob, 0);                       // Unused1
		// SectionContribEntry (28 bytes).  No primary contribution yet:
		// Section = -1 marks "none", everything else stays zero.
		appendU16LE(blob, 0xFFFF);                  // Section
		appendU16LE(blob, 0);                       // Padding1
		appendU32LE(blob, 0);                       // Offset
		appendU32LE(blob, 0);                       // Size
		appendU32LE(blob, 0);                       // Characteristics
		appendU16LE(blob, 0xFFFF);                  // ModuleIndex
		appendU16LE(blob, 0);                       // Padding2
		appendU32LE(blob, 0);                       // DataCrc
		appendU32LE(blob, 0);                       // RelocCrc

		appendU16LE(blob, 0);                       // Flags
		appendU16LE(blob, streamIndex);             // ModuleSymStream
		appendU32LE(blob, symByteSize());
		appendU32LE(blob, 0);                       // C11ByteSize
		appendU32LE(blob, c13ByteSize());
		appendU16LE(blob, sourceFileCount());
		appendU16LE(blob, 0);                       // Padding
		appendU32LE(blob, 0);                       // Unused2
		appendU32LE(blob, 0);                       // SourceFileNameIndex
		appendU32LE(blob, 0);                       // PdbFilePathNameIndex

		blob.insert(blob.end(), objName_.begin(), objName_.end());
		blob.push_back(0);
		blob.insert(blob.end(), libName_.begin(), libName_.end());
		blob.push_back(0);
		padToAlign4(blob);
		return blob;
	}

private:
	std::string objName_;
	std::string libName_;
};

std::vector<uint8_t> buildSourceInfoSubstream(uint32_t numModules)
{
	std::vector<uint8_t> blob;
	appendU16LE(blob, static_cast<uint16_t>(numModules));
	appendU16LE(blob, 0);                           // NumSourceFiles (truncated)
	for (uint32_t i = 0; i < numModules; i++)
		appendU16LE(blob, 0);                       // ModIndices[i]
	for (uint32_t i = 0; i < numModules; i++)
		appendU16LE(blob, 0);                       // ModFileCounts[i]
	// FileNameOffsets and NamesBuffer are empty until source files are
	// plumbed through addLines.
	padToAlign4(blob);
	return blob;
}

class NativePdbWriter : public PdbWriter
{
public:
	explicit NativePdbWriter(const wchar_t* pdbname) : path_(pdbname)
	{
		// Per microsoft-pdb's pdb.cpp, Signature is a creation timestamp.
		signature_ = static_cast<uint32_t>(time(nullptr));
		// UuidCreate is in rpcrt4 and does not require COM initialisation,
		// unlike CoCreateGuid.
		UUID u;
		if (UuidCreate(&u) == RPC_S_OK)
			memcpy(&guid_, &u, sizeof(guid_));
		else
			memset(&guid_, 0, sizeof(guid_));
	}

	~NativePdbWriter() override
	{
		for (ModuleStreamBuilder* m : mods_)
			delete m;
	}

	int initDbi() override { return 1; }
	int initTpi() override { return 1; }
	int initIpi() override { return 1; }

	int setMachineType(unsigned short machine) override
	{
		machine_ = machine;
		return 1;
	}

	int openMod(const char* objName, const char* libName, ModWriter** outMod) override
	{
		auto* m = new ModuleStreamBuilder(
		    objName ? std::string(objName) : std::string(),
		    libName ? std::string(libName) : std::string());
		mods_.push_back(m);
		*outMod = m;
		return 1;
	}

	int addSec(unsigned short, unsigned short, long, long) override { return 1; }
	int addPublic(const char*, unsigned short, long, unsigned long) override { return 1; }

	int querySignature(GUID* guid) override
	{
		if (guid)
			*guid = guid_;
		return 1;
	}

	int queryAge() override { return 1; }

	int queryLastError(char* buf) override
	{
		if (buf)
			buf[0] = 0;
		return 0;
	}

	int commit() override
	{
		MsfBuilder msf;

		TpiStreamBuilder tpi;
		TpiStreamBuilder ipi;
		DbiStreamBuilder dbi;

		// Pre-assign per-module stream indices.  Fixed streams 0..7 are the
		// MSF directory placeholder, PDB Info, TPI, DBI, IPI, TPI hash,
		// IPI hash, and /names.  Module streams take 8..7+N in openMod
		// order; that ordering needs to be visible to the DBI ModInfo
		// substream below before commit() lays out the MSF, so the indices
		// are computed up front rather than being read from MsfBuilder
		// after addStream.
		const uint16_t kFirstModuleIndex = 8;
		std::vector<uint8_t> modInfo;
		for (size_t i = 0; i < mods_.size(); i++)
		{
			uint16_t streamIndex =
			    static_cast<uint16_t>(kFirstModuleIndex + i);
			auto entry = mods_[i]->buildModInfoEntry(streamIndex);
			modInfo.insert(modInfo.end(), entry.begin(), entry.end());
		}
		dbi.setModInfoSubstream(std::move(modInfo));
		dbi.setSourceInfoSubstream(
		    buildSourceInfoSubstream(static_cast<uint32_t>(mods_.size())));

		// Stream 0: "Old MSF Directory" placeholder, empty (lld-link does
		// the same; mspdb keeps 40 stale bytes from the previous commit
		// but no current consumer reads it).
		msf.addStream({});

		// Stream 1: PDB Info Stream.  Layout (little-endian):
		//   PdbStreamHeader { Version (VC70), Signature, Age, Guid }
		//   NameMap        { StringBuffer + open-addressed hash table }
		//   Features       { VC140 = 0x013351DC }
		// The NameMap registers /names so consumers can locate the string
		// table by name lookup; VC140 advertises the IPI stream.
		std::vector<uint8_t> info;
		appendU32LE(info, 20000404);          // VC70
		appendU32LE(info, signature_);
		appendU32LE(info, 1);                 // Age
		const uint8_t* guidBytes = reinterpret_cast<const uint8_t*>(&guid_);
		info.insert(info.end(), guidBytes, guidBytes + sizeof(guid_));
		auto nameMap = buildNameMap({{ "/names", 7 }});
		info.insert(info.end(), nameMap.begin(), nameMap.end());
		appendU32LE(info, 0x013351DC);        // VC140 (= 20140508)
		msf.addStream(std::move(info));

		// Stream layout from here matches the conventional PDB indices:
		//   2 = TPI, 3 = DBI, 4 = IPI.  The TPI/IPI hash sub-streams are
		//   placed at 5 and 6 respectively; those indices must be wired
		//   into the TPI/IPI headers before they are added to the MSF.
		const uint16_t kTpiHashIndex = 5;
		const uint16_t kIpiHashIndex = 6;

		msf.addStream(tpi.buildStream(kTpiHashIndex));   // 2: TPI
		msf.addStream(dbi.buildStream(machine_));        // 3: DBI
		msf.addStream(ipi.buildStream(kIpiHashIndex));   // 4: IPI
		msf.addStream(tpi.buildHashStream());            // 5: TPI hash
		msf.addStream(ipi.buildHashStream());            // 6: IPI hash
		msf.addStream(names_.buildStream());             // 7: /names

		// Per-module symbol streams (8, 9, ...) in openMod order.
		for (ModuleStreamBuilder* m : mods_)
			msf.addStream(m->buildStream());

		return msf.write(path_) ? 1 : 0;
	}

	int close() override { return 1; }

private:
	std::wstring path_;
	GUID guid_;
	uint32_t signature_;
	unsigned short machine_ = 0;
	std::vector<ModuleStreamBuilder*> mods_;
	NamesStreamBuilder names_;
};

}  // namespace

PdbWriter* createNativePdbWriter(const wchar_t* pdbname)
{
	return new NativePdbWriter(pdbname);
}

}  // namespace cv2pdb
