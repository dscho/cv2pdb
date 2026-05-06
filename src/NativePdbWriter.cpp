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

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

namespace cv2pdb {

namespace {

// JamCRC (CRC-32 reversed polynomial, init = 0) used as the per-record hash
// in the TPI/IPI hash streams.  Mirrors llvm::pdb::hashBufferV8, which wraps
// llvm::JamCRC with Init=0.  Polynomial 0xEDB88320 is the bit-reversal of the
// IEEE 802.3 CRC-32 polynomial 0x04C11DB7.  The lookup table is built lazily
// the first time this function runs.
uint32_t hashBufferV8(const uint8_t* buf, size_t size)
{
	static uint32_t table[256];
	static bool initialised = false;
	if (!initialised)
	{
		for (uint32_t i = 0; i < 256; i++)
		{
			uint32_t c = i;
			for (int j = 0; j < 8; j++)
				c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
			table[i] = c;
		}
		initialised = true;
	}
	uint32_t crc = 0;
	for (size_t i = 0; i < size; i++)
		crc = (crc >> 8) ^ table[(crc ^ buf[i]) & 0xff];
	return crc;
}

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
struct SectionContribEntry
{
	int16_t  Section;
	int16_t  Padding1;
	int32_t  Offset;
	int32_t  Size;
	uint32_t Characteristics;
	int16_t  ModuleIndex;
	int16_t  Padding2;
	uint32_t DataCrc;
	uint32_t RelocCrc;
};
#pragma pack(pop)
static_assert(sizeof(SectionContribEntry) == 28, "SectionContribEntry must be 28 bytes");

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
// both.  CodeView records are appended via addRecords(), which also computes
// the hash-stream sidecar (one hashBufferV8 entry per record, plus a sparse
// IndexOffsetBuffer entry on the first record and every IndexOffsetGranBytes
// thereafter).  buildStream emits the 56-byte header with TypeIndexEnd,
// TypeRecordBytes, and the hash-buffer offset/length triples filled in;
// buildHashStream emits the matching hash payload.
class TpiStreamBuilder
{
public:
	static constexpr uint32_t kFirstTypeIndex = 0x1000;
	static constexpr uint32_t kNumHashBuckets = 0x40000 - 1;   // 262143
	static constexpr uint32_t kIndexOffsetGranBytes = 8 * 1024;

	void addRecords(const uint8_t* buf, size_t cb)
	{
		size_t off = 0;
		while (off + 2 <= cb)
		{
			uint16_t len;
			memcpy(&len, buf + off, 2);
			size_t recordSize = 2 + len;       // 2-byte length prefix + payload
			if (recordSize < 4 || off + recordSize > cb)
				break;                         // truncated or malformed

			uint32_t recBlobOffset =
			    static_cast<uint32_t>(records_.size());
			uint32_t typeIndex = kFirstTypeIndex + numRecords_;

			// Sparse IndexOffsetBuffer: always emit on the very first
			// record so that bisection has a starting point at offset 0,
			// then again every kIndexOffsetGranBytes of accumulated record
			// bytes.  Matches LLVM's TpiStreamBuilder.
			if (numRecords_ == 0
			    || (recBlobOffset - lastIobOffset_) >= kIndexOffsetGranBytes)
			{
				indexOffsets_.emplace_back(typeIndex, recBlobOffset);
				lastIobOffset_ = recBlobOffset;
			}

			records_.insert(records_.end(), buf + off, buf + off + recordSize);

			uint32_t hash = hashBufferV8(buf + off, recordSize) % kNumHashBuckets;
			appendU32LE(hashValues_, hash);

			numRecords_++;
			off += recordSize;
		}
	}

	std::vector<uint8_t> buildStream(uint16_t hashStreamIndex) const
	{
		TpiStreamHeader hdr = {};
		hdr.Version            = 20040203;          // V80
		hdr.HeaderSize         = sizeof(hdr);
		hdr.TypeIndexBegin     = kFirstTypeIndex;
		hdr.TypeIndexEnd       = kFirstTypeIndex + numRecords_;
		hdr.TypeRecordBytes    = static_cast<uint32_t>(records_.size());
		hdr.HashStreamIndex    = hashStreamIndex;
		hdr.HashAuxStreamIndex = 0xFFFF;
		hdr.HashKeySize        = 4;
		hdr.NumHashBuckets     = kNumHashBuckets;

		uint32_t hashValueBytes = static_cast<uint32_t>(hashValues_.size());
		uint32_t iobBytes = 8 * static_cast<uint32_t>(indexOffsets_.size());

		hdr.HashValueBufferOffset   = 0;
		hdr.HashValueBufferLength   = hashValueBytes;
		hdr.IndexOffsetBufferOffset = static_cast<int32_t>(hashValueBytes);
		hdr.IndexOffsetBufferLength = iobBytes;
		hdr.HashAdjBufferOffset     =
		    static_cast<int32_t>(hashValueBytes + iobBytes);
		hdr.HashAdjBufferLength     = 0;

		std::vector<uint8_t> blob(sizeof(hdr));
		memcpy(blob.data(), &hdr, sizeof(hdr));
		blob.insert(blob.end(), records_.begin(), records_.end());
		return blob;
	}

	std::vector<uint8_t> buildHashStream() const
	{
		std::vector<uint8_t> blob;
		blob.insert(blob.end(), hashValues_.begin(), hashValues_.end());
		for (const auto& iob : indexOffsets_)
		{
			appendU32LE(blob, iob.first);    // TypeIndex
			appendU32LE(blob, iob.second);   // ByteOffset in record blob
		}
		return blob;
	}

private:
	std::vector<uint8_t> records_;
	std::vector<uint8_t> hashValues_;            // raw little-endian uint32s
	std::vector<std::pair<uint32_t, uint32_t>> indexOffsets_;
	uint32_t numRecords_ = 0;
	uint32_t lastIobOffset_ = 0;
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

	void setSectionContribs(std::vector<SectionContribEntry> entries)
	{
		sectionContribs_ = std::move(entries);
	}

	void setGlobalSymbolStreamIndex(uint16_t idx) { globalSymStream_ = idx; }
	void setPublicSymbolStreamIndex(uint16_t idx) { publicSymStream_ = idx; }
	void setSymRecordStreamIndex(uint16_t idx)    { symRecordStream_ = idx; }
	void setSectionHeaderStreamIndex(uint16_t idx) { sectionHdrStream_ = idx; }

	std::vector<uint8_t> buildStream(uint16_t machine) const
	{
		// Substream payloads, in the order the DBI stream layout requires:
		// ModInfo, SecContrib, SecMap, SourceInfo, TypeServerMap, EC,
		// OptionalDbgHeader.
		std::vector<uint8_t> secContrib;
		appendU32LE(secContrib, 0xF12EBA2D); // DbiSecContribVer60
		for (const auto& e : sectionContribs_)
		{
			const uint8_t* p = reinterpret_cast<const uint8_t*>(&e);
			secContrib.insert(secContrib.end(), p, p + sizeof(e));
		}

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

		// Optional Debug Header substream: 11 stream-index slots.  cv2pdb's
		// in-house writer fills slot [5] (Section Header) when the input
		// PE's section-header array is available.  Everything else is
		// kInvalidStreamIndex (0xFFFF), matching the cv2pdb-mspdb baseline.
		uint16_t optDbgHdrIndices[11] = {
		    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
		    sectionHdrStream_,
		    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
		};
		std::vector<uint8_t> optDbgHdr;
		for (int i = 0; i < 11; i++)
		{
			optDbgHdr.push_back(static_cast<uint8_t>(optDbgHdrIndices[i] & 0xFF));
			optDbgHdr.push_back(static_cast<uint8_t>((optDbgHdrIndices[i] >> 8) & 0xFF));
		}

		DbiStreamHeader hdr = {};
		hdr.VersionSignature        = -1;
		hdr.VersionHeader           = 19990903;     // V70
		hdr.Age                     = 1;
		hdr.GlobalSymbolStreamIndex = globalSymStream_;
		hdr.BuildNumber             = 0x8E0B;       // 36363, lld-link's value
		hdr.PublicSymbolStreamIndex = publicSymStream_;
		hdr.PdbDllVersion           = 0;
		hdr.SymRecordStreamIndex    = symRecordStream_;
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
	std::vector<SectionContribEntry> sectionContribs_;
	uint16_t globalSymStream_ = 0xFFFF;
	uint16_t publicSymStream_ = 0xFFFF;
	uint16_t symRecordStream_ = 0xFFFF;
	uint16_t sectionHdrStream_ = 0xFFFF;
};

// Per-module symbol stream and DBI ModInfo entry.  buildStream emits the C13
// signature, the accumulated symbol bytes (each S_* record cv2pdb pushed via
// addSymbols, padded to 4-byte alignment between records when needed), and
// the trailing GlobalRefCount = 0 / GlobalRefs[] = empty terminator.  C13
// debug subsections (DEBUG_S_LINES, DEBUG_S_FILECHKSMS, ...) come in a later
// commit when addLines is wired up.
//
// addSecContrib stores each (Section, Offset, Size, Characteristics) tuple
// twice: once as the module's primary SectionContribEntry (the first call
// becomes the entry stamped into ModuleInfoHeader.SectionContrib), and once
// in the writer-level vector that DbiStreamBuilder serialises into the DBI
// section-contributions substream.
//
// Format references for the per-module symbol stream and ModuleInfoHeader:
//   - microsoft/microsoft-pdb (PDB/include/dbi.h)
//   - LLVM PDB documentation:
//     https://llvm.org/docs/PDB/ModiStream.html
//     https://llvm.org/docs/PDB/DbiStream.html#dbi-mod-info-substream
class ModuleStreamBuilder : public ModWriter
{
public:
	ModuleStreamBuilder(std::string objName, std::string libName,
	                    TpiStreamBuilder* tpi,
	                    uint16_t moduleIndex,
	                    std::vector<SectionContribEntry>* allSecContribs,
	                    NamesStreamBuilder* names,
	                    PdbWriter* writer)
	    : objName_(std::move(objName)), libName_(std::move(libName)),
	      tpi_(tpi), moduleIndex_(moduleIndex),
	      allSecContribs_(allSecContribs), names_(names), writer_(writer)
	{
		memset(&primaryContrib_, 0, sizeof(primaryContrib_));
		primaryContrib_.Section     = -1;       // "no section" sentinel
		primaryContrib_.ModuleIndex = -1;
	}

	int addSecContrib(unsigned short seg, long off, long size,
	                  unsigned long characteristics) override
	{
		SectionContribEntry e = {};
		e.Section         = static_cast<int16_t>(seg);
		e.Offset          = static_cast<int32_t>(off);
		e.Size            = static_cast<int32_t>(size);
		e.Characteristics = characteristics;
		e.ModuleIndex     = static_cast<int16_t>(moduleIndex_);

		if (!hasPrimary_)
		{
			primaryContrib_ = e;
			hasPrimary_ = true;
		}
		if (allSecContribs_)
			allSecContribs_->push_back(e);
		return 1;
	}

	int addTypes(unsigned char* pTypes, long cbTypes) override
	{
		// cv2pdb routes every CodeView type record through mod->AddTypes;
		// in the in-house writer they all flow into the single global TPI
		// stream that backs every module.  IPI stays empty: cv2pdb has no
		// LF_FUNC_ID / LF_STRING_ID records to emit.
		//
		// cv2pdb prefixes the buffer with a 4-byte CV_SIGNATURE_C13 header
		// (the leading "\x04\x00\x00\x00" written at userTypes[0..3] in
		// dwarf2pdb.cpp and at globalTypes[0..3] in cv2pdb.cpp) before any
		// records start.  Strip it so the record walker does not interpret
		// the signature as a malformed record.
		if (!tpi_ || !pTypes || cbTypes <= 0)
			return 1;
		size_t off = 0;
		if (cbTypes >= 4 && pTypes[0] == 0x04
		    && pTypes[1] == 0 && pTypes[2] == 0 && pTypes[3] == 0)
			off = 4;
		tpi_->addRecords(pTypes + off, static_cast<size_t>(cbTypes) - off);
		return 1;
	}

	int addSymbols(unsigned char* pSymbols, long cbSymbols) override
	{
		// cv2pdb wraps its symbol records in a fake DEBUG_S_SYMBOLS-style
		// envelope and hands the whole thing to AddSymbols.  Per
		// cv2pdb.cpp::CV2PDB::writeSymbols, the buffer is:
		//   data[0] = 4              CV_SIGNATURE_C13
		//   data[1] = 0xF1           DEBUG_S_SYMBOLS subsection kind
		//   data[2] = <length>       payload byte count (= databytes when
		//                              prefix == 3, databytes + 4 when
		//                              prefix == 4)
		//   data[3] = 1              (only when prefix == 4, i.e.
		//                              mspdb::vsVersion < 14, which is the
		//                              case the native backend hits because
		//                              it never loads mspdb and vsVersion
		//                              stays at the default of 8)
		//   data[prefix..]           actual S_* symbol records, then 0..3
		//                              bytes of unrelated tail padding to
		//                              round the buffer to a DWORD count
		// mspdb peels the prefix off and appends just the records (not the
		// tail padding) to the raw symbol-records section of the module
		// stream so consumers find them at offset 4.  The native writer
		// matches that placement: read the declared payload length from
		// data[2], reverse the prefix==4 adjustment, and copy exactly that
		// many bytes from the records area.  Trailing buffer-rounding
		// bytes get dropped, which is what keeps llvm-pdbutil's record
		// walker from running past the last real record into zero bytes
		// that would parse as malformed records.
		if (!pSymbols || cbSymbols <= 0)
			return 1;

		bool wrappedC13 = cbSymbols >= 12
		    && pSymbols[0] == 0x04 && pSymbols[1] == 0
		    && pSymbols[2] == 0    && pSymbols[3] == 0
		    && pSymbols[4] == 0xF1 && pSymbols[5] == 0
		    && pSymbols[6] == 0    && pSymbols[7] == 0;
		if (!wrappedC13)
		{
			// Fallback for any future caller that hands us already-bare
			// records: just append verbatim.
			symbols_.insert(symbols_.end(), pSymbols, pSymbols + cbSymbols);
			while (symbols_.size() % 4 != 0)
				symbols_.push_back(0);
			return 1;
		}

		uint32_t length;
		memcpy(&length, pSymbols + 8, 4);
		bool prefix4 = cbSymbols >= 16
		    && pSymbols[12] == 1 && pSymbols[13] == 0
		    && pSymbols[14] == 0 && pSymbols[15] == 0;
		size_t off = prefix4 ? 16 : 12;
		uint32_t recordBytes = prefix4 ? (length - 4) : length;
		if (off + recordBytes > static_cast<size_t>(cbSymbols))
			return 1;                                  // malformed: drop

		// LLVM's symbol-record walker reads each record by its length
		// field and advances by length + 2 with no separate alignment
		// step.  cv2pdb produces some records (S_COMPILE, S_GPROC32) that
		// are already 4-byte-multiples in size, but others (S_UDT_V3 with
		// short names, etc.) end on an odd byte and would leave the next
		// record at a misaligned offset.  mspdb papers over that by
		// stretching each record's length field so the in-record byte
		// count rounds up to 4.  Do the same here: append the source
		// record verbatim, pad with zero bytes until the on-disk record
		// is a 4-byte multiple, then patch the length field to include
		// the padding.  Trailing zeros in the payload are harmless because
		// every CV symbol record's parser stops at its own structural
		// terminator (null-string, fixed-size fields, ...) before that.
		size_t end = off + recordBytes;
		while (off + 2 <= end)
		{
			uint16_t len;
			memcpy(&len, pSymbols + off, 2);
			if (len < 2)
				break;
			size_t recordSize = 2 + len;
			if (off + recordSize > end)
				break;

			size_t recordStart = symbols_.size();
			symbols_.insert(symbols_.end(), pSymbols + off,
			                pSymbols + off + recordSize);
			while ((symbols_.size() - recordStart) % 4 != 0)
				symbols_.push_back(0);
			uint16_t alignedLen = static_cast<uint16_t>(
			    symbols_.size() - recordStart - 2);
			memcpy(&symbols_[recordStart], &alignedLen, 2);

			off += recordSize;
		}
		return 1;
	}
	int addPublic(const char* name, unsigned short seg, long off,
	              unsigned long type) override
	{
		// mod->AddPublic2 and dbi->AddPublic2 both end up registering the
		// same kind of S_PUB32 record in the shared symbol records stream;
		// route per-module calls through the writer-level addPublic so
		// there is exactly one builder accumulating publics.
		if (writer_)
			return writer_->addPublic(name, seg, off, type);
		return 1;
	}
	int addLines(const char* fname, unsigned short seg, long off, long size,
	             long /*off2*/, unsigned short firstLine,
	             unsigned char* pLineInfo, long cbLineInfo) override
	{
		// cv2pdb hands us one (function, source-file) pair per call.  Each
		// call lands as one DEBUG_S_LINES subsection in the module's C13
		// area, with its NameIndex pointing into a single per-module
		// DEBUG_S_FILECHKSMS subsection that we accumulate alongside.  The
		// source-file path itself goes into the global /names stream and
		// the FileChecksumEntryHeader's NameOffset references that.
		if (!fname || !pLineInfo || cbLineInfo <= 0 || !names_)
			return 1;
		if (cbLineInfo % sizeof(LineInfoEntry) != 0)
			return 1;
		uint32_t numLines =
		    static_cast<uint32_t>(cbLineInfo / sizeof(LineInfoEntry));
		if (numLines == 0)
			return 1;

		uint32_t namesOffset = names_->addName(fname);
		auto it = fileChecksumOffset_.find(namesOffset);
		uint32_t chksumOffset;
		if (it == fileChecksumOffset_.end())
		{
			chksumOffset = static_cast<uint32_t>(checksums_.size());
			fileChecksumOffset_[namesOffset] = chksumOffset;
			sourceFiles_.push_back(fname);
			// FileChecksumEntryHeader: NameOffset (u32) + ChecksumSize (u8)
			// + ChecksumKind (u8) = 6 bytes; pad to 4.  ChecksumKind = 0
			// (None) means "no checksum bytes follow".
			appendU32LE(checksums_, namesOffset);
			checksums_.push_back(0);    // ChecksumSize
			checksums_.push_back(0);    // ChecksumKind = None
			while (checksums_.size() % 4 != 0)
				checksums_.push_back(0);
		}
		else
		{
			chksumOffset = it->second;
		}

		const LineInfoEntry* entries =
		    reinterpret_cast<const LineInfoEntry*>(pLineInfo);

		std::vector<uint8_t> payload;
		// LineFragmentHeader: RelocOffset, RelocSegment, Flags, CodeSize.
		// cv2pdb's dwarflines.cpp computes `size` as the address-range
		// length already minus one: the comment in dwarflines.cpp around
		// the `--high_offset` line says "AddLines will immediately
		// increment it to 0", which is mspdb's mod->AddLines bumping the
		// value back up by one before it writes the CodeSize field on
		// the wire.  We have to do the same; without it the on-disk
		// CodeSize is N-1 for any line range of length N, and worse, a
		// single-instruction range comes in with `size = 0` which
		// underflows to 0xFFFFFFFF and dbghelp drops the entire
		// DEBUG_S_LINES subsection as malformed.
		uint32_t codeSize = static_cast<uint32_t>(size) + 1;
		appendU32LE(payload, static_cast<uint32_t>(off));
		appendU16LE(payload, seg);
		appendU16LE(payload, 0);                  // no columns
		appendU32LE(payload, codeSize);

		// LineBlockFragmentHeader: NameIndex (offset within FILECHKSMS),
		// NumLines, BlockSize (= sizeof(this) + NumLines * 8).
		appendU32LE(payload, chksumOffset);
		appendU32LE(payload, numLines);
		appendU32LE(payload, 12 + numLines * 8);

		for (uint32_t i = 0; i < numLines; i++)
		{
			uint32_t lineOff = entries[i].offset;
			uint32_t actualLine =
			    static_cast<uint32_t>(firstLine) + entries[i].line;
			uint32_t flags = (actualLine & 0xFFFFFF) | (1u << 31);   // IsStatement
			appendU32LE(payload, lineOff);
			appendU32LE(payload, flags);
		}

		// Wrap as DEBUG_S_LINES (Kind 0xF2) subsection.
		appendU32LE(linesSubs_, 0xF2);
		appendU32LE(linesSubs_, static_cast<uint32_t>(payload.size()));
		linesSubs_.insert(linesSubs_.end(), payload.begin(), payload.end());
		while (linesSubs_.size() % 4 != 0)
			linesSubs_.push_back(0);
		return 1;
	}
	int close() override { return 1; }

	std::vector<uint8_t> buildStream() const
	{
		std::vector<uint8_t> blob;
		appendU32LE(blob, 4);   // CV_SIGNATURE_C13
		blob.insert(blob.end(), symbols_.begin(), symbols_.end());

		// C13 subsection block: FILECHKSMS first (so per-line NameIndex
		// references are valid forward into this subsection's payload),
		// then any number of DEBUG_S_LINES subsections.
		if (!checksums_.empty())
		{
			appendU32LE(blob, 0xF4);   // DEBUG_S_FILECHKSMS
			appendU32LE(blob, static_cast<uint32_t>(checksums_.size()));
			blob.insert(blob.end(), checksums_.begin(), checksums_.end());
		}
		blob.insert(blob.end(), linesSubs_.begin(), linesSubs_.end());

		appendU32LE(blob, 0);   // GlobalRefCount
		return blob;
	}

	uint32_t symByteSize() const
	{
		// SymByteSize covers the C13 signature plus the S_* records.
		return 4 + static_cast<uint32_t>(symbols_.size());
	}
	uint32_t c13ByteSize() const
	{
		uint32_t size = static_cast<uint32_t>(linesSubs_.size());
		if (!checksums_.empty())
			size += 8 + static_cast<uint32_t>(checksums_.size());
		return size;
	}
	uint16_t sourceFileCount() const
	{
		return static_cast<uint16_t>(sourceFiles_.size());
	}
	const std::vector<std::string>& sourceFiles() const { return sourceFiles_; }

	std::vector<uint8_t> buildModInfoEntry(uint16_t streamIndex) const
	{
		std::vector<uint8_t> blob;
		appendU32LE(blob, 0);                       // Unused1
		const uint8_t* scBytes =
		    reinterpret_cast<const uint8_t*>(&primaryContrib_);
		blob.insert(blob.end(), scBytes,
		            scBytes + sizeof(primaryContrib_));

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
	TpiStreamBuilder* tpi_;
	uint16_t moduleIndex_;
	std::vector<SectionContribEntry>* allSecContribs_;
	NamesStreamBuilder* names_;
	PdbWriter* writer_;
	std::vector<uint8_t> symbols_;
	std::vector<uint8_t> checksums_;
	std::vector<uint8_t> linesSubs_;
	std::map<uint32_t, uint32_t> fileChecksumOffset_;
	std::vector<std::string> sourceFiles_;
	SectionContribEntry primaryContrib_;
	bool hasPrimary_ = false;
};

// Append-only buffer of CV symbol records that the Globals and Publics
// streams reference by byte offset.  Each record is 4-byte aligned (cv2pdb's
// only caller is addPublic, which builds an aligned S_PUB32 itself, but
// future callers might not).
class SymbolRecordsBuilder
{
public:
	uint32_t append(const std::vector<uint8_t>& record)
	{
		uint32_t offset = static_cast<uint32_t>(records_.size());
		records_.insert(records_.end(), record.begin(), record.end());
		while (records_.size() % 4 != 0)
			records_.push_back(0);
		return offset;
	}

	const std::vector<uint8_t>& bytes() const { return records_; }

private:
	std::vector<uint8_t> records_;
};

// Hash table over (name, recordOffset) tuples used by the Globals stream and,
// extended with an address map, by the Publics stream.  The on-disk layout
// is:
//   GsiHashHeader { VerSignature = 0xFFFFFFFF, VerHdr = 0xF12F091A,
//                   HrSize = NumRecords * 8,
//                   NumBuckets = sizeof(Bitmap) + NumPopulatedBuckets * 4 }
//   PSHashRecord HashRecords[NumRecords]   sorted by (bucket, lower(name))
//   uint32_t Bitmap[129]                   one bit per bucket; 4097 bits
//   uint32_t BucketOffsets[NumPopulatedBuckets]  byte offset of each
//                                                bucket's first HashRecord,
//                                                expressed in units of the
//                                                in-memory record size (12)
//                                                rather than the on-disk
//                                                size (8).
//
// Format references:
//   - microsoft/microsoft-pdb (PDB/dbi/gsi.cpp, IPHR_HASH = 4096)
//   - LLVM:
//     https://llvm.org/docs/PDB/HashStream.html
//     llvm/lib/DebugInfo/PDB/Native/GSIStreamBuilder.cpp
class GsiStreamBuilder
{
public:
	static constexpr uint32_t kHashTableSize = 4096;
	static constexpr uint32_t kBitmapBits = kHashTableSize + 1;     // 4097
	static constexpr uint32_t kBitmapBytes = ((kBitmapBits + 31) / 32) * 4;  // 516

	// Globals stream entries: just (name, record offset).
	void addGlobalEntry(const std::string& name, uint32_t recordOffset)
	{
		entries_.push_back({name, recordOffset, 0, 0, false});
	}

	// Publics stream entries: also carry segment + offset for the address map.
	void addPublicEntry(const std::string& name, uint32_t recordOffset,
	                    uint16_t segment, uint32_t offset)
	{
		entries_.push_back({name, recordOffset, segment, offset, true});
	}

	// Build the GSI hash payload: header + HashRecords + bitmap + bucket
	// offsets.  This is what the standalone Globals stream contains.
	std::vector<uint8_t> buildHashStream() const
	{
		std::vector<uint8_t> blob;
		appendHashStreamPayload(blob);
		return blob;
	}

	// Build the Publics stream: PSGSIHDR + GSI hash payload + AddressMap.
	// AddressMap is a uint32 array of byte offsets into the symbol records
	// stream, sorted by (segment, offset).
	std::vector<uint8_t> buildPublicsStream() const
	{
		std::vector<uint8_t> hashBlob;
		appendHashStreamPayload(hashBlob);

		std::vector<uint32_t> addrMap;
		addrMap.reserve(entries_.size());
		std::vector<size_t> indices(entries_.size());
		for (size_t i = 0; i < entries_.size(); i++)
			indices[i] = i;
		std::sort(indices.begin(), indices.end(),
		          [&](size_t a, size_t b) {
		              const Entry& ea = entries_[a];
		              const Entry& eb = entries_[b];
		              if (ea.segment != eb.segment)
		                  return ea.segment < eb.segment;
		              return ea.offset < eb.offset;
		          });
		for (size_t i : indices)
			addrMap.push_back(entries_[i].recordOffset);

		std::vector<uint8_t> blob;
		// PSGSIHDR (28 bytes)
		appendU32LE(blob, static_cast<uint32_t>(hashBlob.size()));    // SymHash
		appendU32LE(blob, static_cast<uint32_t>(addrMap.size() * 4)); // AddrMap
		appendU32LE(blob, 0);                                          // NumThunks
		appendU32LE(blob, 0);                                          // SizeOfThunk
		appendU16LE(blob, 0);                                          // ISectThunkTable
		appendU16LE(blob, 0);                                          // Padding
		appendU32LE(blob, 0);                                          // OffThunkTable
		appendU32LE(blob, 0);                                          // NumSections

		blob.insert(blob.end(), hashBlob.begin(), hashBlob.end());
		for (uint32_t off : addrMap)
			appendU32LE(blob, off);
		return blob;
	}

private:
	struct Entry
	{
		std::string name;
		uint32_t    recordOffset;
		uint16_t    segment;
		uint32_t    offset;
		bool        isPublic;
	};

	// Within-bucket sort key for the GSI hash table.  microsoft-pdb's
	// gsi.cpp uses caseInsensitiveComparePchPchCchCch which compares by
	// length FIRST and only then by case-insensitive byte content.  Its
	// HashSym lookup walks the bucket chain and exits early as soon as
	// the current entry's name compares greater than the target, so the
	// stored ordering is observable behaviour: getting it wrong makes
	// dbghelp miss the entry entirely and fall through to a different
	// hash table (typically picking the public over the global).  See
	// https://github.com/microsoft/microsoft-pdb/blob/master/PDB/dbi/gsi.cpp
	static bool nameLess(const std::string& a, const std::string& b)
	{
		if (a.size() != b.size())
			return a.size() < b.size();
		for (size_t i = 0; i < a.size(); i++)
		{
			unsigned char ca = static_cast<unsigned char>(a[i]);
			unsigned char cb = static_cast<unsigned char>(b[i]);
			if (ca >= 'A' && ca <= 'Z') ca = static_cast<unsigned char>(ca + 32);
			if (cb >= 'A' && cb <= 'Z') cb = static_cast<unsigned char>(cb + 32);
			if (ca != cb)
				return ca < cb;
		}
		return false;
	}

	void appendHashStreamPayload(std::vector<uint8_t>& blob) const
	{
		// Hash each entry; sort by (bucket, name).
		std::vector<std::pair<uint32_t, size_t>> hashed;
		hashed.reserve(entries_.size());
		for (size_t i = 0; i < entries_.size(); i++)
		{
			uint32_t h = hashStringV1(entries_[i].name.data(),
			                          entries_[i].name.size());
			uint32_t bucket = h % kHashTableSize;
			hashed.emplace_back(bucket, i);
		}
		std::sort(hashed.begin(), hashed.end(),
		          [&](const std::pair<uint32_t, size_t>& a,
		              const std::pair<uint32_t, size_t>& b) {
		              if (a.first != b.first)
		                  return a.first < b.first;
		              return nameLess(entries_[a.second].name,
		                              entries_[b.second].name);
		          });

		// Bitmap of populated buckets (4097 bits) and bucket-offset table.
		// Bucket offsets are expressed in units of 12 bytes (the in-memory
		// record size) per microsoft-pdb's gsi.cpp / LLVM convention, even
		// though on-disk HashRecord is 8 bytes.
		std::vector<uint8_t> bitmap(kBitmapBytes, 0);
		std::vector<uint32_t> bucketOffsets;
		int32_t prevBucket = -1;
		for (uint32_t i = 0; i < hashed.size(); i++)
		{
			uint32_t b = hashed[i].first;
			if (static_cast<int32_t>(b) != prevBucket)
			{
				bitmap[b / 8] |= static_cast<uint8_t>(1u << (b % 8));
				bucketOffsets.push_back(i * 12);
				prevBucket = static_cast<int32_t>(b);
			}
		}

		// HashRecords array: 8 bytes per entry.  Off = recordOffset + 1
		// (off-by-one is part of the format; 0 marks "no record").
		std::vector<uint8_t> hashRecords;
		hashRecords.reserve(hashed.size() * 8);
		for (const auto& kv : hashed)
		{
			appendU32LE(hashRecords, entries_[kv.second].recordOffset + 1);
			appendU32LE(hashRecords, 1);          // CRef
		}

		uint32_t hrSize = static_cast<uint32_t>(hashRecords.size());
		uint32_t numBucketsField =
		    kBitmapBytes + static_cast<uint32_t>(bucketOffsets.size()) * 4;

		appendU32LE(blob, 0xFFFFFFFF);            // VerSignature
		appendU32LE(blob, 0xF12F091A);            // VerHdr (GSIHashSC V70)
		appendU32LE(blob, hrSize);
		appendU32LE(blob, numBucketsField);
		blob.insert(blob.end(), hashRecords.begin(), hashRecords.end());
		blob.insert(blob.end(), bitmap.begin(), bitmap.end());
		for (uint32_t off : bucketOffsets)
			appendU32LE(blob, off);
	}

	std::vector<Entry> entries_;
};

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
		uint16_t modIndex = static_cast<uint16_t>(mods_.size());
		auto* m = new ModuleStreamBuilder(
		    objName ? std::string(objName) : std::string(),
		    libName ? std::string(libName) : std::string(),
		    &tpi_, modIndex, &sectionContribs_, &names_, this);
		mods_.push_back(m);
		*outMod = m;
		return 1;
	}

	int addSec(unsigned short, unsigned short, long, long) override { return 1; }
	int addPublic(const char* name, unsigned short seg, long off,
	              unsigned long /*type*/) override
	{
		// cv2pdb passes a CodeView type index in the type argument; S_PUB32
		// has no type field (the addendum's open question), so the value
		// is dropped on the floor and Flags stays 0.  Build the record
		// manually and stash it in the shared symbol records stream, then
		// register the (name, recordOffset, segment, offset) tuple in the
		// publics builder so the GSI hash and address map cover it.
		if (!name)
			return 1;

		std::string sName(name);
		std::vector<uint8_t> record;
		// Reserve the 2-byte length prefix; fill in once the payload is
		// laid out and padded to 4-byte alignment.
		record.push_back(0);
		record.push_back(0);
		appendU16LE(record, 0x110E);                // S_PUB32
		appendU32LE(record, 0);                     // Flags (none)
		appendU32LE(record, static_cast<uint32_t>(off));
		appendU16LE(record, seg);
		record.insert(record.end(), sName.begin(), sName.end());
		record.push_back(0);                        // null terminator
		while (record.size() % 4 != 0)
			record.push_back(0);
		uint16_t lenField = static_cast<uint16_t>(record.size() - 2);
		record[0] = static_cast<uint8_t>(lenField & 0xFF);
		record[1] = static_cast<uint8_t>((lenField >> 8) & 0xFF);

		uint32_t recordOffset = symbolRecords_.append(record);
		publics_.addPublicEntry(sName, recordOffset, seg,
		                        static_cast<uint32_t>(off));
		return 1;
	}

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

	int setImageSectionHeaders(const void* data, size_t size) override
	{
		if (data && size > 0)
			sectionHeaders_.assign(
			    static_cast<const uint8_t*>(data),
			    static_cast<const uint8_t*>(data) + size);
		return 1;
	}

	int commit() override
	{
		MsfBuilder msf;

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

		// Build SourceInfo from accumulated per-module file lists.  Layout
		// is: NumModules / NumSourceFiles (truncated to u16) / ModIndices
		// (zeros, ignored by readers) / ModFileCounts / FileNameOffsets[
		// totalFiles] / NamesBuffer.  The path strings here are a separate
		// copy from the global /names buffer; older readers consult this
		// substream directly while newer ones go through the C13
		// FILECHKSMS / NameOffset chain.
		std::vector<uint8_t> sourceInfo;
		uint32_t totalFiles = 0;
		for (ModuleStreamBuilder* m : mods_)
			totalFiles += static_cast<uint32_t>(m->sourceFiles().size());
		uint16_t numFilesU16 = totalFiles > 0xFFFF
		    ? static_cast<uint16_t>(0xFFFF)
		    : static_cast<uint16_t>(totalFiles);
		appendU16LE(sourceInfo, static_cast<uint16_t>(mods_.size()));
		appendU16LE(sourceInfo, numFilesU16);
		for (size_t i = 0; i < mods_.size(); i++)
			appendU16LE(sourceInfo, 0);                  // ModIndices
		for (ModuleStreamBuilder* m : mods_)
			appendU16LE(sourceInfo,
			            static_cast<uint16_t>(m->sourceFiles().size()));
		std::vector<uint32_t> fileOffsets;
		std::vector<uint8_t> sourceNames;
		for (ModuleStreamBuilder* m : mods_)
		{
			for (const std::string& path : m->sourceFiles())
			{
				fileOffsets.push_back(
				    static_cast<uint32_t>(sourceNames.size()));
				sourceNames.insert(sourceNames.end(), path.begin(), path.end());
				sourceNames.push_back(0);
			}
		}
		for (uint32_t fo : fileOffsets)
			appendU32LE(sourceInfo, fo);
		sourceInfo.insert(sourceInfo.end(), sourceNames.begin(), sourceNames.end());
		padToAlign4(sourceInfo);
		dbi.setSourceInfoSubstream(std::move(sourceInfo));
		dbi.setSectionContribs(sectionContribs_);

		// Allocate stream indices for the GSI/PSI/SymbolRecords trio that
		// the DBI header points at.  Modules occupy 8..7+N, then Globals,
		// Publics, and SymbolRecords come right after.  Wire those indices
		// into the DBI header *before* DBI is serialised.
		uint16_t globalsIndex =
		    static_cast<uint16_t>(8 + mods_.size());
		uint16_t publicsIndex =
		    static_cast<uint16_t>(8 + mods_.size() + 1);
		uint16_t symRecordsIndex =
		    static_cast<uint16_t>(8 + mods_.size() + 2);
		dbi.setGlobalSymbolStreamIndex(globalsIndex);
		dbi.setPublicSymbolStreamIndex(publicsIndex);
		dbi.setSymRecordStreamIndex(symRecordsIndex);

		// Optional Section Header debug stream sits right after the
		// SymbolRecords stream when present.  When cv2pdb didn't hand us
		// the bytes, leave the slot empty and let DbiStreamBuilder default
		// to kInvalidStreamIndex.
		uint16_t sectionHdrIndex = 0xFFFF;
		if (!sectionHeaders_.empty())
		{
			sectionHdrIndex =
			    static_cast<uint16_t>(8 + mods_.size() + 3);
			dbi.setSectionHeaderStreamIndex(sectionHdrIndex);
		}

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

		msf.addStream(tpi_.buildStream(kTpiHashIndex));  // 2: TPI
		msf.addStream(dbi.buildStream(machine_));        // 3: DBI
		msf.addStream(ipi.buildStream(kIpiHashIndex));   // 4: IPI
		msf.addStream(tpi_.buildHashStream());           // 5: TPI hash
		msf.addStream(ipi.buildHashStream());            // 6: IPI hash
		msf.addStream(names_.buildStream());             // 7: /names

		// Per-module symbol streams (8, 9, ...) in openMod order.
		for (ModuleStreamBuilder* m : mods_)
			msf.addStream(m->buildStream());

		// Globals / Publics / SymbolRecords streams (indices wired into
		// the DBI header above).
		msf.addStream(globals_.buildHashStream());
		msf.addStream(publics_.buildPublicsStream());
		msf.addStream(symbolRecords_.bytes());

		// Optional Section Header debug stream when cv2pdb supplied bytes.
		if (!sectionHeaders_.empty())
			msf.addStream(sectionHeaders_);

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
	TpiStreamBuilder tpi_;
	std::vector<SectionContribEntry> sectionContribs_;
	SymbolRecordsBuilder symbolRecords_;
	GsiStreamBuilder globals_;
	GsiStreamBuilder publics_;
	std::vector<uint8_t> sectionHeaders_;
};

}  // namespace

PdbWriter* createNativePdbWriter(const wchar_t* pdbname)
{
	return new NativePdbWriter(pdbname);
}

}  // namespace cv2pdb
