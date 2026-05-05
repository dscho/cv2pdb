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
#include <string>
#include <vector>

namespace cv2pdb {

namespace {

void appendU32LE(std::vector<uint8_t>& v, uint32_t x)
{
	for (int i = 0; i < 4; i++)
		v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xff));
}

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

class NativeModWriter : public ModWriter
{
public:
	int addSecContrib(unsigned short, long, long, unsigned long) override { return 1; }
	int addTypes(unsigned char*, long) override { return 1; }
	int addSymbols(unsigned char*, long) override { return 1; }
	int addPublic(const char*, unsigned short, long, unsigned long) override { return 1; }
	int addLines(const char*, unsigned short, long, long, long, unsigned short,
	             unsigned char*, long) override { return 1; }
	int close() override { return 1; }
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
		for (NativeModWriter* m : mods_)
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

	int openMod(const char*, const char*, ModWriter** outMod) override
	{
		NativeModWriter* m = new NativeModWriter();
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

		// Stream 0: "Old MSF Directory" placeholder, empty (lld-link does
		// the same; mspdb keeps 40 stale bytes from the previous commit
		// but no current consumer reads it).
		msf.addStream({});

		// Stream 1: PDB Info Stream.  Layout (little-endian):
		//   PdbStreamHeader { Version (VC70), Signature, Age, Guid }
		//   NameMap        { StringBufferSize=0,
		//                    Size=0, Capacity=1,
		//                    PresentBitmapWordCount=0,
		//                    DeletedBitmapWordCount=0 }
		//   Features       { VC140 = 0x01331E94 }
		// VC140 advertises the presence of an IPI stream, which we write
		// below; emitting the feature without the matching stream would
		// be self-contradictory.
		std::vector<uint8_t> info;
		appendU32LE(info, 20000404);          // VC70
		appendU32LE(info, signature_);
		appendU32LE(info, 1);                 // Age
		const uint8_t* guidBytes = reinterpret_cast<const uint8_t*>(&guid_);
		info.insert(info.end(), guidBytes, guidBytes + sizeof(guid_));
		appendU32LE(info, 0);                 // StringBufferSize
		appendU32LE(info, 0);                 // Size
		appendU32LE(info, 1);                 // Capacity (must be > 0)
		appendU32LE(info, 0);                 // PresentBitmapWordCount
		appendU32LE(info, 0);                 // DeletedBitmapWordCount
		appendU32LE(info, 0x013351DC);        // VC140 (= 20140508)
		msf.addStream(std::move(info));

		// Stream layout from here matches the conventional PDB indices:
		//   2 = TPI, 3 = DBI, 4 = IPI.  The TPI/IPI hash sub-streams are
		//   placed at 5 and 6 respectively; those indices must be wired
		//   into the TPI/IPI headers before they are added to the MSF.
		const uint16_t kTpiHashIndex = 5;
		const uint16_t kIpiHashIndex = 6;

		msf.addStream(tpi.buildStream(kTpiHashIndex));   // 2: TPI
		msf.addStream({});                               // 3: DBI placeholder
		msf.addStream(ipi.buildStream(kIpiHashIndex));   // 4: IPI
		msf.addStream(tpi.buildHashStream());            // 5: TPI hash
		msf.addStream(ipi.buildHashStream());            // 6: IPI hash

		return msf.write(path_) ? 1 : 0;
	}

	int close() override { return 1; }

private:
	std::wstring path_;
	GUID guid_;
	uint32_t signature_;
	unsigned short machine_ = 0;
	std::vector<NativeModWriter*> mods_;
};

}  // namespace

PdbWriter* createNativePdbWriter(const wchar_t* pdbname)
{
	return new NativePdbWriter(pdbname);
}

}  // namespace cv2pdb
