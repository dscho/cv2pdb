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
		//   Features       { (none for now; will be VC140 once IPI exists) }
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
		msf.addStream(std::move(info));

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
