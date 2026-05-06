// Convert DMD CodeView debug information to PDB files
// Copyright (c) 2009-2010 by Rainer Schuetze, All Rights Reserved
// Copyright (c) 2026 by Johannes Schindelin, All Rights Reserved
//
// License for redistribution is given by the Artistic License 2.0
// see file LICENSE for further details

// MsPdbWriter: thin pass-through adapter that satisfies the cv2pdb::PdbWriter
// contract by forwarding every operation to the legacy mspdb*.dll-backed
// objects.  This is the only translation unit that talks to mspdb::* types
// directly (apart from mspdb.cpp itself, which keeps the dynamic-load shim).

#include "CV2PdbWriter.h"
#include "mspdb.h"

#include <vector>

namespace cv2pdb {

namespace {

class MsPdbModWriter : public ModWriter
{
public:
	explicit MsPdbModWriter(mspdb::Mod* mod) : mod_(mod) {}

	int addSecContrib(unsigned short seg, long off, long size, unsigned long secflags) override
	{
		return mod_->AddSecContrib(seg, off, size, secflags);
	}

	int addTypes(unsigned char* pTypes, long cbTypes) override
	{
		return mod_->AddTypes(pTypes, cbTypes);
	}

	int addSymbols(unsigned char* pSymbols, long cbSymbols) override
	{
		return mod_->AddSymbols(pSymbols, cbSymbols);
	}

	int addPublic(const char* name, unsigned short seg, long off, unsigned long type) override
	{
		return mod_->AddPublic2(name, seg, off, type);
	}

	int addLines(const char* fname, unsigned short seg, long off, long size,
	             long off2, unsigned short firstLine,
	             unsigned char* pLineInfo, long cbLineInfo) override
	{
		return mod_->AddLines(fname, seg, off, size, off2, firstLine, pLineInfo, cbLineInfo);
	}

	int close() override
	{
		if (!mod_)
			return 1;
		int rc = mod_->Close();
		mod_ = nullptr;
		return rc;
	}

private:
	mspdb::Mod* mod_;
};

class MsPdbWriter : public PdbWriter
{
public:
	explicit MsPdbWriter(mspdb::PDB* pdb) : pdb_(pdb), dbi_(nullptr), tpi_(nullptr), ipi_(nullptr) {}

	~MsPdbWriter() override
	{
		// If commit() was not called the underlying mspdb objects still need
		// to be released to match the legacy cleanup path.
		closeAll(/*commit=*/false);
		for (MsPdbModWriter* m : mods_)
			delete m;
	}

	int initDbi() override
	{
		return pdb_ ? pdb_->CreateDBI("", &dbi_) : 0;
	}

	int initTpi() override
	{
		return pdb_ ? pdb_->OpenTpi("rw", &tpi_) : 0;
	}

	int initIpi() override
	{
		return pdb_ ? pdb_->OpenIpi("rw", &ipi_) : 0;
	}

	int setMachineType(unsigned short machine) override
	{
		if (!dbi_)
			return 0;
		dbi_->SetMachineType(machine);
		return 1;
	}

	int openMod(const char* objName, const char* libName, ModWriter** outMod) override
	{
		*outMod = nullptr;
		if (!dbi_)
			return 0;
		mspdb::Mod* mod = nullptr;
		int rc = dbi_->OpenMod(objName, libName, &mod);
		if (rc <= 0 || !mod)
			return rc;
		MsPdbModWriter* wrap = new MsPdbModWriter(mod);
		mods_.push_back(wrap);
		*outMod = wrap;
		return rc;
	}

	int addSec(unsigned short frame, unsigned short flags, long off, long size) override
	{
		return dbi_ ? dbi_->AddSec(frame, flags, off, size) : 0;
	}

	int addPublic(const char* name, unsigned short seg, long off, unsigned long type) override
	{
		return dbi_ ? dbi_->AddPublic2(name, seg, off, type) : 0;
	}

	int querySignature(GUID* guid) override
	{
		return pdb_ ? pdb_->QuerySignature2(guid) : 0;
	}

	int queryAge() override
	{
		return pdb_ ? pdb_->QueryAge() : 0;
	}

	int queryLastError(char* buf) override
	{
		return pdb_ ? pdb_->QueryLastError(buf) : 0;
	}

	int setImageSectionHeaders(const void*, size_t) override
	{
		// mspdb has no equivalent API.  Return success so callers see the
		// legacy behaviour: no Section Header debug stream in the output.
		return 1;
	}

	int commit() override
	{
		return closeAll(/*commit=*/true);
	}

	int close() override
	{
		return closeAll(/*commit=*/false);
	}

private:
	int closeAll(bool doCommit)
	{
		int rc = 1;
		if (ipi_) { ipi_->Close(); ipi_ = nullptr; }
		if (tpi_) { tpi_->Close(); tpi_ = nullptr; }
		if (dbi_) { dbi_->Close(); dbi_ = nullptr; }
		if (pdb_)
		{
			if (doCommit)
				rc = pdb_->Commit();
			pdb_->Close();
			pdb_ = nullptr;
		}
		return rc;
	}

	mspdb::PDB* pdb_;
	mspdb::DBI* dbi_;
	mspdb::TPI* tpi_;
	mspdb::TPI* ipi_;
	std::vector<MsPdbModWriter*> mods_;
};

}  // namespace

PdbWriter* createMsPdbWriter(const wchar_t* pdbname)
{
	mspdb::PDB* pdb = ::CreatePDB(pdbname);
	if (!pdb)
		return nullptr;
	return new MsPdbWriter(pdb);
}

}  // namespace cv2pdb
