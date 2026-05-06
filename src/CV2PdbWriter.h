// Convert DMD CodeView debug information to PDB files
// Copyright (c) 2009-2010 by Rainer Schuetze, All Rights Reserved
// Copyright (c) 2026 by Johannes Schindelin, All Rights Reserved
//
// License for redistribution is given by the Artistic License 2.0
// see file LICENSE for further details

// Abstract PDB-writer backend used by cv2pdb.
//
// The legacy backend forwards every operation to mspdb*.dll via the
// mspdb::PDB / DBI / TPI / Mod COM-style interfaces.  This header defines a
// thin C++ abstraction that captures the small subset of those calls cv2pdb
// actually uses, so a future in-house backend can be slotted in without
// touching the rest of the converter.  The first concrete implementation,
// MsPdbWriter, is just a pass-through adapter around the existing
// mspdb pointers; see CV2PdbWriter.cpp.

#ifndef __CV2PDBWRITER_H__
#define __CV2PDBWRITER_H__

#include <stdint.h>
#include <windows.h>

namespace cv2pdb {

// One CodeView line-number record, as consumed by ModWriter::addLines.
// Layout matches what mspdb's ModCommon::AddLines expects when an array of
// these is passed as the pLineInfo blob.
#pragma pack(push, 1)
struct LineInfoEntry
{
	unsigned int   offset;
	unsigned short line;
};
#pragma pack(pop)

// Per-module writer.  Lifetime is owned by the parent PdbWriter; callers must
// not delete instances directly.  All methods return a non-zero "true-ish"
// status on success, mirroring the mspdb convention.
class ModWriter
{
public:
	virtual ~ModWriter() {}

	virtual int addSecContrib(unsigned short seg, long off, long size, unsigned long secflags) = 0;
	virtual int addTypes(unsigned char* pTypes, long cbTypes) = 0;
	virtual int addSymbols(unsigned char* pSymbols, long cbSymbols) = 0;
	virtual int addPublic(const char* name, unsigned short seg, long off, unsigned long type) = 0;
	virtual int addLines(const char* fname, unsigned short seg, long off, long size,
	                     long off2, unsigned short firstLine,
	                     unsigned char* pLineInfo, long cbLineInfo) = 0;

	virtual int close() = 0;
};

// Top-level writer.  Wraps the lifecycle of one output PDB.
class PdbWriter
{
public:
	virtual ~PdbWriter() {}

	// Initialise the DBI / TPI / IPI sub-streams.  These are split out from
	// the factory so callers can produce distinct error messages and so
	// IPI can be skipped on older formats (cv2pdb only opens it for
	// VS14+).  Returns the same int convention as mspdb (positive on
	// success, <= 0 on failure).
	virtual int initDbi() = 0;
	virtual int initTpi() = 0;
	virtual int initIpi() = 0;

	virtual int setMachineType(unsigned short machine) = 0;

	// Returns a borrowed pointer; the writer retains ownership.  *outMod is
	// set to NULL on failure.
	virtual int openMod(const char* objName, const char* libName, ModWriter** outMod) = 0;

	virtual int addSec(unsigned short frame, unsigned short flags, long off, long size) = 0;
	virtual int addPublic(const char* name, unsigned short seg, long off, unsigned long type) = 0;

	virtual int querySignature(GUID* guid) = 0;
	virtual int queryAge() = 0;

	// Fills buf with the textual mspdb error message; safe no-op on backends
	// that have no equivalent.  Buffer must be at least 256 bytes.
	virtual int queryLastError(char* buf) = 0;

	// Hand the writer the input PE/COFF image's IMAGE_SECTION_HEADER array
	// so it can be copied verbatim into the optional Section Header debug
	// stream.  Visual Studio uses that stream to map RVAs to symbols when
	// loading a PDB.  Backends without a native API for this (mspdb) may
	// return 1 without doing anything; the resulting PDB simply lacks the
	// auxiliary stream, which matches the legacy cv2pdb-mspdb output.
	virtual int setImageSectionHeaders(const void* data, size_t size) = 0;

	// Closes IPI/TPI/DBI/PDB and writes the file.
	virtual int commit() = 0;

	// Releases the PDB without committing; used on error paths.
	virtual int close() = 0;
};

// Construct a writer backed by mspdb*.dll.  Returns NULL if no compatible DLL
// could be located on the system.  On success, the caller must eventually
// invoke commit() or close() and then delete the writer.
PdbWriter* createMsPdbWriter(const wchar_t* pdbname);

// Construct an in-house writer that produces a PDB file directly, with no
// dependency on the legacy mspdb*.dll runtime.  The returned writer is
// deliberately incomplete at this stage: it accepts every add* call and
// discards the data, then on commit() writes a structurally valid but
// otherwise empty PDB.  Type, symbol, and line tables will be filled in by
// later commits.  Returns NULL on allocation failure.
PdbWriter* createNativePdbWriter(const wchar_t* pdbname);

}  // namespace cv2pdb

#endif  // __CV2PDBWRITER_H__
