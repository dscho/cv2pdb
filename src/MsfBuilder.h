// Convert DMD CodeView debug information to PDB files
// Copyright (c) 2026 by Johannes Schindelin, All Rights Reserved
//
// License for redistribution is given by the Artistic License 2.0
// see file LICENSE for further details

// MsfBuilder lays out a Multi-Stream File container, the on-disk envelope
// every PDB lives in.  The class accepts a list of stream contents and
// produces a structurally valid MSF: SuperBlock, Free Page Map, stream
// directory, and block-map page wired up so that any reader following the
// publicly documented MSF/PDB format can recover the streams.
//
// Format references:
//   - Microsoft's microsoft-pdb reference repository:
//     https://github.com/microsoft/microsoft-pdb (in particular PDB/msf/msf.cpp
//     and PDB/include/msf.h, the original sources behind the format).
//   - LLVM PDB documentation, derived from the same sources and used to
//     drive LLVM's own PDB reader and writer:
//     https://llvm.org/docs/PDB/MsfFile.html
//     https://llvm.org/docs/PDB/index.html
//
// Limited to small files for now: the FPM is laid out across pages 1 and 2
// only, which caps usable file size at BlockSize * 8 pages (= 128 MiB at
// 4 KiB blocks).  Larger PDBs will need the periodic FPM pages at multiples
// of BlockSize.

#ifndef __MSFBUILDER_H__
#define __MSFBUILDER_H__

#include <cstdint>
#include <string>
#include <vector>

namespace cv2pdb {

class MsfBuilder
{
public:
	static constexpr uint32_t kBlockSize = 4096;

	MsfBuilder() = default;

	// Append a stream and return its assigned 0-based index.  The blob is
	// taken by value so the caller can build it incrementally and then
	// hand ownership to the builder.  Stream 0 is conventionally the
	// "Old MSF Directory" placeholder; cv2pdb passes it empty.
	uint32_t addStream(std::vector<uint8_t> data);

	// Compute the page layout, write the file at path, and close it.
	// Returns true on success; on failure lastError() carries a short
	// diagnostic.
	bool write(const std::wstring& path);

	const std::string& lastError() const { return err_; }

private:
	std::vector<std::vector<uint8_t>> streams_;
	std::string err_;
};

}  // namespace cv2pdb

#endif  // __MSFBUILDER_H__
