// Convert DMD CodeView debug information to PDB files
// Copyright (c) 2026 by Johannes Schindelin, All Rights Reserved
//
// License for redistribution is given by the Artistic License 2.0
// see file LICENSE for further details

// MsfBuilder writes the MSF (Multi-Stream File) container that every PDB
// lives in.  Format references:
//   - microsoft/microsoft-pdb (PDB/msf/msf.cpp, PDB/include/msf.h):
//     https://github.com/microsoft/microsoft-pdb
//   - LLVM PDB documentation:
//     https://llvm.org/docs/PDB/MsfFile.html

#include "MsfBuilder.h"

#include <windows.h>

#include <cassert>
#include <cstring>

namespace cv2pdb {

namespace {

constexpr uint32_t kBlockSize = MsfBuilder::kBlockSize;

// "Microsoft C/C++ MSF 7.00\r\n\x1ADS\0\0\0", 32 bytes.
const uint8_t kMsfMagic[32] = {
	'M','i','c','r','o','s','o','f','t',' ',
	'C','/','C','+','+',' ','M','S','F',' ',
	'7','.','0','0','\r','\n', 0x1A,
	'D','S', 0x00, 0x00, 0x00,
};

uint32_t numPagesFor(uint32_t bytes)
{
	return (bytes + kBlockSize - 1) / kBlockSize;
}

void appendU32LE(std::vector<uint8_t>& v, uint32_t x)
{
	for (int i = 0; i < 4; i++)
		v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xff));
}

void putU32LE(uint8_t* p, uint32_t x)
{
	for (int i = 0; i < 4; i++)
		p[i] = static_cast<uint8_t>((x >> (8 * i)) & 0xff);
}

class FileSink
{
public:
	FileSink() : h_(INVALID_HANDLE_VALUE) {}
	~FileSink() { close(); }

	bool open(const std::wstring& path)
	{
		h_ = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr,
		                 CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
		return h_ != INVALID_HANDLE_VALUE;
	}

	bool writePage(const uint8_t* data, uint32_t len)
	{
		uint8_t buf[kBlockSize];
		assert(len <= kBlockSize);
		memcpy(buf, data, len);
		if (len < kBlockSize)
			memset(buf + len, 0, kBlockSize - len);
		DWORD written = 0;
		if (!WriteFile(h_, buf, kBlockSize, &written, nullptr))
			return false;
		return written == kBlockSize;
	}

	bool writePages(const std::vector<uint8_t>& blob, uint32_t pageCount)
	{
		for (uint32_t p = 0; p < pageCount; p++)
		{
			uint32_t off = p * kBlockSize;
			uint32_t take = kBlockSize;
			if (off + take > blob.size())
				take = static_cast<uint32_t>(blob.size() - off);
			if (!writePage(blob.data() + off, take))
				return false;
		}
		return true;
	}

	void close()
	{
		if (h_ != INVALID_HANDLE_VALUE)
		{
			CloseHandle(h_);
			h_ = INVALID_HANDLE_VALUE;
		}
	}

private:
	HANDLE h_;
};

}  // namespace

uint32_t MsfBuilder::addStream(std::vector<uint8_t> data)
{
	uint32_t idx = static_cast<uint32_t>(streams_.size());
	streams_.push_back(std::move(data));
	return idx;
}

bool MsfBuilder::write(const std::wstring& path)
{
	if (streams_.empty())
	{
		err_ = "MsfBuilder: no streams to write";
		return false;
	}

	// Pages 0/1/2 are reserved for SuperBlock and the two FPM banks.
	uint32_t nextPage = 3;

	std::vector<std::vector<uint32_t>> streamPages(streams_.size());
	for (size_t i = 0; i < streams_.size(); i++)
	{
		uint32_t n = numPagesFor(static_cast<uint32_t>(streams_[i].size()));
		streamPages[i].reserve(n);
		for (uint32_t p = 0; p < n; p++)
			streamPages[i].push_back(nextPage++);
	}

	// Stream directory blob: NumStreams, sizes, then page lists.
	std::vector<uint8_t> directory;
	appendU32LE(directory, static_cast<uint32_t>(streams_.size()));
	for (const auto& s : streams_)
		appendU32LE(directory, static_cast<uint32_t>(s.size()));
	for (const auto& pages : streamPages)
		for (uint32_t p : pages)
			appendU32LE(directory, p);

	uint32_t dirNumPages = numPagesFor(static_cast<uint32_t>(directory.size()));
	std::vector<uint32_t> dirPages;
	dirPages.reserve(dirNumPages);
	for (uint32_t p = 0; p < dirNumPages; p++)
		dirPages.push_back(nextPage++);

	// BlockMap: a single page listing the directory's page indices.  The
	// directory must be small enough that its page-index list fits in one
	// page (kBlockSize / 4 entries, i.e. 1024 directory pages = 4 MiB of
	// directory blob).  cv2pdb is nowhere near that.
	if (dirNumPages > kBlockSize / 4)
	{
		err_ = "MsfBuilder: stream directory too large for single block-map page";
		return false;
	}
	uint32_t blockMapAddr = nextPage++;

	uint32_t numBlocks = nextPage;

	// FPM bookkeeping: one bit per page, 1 = free, 0 = allocated.  We hold
	// the bitmap in a single FPM page (page 1), which limits us to
	// kBlockSize * 8 = 32768 pages = 128 MiB at 4 KiB blocks.  Larger
	// files need the periodic FPM pages at multiples of kBlockSize; see
	// the plan document.
	if (numBlocks > kBlockSize * 8)
	{
		err_ = "MsfBuilder: file would exceed single-FPM-page limit";
		return false;
	}
	std::vector<uint8_t> fpm(kBlockSize, 0xff);
	for (uint32_t p = 0; p < numBlocks; p++)
		fpm[p / 8] &= static_cast<uint8_t>(~(1u << (p % 8)));

	// SuperBlock.
	std::vector<uint8_t> superBlock(kBlockSize, 0);
	memcpy(superBlock.data(), kMsfMagic, sizeof(kMsfMagic));
	putU32LE(superBlock.data() + 32, kBlockSize);
	putU32LE(superBlock.data() + 36, 1);  // FreeBlockMapBlock = 1
	putU32LE(superBlock.data() + 40, numBlocks);
	putU32LE(superBlock.data() + 44, static_cast<uint32_t>(directory.size()));
	putU32LE(superBlock.data() + 48, 0);  // Unknown
	putU32LE(superBlock.data() + 52, blockMapAddr);

	// BlockMap: contiguous list of directory page indices, padded to a
	// full page by FileSink::writePage.
	std::vector<uint8_t> blockMap;
	blockMap.reserve(dirPages.size() * 4);
	for (uint32_t p : dirPages)
		appendU32LE(blockMap, p);

	FileSink out;
	if (!out.open(path))
	{
		err_ = "MsfBuilder: CreateFileW failed";
		return false;
	}

	auto fail = [&](const char* what) {
		err_ = what;
		out.close();
		DeleteFileW(path.c_str());
		return false;
	};

	if (!out.writePage(superBlock.data(), kBlockSize))
		return fail("MsfBuilder: write SuperBlock failed");
	if (!out.writePage(fpm.data(), kBlockSize))
		return fail("MsfBuilder: write FPM (page 1) failed");
	if (!out.writePage(fpm.data(), kBlockSize))
		return fail("MsfBuilder: write FPM (page 2) failed");

	for (size_t i = 0; i < streams_.size(); i++)
	{
		uint32_t pageCount = static_cast<uint32_t>(streamPages[i].size());
		if (!out.writePages(streams_[i], pageCount))
			return fail("MsfBuilder: write stream pages failed");
	}

	if (!out.writePages(directory, dirNumPages))
		return fail("MsfBuilder: write directory failed");

	if (!out.writePage(blockMap.data(), static_cast<uint32_t>(blockMap.size())))
		return fail("MsfBuilder: write block map failed");

	out.close();
	return true;
}

}  // namespace cv2pdb
