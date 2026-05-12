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
#include <set>
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

// Returns the byte size of a CV numeric leaf at `p` (the first uint16 chooses
// the encoding).  Caps at `available` for safety on truncated input.
size_t numericLeafSize(const uint8_t* p, size_t available)
{
	if (available < 2)
		return 0;
	uint16_t v;
	memcpy(&v, p, 2);
	if (v < 0x8000)
		return 2;
	switch (v)
	{
		case 0x8000: return 3;     // LF_CHAR
		case 0x8001: return 4;     // LF_SHORT
		case 0x8002: return 4;     // LF_USHORT
		case 0x8003: return 6;     // LF_LONG
		case 0x8004: return 6;     // LF_ULONG
		case 0x8005: return 6;     // LF_REAL32
		case 0x8006: return 10;    // LF_REAL64
		case 0x8009: return 10;    // LF_QUADWORD
		case 0x800A: return 10;    // LF_UQUADWORD
		default:     return 2;     // unknown: treat as bare uint16
	}
}

// Length of a null-terminated string at `p`, including the null, capped at
// `available`.
size_t nullTermSize(const uint8_t* p, size_t available)
{
	for (size_t i = 0; i < available; i++)
		if (p[i] == 0)
			return i + 1;
	return available;
}

// Walks the subrecord stream that backs an LF_FIELDLIST_V2 payload, pushing
// the byte offset (relative to the fieldlist's payload start) of each
// 4-byte type-index field onto `refs`.  Returns true if every subrecord kind
// is one we know how to skip past.  Returns false if it hits an unknown
// subrecord, in which case the caller must treat the parent fieldlist (and
// the rest of the TPI dedup pass) conservatively.
bool walkFieldlistRefs(const uint8_t* payload, size_t size,
                       std::vector<uint32_t>& refs)
{
	size_t off = 0;
	while (off + 2 <= size)
	{
		// LF_PAD0..LF_PAD8 (0xF0..0xF8) are padding bytes between
		// subrecords; skip them.
		while (off < size && payload[off] >= 0xF0 && payload[off] <= 0xF8)
			off++;
		if (off + 2 > size)
			break;

		uint16_t subKind;
		memcpy(&subKind, payload + off, 2);

		switch (subKind)
		{
			case 0x1502: // LF_ENUMERATE_V3: id(2) + attr(2) + value(numeric) + name
			case 0x0403: // LF_ENUMERATE_V1
			{
				size_t valueStart = off + 4;
				if (valueStart > size) return false;
				size_t vsz = numericLeafSize(payload + valueStart, size - valueStart);
				if (vsz == 0) return false;
				size_t nameStart = valueStart + vsz;
				size_t nsz = nullTermSize(payload + nameStart, size - nameStart);
				off = nameStart + nsz;
				break;
			}
			case 0x150D: // LF_MEMBER_V3: id(2) + attr(2) + type(4) + offset(numeric) + name
			case 0x1405: // LF_MEMBER_V2: id(2) + attr(2) + type(4) + offset(numeric) + p_name
			{
				if (off + 8 > size) return false;
				refs.push_back(static_cast<uint32_t>(off + 4));
				size_t offStart = off + 8;
				size_t osz = numericLeafSize(payload + offStart, size - offStart);
				if (osz == 0) return false;
				size_t nameStart = offStart + osz;
				if (subKind == 0x1405)
				{
					// p_name: 1-byte length prefix + chars
					if (nameStart >= size) return false;
					size_t plen = payload[nameStart];
					off = nameStart + 1 + plen;
				}
				else
				{
					size_t nsz = nullTermSize(payload + nameStart, size - nameStart);
					off = nameStart + nsz;
				}
				break;
			}
			case 0x1400: // LF_BCLASS_V2: id(2) + attr(2) + type(4) + offset(numeric)
			{
				if (off + 8 > size) return false;
				refs.push_back(static_cast<uint32_t>(off + 4));
				size_t offStart = off + 8;
				size_t osz = numericLeafSize(payload + offStart, size - offStart);
				if (osz == 0) return false;
				off = offStart + osz;
				break;
			}
			case 0x1401: // LF_VBCLASS: id(2) + attr(2) + btype(4) + vbtype(4) + offset(numeric) + vbpoff(numeric)
			case 0x1402: // LF_IVBCLASS
			{
				if (off + 12 > size) return false;
				refs.push_back(static_cast<uint32_t>(off + 4));    // btype
				refs.push_back(static_cast<uint32_t>(off + 8));    // vbtype
				size_t a = off + 12;
				size_t asz = numericLeafSize(payload + a, size - a);
				if (asz == 0) return false;
				size_t b = a + asz;
				size_t bsz = numericLeafSize(payload + b, size - b);
				if (bsz == 0) return false;
				off = b + bsz;
				break;
			}
			case 0x1404: // LF_INDEX_V2: id(2) + pad(2) + type(4)
			{
				if (off + 8 > size) return false;
				refs.push_back(static_cast<uint32_t>(off + 4));
				off += 8;
				break;
			}
			case 0x1409: // LF_VFUNCTAB_V2: id(2) + pad(2) + type(4)
			{
				if (off + 8 > size) return false;
				refs.push_back(static_cast<uint32_t>(off + 4));
				off += 8;
				break;
			}
			case 0x1510: // LF_NESTTYPE_V3: id(2) + pad(2) + type(4) + name
			case 0x140F: // LF_NESTTYPE_V1: id(2) + pad(2) + type(4) + p_name
			{
				if (off + 8 > size) return false;
				refs.push_back(static_cast<uint32_t>(off + 4));
				size_t nameStart = off + 8;
				if (subKind == 0x140F)
				{
					if (nameStart >= size) return false;
					size_t plen = payload[nameStart];
					off = nameStart + 1 + plen;
				}
				else
				{
					size_t nsz = nullTermSize(payload + nameStart, size - nameStart);
					off = nameStart + nsz;
				}
				break;
			}
			case 0x150E: // LF_STMEMBER_V3: id(2) + attr(2) + type(4) + name
			case 0x1406: // LF_STMEMBER_V2: id(2) + attr(2) + type(4) + p_name
			{
				if (off + 8 > size) return false;
				refs.push_back(static_cast<uint32_t>(off + 4));
				size_t nameStart = off + 8;
				if (subKind == 0x1406)
				{
					if (nameStart >= size) return false;
					size_t plen = payload[nameStart];
					off = nameStart + 1 + plen;
				}
				else
				{
					size_t nsz = nullTermSize(payload + nameStart, size - nameStart);
					off = nameStart + nsz;
				}
				break;
			}
			default:
				return false;          // unknown subrecord: bail
		}
	}
	return true;
}

// Returns the byte offsets (within the record's payload, not counting the
// 2-byte length + 2-byte kind prefix) of every 4-byte type-index field in
// the record.  Sets *isKnown to false if the record's leaf kind isn't one
// we know how to dissect; the caller then bails the whole TPI stream out
// of dedup mode for safety.
std::vector<uint32_t> findTypeIndexRefs(uint16_t leafKind,
                                         const uint8_t* payload,
                                         size_t payloadSize,
                                         bool* isKnown)
{
	*isKnown = true;
	std::vector<uint32_t> refs;
	switch (leafKind)
	{
		case 0x1001:                                  // LF_MODIFIER_V2
			if (payloadSize >= 4) refs.push_back(0);
			break;
		case 0x1002:                                  // LF_POINTER_V2
			if (payloadSize >= 4) refs.push_back(0);
			break;
		case 0x1003:                                  // LF_ARRAY_V2
		case 0x1503:                                  // LF_ARRAY_V3
			if (payloadSize >= 8)
			{
				refs.push_back(0);
				refs.push_back(4);
			}
			break;
		case 0x1004:                                  // LF_CLASS_V2
		case 0x1005:                                  // LF_STRUCTURE_V2
		case 0x1504:                                  // LF_CLASS_V3
		case 0x1505:                                  // LF_STRUCTURE_V3
			if (payloadSize >= 16)
			{
				refs.push_back(4);                    // fieldlist
				refs.push_back(8);                    // derived
				refs.push_back(12);                   // vshape
			}
			break;
		case 0x1006:                                  // LF_UNION_V2
		case 0x1506:                                  // LF_UNION_V3
			if (payloadSize >= 8) refs.push_back(4);
			break;
		case 0x1007:                                  // LF_ENUM_V2
		case 0x1507:                                  // LF_ENUM_V3
			if (payloadSize >= 12)
			{
				refs.push_back(4);                    // underlying type
				refs.push_back(8);                    // fieldlist
			}
			break;
		case 0x1008:                                  // LF_PROCEDURE_V2
			if (payloadSize >= 12)
			{
				refs.push_back(0);                    // rvtype
				refs.push_back(8);                    // arglist
			}
			break;
		case 0x1201:                                  // LF_ARGLIST_V2
			if (payloadSize >= 4)
			{
				uint32_t count;
				memcpy(&count, payload, 4);
				for (uint32_t i = 0; i < count; i++)
				{
					if (4 + (i + 1) * 4 > payloadSize) break;
					refs.push_back(4 + i * 4);
				}
			}
			break;
		case 0x1203:                                  // LF_FIELDLIST_V2
			*isKnown = walkFieldlistRefs(payload, payloadSize, refs);
			if (!*isKnown) refs.clear();
			break;
		case 0x100A:                                  // LF_BITFIELD_V2: type at offset 0
			if (payloadSize >= 4) refs.push_back(0);
			break;
		case 0x1009:                                  // LF_MFUNCTION_V2
			if (payloadSize >= 24)
			{
				refs.push_back(0);                    // rvtype
				refs.push_back(4);                    // class type
				refs.push_back(8);                    // this type
				refs.push_back(16);                   // arglist
			}
			break;
		case 0x000A:                                  // LF_VTSHAPE: no type-index refs
			break;
		default:
			*isKnown = false;
			break;
	}
	return refs;
}

// Returns the byte offset within a CV symbol record's payload (after the
// 2-byte length + 2-byte kind prefix) of every 4-byte type-index field that
// references a TPI record.  *isKnown is set to false if the kind isn't one
// we know how to dissect; the caller then leaves the record alone.
//
// Only kinds that cv2pdb actually emits in the DWARF flow are listed;
// extending this is mechanical (look up the field offsets in mscvpdb.h).
std::vector<uint32_t> findSymbolTypeIndexRefs(uint16_t kind,
                                                const uint8_t* /*payload*/,
                                                size_t payloadSize,
                                                bool* isKnown)
{
	*isKnown = true;
	std::vector<uint32_t> refs;
	switch (kind)
	{
		// No type references:
		case 0x0006: // S_END
		case 0x0001: // S_COMPILE
		case 0x110E: // S_PUB32 (Flags + Off + Seg + Name)
		case 0x1101: // S_OBJNAME
		case 0x113C: // S_COMPILE3
		case 0x1116: // S_COMPILE2
		case 0x1012: // S_FRAMEPROC
		case 0x1103: // S_BLOCK32
		case 0x1105: // S_LABEL32
		case 0x1102: // S_THUNK32
		case 0x114E: // S_INLINESITE_END (scope terminator, no payload)
		case 0x114F: // S_PROC_ID_END (scope terminator, no payload)
		case 0x1136: // S_TRAMPOLINE
		case 0x1141: // S_DEFRANGE
		case 0x1142: // S_DEFRANGE_SUBFIELD
		case 0x1143: // S_DEFRANGE_REGISTER
		case 0x1144: // S_DEFRANGE_FRAMEPOINTER_REL
		case 0x1145: // S_DEFRANGE_SUBFIELD_REGISTER
		case 0x114B: // S_DEFRANGE_REGISTER_REL
		case 0x1107: // S_CONSTANT (V1)
		case 0x1109: // S_CONSTANT (V2)
		case 0x1125: // S_PROCREF
		case 0x1126: // S_DATAREF
		case 0x1127: // S_LPROCREF
		case 0x1115: // S_TOKENREF
		case 0x1124: // S_UNAMESPACE
		case 0x1132: // S_SECTION
		case 0x1133: // S_COFFGROUP
		case 0x1134: // S_EXPORT
			break;

		// type at payload offset 0 (after kind+length):
		case 0x1108: // S_UDT_V3 (TypeIndex + Name)
		case 0x110D: // S_GDATA32 (TypeIndex + Off + Seg + Name)
		case 0x110C: // S_LDATA32
		case 0x1112: // S_LTHREAD32
		case 0x1113: // S_GTHREAD32
		case 0x110A: // S_CONSTANT_V3 (TypeIndex + value-leaf + Name)
		case 0x113D: // S_LOCAL (TypeIndex + Flags + Name)
		case 0x114C: // S_BUILDINFO (TypeIndex only)
			if (payloadSize >= 4) refs.push_back(0);
			break;

		// type at payload offset 4 (after a leading uint32):
		case 0x1111: // S_REGREL32 (Off + TypeIndex + Reg + Name)
		case 0x110B: // S_BPREL32 (Off + TypeIndex + Name)
		case 0x1106: // S_REGISTER (TypeIndex + Reg + Name) - actually offset 0
		case 0x110F: // S_LPROC32 - see below, override
		case 0x1110: // S_GPROC32 - see below, override
		case 0x1147: // S_LPROC32_ID - same layout as LPROC32
		case 0x1148: // S_GPROC32_ID - same layout as GPROC32
			if (kind == 0x1106)
			{
				if (payloadSize >= 4) refs.push_back(0);
			}
			else if (kind == 0x110F || kind == 0x1110
			         || kind == 0x1147 || kind == 0x1148)
			{
				// S_*PROC32: parent(4)+end(4)+next(4)+len(4)+dbgStart(4)+
				// dbgEnd(4)+TypeIndex(4)+offset(4)+segment(2)+flags(1)+name
				if (payloadSize >= 28) refs.push_back(24);
			}
			else
			{
				if (payloadSize >= 8) refs.push_back(4);
			}
			break;

		// Inline site: parent(4)+end(4)+inlinee(TypeIndex,4)+invocation_data
		case 0x114D: // S_INLINESITE
			if (payloadSize >= 12) refs.push_back(8);
			break;

		// Callsite info: offset(4)+seg(2)+pad(2)+TypeIndex(4)
		case 0x114A: // S_CALLSITEINFO
			if (payloadSize >= 12) refs.push_back(8);
			break;

		// Heap alloc site: offset(4)+seg(2)+instr-len(2)+TypeIndex(4)
		case 0x115A: // S_HEAPALLOCSITE
			if (payloadSize >= 12) refs.push_back(8);
			break;

		default:
			*isKnown = false;
			break;
	}
	return refs;
}

// Walk a buffer of length+kind-prefixed CV symbol records, remapping each
// known-kind record's type-index fields via `remap`.  Records of unknown
// kinds are left untouched.  Returns the number of records walked.  The
// records' length fields are not modified, only type-index payload fields
// at known offsets.
size_t remapSymbolTypeIndices(uint8_t* records, size_t size,
                              const std::map<uint32_t, uint32_t>& remap)
{
	size_t off = 0;
	size_t count = 0;
	while (off + 4 <= size)
	{
		uint16_t len, kind;
		memcpy(&len, records + off, 2);
		memcpy(&kind, records + off + 2, 2);
		size_t recordSize = 2 + len;
		if (recordSize < 4 || off + recordSize > size)
			break;

		bool isKnown = true;
		std::vector<uint32_t> refs = findSymbolTypeIndexRefs(
		    kind, records + off + 4, recordSize - 4, &isKnown);
		for (uint32_t roff : refs)
		{
			size_t fieldOff = off + 4 + roff;
			if (fieldOff + 4 > size) continue;
			uint32_t target;
			memcpy(&target, records + fieldOff, 4);
			auto it = remap.find(target);
			if (it != remap.end())
			{
				uint32_t mapped = it->second;
				memcpy(records + fieldOff, &mapped, 4);
			}
		}

		off += recordSize;
		count++;
	}
	return count;
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
// both.  CodeView records are queued via addRecords() and then finalised
// (lazily, on first buildStream/buildHashStream call) into the on-disk
// records blob, the per-record hash buffer, and the sparse IndexOffsetBuffer.
//
// Finalisation does record dedup with type-index remapping, modelled after
// what mspdb does internally: cv2pdb emits identical LF_MODIFIER /
// LF_FIELDLIST / etc. records freely and trusts the writer to coalesce them
// by content.  Two-pass approach so forward references (LF_INDEX_V2 enum
// continuations and the recursive struct/fieldlist pair from cv2pdb #99)
// stay correct: pass 1 deduplicates records whose refs are all backward and
// appends forward-ref records verbatim with their backward refs remapped;
// pass 2 patches forward-ref byte positions once every input index has
// landed in indexRemap_.  If any record contains a kind walkers in
// findTypeIndexRefs / walkFieldlistRefs don't recognise, finalisation bails
// the whole stream out of dedup and just emits the original cv2pdb bytes
// verbatim (correctness-preserving fallback).
class TpiStreamBuilder
{
public:
	static constexpr uint32_t kFirstTypeIndex = 0x1000;
	static constexpr uint32_t kNumHashBuckets = 0x40000 - 1;   // 262143
	static constexpr uint32_t kIndexOffsetGranBytes = 8 * 1024;

	void addRecords(const uint8_t* buf, size_t cb)
	{
		rawInput_.insert(rawInput_.end(), buf, buf + cb);
	}

	std::vector<uint8_t> buildStream(uint16_t hashStreamIndex)
	{
		finalize();
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

	std::vector<uint8_t> buildHashStream()
	{
		finalize();
		std::vector<uint8_t> blob;
		blob.insert(blob.end(), hashValues_.begin(), hashValues_.end());
		for (const auto& iob : indexOffsets_)
		{
			appendU32LE(blob, iob.first);    // TypeIndex
			appendU32LE(blob, iob.second);   // ByteOffset in record blob
		}
		return blob;
	}

	// Returns the input-index -> output-index map after dedup.  Used by
	// ModuleStreamBuilder / NativePdbWriter to remap CV symbol-record type
	// references (S_UDT.type, S_GDATA32.type, S_GPROC32.type, ...) so they
	// keep pointing at the right TPI records after dedup shifts the index
	// space.  The map is also valid for primitive type indices (which it
	// simply doesn't contain entries for; remap callers fall through to
	// identity in that case).
	const std::map<uint32_t, uint32_t>& getIndexRemap()
	{
		finalize();
		return indexRemap_;
	}

private:
	// Append one record (its full bytes, length+kind prefix included) to
	// records_, computing the per-record hash and emitting an IOB entry on
	// 8 KiB boundaries.  Returns the assigned output type index.
	uint32_t emit(const uint8_t* recBytes, size_t recordSize)
	{
		uint32_t recBlobOffset = static_cast<uint32_t>(records_.size());
		uint32_t typeIndex = kFirstTypeIndex + numRecords_;

		if (numRecords_ == 0
		    || (recBlobOffset - lastIobOffset_) >= kIndexOffsetGranBytes)
		{
			indexOffsets_.emplace_back(typeIndex, recBlobOffset);
			lastIobOffset_ = recBlobOffset;
		}

		records_.insert(records_.end(), recBytes, recBytes + recordSize);
		uint32_t hash = hashBufferV8(recBytes, recordSize) % kNumHashBuckets;
		appendU32LE(hashValues_, hash);
		numRecords_++;
		return typeIndex;
	}

	// Walk rawInput_ once just to check that every record (and every
	// fieldlist subrecord) is a kind we know how to dissect.  Bails to
	// no-dedup mode if any unknown is found.
	bool allKindsKnown() const
	{
		size_t off = 0;
		while (off + 2 <= rawInput_.size())
		{
			uint16_t len;
			memcpy(&len, rawInput_.data() + off, 2);
			size_t recordSize = 2 + len;
			if (recordSize < 4 || off + recordSize > rawInput_.size())
				break;
			if (recordSize < 6) { off += recordSize; continue; }

			uint16_t kind;
			memcpy(&kind, rawInput_.data() + off + 2, 2);

			bool isKnown = true;
			std::vector<uint32_t> refs;
			refs = findTypeIndexRefs(kind, rawInput_.data() + off + 4,
			                         recordSize - 4, &isKnown);
			(void)refs;
			if (!isKnown)
				return false;
			off += recordSize;
		}
		return true;
	}

	void finalize()
	{
		if (finalized_) return;
		finalized_ = true;

		bool dedup = allKindsKnown();

		if (!dedup)
		{
			// Verbatim fallback: just walk the input and emit each record
			// unchanged.  Preserves the original behaviour when we hit a
			// record kind we don't yet know how to dedup safely.
			size_t off = 0;
			while (off + 2 <= rawInput_.size())
			{
				uint16_t len;
				memcpy(&len, rawInput_.data() + off, 2);
				size_t recordSize = 2 + len;
				if (recordSize < 4
				    || off + recordSize > rawInput_.size())
					break;
				emit(rawInput_.data() + off, recordSize);
				off += recordSize;
			}
			return;
		}

		// Two-pass dedup with forward-ref fixup.
		// Pass 1: walk records in input order.  For each, find its type
		// refs.  If any ref points forward (>= current input index), we
		// can't compute its final value yet, so we append the record
		// verbatim with backward refs remapped and remember the byte
		// offsets of its forward-ref fields for pass 2.  Otherwise the
		// record is fully deduppable: copy, remap, hash for content match,
		// and either dedup or append.
		size_t off = 0;
		uint32_t inputIdx = kFirstTypeIndex;
		while (off + 2 <= rawInput_.size())
		{
			uint16_t len;
			memcpy(&len, rawInput_.data() + off, 2);
			size_t recordSize = 2 + len;
			if (recordSize < 4 || off + recordSize > rawInput_.size())
				break;

			uint16_t kind = 0;
			if (recordSize >= 4)
				memcpy(&kind, rawInput_.data() + off + 2, 2);

			bool isKnown = true;
			std::vector<uint32_t> refs =
			    findTypeIndexRefs(kind,
			                      rawInput_.data() + off + 4,
			                      recordSize - 4, &isKnown);

			std::vector<uint8_t> rec(rawInput_.data() + off,
			                          rawInput_.data() + off + recordSize);

			bool hasForward = false;
			for (uint32_t roff : refs)
			{
				size_t fieldOffsetInRec = 4 + roff;
				if (fieldOffsetInRec + 4 > rec.size())
					continue;
				uint32_t target;
				memcpy(&target, rec.data() + fieldOffsetInRec, 4);
				if (target >= kFirstTypeIndex && target >= inputIdx)
				{
					hasForward = true;
					break;
				}
			}

			if (hasForward)
			{
				// Append verbatim; remap backward refs in place; record
				// the byte offsets of forward-ref fields for pass 2.
				for (uint32_t roff : refs)
				{
					size_t fieldOffsetInRec = 4 + roff;
					if (fieldOffsetInRec + 4 > rec.size())
						continue;
					uint32_t target;
					memcpy(&target, rec.data() + fieldOffsetInRec, 4);
					if (target >= kFirstTypeIndex && target < inputIdx)
					{
						uint32_t mapped = remap(target);
						memcpy(rec.data() + fieldOffsetInRec, &mapped, 4);
					}
					else if (target >= kFirstTypeIndex)
					{
						// Forward ref.  Remember its byte offset within
						// records_ so pass 2 can patch it.
						forwardFixups_.push_back(
						    records_.size() + fieldOffsetInRec);
					}
				}

				uint32_t outIdx = emit(rec.data(), rec.size());
				indexRemap_[inputIdx] = outIdx;
			}
			else
			{
				// All refs backward (or to primitives).  Remap them all
				// in place, hash, dedup.
				for (uint32_t roff : refs)
				{
					size_t fieldOffsetInRec = 4 + roff;
					if (fieldOffsetInRec + 4 > rec.size())
						continue;
					uint32_t target;
					memcpy(&target, rec.data() + fieldOffsetInRec, 4);
					if (target >= kFirstTypeIndex)
					{
						uint32_t mapped = remap(target);
						memcpy(rec.data() + fieldOffsetInRec, &mapped, 4);
					}
				}

				std::string key(rec.begin(), rec.end());
				auto it = contentMap_.find(key);
				if (it != contentMap_.end())
				{
					indexRemap_[inputIdx] = it->second;
				}
				else
				{
					uint32_t outIdx = emit(rec.data(), rec.size());
					contentMap_[std::move(key)] = outIdx;
					indexRemap_[inputIdx] = outIdx;
				}
			}

			inputIdx++;
			off += recordSize;
		}

		// Pass 2: patch forward-ref byte positions using the now-complete
		// indexRemap_ table.
		for (size_t fixOffset : forwardFixups_)
		{
			if (fixOffset + 4 > records_.size())
				continue;
			uint32_t target;
			memcpy(&target, records_.data() + fixOffset, 4);
			uint32_t mapped = remap(target);
			memcpy(records_.data() + fixOffset, &mapped, 4);
			// Note: the per-record hashBufferV8 in hashValues_ was
			// computed on pre-fixup bytes.  When the forward-ref target
			// did not get deduped (the common case for self-referential
			// struct/fieldlist pairs), pre-fixup and post-fixup bytes
			// match and the hash is still consistent.  When it did get
			// deduped, the hash disagrees with the on-disk bytes by the
			// difference in those four bytes; LLVM's reader does not
			// verify per-record hashes against bytes (the hash is only
			// used by name-keyed lookup), so the mismatch is benign.
		}

		// Pass 3+: fix-point dedup over the post-pass-2 byte buffer.
		// Pass 1's content map only collapsed records whose refs were all
		// backward; forward-ref records (cv2pdb's struct / fieldlist pairs
		// from the #99 fix, and LF_INDEX_V2 enum continuations) were
		// emitted verbatim regardless of whether they duplicated an
		// earlier record.  After pass 2 their bytes are final, so a raw
		// byte-equality dedup over records_ collapses the duplicates.
		// Iterate because a successful dedup in iteration N may equalise
		// further records in iteration N+1 once their refs are remapped
		// through the new reduce map.  Each iteration that fails to merge
		// anything returns false and breaks the loop, so the cap is just
		// a safety net against unforeseen cycles; for cv2pdb's git.exe
		// (59,000 records pre-pass-3) convergence to 15,397 records takes
		// 15 iterations.
		for (int iter = 0; iter < 32; iter++)
			if (!rededupOnce())
				break;
	}

	// One pass of post-pass-2 byte-equality dedup.  Returns true if any
	// records were merged, false if records_ is already minimal.  On
	// success, records_ / hashValues_ / indexOffsets_ / numRecords_ /
	// lastIobOffset_ are rebuilt from the merged set, and indexRemap_'s
	// values are composed with the local reduce map so symbol-side remap
	// stays consistent.
	bool rededupOnce()
	{
		std::vector<uint8_t> newRecords;
		newRecords.reserve(records_.size());
		std::vector<uint8_t> newHashValues;
		newHashValues.reserve(hashValues_.size());
		std::vector<std::pair<uint32_t, uint32_t>> newIndexOffsets;
		std::map<std::string, uint32_t> contentMap;
		std::map<uint32_t, uint32_t> reduce;

		uint32_t curIdx = kFirstTypeIndex;
		uint32_t finalIdx = kFirstTypeIndex;
		uint32_t lastIob = 0;

		size_t pos = 0;
		while (pos + 2 <= records_.size())
		{
			uint16_t len;
			memcpy(&len, records_.data() + pos, 2);
			size_t recSize = 2 + len;
			if (recSize < 4 || pos + recSize > records_.size())
				break;

			std::string key(reinterpret_cast<const char*>(records_.data() + pos),
			                recSize);
			auto it = contentMap.find(key);
			if (it != contentMap.end())
			{
				reduce[curIdx] = it->second;
			}
			else
			{
				uint32_t blobOffset =
				    static_cast<uint32_t>(newRecords.size());
				if (newIndexOffsets.empty()
				    || (blobOffset - lastIob) >= kIndexOffsetGranBytes)
				{
					newIndexOffsets.emplace_back(finalIdx, blobOffset);
					lastIob = blobOffset;
				}
				newRecords.insert(newRecords.end(),
				                   records_.begin() + pos,
				                   records_.begin() + pos + recSize);
				uint32_t hash =
				    hashBufferV8(records_.data() + pos, recSize)
				    % kNumHashBuckets;
				appendU32LE(newHashValues, hash);
				contentMap[std::move(key)] = finalIdx;
				reduce[curIdx] = finalIdx;
				finalIdx++;
			}
			curIdx++;
			pos += recSize;
		}

		uint32_t kept = finalIdx - kFirstTypeIndex;
		uint32_t total = curIdx - kFirstTypeIndex;
		if (kept == total)
			return false;  // no duplicates found, converged

		// Apply reduce to type-index fields in newRecords so the next
		// iteration sees post-reduce content.
		size_t off = 0;
		while (off + 2 <= newRecords.size())
		{
			uint16_t len;
			memcpy(&len, newRecords.data() + off, 2);
			size_t recSize = 2 + len;
			if (recSize < 4 || off + recSize > newRecords.size())
				break;
			if (recSize < 6) { off += recSize; continue; }

			uint16_t kind;
			memcpy(&kind, newRecords.data() + off + 2, 2);
			bool isKnown = true;
			std::vector<uint32_t> refs =
			    findTypeIndexRefs(kind,
			                      newRecords.data() + off + 4,
			                      recSize - 4, &isKnown);
			for (uint32_t roff : refs)
			{
				size_t fieldOff = off + 4 + roff;
				if (fieldOff + 4 > newRecords.size())
					continue;
				uint32_t target;
				memcpy(&target, newRecords.data() + fieldOff, 4);
				auto rit = reduce.find(target);
				if (rit != reduce.end() && rit->second != target)
					memcpy(newRecords.data() + fieldOff,
					        &rit->second, 4);
			}
			off += recSize;
		}

		// Compose indexRemap_ with reduce: input -> old outIdx -> final.
		for (auto& kv : indexRemap_)
		{
			auto rit = reduce.find(kv.second);
			if (rit != reduce.end())
				kv.second = rit->second;
		}

		records_ = std::move(newRecords);
		hashValues_ = std::move(newHashValues);
		indexOffsets_ = std::move(newIndexOffsets);
		numRecords_ = kept;
		lastIobOffset_ = lastIob;
		return true;
	}

	uint32_t remap(uint32_t inputIdx) const
	{
		auto it = indexRemap_.find(inputIdx);
		if (it != indexRemap_.end())
			return it->second;
		return inputIdx;
	}

	std::vector<uint8_t> rawInput_;
	bool finalized_ = false;

	std::vector<uint8_t> records_;
	std::vector<uint8_t> hashValues_;            // raw little-endian uint32s
	std::vector<std::pair<uint32_t, uint32_t>> indexOffsets_;
	uint32_t numRecords_ = 0;
	uint32_t lastIobOffset_ = 0;

	std::map<std::string, uint32_t> contentMap_;
	std::map<uint32_t, uint32_t> indexRemap_;
	std::vector<size_t> forwardFixups_;
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

	// Walk this module's accumulated raw symbol records (the bytes that
	// will land between offsets [4, SymByteSize) of the on-disk module
	// stream) and remap any type-index fields using the TPI dedup map.
	// Called from NativePdbWriter::commit() once the TPI builder has
	// settled on the final input-to-output index mapping.
	void remapSymbolTypeIndices(const std::map<uint32_t, uint32_t>& remap)
	{
		if (symbols_.empty()) return;
		::cv2pdb::remapSymbolTypeIndices(symbols_.data(), symbols_.size(), remap);
	}

	// Patch each scope-opening record's `parent` and `end` (and, where
	// the record type carries one, `next`) field so dbghelp.dll can walk
	// the symbol stream as a tree.  cv2pdb's DWARF translator emits
	// S_GPROC32 / S_LPROC32 / S_BLOCK32 records with these three fields
	// zeroed because mspdb's mod->AddSymbols populates them on the way
	// in; the in-house writer skips that path, so we have to do the same
	// fix-up here once all the records are accumulated.  Without it,
	// dbghelp reports the module as `(pdb symbols)` (public-only) rather
	// than `(private pdb symbols)` and refuses to surface function-local
	// variables, source line info, or scope-aware stepping.
	//
	// Layout per microsoft-pdb cvinfo.h (CV_PROCSYM32, BLOCKSYM32,
	// THUNKSYM32, INLINESITESYM):
	//   S_GPROC32 / S_LPROC32 / S_GPROC32_ID / S_LPROC32_ID: parent(4),
	//     end(4), next(4) at payload offsets 0/4/8.
	//   S_BLOCK32: parent(4), end(4), len(4) at payload offsets 0/4/8.
	//     No `next` slot (sibling chaining is implicit in the linear
	//     scope-end terminator).
	//   S_THUNK32: parent(4), end(4), next(4) at payload offsets 0/4/8.
	//   S_INLINESITE: parent(4), end(4) at payload offsets 0/4.  No
	//     `next` slot here either.
	// Terminators:
	//   S_END (0x0006) closes GPROC32 / LPROC32 / BLOCK32 / THUNK32.
	//   S_PROC_ID_END (0x114F) closes GPROC32_ID / LPROC32_ID.
	//   S_INLINESITE_END (0x114E) closes S_INLINESITE.
	// All offsets are absolute byte positions within the FULL module
	// stream including the leading 4-byte CV_SIGNATURE_C13 prefix; that
	// is what dbghelp expects and what mspdb writes.
	void fixupSymbolScopes()
	{
		if (symbols_.empty()) return;

		struct Scope { size_t openerOffset; uint16_t kind; };
		std::vector<Scope> stack;
		size_t off = 0;
		bool corrupt = false;
		while (off + 2 <= symbols_.size())
		{
			uint16_t len;
			memcpy(&len, symbols_.data() + off, 2);
			size_t recordSize = 2 + len;
			if (recordSize < 4 || off + recordSize > symbols_.size())
			{
				corrupt = true;
				break;
			}
			uint16_t kind;
			memcpy(&kind, symbols_.data() + off + 2, 2);

			size_t streamOff = 4 + off;  // include CV_SIGNATURE_C13

			bool opensScope = false;
			bool closesScope = false;
			uint16_t expectedTerminator = 0;
			switch (kind)
			{
			case 0x1110: // S_GPROC32
			case 0x110F: // S_LPROC32
				opensScope = true;
				expectedTerminator = 0x0006; // S_END
				break;
			case 0x1148: // S_GPROC32_ID
			case 0x1147: // S_LPROC32_ID
				opensScope = true;
				expectedTerminator = 0x114F; // S_PROC_ID_END
				break;
			case 0x1103: // S_BLOCK32
			case 0x1102: // S_THUNK32
				opensScope = true;
				expectedTerminator = 0x0006; // S_END
				break;
			case 0x1132: // S_SEPCODE
				// SEPCODESYM layout (cvinfo.h): pParent(4) + pEnd(4) +
				// length(4) + scf(4) + off(4) + offParent(4) + sect(2)
				// + sectParent(2). Parent and End sit at payload
				// offsets 0 / 4, same as BLOCK32 / INLINESITE, so the
				// generic patch-end-at-openerLocal+8 path covers it.
				// Closed by S_END.  Without this case the closing
				// S_END pops an empty stack, triggers corrupt=true,
				// and every record after split-out cold code in the
				// module loses its End pointer.
				opensScope = true;
				expectedTerminator = 0x0006; // S_END
				break;
			case 0x114D: // S_INLINESITE
				opensScope = true;
				expectedTerminator = 0x114E; // S_INLINESITE_END
				break;
			case 0x0006: // S_END
			case 0x114E: // S_INLINESITE_END
			case 0x114F: // S_PROC_ID_END
				closesScope = true;
				break;
			}

			if (opensScope)
			{
				uint32_t parentOff = stack.empty()
				    ? 0u
				    : static_cast<uint32_t>(stack.back().openerOffset);
				if (recordSize >= 8)
					memcpy(symbols_.data() + off + 4, &parentOff, 4);
				stack.push_back({streamOff, kind});
				(void)expectedTerminator;

				// Procedure records get a Globals-stream S_PROCREF /
				// S_LPROCREF entry; collect (name, ibSym, isLocal) so
				// the writer can synthesize those refs after every
				// module's symbols are finalised.  Per cvinfo.h the
				// V3-style name field starts 39 bytes into the record:
				// len(2)+id(2)+pparent(4)+pend(4)+next(4)+proc_len(4)+
				// debug_start(4)+debug_end(4)+proctype(4)+offset(4)+
				// segment(2)+flags(1), null-terminated.  Top-level
				// procs only -- a nested S_INLINESITE / S_BLOCK32 does
				// not get its own global ref.
				const size_t kNameOffset = 39;
				if ((kind == 0x1110 || kind == 0x110F)
				    && stack.size() == 1
				    && off + kNameOffset < symbols_.size())
				{
					const char* p = reinterpret_cast<const char*>(
					    symbols_.data() + off + kNameOffset);
					size_t maxLen = symbols_.size() - off - kNameOffset;
					size_t nlen = strnlen(p, maxLen);
					if (nlen > 0 && nlen < maxLen)
					{
						ProcRef ref{std::string(p, nlen),
						            static_cast<uint32_t>(streamOff),
						            kind == 0x110F};
						moduleProcs_.push_back(std::move(ref));
					}
				}
			}
			else if (closesScope)
			{
				if (stack.empty())
				{
					corrupt = true;
					break;
				}
				Scope opened = stack.back();
				stack.pop_back();
				uint32_t endOff = static_cast<uint32_t>(streamOff);
				size_t openerLocal = opened.openerOffset - 4;
				if (openerLocal + 12 <= symbols_.size())
					memcpy(symbols_.data() + openerLocal + 8, &endOff, 4);
			}
			else if (kind == 0x1108)
			{
				// S_UDT_V3: len(2)+id(2)+type(4)+name(null-term).
				// Collect (name, full record bytes) so the writer can
				// mirror each UDT into the shared SymbolRecords stream
				// and register it in the Globals GSI hash; dbghelp's
				// `dt <module>!<typename>` resolves type-by-name by
				// scanning that hash for an S_UDT entry, then derefs
				// the embedded TPI type index.
				const size_t kUdtNameOff = 8;
				if (off + kUdtNameOff < symbols_.size())
				{
					const char* np = reinterpret_cast<const char*>(
					    symbols_.data() + off + kUdtNameOff);
					size_t maxLen = symbols_.size() - off - kUdtNameOff;
					size_t nlen = strnlen(np, maxLen);
					if (nlen > 0 && nlen < maxLen)
					{
						UdtRef ref;
						ref.name.assign(np, nlen);
						ref.recordBytes.assign(
						    symbols_.data() + off,
						    symbols_.data() + off + recordSize);
						moduleUdts_.push_back(std::move(ref));
					}
				}
			}

			off += recordSize;
		}

		if (corrupt || !stack.empty())
		{
			// Either the buffer is malformed or there are unbalanced
			// open scopes (a producer that forgot to emit S_END).  Don't
			// rewrite anything in that case; leaving the records intact
			// matches the verbatim fallback we use elsewhere.
			return;
		}
	}

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

	// Procedure-style entries collected by fixupSymbolScopes, exposed so
	// NativePdbWriter::commit() can synthesize the matching S_PROCREF /
	// S_LPROCREF Globals-stream records.  ibSym is the byte offset of
	// the S_GPROC32 / S_LPROC32 record within this module's on-disk
	// symbol stream (with the leading CV_SIGNATURE_C13 prefix counted).
	struct ProcRef
	{
		std::string name;
		uint32_t    ibSym;
		bool        isLocal;
	};
	const std::vector<ProcRef>& moduleProcs() const { return moduleProcs_; }

	// User-defined-type entries: dbghelp resolves `dt <module>!<name>`
	// by looking up an S_UDT-named entry in the Globals GSI hash, so
	// every UDT cv2pdb emits into a module symbol stream is mirrored
	// into the shared SymbolRecords stream too.  The bytes here are
	// the raw S_UDT record copied verbatim (length-prefix and all)
	// from the module stream.
	struct UdtRef
	{
		std::string          name;
		std::vector<uint8_t> recordBytes;
	};
	const std::vector<UdtRef>& moduleUdts() const { return moduleUdts_; }

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
	std::vector<ProcRef> moduleProcs_;
	std::vector<UdtRef> moduleUdts_;
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
	std::vector<uint8_t>& mutableBytes() { return records_; }

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

		// Settle TPI dedup before emitting any module symbol streams or
		// the shared SymbolRecords stream.  Type-index fields embedded in
		// CV symbol records (S_UDT.type, S_*DATA32.type, S_*PROC32.type,
		// ...) carry cv2pdb's pre-dedup type indices; remap them in place
		// so they keep pointing at the right TPI records once dedup has
		// shifted the index space.
		const std::map<uint32_t, uint32_t>& tpiRemap = tpi_.getIndexRemap();
		for (ModuleStreamBuilder* m : mods_)
		{
			m->remapSymbolTypeIndices(tpiRemap);
			m->fixupSymbolScopes();
		}
		::cv2pdb::remapSymbolTypeIndices(
		    symbolRecords_.mutableBytes().data(),
		    symbolRecords_.mutableBytes().size(),
		    tpiRemap);

		// Synthesize a Globals-stream entry per top-level procedure in
		// every module: each S_GPROC32 / S_LPROC32 record gets a matching
		// S_PROCREF (rectyp 0x1125) / S_LPROCREF (0x1127) in the shared
		// SymbolRecords stream, indexed in the Globals GSI hash so
		// dbghelp.dll's "name -> module" dispatch is populated.  Without
		// these entries the resulting PDB loads as `(pdb symbols)`
		// (public-only) rather than `(private pdb symbols)` because
		// dbghelp has no way to find the module hosting any given
		// function name beyond the bare-bones S_PUB32 records that
		// addPublic builds.  Layout per microsoft-pdb cvinfo.h REFSYM2:
		// sumName(4) + ibSym(4, byte offset of the S_GPROC32 within the
		// owning module stream) + imod(2, 1-based module index) + name
		// (null-terminated).  sumName is the name's SUC checksum;
		// real-world readers (LLVM, dbghelp) ignore mismatches there, so
		// emit 0 to match cv2pdb-mspdb's behaviour.
		for (size_t mi = 0; mi < mods_.size(); mi++)
		{
			uint16_t imod = static_cast<uint16_t>(mi + 1);
			for (const auto& ref : mods_[mi]->moduleProcs())
			{
				std::vector<uint8_t> record;
				record.push_back(0);
				record.push_back(0);
				appendU16LE(record, ref.isLocal ? 0x1127 : 0x1125);
				appendU32LE(record, 0);                  // sumName
				appendU32LE(record, ref.ibSym);
				appendU16LE(record, imod);
				record.insert(record.end(), ref.name.begin(), ref.name.end());
				record.push_back(0);                     // null terminator
				while (record.size() % 4 != 0)
					record.push_back(0);
				uint16_t lenField =
				    static_cast<uint16_t>(record.size() - 2);
				record[0] = static_cast<uint8_t>(lenField & 0xFF);
				record[1] = static_cast<uint8_t>((lenField >> 8) & 0xFF);

				uint32_t recOff = symbolRecords_.append(record);
				globals_.addGlobalEntry(ref.name, recOff);
			}

			// Mirror each S_UDT_V3 record collected from the module
			// into the shared SymbolRecords stream and the Globals
			// hash so `dt <module>!<typename>` can resolve the name.
			// Dedup by name across all modules: only the first
			// occurrence of a given typedef-name is emitted as a
			// global, matching cv2pdb-mspdb's behaviour (it keeps
			// per-name uniqueness in the cross-module UDT table).
			for (const auto& udt : mods_[mi]->moduleUdts())
			{
				if (!globalUdtNames_.insert(udt.name).second)
					continue;
				uint32_t recOff = symbolRecords_.append(udt.recordBytes);
				globals_.addGlobalEntry(udt.name, recOff);
			}
		}

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
	std::set<std::string> globalUdtNames_;
};

}  // namespace

PdbWriter* createNativePdbWriter(const wchar_t* pdbname)
{
	return new NativePdbWriter(pdbname);
}

}  // namespace cv2pdb
