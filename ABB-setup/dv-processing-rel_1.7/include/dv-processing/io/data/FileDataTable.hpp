// automatically generated by the FlatBuffers compiler, do not modify

#ifndef FLATBUFFERS_GENERATED_FILEDATATABLE_DV_H_
#define FLATBUFFERS_GENERATED_FILEDATATABLE_DV_H_

#pragma GCC system_header
#pragma clang system_header

#include "../../external/flatbuffers/flatbuffers.h"

#include "../../data/cvector.hpp"

namespace dv {

struct PacketHeader;

struct FileDataDefinitionFlatbuffer;
struct FileDataDefinition;

struct FileDataTableFlatbuffer;
struct FileDataTable;

bool operator==(const PacketHeader &lhs, const PacketHeader &rhs);
bool operator==(const FileDataDefinition &lhs, const FileDataDefinition &rhs);
bool operator==(const FileDataTable &lhs, const FileDataTable &rhs);

inline const flatbuffers::TypeTable *PacketHeaderTypeTable();

inline const flatbuffers::TypeTable *FileDataDefinitionTypeTable();

inline const flatbuffers::TypeTable *FileDataTableTypeTable();

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) PacketHeader FLATBUFFERS_FINAL_CLASS {
	typedef PacketHeader NativeTableType;
	typedef PacketHeader TableType;

private:
	int32_t StreamID_;
	int32_t Size_;

public:
	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.PacketHeader";
	}
	PacketHeader() {
		memset(static_cast<void *>(this), 0, sizeof(PacketHeader));
	}
	PacketHeader(int32_t _StreamID, int32_t _Size) :
		StreamID_(flatbuffers::EndianScalar(_StreamID)),
		Size_(flatbuffers::EndianScalar(_Size)) {
	}
	int32_t StreamID() const {
		return flatbuffers::EndianScalar(StreamID_);
	}
	int32_t Size() const {
		return flatbuffers::EndianScalar(Size_);
	}
};

FLATBUFFERS_STRUCT_END(PacketHeader, 8);

inline bool operator==(const PacketHeader &lhs, const PacketHeader &rhs) {
	return (lhs.StreamID() == rhs.StreamID()) && (lhs.Size() == rhs.Size());
}

struct FileDataDefinition : public flatbuffers::NativeTable {
	typedef FileDataDefinitionFlatbuffer TableType;

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.FileDataDefinition";
	}

	int64_t ByteOffset;
	PacketHeader PacketInfo;
	int64_t NumElements;
	int64_t TimestampStart;
	int64_t TimestampEnd;

	FileDataDefinition() : ByteOffset(0), NumElements(0), TimestampStart(0), TimestampEnd(0) {
	}

	// Generated Constructor
	FileDataDefinition(int64_t _ByteOffset, const PacketHeader &_PacketInfo, int64_t _NumElements,
		int64_t _TimestampStart, int64_t _TimestampEnd) :
		ByteOffset{_ByteOffset},
		PacketInfo{_PacketInfo},
		NumElements{_NumElements},
		TimestampStart{_TimestampStart},
		TimestampEnd{_TimestampEnd} {
	}
};

inline bool operator==(const FileDataDefinition &lhs, const FileDataDefinition &rhs) {
	return (lhs.ByteOffset == rhs.ByteOffset) && (lhs.PacketInfo == rhs.PacketInfo)
		&& (lhs.NumElements == rhs.NumElements) && (lhs.TimestampStart == rhs.TimestampStart)
		&& (lhs.TimestampEnd == rhs.TimestampEnd);
}

struct FileDataDefinitionFlatbuffer FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
	typedef FileDataDefinition NativeTableType;

	static const flatbuffers::TypeTable *MiniReflectTypeTable() {
		return FileDataDefinitionTypeTable();
	}

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.FileDataDefinition";
	}

	enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
		VT_BYTEOFFSET     = 4,
		VT_PACKETINFO     = 6,
		VT_NUMELEMENTS    = 8,
		VT_TIMESTAMPSTART = 10,
		VT_TIMESTAMPEND   = 12
	};

	int64_t ByteOffset() const {
		return GetField<int64_t>(VT_BYTEOFFSET, 0);
	}

	const PacketHeader *PacketInfo() const {
		return GetStruct<const PacketHeader *>(VT_PACKETINFO);
	}

	int64_t NumElements() const {
		return GetField<int64_t>(VT_NUMELEMENTS, 0);
	}

	int64_t TimestampStart() const {
		return GetField<int64_t>(VT_TIMESTAMPSTART, 0);
	}

	int64_t TimestampEnd() const {
		return GetField<int64_t>(VT_TIMESTAMPEND, 0);
	}

	bool Verify(flatbuffers::Verifier &verifier) const {
		return VerifyTableStart(verifier) && VerifyField<int64_t>(verifier, VT_BYTEOFFSET)
			&& VerifyField<PacketHeader>(verifier, VT_PACKETINFO) && VerifyField<int64_t>(verifier, VT_NUMELEMENTS)
			&& VerifyField<int64_t>(verifier, VT_TIMESTAMPSTART) && VerifyField<int64_t>(verifier, VT_TIMESTAMPEND)
			&& verifier.EndTable();
	}

	FileDataDefinition *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	void UnPackTo(FileDataDefinition *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	static void UnPackToFrom(FileDataDefinition *_o, const FileDataDefinitionFlatbuffer *_fb,
		const flatbuffers::resolver_function_t *_resolver = nullptr);
	static flatbuffers::Offset<FileDataDefinitionFlatbuffer> Pack(flatbuffers::FlatBufferBuilder &_fbb,
		const FileDataDefinition *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct FileDataDefinitionBuilder {
	flatbuffers::FlatBufferBuilder &fbb_;
	flatbuffers::uoffset_t start_;

	void add_ByteOffset(int64_t ByteOffset) {
		fbb_.AddElement<int64_t>(FileDataDefinitionFlatbuffer::VT_BYTEOFFSET, ByteOffset, 0);
	}

	void add_PacketInfo(const PacketHeader *PacketInfo) {
		fbb_.AddStruct(FileDataDefinitionFlatbuffer::VT_PACKETINFO, PacketInfo);
	}

	void add_NumElements(int64_t NumElements) {
		fbb_.AddElement<int64_t>(FileDataDefinitionFlatbuffer::VT_NUMELEMENTS, NumElements, 0);
	}

	void add_TimestampStart(int64_t TimestampStart) {
		fbb_.AddElement<int64_t>(FileDataDefinitionFlatbuffer::VT_TIMESTAMPSTART, TimestampStart, 0);
	}

	void add_TimestampEnd(int64_t TimestampEnd) {
		fbb_.AddElement<int64_t>(FileDataDefinitionFlatbuffer::VT_TIMESTAMPEND, TimestampEnd, 0);
	}

	explicit FileDataDefinitionBuilder(flatbuffers::FlatBufferBuilder &_fbb) : fbb_(_fbb) {
		start_ = fbb_.StartTable();
	}

	FileDataDefinitionBuilder &operator=(const FileDataDefinitionBuilder &);

	flatbuffers::Offset<FileDataDefinitionFlatbuffer> Finish() {
		const auto end = fbb_.EndTable(start_);
		auto o         = flatbuffers::Offset<FileDataDefinitionFlatbuffer>(end);
		return o;
	}
};

inline flatbuffers::Offset<FileDataDefinitionFlatbuffer> CreateFileDataDefinition(flatbuffers::FlatBufferBuilder &_fbb,
	int64_t ByteOffset = 0, const PacketHeader *PacketInfo = 0, int64_t NumElements = 0, int64_t TimestampStart = 0,
	int64_t TimestampEnd = 0) {
	FileDataDefinitionBuilder builder_(_fbb);
	builder_.add_TimestampEnd(TimestampEnd);
	builder_.add_TimestampStart(TimestampStart);
	builder_.add_NumElements(NumElements);
	builder_.add_ByteOffset(ByteOffset);
	builder_.add_PacketInfo(PacketInfo);
	return builder_.Finish();
}

flatbuffers::Offset<FileDataDefinitionFlatbuffer> CreateFileDataDefinition(flatbuffers::FlatBufferBuilder &_fbb,
	const FileDataDefinition *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

struct FileDataTable : public flatbuffers::NativeTable {
	typedef FileDataTableFlatbuffer TableType;

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.FileDataTable";
	}

	dv::cvector<FileDataDefinition> Table;

	FileDataTable() {
	}

	// Generated Constructor
	FileDataTable(const dv::cvector<FileDataDefinition> &_Table) : Table{_Table} {
	}
};

inline bool operator==(const FileDataTable &lhs, const FileDataTable &rhs) {
	return (lhs.Table == rhs.Table);
}

struct FileDataTableFlatbuffer FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
	typedef FileDataTable NativeTableType;
	static FLATBUFFERS_CONSTEXPR const char *identifier = "FTAB";

	static const flatbuffers::TypeTable *MiniReflectTypeTable() {
		return FileDataTableTypeTable();
	}

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.FileDataTable";
	}

	enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
		VT_TABLE = 4
	};

	const flatbuffers::Vector<flatbuffers::Offset<FileDataDefinitionFlatbuffer>> *Table() const {
		return GetPointer<const flatbuffers::Vector<flatbuffers::Offset<FileDataDefinitionFlatbuffer>> *>(VT_TABLE);
	}

	bool Verify(flatbuffers::Verifier &verifier) const {
		return VerifyTableStart(verifier) && VerifyOffset(verifier, VT_TABLE) && verifier.VerifyVector(Table())
			&& verifier.VerifyVectorOfTables(Table()) && verifier.EndTable();
	}

	FileDataTable *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	void UnPackTo(FileDataTable *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	static void UnPackToFrom(FileDataTable *_o, const FileDataTableFlatbuffer *_fb,
		const flatbuffers::resolver_function_t *_resolver = nullptr);
	static flatbuffers::Offset<FileDataTableFlatbuffer> Pack(flatbuffers::FlatBufferBuilder &_fbb,
		const FileDataTable *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct FileDataTableBuilder {
	flatbuffers::FlatBufferBuilder &fbb_;
	flatbuffers::uoffset_t start_;

	void add_Table(flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<FileDataDefinitionFlatbuffer>>> Table) {
		fbb_.AddOffset(FileDataTableFlatbuffer::VT_TABLE, Table);
	}

	explicit FileDataTableBuilder(flatbuffers::FlatBufferBuilder &_fbb) : fbb_(_fbb) {
		start_ = fbb_.StartTable();
	}

	FileDataTableBuilder &operator=(const FileDataTableBuilder &);

	flatbuffers::Offset<FileDataTableFlatbuffer> Finish() {
		const auto end = fbb_.EndTable(start_);
		auto o         = flatbuffers::Offset<FileDataTableFlatbuffer>(end);
		return o;
	}
};

inline flatbuffers::Offset<FileDataTableFlatbuffer> CreateFileDataTable(flatbuffers::FlatBufferBuilder &_fbb,
	flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<FileDataDefinitionFlatbuffer>>> Table = 0) {
	FileDataTableBuilder builder_(_fbb);
	builder_.add_Table(Table);
	return builder_.Finish();
}

inline flatbuffers::Offset<FileDataTableFlatbuffer> CreateFileDataTableDirect(flatbuffers::FlatBufferBuilder &_fbb,
	const std::vector<flatbuffers::Offset<FileDataDefinitionFlatbuffer>> *Table = nullptr) {
	auto Table__ = Table ? _fbb.CreateVector<flatbuffers::Offset<FileDataDefinitionFlatbuffer>>(*Table) : 0;
	return dv::CreateFileDataTable(_fbb, Table__);
}

flatbuffers::Offset<FileDataTableFlatbuffer> CreateFileDataTable(flatbuffers::FlatBufferBuilder &_fbb,
	const FileDataTable *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

inline FileDataDefinition *FileDataDefinitionFlatbuffer::UnPack(
	const flatbuffers::resolver_function_t *_resolver) const {
	auto _o = new FileDataDefinition();
	UnPackTo(_o, _resolver);
	return _o;
}

inline void FileDataDefinitionFlatbuffer::UnPackTo(
	FileDataDefinition *_o, const flatbuffers::resolver_function_t *_resolver) const {
	(void) _o;
	(void) _resolver;
	UnPackToFrom(_o, this, _resolver);
}

inline void FileDataDefinitionFlatbuffer::UnPackToFrom(FileDataDefinition *_o, const FileDataDefinitionFlatbuffer *_fb,
	const flatbuffers::resolver_function_t *_resolver) {
	(void) _o;
	(void) _fb;
	(void) _resolver;
	{
		auto _e        = _fb->ByteOffset();
		_o->ByteOffset = _e;
	};
	{
		auto _e = _fb->PacketInfo();
		if (_e)
			_o->PacketInfo = *_e;
	};
	{
		auto _e         = _fb->NumElements();
		_o->NumElements = _e;
	};
	{
		auto _e            = _fb->TimestampStart();
		_o->TimestampStart = _e;
	};
	{
		auto _e          = _fb->TimestampEnd();
		_o->TimestampEnd = _e;
	};
}

inline flatbuffers::Offset<FileDataDefinitionFlatbuffer> FileDataDefinitionFlatbuffer::Pack(
	flatbuffers::FlatBufferBuilder &_fbb, const FileDataDefinition *_o,
	const flatbuffers::rehasher_function_t *_rehasher) {
	return CreateFileDataDefinition(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<FileDataDefinitionFlatbuffer> CreateFileDataDefinition(flatbuffers::FlatBufferBuilder &_fbb,
	const FileDataDefinition *_o, const flatbuffers::rehasher_function_t *_rehasher) {
	(void) _rehasher;
	(void) _o;

	struct _VectorArgs {
		flatbuffers::FlatBufferBuilder *__fbb;
		const FileDataDefinition *__o;
		const flatbuffers::rehasher_function_t *__rehasher;
	} _va = {&_fbb, _o, _rehasher};

	(void) _va;
	auto _ByteOffset     = _o->ByteOffset;
	auto _PacketInfo     = &_o->PacketInfo;
	auto _NumElements    = _o->NumElements;
	auto _TimestampStart = _o->TimestampStart;
	auto _TimestampEnd   = _o->TimestampEnd;
	return dv::CreateFileDataDefinition(_fbb, _ByteOffset, _PacketInfo, _NumElements, _TimestampStart, _TimestampEnd);
}

inline FileDataTable *FileDataTableFlatbuffer::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
	auto _o = new FileDataTable();
	UnPackTo(_o, _resolver);
	return _o;
}

inline void FileDataTableFlatbuffer::UnPackTo(
	FileDataTable *_o, const flatbuffers::resolver_function_t *_resolver) const {
	(void) _o;
	(void) _resolver;
	UnPackToFrom(_o, this, _resolver);
}

inline void FileDataTableFlatbuffer::UnPackToFrom(
	FileDataTable *_o, const FileDataTableFlatbuffer *_fb, const flatbuffers::resolver_function_t *_resolver) {
	(void) _o;
	(void) _fb;
	(void) _resolver;
	{
		auto _e = _fb->Table();
		if (_e) {
			_o->Table.resize(_e->size());
			for (flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) {
				_e->Get(_i)->UnPackTo(&_o->Table[_i], _resolver);
			}
		}
	};
}

inline flatbuffers::Offset<FileDataTableFlatbuffer> FileDataTableFlatbuffer::Pack(
	flatbuffers::FlatBufferBuilder &_fbb, const FileDataTable *_o, const flatbuffers::rehasher_function_t *_rehasher) {
	return CreateFileDataTable(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<FileDataTableFlatbuffer> CreateFileDataTable(
	flatbuffers::FlatBufferBuilder &_fbb, const FileDataTable *_o, const flatbuffers::rehasher_function_t *_rehasher) {
	(void) _rehasher;
	(void) _o;

	struct _VectorArgs {
		flatbuffers::FlatBufferBuilder *__fbb;
		const FileDataTable *__o;
		const flatbuffers::rehasher_function_t *__rehasher;
	} _va = {&_fbb, _o, _rehasher};

	(void) _va;
	auto _Table = _o->Table.size() ? _fbb.CreateVector<flatbuffers::Offset<FileDataDefinitionFlatbuffer>>(
					  _o->Table.size(),
					  [](size_t i, _VectorArgs *__va) {
						  return CreateFileDataDefinition(*__va->__fbb, &__va->__o->Table[i], __va->__rehasher);
					  },
					  &_va)
								   : 0;
	return dv::CreateFileDataTable(_fbb, _Table);
}

inline const flatbuffers::TypeTable *PacketHeaderTypeTable() {
	static const flatbuffers::TypeCode type_codes[] = {
		{flatbuffers::ET_INT, 0, -1},
        {flatbuffers::ET_INT, 0, -1}
    };
	static const int64_t values[]          = {0, 4, 8};
	static const char *const names[]       = {"StreamID", "Size"};
	static const flatbuffers::TypeTable tt = {flatbuffers::ST_STRUCT, 2, type_codes, nullptr, values, names};
	return &tt;
}

inline const flatbuffers::TypeTable *FileDataDefinitionTypeTable() {
	static const flatbuffers::TypeCode type_codes[] = {
		{flatbuffers::ET_LONG,     0, -1},
        {flatbuffers::ET_SEQUENCE, 0, 0 },
        {flatbuffers::ET_LONG,     0, -1},
		{flatbuffers::ET_LONG,     0, -1},
        {flatbuffers::ET_LONG,     0, -1}
    };
	static const flatbuffers::TypeFunction type_refs[] = {PacketHeaderTypeTable};
	static const char *const names[] = {"ByteOffset", "PacketInfo", "NumElements", "TimestampStart", "TimestampEnd"};
	static const flatbuffers::TypeTable tt = {flatbuffers::ST_TABLE, 5, type_codes, type_refs, nullptr, names};
	return &tt;
}

inline const flatbuffers::TypeTable *FileDataTableTypeTable() {
	static const flatbuffers::TypeCode type_codes[] = {
		{flatbuffers::ET_SEQUENCE, 1, 0}
    };
	static const flatbuffers::TypeFunction type_refs[] = {FileDataDefinitionTypeTable};
	static const char *const names[]                   = {"Table"};
	static const flatbuffers::TypeTable tt = {flatbuffers::ST_TABLE, 1, type_codes, type_refs, nullptr, names};
	return &tt;
}

inline const dv::FileDataTableFlatbuffer *GetFileDataTable(const void *buf) {
	return flatbuffers::GetRoot<dv::FileDataTableFlatbuffer>(buf);
}

inline const dv::FileDataTableFlatbuffer *GetSizePrefixedFileDataTable(const void *buf) {
	return flatbuffers::GetSizePrefixedRoot<dv::FileDataTableFlatbuffer>(buf);
}

inline const char *FileDataTableIdentifier() {
	return "FTAB";
}

inline bool FileDataTableBufferHasIdentifier(const void *buf) {
	return flatbuffers::BufferHasIdentifier(buf, FileDataTableIdentifier());
}

inline bool VerifyFileDataTableBuffer(flatbuffers::Verifier &verifier) {
	return verifier.VerifyBuffer<dv::FileDataTableFlatbuffer>(FileDataTableIdentifier());
}

inline bool VerifySizePrefixedFileDataTableBuffer(flatbuffers::Verifier &verifier) {
	return verifier.VerifySizePrefixedBuffer<dv::FileDataTableFlatbuffer>(FileDataTableIdentifier());
}

inline void FinishFileDataTableBuffer(
	flatbuffers::FlatBufferBuilder &fbb, flatbuffers::Offset<dv::FileDataTableFlatbuffer> root) {
	fbb.Finish(root, FileDataTableIdentifier());
}

inline void FinishSizePrefixedFileDataTableBuffer(
	flatbuffers::FlatBufferBuilder &fbb, flatbuffers::Offset<dv::FileDataTableFlatbuffer> root) {
	fbb.FinishSizePrefixed(root, FileDataTableIdentifier());
}

inline std::unique_ptr<FileDataTable> UnPackFileDataTable(
	const void *buf, const flatbuffers::resolver_function_t *res = nullptr) {
	return std::unique_ptr<FileDataTable>(GetFileDataTable(buf)->UnPack(res));
}

} // namespace dv

#endif // FLATBUFFERS_GENERATED_FILEDATATABLE_DV_H_
