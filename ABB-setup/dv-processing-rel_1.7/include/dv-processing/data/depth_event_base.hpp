// automatically generated by the FlatBuffers compiler, do not modify

#ifndef FLATBUFFERS_GENERATED_DEPTHEVENT_DV_H_
#define FLATBUFFERS_GENERATED_DEPTHEVENT_DV_H_

#include "../external/flatbuffers/flatbuffers.h"

#include "cvector.hpp"

namespace dv {

struct DepthEvent;

struct DepthEventPacketFlatbuffer;
struct DepthEventPacket;

bool operator==(const DepthEvent &lhs, const DepthEvent &rhs);
bool operator==(const DepthEventPacket &lhs, const DepthEventPacket &rhs);

inline const flatbuffers::TypeTable *DepthEventTypeTable();

inline const flatbuffers::TypeTable *DepthEventPacketTypeTable();

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(8) DepthEvent FLATBUFFERS_FINAL_CLASS {
	typedef DepthEvent NativeTableType;
	typedef DepthEvent TableType;

private:
	int64_t timestamp_;
	int16_t x_;
	int16_t y_;
	uint8_t polarity_;
	int8_t padding0__;
	uint16_t depth_;

public:
	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.DepthEvent";
	}
	DepthEvent() {
		memset(static_cast<void *>(this), 0, sizeof(DepthEvent));
	}
	DepthEvent(int64_t _timestamp, int16_t _x, int16_t _y, bool _polarity, uint16_t _depth) :
		timestamp_(flatbuffers::EndianScalar(_timestamp)),
		x_(flatbuffers::EndianScalar(_x)),
		y_(flatbuffers::EndianScalar(_y)),
		polarity_(flatbuffers::EndianScalar(static_cast<uint8_t>(_polarity))),
		padding0__(0),
		depth_(flatbuffers::EndianScalar(_depth)) {
		(void) padding0__;
	}
	/// Timestamp (µs).
	int64_t timestamp() const {
		return flatbuffers::EndianScalar(timestamp_);
	}
	int16_t x() const {
		return flatbuffers::EndianScalar(x_);
	}
	int16_t y() const {
		return flatbuffers::EndianScalar(y_);
	}
	bool polarity() const {
		return flatbuffers::EndianScalar(polarity_) != 0;
	}
	uint16_t depth() const {
		return flatbuffers::EndianScalar(depth_);
	}
};

FLATBUFFERS_STRUCT_END(DepthEvent, 16);

inline bool operator==(const DepthEvent &lhs, const DepthEvent &rhs) {
	return (lhs.timestamp() == rhs.timestamp()) && (lhs.x() == rhs.x()) && (lhs.y() == rhs.y())
		&& (lhs.polarity() == rhs.polarity()) && (lhs.depth() == rhs.depth());
}

struct DepthEventPacket : public flatbuffers::NativeTable {
	typedef DepthEventPacketFlatbuffer TableType;

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.DepthEventPacket";
	}

	dv::cvector<DepthEvent> elements;

	DepthEventPacket() {
	}

	// Generated Constructor
	DepthEventPacket(const dv::cvector<DepthEvent> &_elements) : elements{_elements} {
	}

	friend std::ostream &operator<<(std::ostream &os, const DepthEventPacket &packet) {
		if (packet.elements.empty()) {
			os << fmt::format("DepthEventPacket containing 0 events");
			return os;
		}

		const int64_t lowestTime  = packet.elements.front().timestamp();
		const int64_t highestTime = packet.elements.back().timestamp();

		os << fmt::format("DepthEventPacket containing {} events within {}μs duration; time range within [{}; {}]",
			packet.elements.size(), highestTime - lowestTime, lowestTime, highestTime);
		return os;
	}
};

inline bool operator==(const DepthEventPacket &lhs, const DepthEventPacket &rhs) {
	return (lhs.elements == rhs.elements);
}

struct DepthEventPacketFlatbuffer FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
	typedef DepthEventPacket NativeTableType;
	static FLATBUFFERS_CONSTEXPR const char *identifier = "DEVT";

	static const flatbuffers::TypeTable *MiniReflectTypeTable() {
		return DepthEventPacketTypeTable();
	}

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.DepthEventPacket";
	}

	enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
		VT_ELEMENTS = 4
	};

	const flatbuffers::Vector<const DepthEvent *> *elements() const {
		return GetPointer<const flatbuffers::Vector<const DepthEvent *> *>(VT_ELEMENTS);
	}

	bool Verify(flatbuffers::Verifier &verifier) const {
		return VerifyTableStart(verifier) && VerifyOffset(verifier, VT_ELEMENTS) && verifier.VerifyVector(elements())
			&& verifier.EndTable();
	}

	DepthEventPacket *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	void UnPackTo(DepthEventPacket *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	static void UnPackToFrom(DepthEventPacket *_o, const DepthEventPacketFlatbuffer *_fb,
		const flatbuffers::resolver_function_t *_resolver = nullptr);
	static flatbuffers::Offset<DepthEventPacketFlatbuffer> Pack(flatbuffers::FlatBufferBuilder &_fbb,
		const DepthEventPacket *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct DepthEventPacketBuilder {
	flatbuffers::FlatBufferBuilder &fbb_;
	flatbuffers::uoffset_t start_;

	void add_elements(flatbuffers::Offset<flatbuffers::Vector<const DepthEvent *>> elements) {
		fbb_.AddOffset(DepthEventPacketFlatbuffer::VT_ELEMENTS, elements);
	}

	explicit DepthEventPacketBuilder(flatbuffers::FlatBufferBuilder &_fbb) : fbb_(_fbb) {
		start_ = fbb_.StartTable();
	}

	DepthEventPacketBuilder &operator=(const DepthEventPacketBuilder &);

	flatbuffers::Offset<DepthEventPacketFlatbuffer> Finish() {
		const auto end = fbb_.EndTable(start_);
		auto o         = flatbuffers::Offset<DepthEventPacketFlatbuffer>(end);
		return o;
	}
};

inline flatbuffers::Offset<DepthEventPacketFlatbuffer> CreateDepthEventPacket(
	flatbuffers::FlatBufferBuilder &_fbb, flatbuffers::Offset<flatbuffers::Vector<const DepthEvent *>> elements = 0) {
	DepthEventPacketBuilder builder_(_fbb);
	builder_.add_elements(elements);
	return builder_.Finish();
}

inline flatbuffers::Offset<DepthEventPacketFlatbuffer> CreateDepthEventPacketDirect(
	flatbuffers::FlatBufferBuilder &_fbb, const std::vector<DepthEvent> *elements = nullptr) {
	auto elements__ = elements ? _fbb.CreateVectorOfStructs<DepthEvent>(*elements) : 0;
	return dv::CreateDepthEventPacket(_fbb, elements__);
}

flatbuffers::Offset<DepthEventPacketFlatbuffer> CreateDepthEventPacket(flatbuffers::FlatBufferBuilder &_fbb,
	const DepthEventPacket *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

inline DepthEventPacket *DepthEventPacketFlatbuffer::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
	auto _o = new DepthEventPacket();
	UnPackTo(_o, _resolver);
	return _o;
}

inline void DepthEventPacketFlatbuffer::UnPackTo(
	DepthEventPacket *_o, const flatbuffers::resolver_function_t *_resolver) const {
	(void) _o;
	(void) _resolver;
	UnPackToFrom(_o, this, _resolver);
}

inline void DepthEventPacketFlatbuffer::UnPackToFrom(
	DepthEventPacket *_o, const DepthEventPacketFlatbuffer *_fb, const flatbuffers::resolver_function_t *_resolver) {
	(void) _o;
	(void) _fb;
	(void) _resolver;
	{
		auto _e = _fb->elements();
		if (_e) {
			_o->elements.resize(_e->size());
			for (flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) {
				_o->elements[_i] = *_e->Get(_i);
			}
		}
	};
}

inline flatbuffers::Offset<DepthEventPacketFlatbuffer> DepthEventPacketFlatbuffer::Pack(
	flatbuffers::FlatBufferBuilder &_fbb, const DepthEventPacket *_o,
	const flatbuffers::rehasher_function_t *_rehasher) {
	return CreateDepthEventPacket(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<DepthEventPacketFlatbuffer> CreateDepthEventPacket(flatbuffers::FlatBufferBuilder &_fbb,
	const DepthEventPacket *_o, const flatbuffers::rehasher_function_t *_rehasher) {
	(void) _rehasher;
	(void) _o;

	struct _VectorArgs {
		flatbuffers::FlatBufferBuilder *__fbb;
		const DepthEventPacket *__o;
		const flatbuffers::rehasher_function_t *__rehasher;
	} _va = {&_fbb, _o, _rehasher};

	(void) _va;
	auto _elements = _o->elements.size() ? _fbb.CreateVectorOfStructs(_o->elements.data(), _o->elements.size()) : 0;
	return dv::CreateDepthEventPacket(_fbb, _elements);
}

inline const flatbuffers::TypeTable *DepthEventTypeTable() {
	static const flatbuffers::TypeCode type_codes[] = {
		{flatbuffers::ET_LONG,   0, -1},
        {flatbuffers::ET_SHORT,  0, -1},
        {flatbuffers::ET_SHORT,  0, -1},
		{flatbuffers::ET_BOOL,   0, -1},
        {flatbuffers::ET_USHORT, 0, -1}
    };
	static const int64_t values[]          = {0, 8, 10, 12, 14, 16};
	static const char *const names[]       = {"timestamp", "x", "y", "polarity", "depth"};
	static const flatbuffers::TypeTable tt = {flatbuffers::ST_STRUCT, 5, type_codes, nullptr, values, names};
	return &tt;
}

inline const flatbuffers::TypeTable *DepthEventPacketTypeTable() {
	static const flatbuffers::TypeCode type_codes[] = {
		{flatbuffers::ET_SEQUENCE, 1, 0}
    };
	static const flatbuffers::TypeFunction type_refs[] = {DepthEventTypeTable};
	static const char *const names[]                   = {"elements"};
	static const flatbuffers::TypeTable tt = {flatbuffers::ST_TABLE, 1, type_codes, type_refs, nullptr, names};
	return &tt;
}

inline const dv::DepthEventPacketFlatbuffer *GetDepthEventPacket(const void *buf) {
	return flatbuffers::GetRoot<dv::DepthEventPacketFlatbuffer>(buf);
}

inline const dv::DepthEventPacketFlatbuffer *GetSizePrefixedDepthEventPacket(const void *buf) {
	return flatbuffers::GetSizePrefixedRoot<dv::DepthEventPacketFlatbuffer>(buf);
}

inline const char *DepthEventPacketIdentifier() {
	return "DEVT";
}

inline bool DepthEventPacketBufferHasIdentifier(const void *buf) {
	return flatbuffers::BufferHasIdentifier(buf, DepthEventPacketIdentifier());
}

inline bool VerifyDepthEventPacketBuffer(flatbuffers::Verifier &verifier) {
	return verifier.VerifyBuffer<dv::DepthEventPacketFlatbuffer>(DepthEventPacketIdentifier());
}

inline bool VerifySizePrefixedDepthEventPacketBuffer(flatbuffers::Verifier &verifier) {
	return verifier.VerifySizePrefixedBuffer<dv::DepthEventPacketFlatbuffer>(DepthEventPacketIdentifier());
}

inline void FinishDepthEventPacketBuffer(
	flatbuffers::FlatBufferBuilder &fbb, flatbuffers::Offset<dv::DepthEventPacketFlatbuffer> root) {
	fbb.Finish(root, DepthEventPacketIdentifier());
}

inline void FinishSizePrefixedDepthEventPacketBuffer(
	flatbuffers::FlatBufferBuilder &fbb, flatbuffers::Offset<dv::DepthEventPacketFlatbuffer> root) {
	fbb.FinishSizePrefixed(root, DepthEventPacketIdentifier());
}

inline std::unique_ptr<DepthEventPacket> UnPackDepthEventPacket(
	const void *buf, const flatbuffers::resolver_function_t *res = nullptr) {
	return std::unique_ptr<DepthEventPacket>(GetDepthEventPacket(buf)->UnPack(res));
}

} // namespace dv

// fmt compatibility for ostream class printing.
template<>
struct fmt::formatter<dv::DepthEventPacket> : fmt::ostream_formatter {};

#endif // FLATBUFFERS_GENERATED_DEPTHEVENT_DV_H_
