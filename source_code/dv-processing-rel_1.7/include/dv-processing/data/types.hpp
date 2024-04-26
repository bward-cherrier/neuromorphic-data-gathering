#pragma once

#include "../external/flatbuffers/flatbuffers.h"

#include "../core/utils.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"

namespace dv::types {

constexpr uint32_t IdentifierStringToId(const std::string_view id) noexcept {
	auto ret = static_cast<uint32_t>(id.at(3));
	ret      |= (static_cast<uint32_t>(id.at(2)) << 8);
	ret      |= (static_cast<uint32_t>(id.at(1)) << 16);
	ret      |= (static_cast<uint32_t>(id.at(0)) << 24);
	return ret;
}

constexpr std::array<char, 5> IdToIdentifierString(const uint32_t id) noexcept {
	std::array<char, 5> ret{};
	ret.at(0) = static_cast<char>((id >> 24) & 0x00FF);
	ret.at(1) = static_cast<char>((id >> 16) & 0x00FF);
	ret.at(2) = static_cast<char>((id >> 8) & 0x00FF);
	ret.at(3) = static_cast<char>((id) &0x00FF);
	return ret;
}

struct TimeElementExtractor {
	int64_t startTimestamp;
	int64_t endTimestamp;
	int64_t numElements;

	constexpr TimeElementExtractor() noexcept : startTimestamp(-1), endTimestamp(-1), numElements(-1) {
	}

	constexpr TimeElementExtractor(const int64_t startTimestamp_, const int64_t endTimestamp_) noexcept :
		startTimestamp(startTimestamp_),
		endTimestamp(endTimestamp_),
		numElements(-1) {
	}

	~TimeElementExtractor() = default;

	// Types should be copyable and movable.
	TimeElementExtractor(const TimeElementExtractor &t)              = default;
	TimeElementExtractor &operator=(const TimeElementExtractor &rhs) = default;
	TimeElementExtractor(TimeElementExtractor &&t)                   = default;
	TimeElementExtractor &operator=(TimeElementExtractor &&rhs)      = default;

	constexpr bool operator==(const TimeElementExtractor &rhs) const noexcept {
		return ((startTimestamp == rhs.startTimestamp) && (endTimestamp == rhs.endTimestamp)
				&& (numElements == rhs.numElements));
	}

	constexpr bool operator!=(const TimeElementExtractor &rhs) const noexcept {
		return (!operator==(rhs));
	}
};

static_assert(std::is_standard_layout_v<TimeElementExtractor>, "TimeElementExtractor is not standard layout");
static_assert(std::is_trivially_copyable_v<TimeElementExtractor>, "TimeElementExtractor is not trivially copyable");
static_assert(
	std::is_trivially_destructible_v<TimeElementExtractor>, "TimeElementExtractor is not trivially destructible");
static_assert(sizeof(TimeElementExtractor) == (3 * sizeof(int64_t)), "TimeElementExtractor size is unexpected");
static_assert((alignof(TimeElementExtractor) == sizeof(int64_t))
				  || ((sizeof(void *) == sizeof(int32_t)) && (alignof(TimeElementExtractor) == sizeof(int32_t))),
	"TimeElementExtractor alignment is unexpected");

using PackFuncPtr             = std::add_pointer_t<uint32_t(void *toFlatBufferBuilder, const void *fromObject)>;
using UnpackFuncPtr           = std::add_pointer_t<void(void *toObject, const void *fromFlatBuffer)>;
using ConstructPtr            = std::add_pointer_t<void *(const size_t sizeOfObject)>;
using DestructPtr             = std::add_pointer_t<void(void *object)>;
using TimeElementExtractorPtr = std::add_pointer_t<void(const void *object, TimeElementExtractor *rangeOut)>;
using TimeRangeExtractorPtr   = std::add_pointer_t<void(void *toObject, const void *fromObject,
    const TimeElementExtractor *rangeIn, uint32_t *commitNowOut, uint32_t *exceedsTimeRangeOut)>;

// Used for Type System.
struct Type {
	uint32_t id;
	size_t sizeOfType;
	PackFuncPtr pack;
	UnpackFuncPtr unpack;
	ConstructPtr construct;
	DestructPtr destruct;
	TimeElementExtractorPtr timeElementExtractor;
	TimeRangeExtractorPtr timeRangeExtractor;

	constexpr Type() noexcept :
		id(0),
		sizeOfType(0),
		pack(nullptr),
		unpack(nullptr),
		construct(nullptr),
		destruct(nullptr),
		timeElementExtractor(nullptr),
		timeRangeExtractor(nullptr) {
	}

	constexpr Type(const std::string_view identifier_, const size_t sizeOfType_, PackFuncPtr pack_,
		UnpackFuncPtr unpack_, ConstructPtr construct_, DestructPtr destruct_,
		TimeElementExtractorPtr timeElementExtractor_, TimeRangeExtractorPtr timeRangeExtractor_) :
		id(0),
		sizeOfType(sizeOfType_),
		pack(pack_),
		unpack(unpack_),
		construct(construct_),
		destruct(destruct_),
		timeElementExtractor(timeElementExtractor_),
		timeRangeExtractor(timeRangeExtractor_) {
		if (identifier_.empty() || (identifier_.size() != 4)) {
			throw std::invalid_argument("Type identifier must be exactly four characters long.");
		}

		id = IdentifierStringToId(identifier_);
	}

	~Type() = default;

	// Types should be copyable and movable.
	Type(const Type &t)              = default;
	Type &operator=(const Type &rhs) = default;
	Type(Type &&t)                   = default;
	Type &operator=(Type &&rhs)      = default;

	constexpr bool operator==(const Type &rhs) const noexcept {
		return ((id == rhs.id) && (sizeOfType == rhs.sizeOfType));
	}

	constexpr bool operator!=(const Type &rhs) const noexcept {
		return (!operator==(rhs));
	}
};

static_assert(std::is_standard_layout_v<Type>, "Type is not standard layout");
static_assert(std::is_trivially_copyable_v<Type>, "Type is not trivially copyable");
static_assert(std::is_trivially_destructible_v<Type>, "Type is not trivially destructible");
static_assert(sizeof(Type) == (8 * sizeof(size_t)), "Type size is unexpected");
static_assert(
	(sizeof(void *) == sizeof(int32_t)) ? (alignof(Type) == sizeof(int32_t)) : (alignof(Type) == sizeof(int64_t)),
	"Type alignment is unexpected");

// Used for AEDAT4 I/O.
struct TypedObject {
	void *obj;
	Type type;

	constexpr TypedObject(const Type &type_) : obj{nullptr}, type(type_) {
		obj = (*type.construct)(type.sizeOfType);
		if (obj == nullptr) {
			throw std::bad_alloc();
		}
	}

	~TypedObject() noexcept {
		if (obj != nullptr) {
			(*type.destruct)(obj);
		}
	}

	// Typed objects should not be copyable, but movable.
	TypedObject(const TypedObject &t)              = delete;
	TypedObject &operator=(const TypedObject &rhs) = delete;

	TypedObject(TypedObject &&t) {
		obj  = t.obj;
		type = t.type;

		t.obj  = nullptr;
		t.type = Type{}; // NULL placeholder type.
	}

	TypedObject &operator=(TypedObject &&rhs) {
		assert(this != &rhs);

		if (obj != nullptr) {
			(*type.destruct)(obj);
		}

		obj  = rhs.obj;
		type = rhs.type;

		rhs.obj  = nullptr;
		rhs.type = Type{}; // NULL placeholder type.

		return *this;
	}

	constexpr bool operator==(const TypedObject &rhs) const noexcept {
		return ((obj == rhs.obj) && (type == rhs.type));
	}

	constexpr bool operator!=(const TypedObject &rhs) const noexcept {
		return (!operator==(rhs));
	}

	/**
	 * Cast and move the pointer to the data into a shared pointer. The underlying data is not affected, but it
	 * invalidates this instance and passes the ownership of the data to the shared pointer - it will take care of
	 * memory management from the point of this method call.
	 *
	 * @tparam TargetType Target type to cast the typed obect into
	 * @return
	 */
	template<class TargetType>
	requires dv::concepts::FlatbufferPacket<TargetType>
	[[nodiscard]] std::shared_ptr<TargetType> moveToSharedPtr() {
		if (obj == nullptr) {
			throw dv::exceptions::RuntimeError("TypedObject does not contain any data");
		}

		static constexpr int32_t typeId = dv::types::IdentifierStringToId(TargetType::TableType::identifier);
		if (type.id != typeId) {
			throw dv::exceptions::InvalidArgument<std::string>(
				fmt::format("Incompatible type to cast the data into; compatible type identifier: {}",
					dv::types::IdToIdentifierString(type.id).data()),
				TargetType::TableType::identifier);
		}

		using NonConstTargetType = typename std::remove_const_t<TargetType>;
		std::shared_ptr<TargetType> ptr(
			static_cast<NonConstTargetType *>(obj), [type = this->type](NonConstTargetType *objPtr) {
				(*type.destruct)(static_cast<void *>(objPtr));
			});

		// Invalidate this
		obj  = nullptr;
		type = Type{}; // NULL placeholder type.

		return ptr;
	}
};

static_assert(std::is_standard_layout_v<TypedObject>, "TypedObject is not standard layout");
static_assert(sizeof(TypedObject) == (9 * sizeof(size_t)), "TypedObject size is unexpected");
static_assert((sizeof(void *) == sizeof(int32_t)) ? (alignof(TypedObject) == sizeof(int32_t))
												  : (alignof(TypedObject) == sizeof(int64_t)),
	"TypedObject alignment is unexpected");

template<typename ObjectAPIType>
inline uint32_t Packer(void *toFlatBufferBuilder, const void *fromObject) {
	using FBType = typename ObjectAPIType::TableType;

	assert(toFlatBufferBuilder != nullptr);
	assert(fromObject != nullptr);

	return (FBType::Pack(*(static_cast<flatbuffers::FlatBufferBuilder *>(toFlatBufferBuilder)),
		static_cast<const ObjectAPIType *>(fromObject), nullptr)
				.o);
}

template<typename ObjectAPIType>
inline void Unpacker(void *toObject, const void *fromFlatBuffer) {
	using FBType = typename ObjectAPIType::TableType;

	assert(toObject != nullptr);
	assert(fromFlatBuffer != nullptr);

	FBType::UnPackToFrom(static_cast<ObjectAPIType *>(toObject), static_cast<const FBType *>(fromFlatBuffer), nullptr);
}

template<typename ObjectAPIType, typename SubObjectAPIType>
inline void TimeElementExtractorDefault(const void *object, TimeElementExtractor *rangeOut) noexcept {
	assert(object != nullptr);
	assert(rangeOut != nullptr);

	const auto *obj = static_cast<const ObjectAPIType *>(object);

	// Information extraction mode: get number of contained elements
	// and timestamp related information.
	if constexpr (dv::concepts::HasElementsVector<ObjectAPIType>) {
		rangeOut->numElements = static_cast<int64_t>(obj->elements.size());

		if (rangeOut->numElements > 0) {
			if constexpr (dv::concepts::TimestampedByMember<SubObjectAPIType>) {
				rangeOut->startTimestamp = obj->elements.front().timestamp;
				rangeOut->endTimestamp   = obj->elements.back().timestamp;
			}
			else if constexpr (dv::concepts::TimestampedByAccessor<SubObjectAPIType>) {
				rangeOut->startTimestamp = obj->elements.front().timestamp();
				rangeOut->endTimestamp   = obj->elements.back().timestamp();
			}
			else {
				static_assert(dv::concepts::TimestampedByMember<SubObjectAPIType>
								  || dv::concepts::TimestampedByAccessor<SubObjectAPIType>,
					"DV types must have a timestamp, either as a function or member variable.");
			}
		}
		else {
			rangeOut->startTimestamp = rangeOut->endTimestamp = -1;
		}
	}
	else {
		rangeOut->numElements = 1;

		if constexpr (dv::concepts::TimestampedByMember<ObjectAPIType>) {
			rangeOut->startTimestamp = rangeOut->endTimestamp = obj->timestamp;
		}
		else if constexpr (dv::concepts::TimestampedByAccessor<ObjectAPIType>) {
			rangeOut->startTimestamp = rangeOut->endTimestamp = obj->timestamp();
		}
		else {
			static_assert(
				dv::concepts::TimestampedByMember<ObjectAPIType> || dv::concepts::TimestampedByAccessor<ObjectAPIType>,
				"DV types must have a timestamp, either as a function or member variable.");
		}
	}
}

template<typename ObjectAPIType, typename SubObjectAPIType>
inline void TimeRangeExtractorDefault(void *toObject, const void *fromObject, const TimeElementExtractor *rangeIn,
	uint32_t *commitNowOut, uint32_t *exceedsTimeRangeAndKeepPacketOut) {
	assert(toObject != nullptr);
	assert(fromObject != nullptr);
	assert(rangeIn != nullptr);
	assert(rangeIn->numElements == -1); // Element-based extraction not supported.
	assert(commitNowOut != nullptr);
	assert(exceedsTimeRangeAndKeepPacketOut != nullptr);

	auto *toObj         = static_cast<ObjectAPIType *>(toObject);
	const auto *fromObj = static_cast<const ObjectAPIType *>(fromObject);

	if constexpr (dv::concepts::HasElementsVector<ObjectAPIType>) {
		const auto &fromElements = fromObj->elements;

		if (fromElements.size() > 0) {
			const auto lower = std::lower_bound(fromElements.cbegin(), fromElements.cend(), rangeIn->startTimestamp,
				[](const SubObjectAPIType &elem, const int64_t val) {
					if constexpr (dv::concepts::TimestampedByMember<SubObjectAPIType>) {
						return (elem.timestamp < val);
					}
					else if constexpr (dv::concepts::TimestampedByAccessor<SubObjectAPIType>) {
						return (elem.timestamp() < val);
					}
					else {
						static_assert(dv::concepts::TimestampedByMember<SubObjectAPIType>
										  || dv::concepts::TimestampedByAccessor<SubObjectAPIType>,
							"DV types must have a timestamp, either as a function or member variable.");
					}
				});

			if (lower == fromElements.cend()) {
				// Packet ignored because it lies in the past and the current time range is already beyond that.
				// It is likely that the timestamps are corrupted, and not monotonically increasing.
				// All timestamps are smaller than the start timestamp, commit data to flush any old data, do not keep
				// data because it could never be used again.
				*commitNowOut                     = true;
				*exceedsTimeRangeAndKeepPacketOut = false;
				return;
			}

			const auto upper = std::upper_bound(
				lower, fromElements.cend(), rangeIn->endTimestamp, [](const int64_t val, const SubObjectAPIType &elem) {
					if constexpr (dv::concepts::TimestampedByMember<SubObjectAPIType>) {
						return (elem.timestamp > val);
					}
					else if constexpr (dv::concepts::TimestampedByAccessor<SubObjectAPIType>) {
						return (elem.timestamp() > val);
					}
					else {
						static_assert(dv::concepts::TimestampedByMember<SubObjectAPIType>
										  || dv::concepts::TimestampedByAccessor<SubObjectAPIType>,
							"DV types must have a timestamp, either as a function or member variable.");
					}
				});

			auto objSize = toObj->elements.size();

			toObj->elements.resize(objSize + static_cast<size_t>(std::distance(lower, upper)));

			std::for_each(lower, upper, [toObj, objSize](const SubObjectAPIType &elem) mutable {
				toObj->elements[objSize++] = elem;
			});

			if (upper != fromElements.cend()) {
				// We have at least one element that came later than the time range, commit, and signal that data
				// exceeds time range and should be kept for later use.
				*commitNowOut                     = true;
				*exceedsTimeRangeAndKeepPacketOut = true;
				return;
			}
			else {
				*commitNowOut                     = false;
				*exceedsTimeRangeAndKeepPacketOut = false;
				return;
			}
		}
	}
	else {
		if constexpr (dv::concepts::TimestampedByMember<ObjectAPIType>) {
			if ((fromObj->timestamp >= rangeIn->startTimestamp) && (fromObj->timestamp <= rangeIn->endTimestamp)) {
				*toObj = *fromObj;

				// Each unpacked frame needs to be committed as multiple frames cannot be committed simultaneously.
				// Data does not exceed time range and we can remove it.
				*commitNowOut                     = true;
				*exceedsTimeRangeAndKeepPacketOut = false;
				return;
			}
			else {
				// Timestamp outside of range. No commit, as no new data unpacked. Data exceeds time range and
				// we keep it around for later.
				*commitNowOut                     = false;
				*exceedsTimeRangeAndKeepPacketOut = true;
				return;
			}
		}
		else if constexpr (dv::concepts::TimestampedByAccessor<ObjectAPIType>) {
			if ((fromObj->timestamp() >= rangeIn->startTimestamp) && (fromObj->timestamp() <= rangeIn->endTimestamp)) {
				*toObj = *fromObj;

				// Each unpacked frame needs to be committed as multiple frames cannot be committed simultaneously.
				// Data does not exceed time range and we can remove it.
				*commitNowOut                     = true;
				*exceedsTimeRangeAndKeepPacketOut = false;
				return;
			}
			else {
				// Timestamp outside of range. No commit, as no new data unpacked. Data exceeds time range and
				// we keep it around for later.
				*commitNowOut                     = false;
				*exceedsTimeRangeAndKeepPacketOut = true;
				return;
			}
		}
		else {
			static_assert(
				dv::concepts::TimestampedByMember<ObjectAPIType> || dv::concepts::TimestampedByAccessor<ObjectAPIType>,
				"DV types must have a timestamp, either as a function or member variable.");
		}
	}

	// No commit, no knowledge whether data exceeds time range.
	*commitNowOut                     = false;
	*exceedsTimeRangeAndKeepPacketOut = false;
}

template<typename ObjectAPIType, typename SubObjectAPIType>
constexpr Type makeTypeDefinition() {
	static_assert(std::is_standard_layout_v<ObjectAPIType>, "ObjectAPIType is not standard layout");
	static_assert(std::is_standard_layout_v<SubObjectAPIType>, "SubObjectAPIType is not standard layout");

	return Type{ObjectAPIType::TableType::identifier, sizeof(ObjectAPIType), &Packer<ObjectAPIType>,
		&Unpacker<ObjectAPIType>, &dv::mallocConstructorSize<ObjectAPIType>, &dv::mallocDestructor<ObjectAPIType>,
		&TimeElementExtractorDefault<ObjectAPIType, SubObjectAPIType>,
		&TimeRangeExtractorDefault<ObjectAPIType, SubObjectAPIType>};
}

template<typename ObjectAPIType, typename SubObjectAPIType>
[[deprecated("Type description is not used anymore and will be removed, please use makeTypeDefinition() with no "
			 "arguments instead.")]] constexpr Type
	makeTypeDefinition([[maybe_unused]] const std::string_view description) {
	return makeTypeDefinition<ObjectAPIType, SubObjectAPIType>();
}

} // namespace dv::types
