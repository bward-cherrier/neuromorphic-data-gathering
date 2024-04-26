#pragma once

// Common includes, useful for everyone.
#include "../external/compare_compat.hpp"
#include "../external/fmt_compat.hpp"

#include "../exception/exceptions/generic_exceptions.hpp"
#include "concepts.hpp"
#include "dvassert.hpp"
#include "time.hpp"
#include "time_window.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

// Check that basic fixed size types have expected size.
static_assert(sizeof(int8_t) == 1, "int8_t size not 1");
static_assert(sizeof(uint8_t) == 1, "uint8_t size not 1");
static_assert(sizeof(int16_t) == 2, "int16_t size not 2");
static_assert(sizeof(uint16_t) == 2, "uint16_t size not 2");
static_assert(sizeof(int32_t) == 4, "int32_t size not 4");
static_assert(sizeof(uint32_t) == 4, "uint32_t size not 4");
static_assert(sizeof(float) == 4, "float size not 4");
static_assert(sizeof(int64_t) == 8, "int64_t size not 8");
static_assert(sizeof(uint64_t) == 8, "uint64_t size not 8");
static_assert(sizeof(double) == 8, "double size not 8");

static_assert(sizeof(size_t) == sizeof(void *), "void * and size_t are not coherent");

// Check that default alignment is sane.
static_assert(alignof(int8_t) == 1, "int8_t alignment not 1");
static_assert(alignof(uint8_t) == 1, "uint8_t alignment not 1");
static_assert(alignof(int16_t) == 2, "int16_t alignment not 2");
static_assert(alignof(uint16_t) == 2, "uint16_t alignment not 2");
static_assert(alignof(int32_t) == 4, "int32_t alignment not 4");
static_assert(alignof(uint32_t) == 4, "uint32_t alignment not 4");
static_assert(alignof(float) == 4, "float alignment not 4");
static_assert((alignof(int64_t) == 8) || ((sizeof(void *) == 4) && (alignof(int64_t) == 4)),
	"int64_t alignment not 8 (or 4 on some 32bit systems)");
static_assert((alignof(uint64_t) == 8) || ((sizeof(void *) == 4) && (alignof(uint64_t) == 4)),
	"uint64_t alignment not 8 (or 4 on some 32bit systems)");
static_assert((alignof(double) == 8) || ((sizeof(void *) == 4) && (alignof(double) == 4)),
	"double alignment not 8 (or 4 on some 32bit systems)");

static_assert(alignof(size_t) == sizeof(size_t), "size_t alignment not coherent");
static_assert(alignof(void *) == sizeof(void *), "void * alignment not coherent");
static_assert(alignof(size_t) == alignof(void *), "void * and size_t alignment are not coherent");

namespace dv {

/**
 * std::function substitute with exact signature matching.
 * Requires boost::callable_traits installed, which is only available
 * with boost >= 1.66.
 */
template<typename>
struct std_function_exact;

template<typename R, typename... Args>
struct std_function_exact<R(Args...)> : public std::function<R(Args...)> {
	template<typename T, std::enable_if_t<std::is_same_v<boost::callable_traits::return_type_t<T>, R>
											  && std::is_same_v<boost::callable_traits::args_t<T>, std::tuple<Args...>>,
							 bool>
						 = true>
	std_function_exact(T &&t) : std::function<R(Args...)>(std::forward<T>(t)) {
	}
};

/**
 * Functions to help handle enumerations and their values.
 */
template<dv::concepts::Enum Enumeration>
[[nodiscard]] constexpr std::underlying_type_t<Enumeration> EnumAsInteger(const Enumeration value) noexcept {
	return static_cast<typename std::underlying_type_t<Enumeration>>(value);
}

template<dv::concepts::Enum Enumeration, std::integral T>
[[nodiscard]] constexpr Enumeration IntegerAsEnum(const T value) noexcept {
	return static_cast<Enumeration>(static_cast<std::underlying_type_t<Enumeration>>(value));
}

/**
 * Functions to help deal with common vector operations:
 * bool vectorContains(vec, item)
 * bool vectorContainsIf(vec, predicate)
 * bool vectorRemove(vec, item)
 * bool vectorRemoveIf(vec, predicate)
 * void vectorSortUnique(vec)
 * void vectorSortUnique(vec, comparator)
 */
template<typename T, typename U>
[[nodiscard]] inline bool vectorContains(const std::vector<T> &vec, const U &item) {
	const auto result = std::find(vec.cbegin(), vec.cend(), item);

	if (result == vec.cend()) {
		return false;
	}

	return true;
}

template<typename T, typename Pred>
[[nodiscard]] inline bool vectorContainsIf(const std::vector<T> &vec, Pred predicate) {
	const auto result = std::find_if(vec.cbegin(), vec.cend(), predicate);

	if (result == vec.cend()) {
		return false;
	}

	return true;
}

template<typename T, typename U>
inline size_t vectorRemove(std::vector<T> &vec, const U &item) {
	auto firstRemove = std::remove(vec.begin(), vec.end(), item);
	const auto dist  = std::distance(firstRemove, vec.end());

	vec.erase(firstRemove, vec.end());

	return static_cast<size_t>(dist); // Deleted N elements.
}

template<typename T, typename Pred>
inline size_t vectorRemoveIf(std::vector<T> &vec, Pred predicate) {
	auto firstRemove = std::remove_if(vec.begin(), vec.end(), predicate);
	const auto dist  = std::distance(firstRemove, vec.end());

	vec.erase(firstRemove, vec.end());

	return static_cast<size_t>(dist); // Deleted N elements.
}

template<typename T>
inline void vectorSortUnique(std::vector<T> &vec) {
	if (!vec.empty()) {
		std::sort(vec.begin(), vec.end());
		vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
	}
}

template<typename T, typename Compare>
inline void vectorSortUnique(std::vector<T> &vec, Compare comp) {
	if (!vec.empty()) {
		std::sort(vec.begin(), vec.end(), comp);
		vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
	}
}

/**
 * Path cleanup functions for existing paths (canonical) and
 * possibly non-existing ones (absolute).
 */
inline std::filesystem::path pathResolveNonExisting(const std::filesystem::path &path) {
	return std::filesystem::absolute(path).lexically_normal().make_preferred();
}

inline std::filesystem::path pathResolveExisting(const std::filesystem::path &path) {
	return std::filesystem::canonical(path).make_preferred();
}

template<typename ObjectT, typename... Args>
inline void *mallocConstructorSize(const size_t sizeOfObject, Args &&...args) {
	auto *object = static_cast<ObjectT *>(malloc(sizeOfObject));
	if (object == nullptr) {
		throw std::bad_alloc();
	}

	new (object) ObjectT(std::forward<Args>(args)...);

	return object;
}

template<typename ObjectT, typename... Args>
inline void *mallocConstructor(Args &&...args) {
	return dv::mallocConstructorSize<ObjectT>(sizeof(ObjectT), std::forward<Args>(args)...);
}

template<typename ObjectT>
inline void mallocDestructor(void *object) noexcept {
	dv::runtime_assert(object != nullptr, "object to be destroyed cannot be null");

	auto *obj = static_cast<ObjectT *>(object);

	std::destroy_at(obj);

	free(obj);
}

namespace packets {

enum class Timestamp {
	START,
	END
};

/**
 * Template method that retrieves timestamp from a Timestamped structure.
 * @tparam ElementType Type of the element
 * @param element Instance of the element
 * @return Timestamp of this element
 */
template<class ElementType>
requires dv::concepts::Timestamped<ElementType>
[[nodiscard]] inline int64_t getTimestamp(const ElementType &element) {
	if constexpr (dv::concepts::TimestampedByMember<ElementType>) {
		return element.timestamp;
	}
	else if constexpr (dv::concepts::TimestampedByAccessor<ElementType>) {
		return element.timestamp();
	}
}

/**
 * Check if a packet is empty.
 * @tparam PacketType
 * @param packet
 * @return True if the given packet is empty, false otherwise.
 */
template<class PacketType>
requires dv::concepts::EventStorage<PacketType> || dv::concepts::DataPacket<PacketType>
	  || dv::concepts::Iterable<PacketType>
[[nodiscard]] inline bool isPacketEmpty(const PacketType &packet) {
	if constexpr (dv::concepts::EventStorage<PacketType>) {
		return packet.isEmpty();
	}
	else if constexpr (dv::concepts::DataPacket<PacketType>) {
		return packet.elements.size() == 0;
	}
	else {
		return packet.size() == 0;
	}
}

/**
 * Get packet size. This utility template method can be used to generically get size of a EventStore,
 * data packet or any container satisfying the iterable concept.
 * @tparam PacketType
 * @param packet
 * @return Size of the given packet
 */
template<class PacketType>
requires dv::concepts::EventStorage<PacketType> || dv::concepts::DataPacket<PacketType>
	  || dv::concepts::Iterable<PacketType>
[[nodiscard]] inline size_t getPacketSize(const PacketType &packet) {
	if constexpr (dv::concepts::DataPacket<PacketType>) {
		return packet.elements.size();
	}
	else {
		return packet.size();
	}
}

/**
 * Generic getter of a begin iterator of a packet.
 * @tparam PacketType
 * @param packet
 * @return
 */
template<class PacketType>
requires dv::concepts::EventStorage<PacketType> || dv::concepts::DataPacket<PacketType>
	  || dv::concepts::Iterable<PacketType>
[[nodiscard]] inline auto getPacketBegin(const PacketType &packet) {
	if constexpr (dv::concepts::DataPacket<PacketType>) {
		return packet.elements.begin();
	}
	else {
		return packet.begin();
	}
}

/**
 * Generic getter of an end iterator of a packet.
 * @tparam PacketType
 * @param packet
 * @return
 */
template<class PacketType>
requires dv::concepts::EventStorage<PacketType> || dv::concepts::DataPacket<PacketType>
	  || dv::concepts::Iterable<PacketType>
[[nodiscard]] inline auto getPacketEnd(const PacketType &packet) {
	if constexpr (dv::concepts::DataPacket<PacketType>) {
		return packet.elements.end();
	}
	else {
		return packet.end();
	}
}

/**
 * Retrieve packet start or end timestamp using template generation.
 * @tparam startTime Use enum to select whether you want start or end timestamp.
 * @tparam PacketType Packet type, inferred from argument type.
 * @param packet Non-empty data packet.
 * @throws InvalidArgument exception is thrown if the packet is empty.
 * @return Timestamp of the first or last element in the packet.
 */
template<Timestamp startTime, class PacketType>
requires dv::concepts::EventStorage<PacketType> || dv::concepts::DataPacket<PacketType>
	  || dv::concepts::TimestampedIterable<PacketType>
[[nodiscard]] inline int64_t getPacketTimestamp(const PacketType &packet) {
	if (isPacketEmpty(packet)) {
		throw dv::exceptions::InvalidArgument<bool>(
			"Packet can't be empty to access the timestamp!", isPacketEmpty(packet));
	}

	if constexpr (startTime == Timestamp::START) {
		return getTimestamp(*getPacketBegin(packet));
	}
	else {
		return getTimestamp(*std::prev(getPacketEnd(packet)));
	}
}

/**
 * Get time window for a given packet.
 * @tparam PacketType
 * @param packet Non-empty data packet.
 * @throws InvalidArgument exception is thrown if the packet is empty.
 * @return Time window with start and end timestamps of this packet.
 */
template<class PacketType>
requires dv::concepts::EventStorage<PacketType> || dv::concepts::DataPacket<PacketType>
	  || dv::concepts::TimestampedIterable<PacketType>
[[nodiscard]] inline dv::TimeWindow getPacketTimeWindow(const PacketType &packet) {
	if (isPacketEmpty(packet)) {
		throw dv::exceptions::InvalidArgument<bool>(
			"Packet can't be empty to access the timestamp!", isPacketEmpty(packet));
	}
	if constexpr (dv::concepts::EventStorage<PacketType>) {
		return {packet.getLowestTime(), packet.getHighestTime()};
	}
	else {
		return {getTimestamp(*getPacketBegin(packet)), getTimestamp(*std::prev(getPacketEnd(packet)))};
	}
}

} // namespace packets

namespace {

constexpr size_t INTERNAL_STRERROR_BUF_SIZE{512};

#if defined(_WIN32) || defined(_WIN64) || defined(__CYGWIN__)

inline std::string internal_errno_to_string(int errorNumber) {
	char buffer[INTERNAL_STRERROR_BUF_SIZE] = {0x00};
	strerror_s(buffer, INTERNAL_STRERROR_BUF_SIZE, errorNumber);
	return std::string{buffer};
}

#else // POSIX

[[maybe_unused]] inline char *internal_check_strerror(const int result, char *buffer, const int errorNumber) {
	if (result != 0) {
		fmt::format_to_n(buffer, INTERNAL_STRERROR_BUF_SIZE, "unknown error: {}", errorNumber);
	}

	return buffer;
}

[[maybe_unused]] inline char *internal_check_strerror(
	char *result, [[maybe_unused]] char *buffer, [[maybe_unused]] int const errorNumber) {
	return result;
}

inline std::string internal_errno_to_string(int errorNumber) {
	char buffer[INTERNAL_STRERROR_BUF_SIZE] = {0x00};
	return std::string{
		internal_check_strerror(strerror_r(errorNumber, buffer, INTERNAL_STRERROR_BUF_SIZE), buffer, errorNumber)};
}

#endif

} // namespace

inline std::string errnoToString(int errorNumber) {
	return internal_errno_to_string(errorNumber);
}

/**
 * Check whether given point is non-negative and within dimensions of given resolution. The following check is
 * performed: X ∈ [0; (width - 1)] and Y ∈ [0; (height - 1)]. Function will check floating point coordinate
 * fractional part overflow, it will return false in case even fractional part is beyond the valid range.
 * @param point 		Coordinates to check.
 * @param resolution 	Pixel space resolution.
 * @return				True if coordinates are within valid range, false otherwise.
 */
template<concepts::Coordinate2D Input>
[[nodiscard]] inline bool isWithinDimensions(const Input &point, const cv::Size &resolution) {
	if constexpr (concepts::Coordinate2DMembers<Input>) {
		if constexpr (std::floating_point<decltype(point.x)>) {
			return point.x >= 0 && std::ceil(point.x) < resolution.width && point.y >= 0
				&& std::ceil(point.y) < resolution.height;
		}
		else {
			return point.x >= 0 && point.x < resolution.width && point.y >= 0 && point.y < resolution.height;
		}
	}
	else if constexpr (concepts::Coordinate2DAccessors<Input>) {
		if constexpr (std::floating_point<decltype(point.x())>) {
			return point.x() >= 0 && std::ceil(point.x()) < resolution.width && point.y() >= 0
				&& std::ceil(point.y()) < resolution.height;
		}
		else {
			return point.x() >= 0 && point.x() < resolution.width && point.y() >= 0 && point.y() < resolution.height;
		}
	}
}

} // namespace dv

/**
 * fmt formatting support, adds automatic direct formatting
 * support for common data structures:
 * - std::filesystem::path
 * - std::vector<T>
 */
namespace fmt {

template<>
struct formatter<std::filesystem::path> : fmt::formatter<std::string> {
	// parse is inherited from fmt::formatter<std::string>.
	// All paths in DV SDK are UTF-8 at I/O points (construction, printing, usage).
	template<typename FormatContext>
	auto format(const std::filesystem::path &path, FormatContext &ctx) DV_EXT_FMT_CONST {
		return fmt::formatter<std::string>::format(path.string(), ctx);
	}
};

template<typename T>
class formatter<std::vector<T>> {
public:
	constexpr auto parse(format_parse_context &ctx) {
		// Parse the presentation format and store it in the formatter:
		auto it       = ctx.begin();
		auto end      = ctx.end();
		size_t strPos = 0;

		mFmtForward[strPos] = '{';
		strPos++;

		// Any characters before the second : are part of the forwarded
		// format specifier (direct copy), then the separator.
		while ((it != end) && (*it != '}') && (*it != ':')) {
			if (strPos == 1) {
				// If there actually are chars here to copy, we need to
				// also inject a : in the formatter to forward.
				mFmtForward[strPos] = ':';
				strPos++;
			}

			mFmtForward[strPos] = *it;
			strPos++;
			it++;

			if (strPos == (FORMATTER_MAX_LEN - 1)) {
				throw std::out_of_range("Formatter too long, cannot forward.");
			}
		}

		// Close and terminate formatter string.
		mFmtForward[strPos] = '}';
		strPos++;
		mFmtForward[strPos] = 0x00;

		// Skip second : if exists.
		if ((it != end) && (*it == ':')) {
			it++;
		}

		strPos = 0;

		while ((it != end) && (*it != '}')) {
			mSeparator[strPos] = *it;
			strPos++;
			it++;
		}

		if (strPos == 0) {
			// Default separator.
			mSeparator[strPos] = ',';
			strPos++;
			mSeparator[strPos] = ' ';
			strPos++;
		}

		if (strPos == FORMATTER_MAX_LEN) {
			throw std::out_of_range("Separator too long, cannot setup.");
		}

		mSeparator[strPos] = 0x00;

		// Return an iterator past the end of the parsed range:
		return it;
	}

	template<typename FormatContext>
	auto format(const std::vector<T> &vec, FormatContext &ctx) DV_EXT_FMT_CONST {
		return fmt::format_to(ctx.out(), DV_EXT_FMT_RUNTIME(mFmtForward.data()),
			fmt::join(vec.cbegin(), vec.cend(), std::string_view{mSeparator.data()}));
	}

private:
	static constexpr size_t FORMATTER_MAX_LEN{32};

	std::array<char, FORMATTER_MAX_LEN> mFmtForward;
	std::array<char, FORMATTER_MAX_LEN> mSeparator;
};

} // namespace fmt
