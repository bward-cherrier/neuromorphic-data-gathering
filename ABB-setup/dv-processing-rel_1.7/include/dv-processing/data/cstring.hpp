#pragma once

#include "../external/compare_compat.hpp"
#include "../external/fmt_compat.hpp"

#include "../core/dvassert.hpp"
#include "cptriterator.hpp"

#include <array>
#include <concepts>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace dv {

template<class T>
class basic_cstring {
public:
	// Container traits.
	using value_type       = T;
	using const_value_type = const T;
	using pointer          = T *;
	using const_pointer    = const T *;
	using reference        = T &;
	using const_reference  = const T &;
	using size_type        = size_t;
	using difference_type  = ptrdiff_t;

	static constexpr size_type npos{static_cast<size_type>(-1)};

	static_assert(std::is_standard_layout_v<value_type>, "basic_cstring type is not standard layout");
	static_assert(std::is_trivial_v<value_type>, "basic_cstring type is not trivial");

private:
	static inline T NULL_CHAR{0};

	size_type mCurrSize{0};
	size_type mMaximumSize{0};
	pointer mDataPtr{&NULL_CHAR};

	static_assert(sizeof(size_type) == sizeof(pointer), "size_type and pointer size are not coherent");
	static_assert(alignof(size_type) == sizeof(size_type), "size_type alignment not coherent");
	static_assert(alignof(pointer) == sizeof(pointer), "pointer alignment not coherent");

public:
	// Default constexpr constructor. Initialize empty string with no memory.
	constexpr basic_cstring() noexcept = default;

	// Destructor.
	~basic_cstring() noexcept {
		mCurrSize = 0;
		reallocateMemory(mCurrSize);
	}

	// Copy constructor.
	basic_cstring(const basic_cstring &str, const size_type pos = 0, const size_type count = npos) :
		basic_cstring(str.data(), str.length(), pos, count) {
	}

	// C++23: forbid initialization from nullptr.
	constexpr basic_cstring(std::nullptr_t) = delete;

	// Convenience to initialize from a C string.
	basic_cstring(const_pointer str) : basic_cstring(str, (str == nullptr) ? (0) : (strlen(str))) {
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	basic_cstring(const U &str, const size_type pos = 0, const size_type count = npos) :
		basic_cstring(str.data(), str.size(), pos, count) {
	}

	// Lowest common denominator: a ptr and sizes. Most constructors call this.
	basic_cstring(const_pointer str, const size_type strLength, const size_type pos = 0, const size_type count = npos) {
		if (strLength == 0) {
			return;
		}

		if (str == nullptr) {
			throw std::invalid_argument("string resolves to nullptr.");
		}

		if (pos > strLength) {
			throw std::length_error("position bigger than string length.");
		}

		// Ensure number of characters to copy is within range.
		const auto toCopyCount = (count > (strLength - pos)) ? (strLength - pos) : (count);
		if (toCopyCount == 0) {
			return;
		}

		ensureCapacity(toCopyCount);
		mCurrSize = toCopyCount;

		std::copy_n(const_iterator(str + pos), toCopyCount, begin());

		nullTerminate();
	}

	// Initialize string with N NULL characters.
	explicit basic_cstring(const size_type count) : basic_cstring(count, value_type{}) {
	}

	// Initialize string with N times the given character.
	basic_cstring(const size_type count, const value_type value) {
		if (count == 0) {
			return;
		}

		ensureCapacity(count);
		mCurrSize = count;

		// Initialize elements to copy of value.
		std::fill_n(begin(), count, value);

		nullTerminate();
	}

	// Initialize string with characters from range.
	template<typename InputIt,
		std::enable_if_t<
			std::is_base_of_v<std::input_iterator_tag, typename std::iterator_traits<InputIt>::iterator_category>, bool>
		= true>
	basic_cstring(InputIt first, InputIt last) {
		const auto difference = std::distance(first, last);
		if (difference < 0) {
			throw std::invalid_argument("Inverted iterators (last < first). This is never what you really want.");
		}

		const auto count = static_cast<size_type>(difference);
		if (count == 0) {
			return;
		}

		ensureCapacity(count);
		mCurrSize = count;

		// Initialize elements to copy of range's values.
		std::copy_n(first, count, begin());

		nullTerminate();
	}

	// Initialize vector via initializer list {x, y, z}.
	basic_cstring(std::initializer_list<value_type> init_list) : basic_cstring(init_list.begin(), init_list.end()) {
	}

	// Paths are always char or wchar_t (below).
	template<typename U = T, std::enable_if_t<std::is_same_v<U, char>, bool> = true>
	basic_cstring(const std::filesystem::path &path) : basic_cstring(path.string()) {
	}

	// Paths are always wchar_t or char (above).
	template<typename U = T, std::enable_if_t<std::is_same_v<U, wchar_t>, bool> = true>
	basic_cstring(const std::filesystem::path &path) : basic_cstring(path.wstring()) {
	}

	// Support UTF-8 string types and paths.
	template<typename U = T, std::enable_if_t<std::is_same_v<U, char8_t>, bool> = true>
	basic_cstring(const std::filesystem::path &path) : basic_cstring(path.u8string()) {
	}

	// Support UTF-16 string types and paths.
	template<typename U = T, std::enable_if_t<std::is_same_v<U, char16_t>, bool> = true>
	basic_cstring(const std::filesystem::path &path) : basic_cstring(path.u16string()) {
	}

	// Support UTF-32 string types and paths.
	template<typename U = T, std::enable_if_t<std::is_same_v<U, char32_t>, bool> = true>
	basic_cstring(const std::filesystem::path &path) : basic_cstring(path.u32string()) {
	}

	// Move constructor.
	basic_cstring(basic_cstring &&rhs) noexcept {
		// Moved-from object must remain in a valid state. We can define
		// valid-state-after-move to be nothing allowed but a destructor
		// call, which is what normally happens, and helps us a lot here.

		// Move data here.
		mCurrSize    = rhs.mCurrSize;
		mMaximumSize = rhs.mMaximumSize;
		mDataPtr     = rhs.mDataPtr;

		// Reset to empty string (ready for destruction or reuse).
		rhs.mCurrSize    = 0;
		rhs.mMaximumSize = 0;
		rhs.mDataPtr     = &NULL_CHAR;
	}

	// Move assignment.
	basic_cstring &operator=(basic_cstring &&rhs) noexcept {
		return assign(std::move(rhs));
	}

	// Copy assignment.
	basic_cstring &operator=(const basic_cstring &rhs) {
		return assign(rhs);
	}

	// Extra assignment operators.
	basic_cstring &operator=(const_pointer str) {
		return assign(str);
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	basic_cstring &operator=(const U &rhs) {
		return assign(rhs.data(), rhs.size());
	}

	basic_cstring &operator=(const value_type value) {
		return assign(value);
	}

	basic_cstring &operator=(std::initializer_list<value_type> rhs_list) {
		return assign(rhs_list);
	}

	// Comparison operators.
	bool operator==(const basic_cstring &rhs) const noexcept {
		// Check zero size first: if both have size zero they're
		// equal, if only one of them has a size of zero and the
		// other doesn't they must be different. This is both an
		// optimization and a safety check to avoid weird edge
		// cases in the comparison (esp. for std::string_view).
		if (empty() || rhs.empty()) {
			return (empty() && rhs.empty()) ? (true) : (false);
		}

		// Sizes are different, can't be equal.
		if (size() != rhs.size()) {
			return false;
		}

		return std::equal(cbegin(), cend(), rhs.cbegin(), rhs.cend());
	}

	auto operator<=>(const basic_cstring &rhs) const noexcept {
		// Check zero size first: only if both have size zero
		// are they equal, else it depends on who has a size.
		// This is both an optimization and a safety check to
		// avoid weird edge cases in the comparison (esp. for
		// std::string_view).
		if (empty() || rhs.empty()) {
			if (empty() && !rhs.empty()) {
				// this is zero, LHS has a size. this is less.
				return std::strong_ordering::less;
			}
			else if (!empty() && rhs.empty()) {
				// this has a size, LHS is zero. this is greater.
				return std::strong_ordering::greater;
			}
			else {
				// Both have zero length. Both are equal.
				return std::strong_ordering::equal;
			}
		}

		return std::lexicographical_compare_three_way(cbegin(), cend(), rhs.cbegin(), rhs.cend());
	}

	bool operator==(const_pointer rhs) const noexcept {
		const size_t rhsSize = (rhs == nullptr) ? (0) : (strlen(rhs));

		// Check zero size first: if both have size zero they're
		// equal, if only one of them has a size of zero and the
		// other doesn't they must be different. This is both an
		// optimization and a safety check to avoid weird edge
		// cases in the comparison (esp. for std::string_view).
		if (empty() || (rhsSize == 0)) {
			return (empty() && (rhsSize == 0)) ? (true) : (false);
		}

		// Sizes are different, can't be equal.
		if (size() != rhsSize) {
			return false;
		}

		const std::basic_string_view<value_type> rhsView{rhs, rhsSize};
		return std::equal(cbegin(), cend(), rhsView.cbegin(), rhsView.cend());
	}

	auto operator<=>(const_pointer rhs) const noexcept {
		const size_t rhsSize = (rhs == nullptr) ? (0) : (strlen(rhs));

		// Check zero size first: only if both have size zero
		// are they equal, else it depends on who has a size.
		// This is both an optimization and a safety check to
		// avoid weird edge cases in the comparison (esp. for
		// std::string_view).
		if (empty() || (rhsSize == 0)) {
			if (empty() && (rhsSize != 0)) {
				// this is zero, LHS has a size. this is less.
				return std::strong_ordering::less;
			}
			else if (!empty() && (rhsSize == 0)) {
				// this has a size, LHS is zero. this is greater.
				return std::strong_ordering::greater;
			}
			else {
				// Both have zero length. Both are equal.
				return std::strong_ordering::equal;
			}
		}

		const std::basic_string_view<value_type> rhsView{rhs, rhsSize};
		return std::lexicographical_compare_three_way(cbegin(), cend(), rhsView.cbegin(), rhsView.cend());
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 } bool
	operator==(const U &rhs) const noexcept {
		// Check zero size first: if both have size zero they're
		// equal, if only one of them has a size of zero and the
		// other doesn't they must be different. This is both an
		// optimization and a safety check to avoid weird edge
		// cases in the comparison (esp. for std::string_view).
		if (empty() || (rhs.size() == 0)) {
			return (empty() && (rhs.size() == 0)) ? (true) : (false);
		}

		// Sizes are different, can't be equal.
		if (size() != rhs.size()) {
			return false;
		}

		const std::basic_string_view<value_type> rhsView{rhs.data(), rhs.size()};
		return std::equal(cbegin(), cend(), rhsView.cbegin(), rhsView.cend());
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	auto operator<=>(const U &rhs) const noexcept {
		// Check zero size first: only if both have size zero
		// are they equal, else it depends on who has a size.
		// This is both an optimization and a safety check to
		// avoid weird edge cases in the comparison (esp. for
		// std::string_view).
		if (empty() || (rhs.size() == 0)) {
			if (empty() && (rhs.size() != 0)) {
				// this is zero, LHS has a size. this is less.
				return std::strong_ordering::less;
			}
			else if (!empty() && (rhs.size() == 0)) {
				// this has a size, LHS is zero. this is greater.
				return std::strong_ordering::greater;
			}
			else {
				// Both have zero length. Both are equal.
				return std::strong_ordering::equal;
			}
		}

		const std::basic_string_view<value_type> rhsView{rhs.data(), rhs.size()};
		return std::lexicographical_compare_three_way(cbegin(), cend(), rhsView.cbegin(), rhsView.cend());
	}

	basic_cstring &assign(basic_cstring &&str) {
		dv::runtime_assert(this != &str, "cannot move-assign into self");

		// Moved-from object must remain in a valid state. We can define
		// valid-state-after-move to be nothing allowed but a destructor
		// call, which is what normally happens, and helps us a lot here.

		// Destroy current data.
		mCurrSize = 0;
		reallocateMemory(mCurrSize);

		// Move data here.
		mCurrSize    = str.mCurrSize;
		mMaximumSize = str.mMaximumSize;
		mDataPtr     = str.mDataPtr;

		// Reset to empty string (ready for destruction or reuse).
		str.mCurrSize    = 0;
		str.mMaximumSize = 0;
		str.mDataPtr     = &NULL_CHAR;

		return *this;
	}

	basic_cstring &assign(const basic_cstring &str, const size_type pos = 0, const size_type count = npos) {
		// If operation would have no effect, do nothing.
		if ((this == &str) && (pos == 0) && (count >= str.length())) {
			return *this;
		}

		return assign(str.data(), str.length(), pos, count);
	}

	basic_cstring &assign(const_pointer str) {
		return assign(str, (str == nullptr) ? (0) : (strlen(str)));
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	basic_cstring &assign(const U &str, const size_type pos = 0, const size_type count = npos) {
		return assign(str.data(), str.size(), pos, count);
	}

	// Lowest common denominator: a ptr and sizes. Most assignments call this.
	basic_cstring &assign(
		const_pointer str, const size_type strLength, const size_type pos = 0, const size_type count = npos) {
		if (strLength == 0) {
			mCurrSize = 0;
			nullTerminate();

			return *this;
		}

		if (str == nullptr) {
			throw std::invalid_argument("string resolves to nullptr.");
		}

		if (pos > strLength) {
			throw std::length_error("position bigger than string length.");
		}

		// Ensure number of characters to copy is within range.
		const auto toCopyCount = (count > (strLength - pos)) ? (strLength - pos) : (count);
		if (toCopyCount == 0) {
			mCurrSize = 0;
			nullTerminate();

			return *this;
		}

		ensureCapacity(toCopyCount);
		mCurrSize = toCopyCount;

		std::copy_n(const_iterator(str + pos), toCopyCount, begin());

		nullTerminate();

		return *this;
	}

	// Replace string with the given character.
	basic_cstring &assign(const value_type value) {
		return assign(static_cast<size_type>(1), value);
	}

	// Replace string with N times the given character.
	basic_cstring &assign(const size_type count, const value_type value) {
		if (count == 0) {
			mCurrSize = 0;
			nullTerminate();

			return *this;
		}

		ensureCapacity(count);
		mCurrSize = count;

		// Initialize elements to copy of value.
		std::fill_n(begin(), count, value);

		nullTerminate();

		return *this;
	}

	// Replace string with characters from range.
	template<typename InputIt,
		std::enable_if_t<
			std::is_base_of_v<std::input_iterator_tag, typename std::iterator_traits<InputIt>::iterator_category>, bool>
		= true>
	basic_cstring &assign(InputIt first, InputIt last) {
		const auto difference = std::distance(first, last);
		if (difference < 0) {
			throw std::invalid_argument("Inverted iterators (last < first). This is never what you really want.");
		}

		const auto count = static_cast<size_type>(difference);
		if (count == 0) {
			mCurrSize = 0;
			nullTerminate();

			return *this;
		}

		ensureCapacity(count);
		mCurrSize = count;

		// Initialize elements to copy of range's values.
		std::copy_n(first, count, begin());

		nullTerminate();

		return *this;
	}

	// Replace string via initializer list {x, y, z}.
	basic_cstring &assign(std::initializer_list<value_type> init_list) {
		return assign(init_list.begin(), init_list.end());
	}

	[[nodiscard]] pointer data() noexcept {
		return mDataPtr;
	}

	[[nodiscard]] const_pointer data() const noexcept {
		return mDataPtr;
	}

	[[nodiscard]] const_pointer c_str() const noexcept {
		return data();
	}

	[[nodiscard]] size_type size() const noexcept {
		return mCurrSize;
	}

	[[nodiscard]] size_type length() const noexcept {
		return size();
	}

	[[nodiscard]] size_type capacity() const noexcept {
		return mMaximumSize;
	}

	[[nodiscard]] size_type max_size() const noexcept {
		// -1 to account for NULL termination.
		return static_cast<size_type>((std::numeric_limits<difference_type>::max() / sizeof(value_type)) - 1);
	}

	[[nodiscard]] bool empty() const noexcept {
		return (mCurrSize == 0);
	}

	void resize(const size_type newSize) {
		resize(newSize, value_type{});
	}

	void resize(const size_type newSize, const value_type value) {
		if (newSize == mCurrSize) {
			return;
		}

		ensureCapacity(newSize);

		if (newSize > mCurrSize) {
			// Add new characters on expansion.
			std::fill_n(end(), (newSize - mCurrSize), value);
		}

		mCurrSize = newSize;
		nullTerminate();
	}

	void reserve(const size_type minCapacity) {
		ensureCapacity(minCapacity);
	}

	void shrink_to_fit() {
		if (mCurrSize == mMaximumSize) {
			return; // Already smallest possible size.
		}

		// Capacity is bigger than size, so we shrink the allocation.
		reallocateMemory(mCurrSize);
	}

	template<typename INT>
	[[nodiscard]] reference operator[](const INT index) {
		return at(index);
	}

	template<typename INT>
	[[nodiscard]] const_reference operator[](const INT index) const {
		return at(index);
	}

	template<typename INT>
	[[nodiscard]] reference at(const INT index) {
		static_assert(std::is_integral_v<INT>, "CString subscript operator index must be an integer.");

		if constexpr (std::is_unsigned_v<INT>) {
			return mDataPtr[getIndex(static_cast<size_type>(index))];
		}
		else {
			return mDataPtr[getIndex(static_cast<difference_type>(index))];
		}
	}

	template<typename INT>
	[[nodiscard]] const_reference at(const INT index) const {
		static_assert(std::is_integral_v<INT>, "CString subscript operator index must be an integer.");

		if constexpr (std::is_unsigned_v<INT>) {
			return mDataPtr[getIndex(static_cast<size_type>(index))];
		}
		else {
			return mDataPtr[getIndex(static_cast<difference_type>(index))];
		}
	}

	[[nodiscard]] operator std::basic_string_view<value_type>() const {
		return std::basic_string_view<value_type>{c_str(), length()};
	}

	[[nodiscard]] explicit operator std::basic_string<value_type>() const {
		return std::basic_string<value_type>{c_str(), length()};
	}

	[[nodiscard]] reference front() {
		return at(0);
	}

	[[nodiscard]] const_reference front() const {
		return at(0);
	}

	[[nodiscard]] reference back() {
		return at(-1);
	}

	[[nodiscard]] const_reference back() const {
		return at(-1);
	}

	void push_back(const value_type value) {
		ensureCapacity(mCurrSize + 1);

		mDataPtr[mCurrSize] = value;

		mCurrSize++;
		nullTerminate();
	}

	void pop_back() {
		if (empty()) {
			throw std::out_of_range("string is empty.");
		}

		mCurrSize--;
		nullTerminate();
	}

	void clear() noexcept {
		mCurrSize = 0;
		nullTerminate();
	}

	void swap(basic_cstring &rhs) noexcept {
		std::swap(mCurrSize, rhs.mCurrSize);
		std::swap(mMaximumSize, rhs.mMaximumSize);
		std::swap(mDataPtr, rhs.mDataPtr);
	}

	// Iterator support.
	using iterator               = cPtrIterator<value_type>;
	using const_iterator         = cPtrIterator<const_value_type>;
	using reverse_iterator       = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	[[nodiscard]] iterator begin() noexcept {
		// Empty container is a special case. front/back would throw.
		if (empty()) {
			return iterator(mDataPtr);
		}

		return iterator(&front());
	}

	[[nodiscard]] iterator end() noexcept {
		// Empty container is a special case. front/back would throw.
		if (empty()) {
			return iterator(mDataPtr);
		}

		// Pointer must be to element one past the end!
		return iterator((&back()) + 1);
	}

	[[nodiscard]] const_iterator begin() const noexcept {
		return cbegin();
	}

	[[nodiscard]] const_iterator end() const noexcept {
		return cend();
	}

	[[nodiscard]] const_iterator cbegin() const noexcept {
		// Empty container is a special case. front/back would throw.
		if (empty()) {
			return const_iterator(mDataPtr);
		}

		return const_iterator(&front());
	}

	[[nodiscard]] const_iterator cend() const noexcept {
		// Empty container is a special case. front/back would throw.
		if (empty()) {
			return const_iterator(mDataPtr);
		}

		// Pointer must be to element one past the end!
		return const_iterator((&back()) + 1);
	}

	[[nodiscard]] reverse_iterator rbegin() noexcept {
		return reverse_iterator(end());
	}

	[[nodiscard]] reverse_iterator rend() noexcept {
		return reverse_iterator(begin());
	}

	[[nodiscard]] const_reverse_iterator rbegin() const noexcept {
		return crbegin();
	}

	[[nodiscard]] const_reverse_iterator rend() const noexcept {
		return crend();
	}

	[[nodiscard]] const_reverse_iterator crbegin() const noexcept {
		return const_reverse_iterator(cend());
	}

	[[nodiscard]] const_reverse_iterator crend() const noexcept {
		return const_reverse_iterator(cbegin());
	}

	iterator insert(const_iterator pos, const value_type value) {
		return insert(pos, static_cast<size_type>(1), value);
	}

	iterator insert(const_iterator pos, const size_type count, const value_type value) {
		const auto idx = static_cast<size_type>(std::distance(cbegin(), pos));

		if (count == 0) {
			return (begin() + idx);
		}

		// Careful: ensureCapacity() can invalidate iterators!
		// That's why we get the index first and regenerate pos.
		ensureCapacity(mCurrSize + count);
		auto wrPos = begin() + idx;

		// Move by N to make space.
		std::move_backward(wrPos, end(), end() + count);

		// Copy construct new elements at insertion position.
		std::fill_n(wrPos, count, value);

		mCurrSize += count;
		nullTerminate();

		return wrPos;
	}

	template<typename InputIt,
		std::enable_if_t<
			std::is_base_of_v<std::input_iterator_tag, typename std::iterator_traits<InputIt>::iterator_category>, bool>
		= true>
	iterator insert(const_iterator pos, InputIt first, InputIt last) {
		const auto difference = std::distance(first, last);
		if (difference < 0) {
			throw std::invalid_argument("Inverted iterators (last < first). This is never what you really want.");
		}

		const auto count = static_cast<size_type>(difference);
		const auto idx   = static_cast<size_type>(std::distance(cbegin(), pos));

		if (count == 0) {
			return (begin() + idx);
		}

		// Careful: ensureCapacity() can invalidate iterators!
		// That's why we get the index first and regenerate pos.
		ensureCapacity(mCurrSize + count);
		auto wrPos = begin() + idx;

		// Move by N to make space.
		std::move_backward(wrPos, end(), end() + count);

		// Copy construct new elements at insertion position from external range.
		std::copy_n(first, count, wrPos);

		mCurrSize += count;
		nullTerminate();

		return wrPos;
	}

	iterator insert(const_iterator pos, std::initializer_list<value_type> init_list) {
		return insert(pos, init_list.begin(), init_list.end());
	}

	iterator erase(const_iterator pos) {
		// If position is end or string empty, nothing to do.
		if ((pos == cend()) || empty()) {
			return end();
		}

		const auto idx = static_cast<size_type>(std::distance(cbegin(), pos));
		auto wrPos     = begin() + idx;

		// Move elements over, this will move assign into the
		// to be erased element, effectively erasing it.
		std::move(wrPos + 1, end(), wrPos);

		// Destroy object at end, this was moved from and is
		// now waiting on destruction.
		mCurrSize--;
		nullTerminate();

		return wrPos;
	}

	iterator erase(const_iterator first, const_iterator last) {
		const auto difference = std::distance(first, last);
		if (difference < 0) {
			throw std::invalid_argument("Inverted iterators (last < first). This is never what you really want.");
		}

		// If start position is end or string empty, nothing to do.
		if ((first == cend()) || empty()) {
			return end();
		}

		const auto count = static_cast<size_type>(difference);
		const auto idx   = static_cast<size_type>(std::distance(cbegin(), first));
		auto wrFirst     = begin() + idx;

		if (count == 0) {
			return wrFirst;
		}

		// Move elements over, this will move assign into the
		// to be erased element, effectively erasing it.
		std::move(wrFirst + count, end(), wrFirst);

		// Destroy objects at end, they were moved from and are
		// now waiting on destruction.
		mCurrSize -= count;
		nullTerminate();

		return wrFirst;
	}

	[[nodiscard]] constexpr size_type find(const basic_cstring &str, size_type pos = 0) const noexcept {
		return std::basic_string_view<value_type>{*this}.find(str, pos);
	}

	[[nodiscard]] constexpr size_type find(value_type c, size_type pos = 0) const noexcept {
		return std::basic_string_view<value_type>{*this}.find(c, pos);
	}

	[[nodiscard]] constexpr size_type find(const_pointer s, size_type pos, size_type count) const {
		return std::basic_string_view<value_type>{*this}.find(s, pos, count);
	}

	[[nodiscard]] constexpr size_type find(const_pointer s, size_type pos = 0) const {
		return std::basic_string_view<value_type>{*this}.find(s, pos);
	}

	[[nodiscard]] constexpr size_type rfind(const basic_cstring &str, size_type pos = npos) const noexcept {
		return std::basic_string_view<value_type>{*this}.rfind(str, pos);
	}

	[[nodiscard]] constexpr size_type rfind(value_type c, size_type pos = npos) const noexcept {
		return std::basic_string_view<value_type>{*this}.rfind(c, pos);
	}

	[[nodiscard]] constexpr size_type rfind(const_pointer s, size_type pos, size_type count) const {
		return std::basic_string_view<value_type>{*this}.rfind(s, pos, count);
	}

	[[nodiscard]] constexpr size_type rfind(const_pointer s, size_type pos = npos) const {
		return std::basic_string_view<value_type>{*this}.rfind(s, pos);
	}

	// TODO: iterator insert(const_iterator pos, const STRINGS t, const size_type index_str = 0, const
	// size_type count = npos)
	// TODO: basic_cstring& insert(const size_type index, const value_type value)
	// TODO: basic_cstring& insert(const size_type index, const size_type count, const value_type value)
	// TODO: basic_cstring& insert(const size_type index, const STRINGS t, const size_type index_str = 0, const
	// size_type count = npos)
	// TODO: template<class InputIt> basic_cstring& insert(const size_type index, InputIt first, InputIt last)
	// TODO: basic_cstring& insert(const size_type index, std::initializer_list<value_type> init_list)
	// TODO: basic_cstring& erase(const size_type index = 0, const size_type count = npos)
	// TODO: void remove_front(const size_type count = 1) -> alias remove_prefix()
	// TODO: void remove_back(const size_type count = 1) -> alias remove_suffix()
	// TODO: bool starts_with(const STRINGS t) const
	// TODO: bool starts_with(const value_type value) const
	// TODO: bool ends_with(const STRINGS t) const
	// TODO: bool ends_with(const value_type value) const
	// TODO: bool contains(const STRINGS t) const
	// TODO: bool contains(const value_type value) const
	// TODO: size_type remove(const STRINGS t)
	// TODO: size_type remove(const value_type value)
	// TODO: basic_cstring substr(const size_type pos = 0, const size_type count = npos) const -> alias slice()
	// TODO: template<class InputIt> basic_cstring substr(InputIt first, InputIt last) const -> alias slice()
	// TODO: std::string_view substrView(const size_type pos = 0, const size_type count = npos) const -> alias
	// sliceView()
	// TODO: template<class InputIt> std::string_view substrView(InputIt first, InputIt last) const -> alias sliceView()
	// TODO: find_first_of, find_first_not_of, find_last_of, find_last_not_of (all const)

	basic_cstring &append(const basic_cstring &str, const size_type pos = 0, const size_type count = npos) {
		return append(str.data(), str.length(), pos, count);
	}

	basic_cstring &append(const_pointer str) {
		return append(str, (str == nullptr) ? (0) : (strlen(str)));
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	basic_cstring &append(const U &str, const size_type pos = 0, const size_type count = npos) {
		return append(str.data(), str.size(), pos, count);
	}

	// Lowest common denominator: a ptr and sizes.
	basic_cstring &append(
		const_pointer str, const size_type strLength, const size_type pos = 0, const size_type count = npos) {
		if (strLength == 0) {
			return *this;
		}

		if (str == nullptr) {
			throw std::invalid_argument("string resolves to nullptr.");
		}

		if (pos > strLength) {
			throw std::length_error("position bigger than string length.");
		}

		// Ensure number of characters to copy is within range.
		const auto toCopyCount = (count > (strLength - pos)) ? (strLength - pos) : (count);
		if (toCopyCount == 0) {
			return *this;
		}

		ensureCapacity(mCurrSize + toCopyCount);

		std::copy_n(const_iterator(str + pos), toCopyCount, end());

		mCurrSize += toCopyCount;
		nullTerminate();

		return *this;
	}

	// Enlarge string with the given character.
	basic_cstring &append(const value_type value) {
		return append(static_cast<size_type>(1), value);
	}

	// Enlarge string with N times the given character.
	basic_cstring &append(const size_type count, const value_type value) {
		if (count == 0) {
			return *this;
		}

		ensureCapacity(mCurrSize + count);

		// Initialize elements to copy of value.
		std::fill_n(end(), count, value);

		mCurrSize += count;
		nullTerminate();

		return *this;
	}

	// Enlarge string with characters from range.
	template<typename InputIt,
		std::enable_if_t<
			std::is_base_of_v<std::input_iterator_tag, typename std::iterator_traits<InputIt>::iterator_category>, bool>
		= true>
	basic_cstring &append(InputIt first, InputIt last) {
		const auto difference = std::distance(first, last);
		if (difference < 0) {
			throw std::invalid_argument("Inverted iterators (last < first). This is never what you really want.");
		}

		const auto count = static_cast<size_type>(difference);
		if (count == 0) {
			return *this;
		}

		ensureCapacity(mCurrSize + count);

		// Initialize elements to copy of range's values.
		std::copy_n(first, count, end());

		mCurrSize += count;
		nullTerminate();

		return *this;
	}

	// Enlarge string via initializer list {x, y, z}.
	basic_cstring &append(std::initializer_list<value_type> init_list) {
		return append(init_list.begin(), init_list.end());
	}

	basic_cstring &operator+=(const basic_cstring &rhs) {
		return append(rhs);
	}

	basic_cstring &operator+=(const_pointer str) {
		return append(str);
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	basic_cstring &operator+=(const U &str) {
		return append(str.data(), str.size());
	}

	basic_cstring &operator+=(const value_type value) {
		return append(value);
	}

	basic_cstring &operator+=(std::initializer_list<value_type> rhs_list) {
		return append(rhs_list);
	}

	basic_cstring operator+(const basic_cstring &rhs) const {
		basic_cstring sum;
		sum.reserve(size() + rhs.size());

		sum.assign(*this);
		sum.append(rhs);

		return sum;
	}

	basic_cstring operator+(const_pointer rhs) const {
		const size_t rhsSize = (rhs == nullptr) ? (0) : (strlen(rhs));

		basic_cstring sum;
		sum.reserve(size() + rhsSize);

		sum.assign(*this);
		sum.append(rhs, rhsSize);

		return sum;
	}

	friend basic_cstring operator+(const_pointer lhs, const basic_cstring &rhs) {
		const size_t lhsSize = (lhs == nullptr) ? (0) : (strlen(lhs));

		basic_cstring sum;
		sum.reserve(lhsSize + rhs.size());

		sum.assign(lhs, lhsSize);
		sum.append(rhs);

		return sum;
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	basic_cstring operator+(const U &rhs) const {
		basic_cstring sum;
		sum.reserve(size() + rhs.size());

		sum.assign(*this);
		sum.append(rhs.data(), rhs.size());

		return sum;
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	friend basic_cstring operator+(const U &lhs, const basic_cstring &rhs) {
		basic_cstring sum;
		sum.reserve(lhs.size() + rhs.size());

		sum.assign(lhs.data(), lhs.size());
		sum.append(rhs);

		return sum;
	}

	basic_cstring operator+(const value_type value) const {
		basic_cstring sum;
		sum.reserve(size() + 1);

		sum.assign(*this);
		sum.append(value);

		return sum;
	}

	friend basic_cstring operator+(const value_type value, const basic_cstring &rhs) {
		basic_cstring sum;
		sum.reserve(1 + rhs.size());

		sum.assign(value);
		sum.append(rhs);

		return sum;
	}

	basic_cstring operator+(std::initializer_list<value_type> rhs_list) const {
		basic_cstring sum;
		sum.reserve(size() + rhs_list.size());

		sum.assign(*this);
		sum.append(rhs_list);

		return sum;
	}

	friend basic_cstring operator+(std::initializer_list<value_type> lhs_list, const basic_cstring &rhs) {
		basic_cstring sum;
		sum.reserve(lhs_list.size() + rhs.size());

		sum.assign(lhs_list);
		sum.append(rhs);

		return sum;
	}

	friend std::ostream &operator<<(std::ostream &os, const basic_cstring &rhs) {
		os << std::string_view{rhs};
		return os;
	}

private:
	void nullTerminate() {
		// Only terminate if memory is allocated, else mDataPtr will
		// already point to the static zero termination char.
		if (mMaximumSize > 0) {
			mDataPtr[mCurrSize] = 0;
		}
	}

	void ensureCapacity(const size_type newSize) {
		// Do we have enough space left?
		if (newSize <= mMaximumSize) {
			return; // Yes.
		}

		// No, we must grow.
		// Let's use factor 1.5 for growing, for rationale see:
		// https://github.com/facebook/folly/blob/main/folly/docs/FBVector.md
		// Also let's make sure to not have weird micro-allocations,
		// so smallest size we request is 32 bytes, which due
		// to extra NUL character means 31 chars (+1 NUL byte).
		auto newCapacity = static_cast<size_type>(static_cast<float>(mMaximumSize) * 1.5f);
		if (newCapacity < (32 - 1)) {
			newCapacity = (32 - 1);
		}

		reallocateMemory(((newCapacity >= newSize) && (newCapacity <= max_size())) ? (newCapacity) : (newSize));
	}

	void reallocateMemory(const size_type newSize) {
		if (newSize > max_size()) {
			throw std::length_error("requested size exceeds max_size() limit.");
		}

		// If wanted size is zero (shrink_to_fit() on empty string for example),
		// reset to the empty string using the static zero termination char.
		if (newSize == 0) {
			if (mMaximumSize > 0) {
				free(mDataPtr);
				mDataPtr     = &NULL_CHAR;
				mMaximumSize = 0;
			}

			return;
		}

		// Always allocate one byte more for NULL termination (C compatibility).
		// Only use mDataPtr is memory was previously allocated, else do a completely
		// new allocation using nullptr, since mDataPtr must have been pointing
		// to the static zero termination char.
		// Character types are always trivial, so we can just use realloc().
		auto *new_data_ptr = static_cast<pointer>(
			realloc((mMaximumSize > 0) ? (mDataPtr) : (nullptr), ((newSize + 1) * sizeof(value_type))));
		if (new_data_ptr == nullptr) {
			// Failed to allocate memory.
			throw std::bad_alloc();
		}

		// Succeeded, update ptr + capacity.
		mDataPtr     = new_data_ptr;
		mMaximumSize = newSize;
	}

	[[nodiscard]] size_type getIndex(const size_type index) const {
		if (index >= mCurrSize) {
			throw std::out_of_range("Index out of range.");
		}

		return index;
	}

	[[nodiscard]] size_type getIndex(const difference_type index) const {
		// Support negative indexes to go from the last existing/defined element
		// backwards (not from the capacity!).
		size_type realIndex;

		if (index < 0) {
			const difference_type negIndex = static_cast<difference_type>(mCurrSize) + index;

			if (negIndex < 0) {
				throw std::out_of_range("Index out of range.");
			}

			realIndex = static_cast<size_type>(negIndex);
		}
		else {
			realIndex = static_cast<size_type>(index);
		}

		return getIndex(realIndex);
	}
};

using cstring    = basic_cstring<char>;
using cwstring   = basic_cstring<wchar_t>;
using cu8string  = basic_cstring<char8_t>;
using cu16string = basic_cstring<char16_t>;
using cu32string = basic_cstring<char32_t>;

} // namespace dv

static_assert(std::is_standard_layout_v<dv::cstring>, "cstring is not standard layout");
static_assert(std::is_standard_layout_v<dv::cwstring>, "cwstring is not standard layout");
static_assert(std::is_standard_layout_v<dv::cu8string>, "cu8string is not standard layout");
static_assert(std::is_standard_layout_v<dv::cu16string>, "cu16string is not standard layout");
static_assert(std::is_standard_layout_v<dv::cu32string>, "cu32string is not standard layout");

static_assert(sizeof(dv::cstring) == (3 * sizeof(size_t)), "cstring size is unexpected");
static_assert(sizeof(dv::cwstring) == (3 * sizeof(size_t)), "cwstring size is unexpected");
static_assert(sizeof(dv::cu8string) == (3 * sizeof(size_t)), "cu8string size is unexpected");
static_assert(sizeof(dv::cu16string) == (3 * sizeof(size_t)), "cu16string size is unexpected");
static_assert(sizeof(dv::cu32string) == (3 * sizeof(size_t)), "cu32string size is unexpected");

// fmt formatting compatibility.
namespace fmt {

template<typename T>
struct formatter<dv::basic_cstring<T>> : fmt::formatter<std::basic_string_view<T>> {
	// parse is inherited from fmt::formatter<std::basic_string_view<T>>.
	template<typename FormatContext>
	auto format(const dv::basic_cstring<T> &str, FormatContext &ctx) DV_EXT_FMT_CONST {
		return fmt::formatter<std::basic_string_view<T>>::format(std::basic_string_view<T>{str}, ctx);
	}
};

} // namespace fmt
