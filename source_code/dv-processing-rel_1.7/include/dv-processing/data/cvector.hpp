#pragma once

#include "../external/compare_compat.hpp"
#include "../external/fmt_compat.hpp"

#include "../core/dvassert.hpp"
#include "cptriterator.hpp"

#include <array>
#include <concepts>
#include <limits>
#include <span>
#include <stdexcept>
#include <string_view>

namespace dv {

namespace concepts {

namespace internal {

template<class T>
concept OutputStreamable = requires(std::ostream &os, T value) {
							   { os << value } -> std::convertible_to<std::ostream &>;
						   };

} // namespace internal

} // namespace concepts

template<class T>
class cvector {
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

	static_assert(std::is_standard_layout_v<value_type>, "cvector type is not standard layout");

private:
	size_type mCurrSize{0};
	size_type mMaximumSize{0};
	pointer mDataPtr{nullptr};

	static_assert(sizeof(size_type) == sizeof(pointer), "size_type and pointer size are not coherent");
	static_assert(alignof(size_type) == sizeof(size_type), "size_type alignment not coherent");
	static_assert(alignof(pointer) == sizeof(pointer), "pointer alignment not coherent");

public:
	// Default constructor. Initialize empty vector with no memory.
	constexpr cvector() noexcept = default;

	// Destructor.
	~cvector() noexcept {
		// Destroy all elements.
		std::destroy_n(begin(), mCurrSize);
		mCurrSize = 0;
		reallocateMemory(mCurrSize);
	}

	// Copy constructor.
	cvector(const cvector &vec, const size_type pos = 0, const size_type count = npos) :
		cvector(vec.data(), vec.size(), pos, count) {
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	cvector(const U &vec, const size_type pos = 0, const size_type count = npos) :
		cvector(vec.data(), vec.size(), pos, count) {
	}

	// Lowest common denominator: a ptr and sizes. Most constructors call this.
	cvector(const_pointer vec, const size_type vecLength, const size_type pos = 0, const size_type count = npos) {
		if (vecLength == 0) {
			return;
		}

		if (vec == nullptr) {
			throw std::invalid_argument("vector resolves to nullptr.");
		}

		if (pos > vecLength) {
			throw std::length_error("position bigger than vector length.");
		}

		// Ensure number of elements to copy is within range.
		const auto toCopyCount = (count > (vecLength - pos)) ? (vecLength - pos) : (count);
		if (toCopyCount == 0) {
			return;
		}

		ensureCapacity(toCopyCount);
		mCurrSize = toCopyCount;

		std::uninitialized_copy_n(const_iterator(vec + pos), toCopyCount, begin());
	}

	// Initialize vector with N value constructed elements.
	explicit cvector(const size_type count) {
		if (count == 0) {
			return;
		}

		ensureCapacity(count);
		mCurrSize = count;

		// Value initialize elements.
		std::uninitialized_value_construct_n(begin(), count);
	}

	// Initialize vector with N copies of given value.
	cvector(const size_type count, const_reference value) {
		if (count == 0) {
			return;
		}

		ensureCapacity(count);
		mCurrSize = count;

		// Initialize elements to copy of value.
		std::uninitialized_fill_n(begin(), count, value);
	}

	// Initialize vector with elements from range.
	template<typename InputIt,
		std::enable_if_t<
			std::is_base_of_v<std::input_iterator_tag, typename std::iterator_traits<InputIt>::iterator_category>, bool>
		= true>
	cvector(InputIt first, InputIt last) {
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
		std::uninitialized_copy_n(first, count, begin());
	}

	// Initialize vector via initializer list {x, y, z}.
	cvector(std::initializer_list<value_type> init_list) : cvector(init_list.begin(), init_list.end()) {
	}

	// Move constructor.
	cvector(cvector &&rhs) noexcept {
		// Moved-from object must remain in a valid state. We can define
		// valid-state-after-move to be nothing allowed but a destructor
		// call, which is what normally happens, and helps us a lot here.

		// Move data here.
		mCurrSize    = rhs.mCurrSize;
		mMaximumSize = rhs.mMaximumSize;
		mDataPtr     = rhs.mDataPtr;

		// Reset old data (ready for destruction).
		rhs.mCurrSize    = 0;
		rhs.mMaximumSize = 0;
		rhs.mDataPtr     = nullptr;
	}

	// Move assignment.
	cvector &operator=(cvector &&rhs) noexcept {
		return (assign(std::move(rhs)));
	}

	// Copy assignment.
	cvector &operator=(const cvector &rhs) {
		return (assign(rhs));
	}

	// Extra assignment operators.
	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	cvector &operator=(const U &rhs) {
		return (assign(rhs.data(), rhs.size()));
	}

	cvector &operator=(const_reference value) {
		return (assign(value));
	}

	cvector &operator=(std::initializer_list<value_type> rhs_list) {
		return (assign(rhs_list));
	}

	// Comparison operators.
	bool operator==(const cvector &rhs) const noexcept {
		// Check zero size first: if both have size zero they're
		// equal, if only one of them has a size of zero and the
		// other doesn't they must be different. This is both an
		// optimization and a safety check to avoid weird edge
		// cases in the comparison.
		if (empty() || rhs.empty()) {
			return (empty() && rhs.empty()) ? (true) : (false);
		}

		// Sizes are different, can't be equal.
		if (size() != rhs.size()) {
			return false;
		}

		return std::equal(cbegin(), cend(), rhs.cbegin(), rhs.cend());
	}

	auto operator<=>(const cvector &rhs) const noexcept {
		// Check zero size first: only if both have size zero
		// are they equal, else it depends on who has a size.
		// This is both an optimization and a safety check to
		// avoid weird edge cases in the comparison.
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
		// cases in the comparison.
		if (empty() || (rhs.size() == 0)) {
			return (empty() && (rhs.size() == 0)) ? (true) : (false);
		}

		// Sizes are different, can't be equal.
		if (size() != rhs.size()) {
			return false;
		}

		const std::span<value_type> rhsView{rhs.data(), rhs.size()};
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
		// avoid weird edge cases in the comparison.
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

		const std::span<value_type> rhsView{rhs.data(), rhs.size()};
		return std::lexicographical_compare_three_way(cbegin(), cend(), rhsView.cbegin(), rhsView.cend());
	}

	cvector &assign(cvector &&vec) {
		dv::runtime_assert(this != &vec, "cannot move-assign into self");

		// Moved-from object must remain in a valid state. We can define
		// valid-state-after-move to be nothing allowed but a destructor
		// call, which is what normally happens, and helps us a lot here.

		// Destroy current data.
		std::destroy_n(begin(), mCurrSize);
		mCurrSize = 0;
		reallocateMemory(mCurrSize);

		// Move data here.
		mCurrSize    = vec.mCurrSize;
		mMaximumSize = vec.mMaximumSize;
		mDataPtr     = vec.mDataPtr;

		// Reset old data (ready for destruction).
		vec.mCurrSize    = 0;
		vec.mMaximumSize = 0;
		vec.mDataPtr     = nullptr;

		return *this;
	}

	cvector &assign(const cvector &vec, const size_type pos = 0, const size_type count = npos) {
		// If operation would have no effect, do nothing.
		if ((this == &vec) && (pos == 0) && (count >= vec.size())) {
			return *this;
		}

		return assign(vec.data(), vec.size(), pos, count);
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	cvector &assign(const U &vec, const size_type pos = 0, const size_type count = npos) {
		return assign(vec.data(), vec.size(), pos, count);
	}

	// Lowest common denominator: a ptr and sizes. Most assignments call this.
	cvector &assign(
		const_pointer vec, const size_type vecLength, const size_type pos = 0, const size_type count = npos) {
		if (vecLength == 0) {
			std::destroy_n(begin(), mCurrSize);
			mCurrSize = 0;

			return *this;
		}

		if (vec == nullptr) {
			throw std::invalid_argument("vector resolves to nullptr.");
		}

		if (pos > vecLength) {
			throw std::length_error("position bigger than vector length.");
		}

		// Ensure number of elements to copy is within range.
		const auto toCopyCount = (count > (vecLength - pos)) ? (vecLength - pos) : (count);
		if (toCopyCount == 0) {
			std::destroy_n(begin(), mCurrSize);
			mCurrSize = 0;

			return *this;
		}

		ensureCapacity(toCopyCount);

		std::destroy_n(begin(), mCurrSize);
		mCurrSize = toCopyCount;

		std::uninitialized_copy_n(const_iterator(vec + pos), toCopyCount, begin());

		return *this;
	}

	// Replace vector with one value.
	cvector &assign(const_reference value) {
		return assign(static_cast<size_type>(1), value);
	}

	// Replace vector with N copies of given value.
	cvector &assign(const size_type count, const_reference value) {
		if (count == 0) {
			std::destroy_n(begin(), mCurrSize);
			mCurrSize = 0;

			return *this;
		}

		ensureCapacity(count);

		std::destroy_n(begin(), mCurrSize);
		mCurrSize = count;

		// Initialize elements to copy of value.
		std::uninitialized_fill_n(begin(), count, value);

		return *this;
	}

	// Replace vector with elements from range.
	template<typename InputIt,
		std::enable_if_t<
			std::is_base_of_v<std::input_iterator_tag, typename std::iterator_traits<InputIt>::iterator_category>, bool>
		= true>
	cvector &assign(InputIt first, InputIt last) {
		const auto difference = std::distance(first, last);
		if (difference < 0) {
			throw std::invalid_argument("Inverted iterators (last < first). This is never what you really want.");
		}

		const auto count = static_cast<size_type>(difference);
		if (count == 0) {
			std::destroy_n(begin(), mCurrSize);
			mCurrSize = 0;

			return *this;
		}

		ensureCapacity(count);

		std::destroy_n(begin(), mCurrSize);
		mCurrSize = count;

		// Initialize elements to copy of range's values.
		std::uninitialized_copy_n(first, count, begin());

		return *this;
	}

	// Replace vector via initializer list {x, y, z}.
	cvector &assign(std::initializer_list<value_type> init_list) {
		return assign(init_list.begin(), init_list.end());
	}

	[[nodiscard]] pointer data() noexcept {
		return mDataPtr;
	}

	[[nodiscard]] const_pointer data() const noexcept {
		return mDataPtr;
	}

	[[nodiscard]] size_type size() const noexcept {
		return mCurrSize;
	}

	[[nodiscard]] size_type capacity() const noexcept {
		return mMaximumSize;
	}

	[[nodiscard]] size_type max_size() const noexcept {
		return static_cast<size_type>(std::numeric_limits<difference_type>::max() / sizeof(value_type));
	}

	[[nodiscard]] bool empty() const noexcept {
		return (mCurrSize == 0);
	}

	void resize(const size_type newSize) {
		resize(newSize, value_type{});
	}

	void resize(const size_type newSize, const_reference value) {
		if (newSize == mCurrSize) {
			return;
		}

		ensureCapacity(newSize);

		if (newSize > mCurrSize) {
			// Construct new values on expansion.
			std::uninitialized_fill_n(end(), (newSize - mCurrSize), value);
		}
		else {
			// Destroy on shrinking.
			std::destroy_n(begin() + newSize, (mCurrSize - newSize));
		}

		mCurrSize = newSize;
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
		static_assert(std::is_integral_v<INT>, "CVector subscript operator index must be an integer.");

		if constexpr (std::is_unsigned_v<INT>) {
			return mDataPtr[getIndex<false>(static_cast<size_type>(index))];
		}
		else {
			return mDataPtr[getIndex<false>(static_cast<difference_type>(index))];
		}
	}

	template<typename INT>
	[[nodiscard]] const_reference operator[](const INT index) const {
		static_assert(std::is_integral_v<INT>, "CVector subscript operator index must be an integer.");

		if constexpr (std::is_unsigned_v<INT>) {
			return mDataPtr[getIndex<false>(static_cast<size_type>(index))];
		}
		else {
			return mDataPtr[getIndex<false>(static_cast<difference_type>(index))];
		}
	}

	template<typename INT>
	[[nodiscard]] reference at(const INT index) {
		static_assert(std::is_integral_v<INT>, "CVector subscript operator index must be an integer.");

		if constexpr (std::is_unsigned_v<INT>) {
			return mDataPtr[getIndex<true>(static_cast<size_type>(index))];
		}
		else {
			return mDataPtr[getIndex<true>(static_cast<difference_type>(index))];
		}
	}

	template<typename INT>
	[[nodiscard]] const_reference at(const INT index) const {
		static_assert(std::is_integral_v<INT>, "CVector subscript operator index must be an integer.");

		if constexpr (std::is_unsigned_v<INT>) {
			return mDataPtr[getIndex<true>(static_cast<size_type>(index))];
		}
		else {
			return mDataPtr[getIndex<true>(static_cast<difference_type>(index))];
		}
	}

	[[nodiscard]] explicit operator std::vector<value_type>() const {
		return std::vector<value_type>{cbegin(), cend()};
	}

	[[nodiscard]] reference front() {
		return this->template operator[](static_cast<size_type>(0));
	}

	[[nodiscard]] const_reference front() const {
		return this->template operator[](static_cast<size_type>(0));
	}

	[[nodiscard]] reference back() {
		return this->template operator[](static_cast<difference_type>(-1));
	}

	[[nodiscard]] const_reference back() const {
		return this->template operator[](static_cast<difference_type>(-1));
	}

	void push_back(const_reference value) {
		ensureCapacity(mCurrSize + 1);

		// Call copy constructor.
		new (&mDataPtr[mCurrSize]) value_type{value};

		mCurrSize++;
	}

	void push_back(value_type &&value) {
		ensureCapacity(mCurrSize + 1);

		// Call move constructor.
		new (&mDataPtr[mCurrSize]) value_type{std::move(value)};

		mCurrSize++;
	}

	template<class... Args>
	reference emplace_back(Args &&...args) {
		ensureCapacity(mCurrSize + 1);

		// Call constructor with forwarded arguments.
		new (&mDataPtr[mCurrSize]) value_type(std::forward<Args>(args)...);

		mCurrSize++;

		return back();
	}

	void pop_back() {
		if (empty()) {
			throw std::out_of_range("vector is empty.");
		}

		mCurrSize--;
		std::destroy_n(end(), 1);
	}

	void clear() noexcept {
		std::destroy_n(begin(), mCurrSize);
		mCurrSize = 0;
	}

	void swap(cvector &rhs) noexcept {
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

	iterator insert(const_iterator pos, const_reference value) {
		return insert(pos, static_cast<size_type>(1), value);
	}

	iterator insert(const_iterator pos, value_type &&value) {
		// Careful: ensureCapacity() can invalidate iterators!
		// That's why we get the index first and regenerate pos.
		const auto idx = static_cast<size_type>(std::distance(cbegin(), pos));

		ensureCapacity(mCurrSize + 1);
		auto wrPos = begin() + idx;

		// Default construct so we can move into this.
		std::uninitialized_default_construct_n(end(), 1);

		// Move by one to make space.
		std::move_backward(wrPos, end(), end() + 1);

		// Destroy object at insertion position.
		std::destroy_n(wrPos, 1);

		// Move construct new element at insertion index.
		new (&mDataPtr[idx]) value_type{std::move(value)};

		mCurrSize += 1;

		return wrPos;
	}

	iterator insert(const_iterator pos, const size_type count, const_reference value) {
		const auto idx = static_cast<size_type>(std::distance(cbegin(), pos));

		if (count == 0) {
			return (begin() + idx);
		}

		// Careful: ensureCapacity() can invalidate iterators!
		// That's why we get the index first and regenerate pos.
		ensureCapacity(mCurrSize + count);
		auto wrPos = begin() + idx;

		// Default construct so we can move into this.
		std::uninitialized_default_construct_n(end(), count);

		// Move by N to make space.
		std::move_backward(wrPos, end(), end() + count);

		// Destroy objects at insertion position.
		std::destroy_n(wrPos, count);

		// Copy construct new elements at insertion position.
		std::uninitialized_fill_n(wrPos, count, value);

		mCurrSize += count;

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

		// Default construct so we can move into this.
		std::uninitialized_default_construct_n(end(), count);

		// Move by N to make space.
		std::move_backward(wrPos, end(), end() + count);

		// Destroy objects at insertion position.
		std::destroy_n(wrPos, count);

		// Copy construct new elements at insertion position from external range.
		std::uninitialized_copy_n(first, count, wrPos);

		mCurrSize += count;

		return wrPos;
	}

	iterator insert(const_iterator pos, std::initializer_list<value_type> init_list) {
		return insert(pos, init_list.begin(), init_list.end());
	}

	template<class... Args>
	iterator emplace(const_iterator pos, Args &&...args) {
		// Careful: ensureCapacity() can invalidate iterators!
		// That's why we get the index first and regenerate pos.
		const auto idx = static_cast<size_type>(std::distance(cbegin(), pos));

		ensureCapacity(mCurrSize + 1);
		auto wrPos = begin() + idx;

		// Default construct so we can move into this.
		std::uninitialized_default_construct_n(end(), 1);

		// Move by one to make space.
		std::move_backward(wrPos, end(), end() + 1);

		// Destroy object at insertion position.
		std::destroy_n(wrPos, 1);

		// Move construct new element at insertion index.
		new (&mDataPtr[idx]) value_type(std::forward<Args>(args)...);

		mCurrSize += 1;

		return wrPos;
	}

	iterator erase(const_iterator pos) {
		// If position is end or vector empty, nothing to do.
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
		std::destroy_n(end(), 1);

		return wrPos;
	}

	iterator erase(const_iterator first, const_iterator last) {
		const auto difference = std::distance(first, last);
		if (difference < 0) {
			throw std::invalid_argument("Inverted iterators (last < first). This is never what you really want.");
		}

		// If start position is end or vector empty, nothing to do.
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
		std::destroy_n(end(), count);

		return wrFirst;
	}

	cvector &append(const cvector &vec, const size_type pos = 0, const size_type count = npos) {
		return append(vec.data(), vec.size(), pos, count);
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	cvector &append(const U &vec, const size_type pos = 0, const size_type count = npos) {
		return append(vec.data(), vec.size(), pos, count);
	}

	// Lowest common denominator: a ptr and sizes.
	cvector &append(
		const_pointer vec, const size_type vecLength, const size_type pos = 0, const size_type count = npos) {
		if (vecLength == 0) {
			return *this;
		}

		if (vec == nullptr) {
			throw std::invalid_argument("vector resolves to nullptr.");
		}

		if (pos > vecLength) {
			throw std::length_error("position bigger than vector length.");
		}

		// Ensure number of elements to copy is within range.
		const auto toCopyCount = (count > (vecLength - pos)) ? (vecLength - pos) : (count);
		if (toCopyCount == 0) {
			return *this;
		}

		ensureCapacity(mCurrSize + toCopyCount);

		std::uninitialized_copy_n(const_iterator(vec + pos), toCopyCount, end());

		mCurrSize += toCopyCount;

		return *this;
	}

	// Enlarge vector with one value.
	cvector &append(const_reference value) {
		return append(static_cast<size_type>(1), value);
	}

	// Enlarge vector with N copies of given value.
	cvector &append(const size_type count, const_reference value) {
		if (count == 0) {
			return *this;
		}

		ensureCapacity(mCurrSize + count);

		// Initialize elements to copy of value.
		std::uninitialized_fill_n(end(), count, value);

		mCurrSize += count;

		return *this;
	}

	// Enlarge vector with elements from range.
	template<typename InputIt,
		std::enable_if_t<
			std::is_base_of_v<std::input_iterator_tag, typename std::iterator_traits<InputIt>::iterator_category>, bool>
		= true>
	cvector &append(InputIt first, InputIt last) {
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
		std::uninitialized_copy_n(first, count, end());

		mCurrSize += count;

		return *this;
	}

	// Enlarge vector via initializer list {x, y, z}.
	cvector &append(std::initializer_list<value_type> init_list) {
		return append(init_list.begin(), init_list.end());
	}

	cvector &operator+=(const cvector &rhs) {
		return append(rhs);
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	cvector &operator+=(const U &rhs) {
		return append(rhs.data(), rhs.size());
	}

	cvector &operator+=(const_reference value) {
		return append(value);
	}

	cvector &operator+=(std::initializer_list<value_type> rhs_list) {
		return append(rhs_list);
	}

	cvector operator+(const cvector &rhs) const {
		cvector sum;
		sum.reserve(size() + rhs.size());

		sum.assign(*this);
		sum.append(rhs);

		return sum;
	}

	template<typename U>
	requires requires(U u) {
				 { u.data() } -> std::convertible_to<const_pointer>;
				 { u.size() } -> std::convertible_to<size_type>;
			 }
	cvector operator+(const U &rhs) const {
		cvector sum;
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
	friend cvector operator+(const U &lhs, const cvector &rhs) {
		cvector sum;
		sum.reserve(lhs.size() + rhs.size());

		sum.assign(lhs.data(), lhs.size());
		sum.append(rhs);

		return sum;
	}

	cvector operator+(const_reference value) const {
		cvector sum;
		sum.reserve(size() + 1);

		sum.assign(*this);
		sum.append(value);

		return sum;
	}

	friend cvector operator+(const_reference value, const cvector &rhs) {
		cvector sum;
		sum.reserve(1 + rhs.size());

		sum.assign(value);
		sum.append(rhs);

		return sum;
	}

	cvector operator+(std::initializer_list<value_type> rhs_list) const {
		cvector sum;
		sum.reserve(size() + rhs_list.size());

		sum.assign(*this);
		sum.append(rhs_list);

		return sum;
	}

	friend cvector operator+(std::initializer_list<value_type> lhs_list, const cvector &rhs) {
		cvector sum;
		sum.reserve(lhs_list.size() + rhs.size());

		sum.assign(lhs_list);
		sum.append(rhs);

		return sum;
	}

	// TODO: iterator insert(const_iterator pos, const VECTORS t, const size_type index_vec = 0, const
	// size_type count = npos)
	// TODO: basic_cvector& insert(const size_type index, const_reference value)
	// TODO: basic_cvector& insert(const size_type index, const size_type count, const_reference value)
	// TODO: basic_cvector& insert(const size_type index, const VECTORS t, const size_type index_vec = 0, const
	// size_type count = npos)
	// TODO: template<class InputIt> basic_cvector& insert(const size_type index, InputIt first, InputIt last)
	// TODO: basic_cvector& insert(const size_type index, std::initializer_list<value_type> init_list)
	// TODO: template<class... Args> reference emplace(const size_type index, Args &&...args)
	// TODO: basic_cvector& erase(const size_type index = 0, const size_type count = npos)
	// TODO: void remove_front(const size_type count = 1)
	// TODO: void remove_back(const size_type count = 1)
	// TODO: basic_cvector subvec(const size_type pos = 0, const size_type count = npos) const -> alias slice()
	// TODO: template<class InputIt> basic_cvector subvec(InputIt first, InputIt last) const -> alias slice()
	// TODO: std::span subvecView(const size_type pos = 0, const size_type count = npos) const -> alias sliceView()
	// TODO: template<class InputIt> std::span subvecView(InputIt first, InputIt last) const -> alias sliceView()
	// TODO: find, rfind (all const)

	// Convenience functions for often executed operations.
	template<typename U>
	[[nodiscard]] bool contains(const U &item) const {
		const auto result = std::find(cbegin(), cend(), item);

		if (result == cend()) {
			return false;
		}

		return true;
	}

	template<typename Pred>
	[[nodiscard]] bool containsIf(Pred predicate) const {
		const auto result = std::find_if(cbegin(), cend(), predicate);

		if (result == cend()) {
			return false;
		}

		return true;
	}

	void sortUnique() {
		if (!empty()) {
			std::sort(begin(), end());
			erase(std::unique(begin(), end()), end());
		}
	}

	template<typename Compare>
	void sortUnique(Compare comp) {
		if (!empty()) {
			std::sort(begin(), end(), comp);
			erase(std::unique(begin(), end()), end());
		}
	}

	template<typename U>
	size_type remove(const U &item) {
		auto firstRemove = std::remove(begin(), end(), item);
		const auto dist  = std::distance(firstRemove, end());

		erase(firstRemove, end());

		return static_cast<size_type>(dist); // Deleted N elements.
	}

	template<typename Pred>
	size_type removeIf(Pred predicate) {
		auto firstRemove = std::remove_if(begin(), end(), predicate);
		const auto dist  = std::distance(firstRemove, end());

		erase(firstRemove, end());

		return static_cast<size_type>(dist); // Deleted N elements.
	}

	friend std::ostream &operator<<(std::ostream &os, const cvector &rhs) {
		os << "size: " << rhs.size() << ", elements: [";

		if constexpr (dv::concepts::internal::OutputStreamable<value_type>) {
			for (size_t i = 0; i < rhs.size(); i++) {
				// Print vector element.
				os << rhs[i];

				// Add comma spacer.
				if (i < (rhs.size() - 1)) {
					os << ", ";
				}
			}
		}
		else {
			os << "not printable";
		}

		os << "]";

		return os;
	}

private:
	void ensureCapacity(const size_type newSize) {
		// Do we have enough space left?
		if (newSize <= mMaximumSize) {
			return; // Yes.
		}

		// No, we must grow.
		// Let's use factor 1.5 for growing, for rationale see:
		// https://github.com/facebook/folly/blob/main/folly/docs/FBVector.md
		// Also let's make sure to not have weird micro-allocations,
		// so smallest size we request is 16 elements.
		auto newCapacity = static_cast<size_type>(static_cast<float>(mMaximumSize) * 1.5f);
		if (newCapacity < 16) {
			newCapacity = 16;
		}

		reallocateMemory(((newCapacity >= newSize) && (newCapacity <= max_size())) ? (newCapacity) : (newSize));
	}

	void reallocateMemory(const size_type newSize) {
		if (newSize > max_size()) {
			throw std::length_error("requested size exceeds max_size() limit.");
		}

		// If wanted size is zero (shrink_to_fit() on empty vector for example),
		// reset to the empty vector using the nullptr placeholder.
		if (newSize == 0) {
			if (mMaximumSize > 0) {
				free(mDataPtr);
				mDataPtr     = nullptr;
				mMaximumSize = 0;
			}

			return;
		}

		pointer new_data_ptr = nullptr;

		if constexpr (std::is_trivially_copyable_v<value_type>) {
			// Type is POD, we can just use realloc.
			new_data_ptr = static_cast<pointer>(realloc(mDataPtr, newSize * sizeof(value_type)));
			if (new_data_ptr == nullptr) {
				// Failed to allocate memory.
				throw std::bad_alloc();
			}
		}
		else {
			// Type is not POD (C++ object instead), we cannot use realloc in one step directly.
			// So we allocate the new size, move objects over, and then free.
			new_data_ptr = static_cast<pointer>(realloc(nullptr, newSize * sizeof(value_type)));
			if (new_data_ptr == nullptr) {
				// Failed to allocate memory.
				throw std::bad_alloc();
			}

			// If memory was previously allocated:
			if (mMaximumSize > 0) {
				// Move construct new memory.
				std::uninitialized_move_n(begin(), mCurrSize, iterator(new_data_ptr));

				// Destroy old objects.
				std::destroy_n(begin(), mCurrSize);

				// Free old memory. Objects have been cleaned up above.
				free(mDataPtr);
			}
		}

		// Succeeded, update ptr + capacity.
		mDataPtr     = new_data_ptr;
		mMaximumSize = newSize;
	}

	template<bool CHECKED>
	[[nodiscard]] size_type getIndex(const size_type index) const {
		if constexpr (CHECKED) {
			if (index >= mCurrSize) {
				throw std::out_of_range("Index out of range.");
			}
		}
		else {
			dv::runtime_assert(index < mCurrSize, "Index out of range.");
		}

		return index;
	}

	template<bool CHECKED>
	[[nodiscard]] size_type getIndex(const difference_type index) const {
		// Support negative indexes to go from the last existing/defined element
		// backwards (not from the capacity!).
		size_type realIndex;

		if (index < 0) {
			const difference_type negIndex = static_cast<difference_type>(mCurrSize) + index;

			if constexpr (CHECKED) {
				if (negIndex < 0) {
					throw std::out_of_range("Negative index out of range.");
				}
			}
			else {
				dv::runtime_assert(negIndex >= 0, "Negative index out of range.");
			}

			realIndex = static_cast<size_type>(negIndex);
		}
		else {
			realIndex = static_cast<size_type>(index);
		}

		return getIndex<CHECKED>(realIndex);
	}
};

} // namespace dv

static_assert(std::is_standard_layout_v<dv::cvector<int>>, "cvector is not standard layout");

static_assert(sizeof(dv::cvector<int>) == (3 * sizeof(size_t)), "cvector size is unexpected");

// fmt formatting compatibility.
namespace fmt {

template<typename T>
class formatter<dv::cvector<T>> {
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
	auto format(const dv::cvector<T> &vec, FormatContext &ctx) DV_EXT_FMT_CONST {
		return fmt::format_to(ctx.out(), DV_EXT_FMT_RUNTIME(mFmtForward.data()),
			fmt::join(vec.cbegin(), vec.cend(), std::string_view{mSeparator.data()}));
	}

private:
	static constexpr size_t FORMATTER_MAX_LEN{32};

	std::array<char, FORMATTER_MAX_LEN> mFmtForward;
	std::array<char, FORMATTER_MAX_LEN> mSeparator;
};

} // namespace fmt
