#pragma once

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <type_traits>

namespace dv {

template<class T>
class cPtrIterator {
public:
	// Iterator traits.
	using iterator_category = std::random_access_iterator_tag;
	using value_type        = typename std::remove_cv_t<T>;
	using pointer           = T *;
	using reference         = T &;
	using size_type         = size_t;
	using difference_type   = ptrdiff_t;

private:
	pointer mElementPtr{nullptr};

public:
	// Constructors.
	constexpr cPtrIterator() noexcept = default;

	constexpr cPtrIterator(pointer elementPtr) noexcept : mElementPtr(elementPtr) {
	}

	// Data access operators.
	[[nodiscard]] constexpr reference operator*() const noexcept {
		return *mElementPtr;
	}

	[[nodiscard]] constexpr pointer operator->() const noexcept {
		return mElementPtr;
	}

	[[nodiscard]] constexpr reference operator[](const size_type index) const noexcept {
		return mElementPtr[index];
	}

	// Comparison operators.
	[[nodiscard]] constexpr bool operator==(const cPtrIterator &rhs) const noexcept {
		return (mElementPtr == rhs.mElementPtr);
	}

	[[nodiscard]] constexpr bool operator!=(const cPtrIterator &rhs) const noexcept {
		return (mElementPtr != rhs.mElementPtr);
	}

	[[nodiscard]] constexpr bool operator<(const cPtrIterator &rhs) const noexcept {
		return (mElementPtr < rhs.mElementPtr);
	}

	[[nodiscard]] constexpr bool operator>(const cPtrIterator &rhs) const noexcept {
		return (mElementPtr > rhs.mElementPtr);
	}

	[[nodiscard]] constexpr bool operator<=(const cPtrIterator &rhs) const noexcept {
		return (mElementPtr <= rhs.mElementPtr);
	}

	[[nodiscard]] constexpr bool operator>=(const cPtrIterator &rhs) const noexcept {
		return (mElementPtr >= rhs.mElementPtr);
	}

	// Prefix increment.
	cPtrIterator &operator++() noexcept {
		mElementPtr++;
		return *this;
	}

	// Postfix increment.
	cPtrIterator operator++(int) noexcept {
		return cPtrIterator(mElementPtr++);
	}

	// Prefix decrement.
	cPtrIterator &operator--() noexcept {
		mElementPtr--;
		return *this;
	}

	// Postfix decrement.
	cPtrIterator operator--(int) noexcept {
		return cPtrIterator(mElementPtr--);
	}

	// Iter += N.
	cPtrIterator &operator+=(const size_type add) noexcept {
		mElementPtr += add;
		return *this;
	}

	// Iter + N.
	[[nodiscard]] constexpr cPtrIterator operator+(const size_type add) const noexcept {
		return cPtrIterator(mElementPtr + add);
	}

	// N + Iter. Must be friend as Iter is right-hand-side.
	[[nodiscard]] friend constexpr cPtrIterator operator+(const size_type lhs, const cPtrIterator &rhs) noexcept {
		return cPtrIterator(rhs.mElementPtr + lhs);
	}

	// Iter -= N.
	cPtrIterator &operator-=(const size_type sub) noexcept {
		mElementPtr -= sub;
		return *this;
	}

	// Iter - N. (N - Iter doesn't make sense!)
	[[nodiscard]] constexpr cPtrIterator operator-(const size_type sub) const noexcept {
		return cPtrIterator(mElementPtr - sub);
	}

	// Iter - Iter. (Iter + Iter doesn't make sense!)
	[[nodiscard]] constexpr difference_type operator-(const cPtrIterator &rhs) const noexcept {
		return (mElementPtr - rhs.mElementPtr);
	}

	// Swap two iterators.
	void swap(cPtrIterator &rhs) noexcept {
		std::swap(mElementPtr, rhs.mElementPtr);
	}

	[[nodiscard]] constexpr operator cPtrIterator<const value_type>() const noexcept {
		return cPtrIterator<const value_type>{mElementPtr};
	}
};

} // namespace dv
