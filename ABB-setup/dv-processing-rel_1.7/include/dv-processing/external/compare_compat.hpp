#pragma once

#include <compare>

#if !(__cplusplus > 201703L && defined(__cpp_lib_three_way_comparison) && __cpp_lib_three_way_comparison >= 201907L) \
	&& !defined(_MSC_VER)

#	include <concepts>
#	include <functional>
#	include <iterator>

namespace std {

// Provide compatibility implementation of std::lexicographical_compare_three_way() from libc++ bug tracker.
template<class _InputIterator1, class _InputIterator2, class _Cmp>
_LIBCPP_HIDE_FROM_ABI constexpr auto lexicographical_compare_three_way(
	_InputIterator1 __first1, _InputIterator1 __last1, _InputIterator2 __first2, _InputIterator2 __last2, _Cmp __comp)
	-> decltype(__comp(*__first1, *__first2)) {
	using return_type_t = decltype(__comp(*__first1, *__first2));
	static_assert(
		std::disjunction_v<std::is_same<return_type_t, std::strong_ordering>,
			std::is_same<return_type_t, std::weak_ordering>, std::is_same<return_type_t, std::partial_ordering>>,
		"The return type must be a comparison category type.");

	if constexpr (__is_cpp17_random_access_iterator<_InputIterator1>::value
				  && __is_cpp17_random_access_iterator<_InputIterator2>::value) {
		// Fast path for random access iterators which computes the number of loop iterations up-front and
		// then skips the iterator comparisons inside the loop.
		static_assert(std::is_integral_v<typename std::iterator_traits<_InputIterator1>::difference_type>,
			"Using a non-integral difference_type is undefined behavior");
		static_assert(std::is_integral_v<typename std::iterator_traits<_InputIterator2>::difference_type>,
			"Using a non-integral difference_type is undefined behavior");

		auto __len1    = __last1 - __first1;
		auto __len2    = __last2 - __first2;
		auto __min_len = __len1 < __len2 ? __len1 : __len2;

		for (decltype(__min_len) __i = 0; __i < __min_len; ++__i) {
			auto __c = __comp(*__first1, *__first2);
			if (__c != 0) {
				return __c;
			}
			++__first1;
			++__first2;
		}

		return __len1 <=> __len2;
	}
	else {
		// Unoptimized implementation which compares the iterators against the end in every loop iteration
		while (true) {
			bool __exhausted1 = __first1 == __last1;
			bool __exhausted2 = __first2 == __last2;

			if (__exhausted1 || __exhausted2) {
				if (!__exhausted1)
					return strong_ordering::greater;
				if (!__exhausted2)
					return strong_ordering::less;
				return strong_ordering::equal;
			}

			auto __c = __comp(*__first1, *__first2);
			if (__c != 0) {
				return __c;
			}

			++__first1;
			++__first2;
		}
	}
}

template<class _InputIterator1, class _InputIterator2>
_LIBCPP_HIDE_FROM_ABI constexpr auto lexicographical_compare_three_way(
	_InputIterator1 __first1, _InputIterator1 __last1, _InputIterator2 __first2, _InputIterator2 __last2) {
	return std::lexicographical_compare_three_way(__first1, __last1, __first2, __last2, std::compare_three_way{});
}

} // namespace std

#endif
