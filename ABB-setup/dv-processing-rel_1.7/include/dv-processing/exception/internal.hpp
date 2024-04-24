#pragma once

#include "../external/fmt_compat.hpp"

#include <concepts>
#include <string>

namespace dv::exceptions::internal {

template<typename T>
concept HasExtraExceptionInfo = not
std::same_as<typename T::Info, void>;

template<typename T>
concept HasCustomExceptionFormatter = HasExtraExceptionInfo<T> && requires(const typename T::Info &info) {
																	  {
																		  T::format(info)
																		  } -> std::convertible_to<std::string>;
																  };

template<HasCustomExceptionFormatter T>
[[nodiscard]] std::string format(const typename T::Info &info) {
	return T::format(info);
}

template<typename T>
concept NoCustomExceptionFormatter = not
HasCustomExceptionFormatter<T>;

template<NoCustomExceptionFormatter T>
[[nodiscard]] std::string format(const typename T::Info &info) {
	return fmt::format("{}", info);
}

} // namespace dv::exceptions::internal
