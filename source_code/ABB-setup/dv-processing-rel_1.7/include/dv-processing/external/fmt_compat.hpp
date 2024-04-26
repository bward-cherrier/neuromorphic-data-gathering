#pragma once

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <fmt/ostream.h>

#if defined(FMT_VERSION) && FMT_VERSION >= 80000

#	define DV_EXT_FMT_RUNTIME(VAR) fmt::runtime((VAR))
#	define DV_EXT_FMT_CONST        const

#else

#	define DV_EXT_FMT_RUNTIME(VAR) (VAR)
#	define DV_EXT_FMT_CONST

#endif

#if defined(FMT_VERSION) && FMT_VERSION < 90000

namespace fmt {

// Formats an object of type T that has an overloaded ostream operator<<.
template<typename Char>
struct basic_ostream_formatter : fmt::formatter<std::basic_string_view<Char>, Char> {
	template<typename T, typename OutputIt>
	auto format(const T &value, fmt::basic_format_context<OutputIt, Char> &ctx) DV_EXT_FMT_CONST -> OutputIt {
		auto buffer = fmt::basic_memory_buffer<Char>();
		format_value(buffer, value, ctx.locale());
		return fmt::formatter<std::basic_string_view<Char>, Char>::format({buffer.data(), buffer.size()}, ctx);
	}
};

using ostream_formatter = fmt::basic_ostream_formatter<char>;

} // namespace fmt

#endif
