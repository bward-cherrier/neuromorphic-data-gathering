#pragma once

#include "../external/fmt_compat.hpp"
#include "../external/source_location_compat.hpp"

#include <boost/stacktrace.hpp>

#include <cstdlib>
#include <filesystem>
#include <string_view>

namespace dv {

#if defined(NDEBUG) || (defined(_WIN32) && !defined(_DEBUG))

static constexpr bool DEBUG_ENABLED{false};

inline void runtime_assert([[maybe_unused]] const bool expression, [[maybe_unused]] const std::string_view message) {
	// Empty. Optimized away. Keeps side-effects of expression if any.
}

#else

static constexpr bool DEBUG_ENABLED{true};

inline void runtime_assert(const bool expression, const std::string_view message,
	const std::source_location &location = std::source_location::current()) {
	if (!expression) {
		// Clean up file path. File here comes from std::source_location::file_name(), so it's just a 'bag-of-bytes'.
		// We simply treat it as a UTF-8 string to print, which will work anywhere minus Windows, where the
		// wide-characters aren't accessible anyway by design. No way to get that in a portable way.
		std::filesystem::path filePath{location.file_name()};
		filePath = std::filesystem::absolute(filePath).lexically_normal().make_preferred();

		fmt::print(stderr, "{:s}({:d}): {:s}()\nruntime assertion failed: {:s}\nStacktrace:\n{:s}\n", filePath.string(),
			location.line(), location.function_name(), message,
			boost::stacktrace::to_string(boost::stacktrace::stacktrace()));
		std::abort();
	}
}

#endif

} // namespace dv
