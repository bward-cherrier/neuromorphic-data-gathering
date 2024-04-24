#pragma once

#include <version>

#if (__cplusplus > 201703L && __has_include(<source_location>) && defined(__cpp_lib_source_location) && __cpp_lib_source_location >= 201907L) || defined(_MSC_VER)
#	include <source_location>
#else

#	include <string_view>

namespace std {

class source_location {
public:
	// This should be consteval, but clang doesn't support this yet in default function arguments:
	// https://stackoverflow.com/questions/68789984/immediate-function-as-default-function-argument-initializer-in-clang
	static constexpr source_location current(const std::string_view file = __builtin_FILE(),
		const std::string_view function = __builtin_FUNCTION(), const uint32_t line = __builtin_LINE()) noexcept {
		return source_location{file, function, line};
	}

	constexpr source_location() noexcept = default;

	constexpr source_location(const source_location &other)     = default;
	constexpr source_location(source_location &&other) noexcept = default;

	[[nodiscard]] constexpr const char *file_name() const noexcept {
		return mFile.data();
	}

	[[nodiscard]] constexpr const char *function_name() const noexcept {
		return mFunction.data();
	}

	[[nodiscard]] constexpr std::uint_least32_t line() const noexcept {
		return mLine;
	}

private:
	std::string_view mFile;
	std::string_view mFunction;
	uint32_t mLine{0};

	constexpr source_location(
		const std::string_view file, const std::string_view function, const uint32_t line) noexcept :
		mFile(file),
		mFunction(function),
		mLine(line) {
	}
};

} // namespace std

#endif
