#pragma once

#include "../external/source_location_compat.hpp"

#include "internal.hpp"

#include <boost/core/demangle.hpp>
#include <boost/stacktrace.hpp>

#include <filesystem>
#include <string>

namespace dv::exceptions {

namespace info {
struct EmptyException {
	using Info = void;
};
} // namespace info

class Exception : public std::exception {
public:
	explicit Exception(const std::source_location &location = std::source_location::current(),
		const boost::stacktrace::stacktrace &stacktrace     = boost::stacktrace::stacktrace(),
		const std::string_view type                         = boost::core::demangle(typeid(Exception).name())) {
		createInfo("", location.file_name(), location.function_name(), location.line(),
			boost::stacktrace::to_string(stacktrace), type);
	}

	explicit Exception(const std::string_view whatInfo,
		const std::source_location &location            = std::source_location::current(),
		const boost::stacktrace::stacktrace &stacktrace = boost::stacktrace::stacktrace(),
		const std::string_view type                     = boost::core::demangle(typeid(Exception).name())) {
		createInfo(whatInfo, location.file_name(), location.function_name(), location.line(),
			boost::stacktrace::to_string(stacktrace), type);
	}

	~Exception() override = default;

	Exception(const Exception &other) = default;
	Exception(Exception &&other)      = default;

	Exception operator<<(const std::string_view info) {
		mInfo += fmt::format("\n{:s}", info);
		return *this;
	}

	[[nodiscard]] const char *what() const noexcept override {
		return mInfo.c_str();
	}

protected:
	std::string mInfo;

private:
	void createInfo(const std::string_view whatInfo, const std::string_view file, const std::string_view function,
		const uint32_t line, const std::string_view stacktrace, const std::string_view type) {
		// Remove the namespace information from "type"
		auto t = type;

		if (const auto pos = t.rfind("::"); pos != std::string_view::npos) {
			t = t.substr(pos + 2);
		}

		const auto filePath = std::filesystem::absolute(file).lexically_normal().make_preferred();

		mInfo = fmt::format("{:s}({:d}): {:s}()\n{:s}: {:s}\nStacktrace:\n{:s}", filePath.string(), line, function, t,
			whatInfo, stacktrace);
	}
};

template<typename EXCEPTION_TYPE, typename BASE_TYPE = Exception>
class Exception_ : public BASE_TYPE {
public:
	using Info = typename EXCEPTION_TYPE::Info;

	template<internal::HasExtraExceptionInfo T = EXCEPTION_TYPE>
	Exception_(const std::string_view whatInfo, const typename T::Info &errorInfo,
		const std::source_location &location            = std::source_location::current(),
		const boost::stacktrace::stacktrace &stacktrace = boost::stacktrace::stacktrace(),
		const std::string_view type                     = boost::core::demangle(typeid(EXCEPTION_TYPE).name())) :
		BASE_TYPE(fmt::format("{:s} - Error info: {:s}", whatInfo, internal::format<T>(errorInfo)), location,
			stacktrace, type) {
	}

	template<internal::HasExtraExceptionInfo T = EXCEPTION_TYPE>
	Exception_(const typename T::Info &errorInfo,
		const std::source_location &location            = std::source_location::current(),
		const boost::stacktrace::stacktrace &stacktrace = boost::stacktrace::stacktrace(),
		const std::string_view type                     = boost::core::demangle(typeid(EXCEPTION_TYPE).name())) :
		BASE_TYPE(fmt::format("Error info: {:s}", internal::format<T>(errorInfo)), location, stacktrace, type) {
	}

	Exception_(const std::string_view whatInfo, const std::source_location &location = std::source_location::current(),
		const boost::stacktrace::stacktrace &stacktrace = boost::stacktrace::stacktrace(),
		const std::string_view type                     = boost::core::demangle(typeid(EXCEPTION_TYPE).name())) :
		BASE_TYPE(whatInfo, location, stacktrace, type) {
	}

	Exception_(const std::source_location &location     = std::source_location::current(),
		const boost::stacktrace::stacktrace &stacktrace = boost::stacktrace::stacktrace(),
		const std::string_view type                     = boost::core::demangle(typeid(EXCEPTION_TYPE).name())) :
		BASE_TYPE(location, stacktrace, type) {
	}

	~Exception_() override = default;

	Exception_(const Exception_ &other) = default;
	Exception_(Exception_ &&other)      = default;

	template<internal::HasExtraExceptionInfo T = EXCEPTION_TYPE>
	Exception_ operator<<(const typename T::Info &errorInfo) {
		Exception::operator<<(internal::format<T>(errorInfo));
		return *this;
	}

	Exception_ operator<<(const std::string_view whatInfo) {
		Exception::operator<<(whatInfo);
		return *this;
	}
};

} // namespace dv::exceptions
