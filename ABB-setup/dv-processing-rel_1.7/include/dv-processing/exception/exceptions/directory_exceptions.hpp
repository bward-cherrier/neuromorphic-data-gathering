#pragma once

#include "../exception_base.hpp"

namespace dv::exceptions {

namespace info {
struct DirectoryError {
	using Info = std::filesystem::path;

	static std::string format(const Info &info) {
		return fmt::format("Directory: {}", info.string());
	}
};
} // namespace info

using DirectoryError = Exception_<info::DirectoryError>;

namespace info {
struct DirectoryNotFound {
	using Info = std::filesystem::path;

	static std::string format(const Info &info) {
		return fmt::format("Directory {} not found", info.string());
	}
};
} // namespace info

using DirectoryNotFound = Exception_<info::DirectoryNotFound, DirectoryError>;

} // namespace dv::exceptions
