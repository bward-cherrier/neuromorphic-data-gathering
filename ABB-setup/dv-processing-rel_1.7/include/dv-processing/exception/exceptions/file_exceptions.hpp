#pragma once

#include "../exception_base.hpp"

namespace dv::exceptions {

namespace info {
struct FileError {
	using Info = std::filesystem::path;

	static std::string format(const Info &info) {
		return fmt::format("File: {}", info.string());
	}
};
} // namespace info

using FileError = Exception_<info::FileError>;

namespace info {
struct FileOpenError {
	using Info = std::filesystem::path;

	static std::string format(const Info &info) {
		return fmt::format("File {} could not be opened", info.string());
	}
};
} // namespace info

using FileOpenError = Exception_<info::FileOpenError, FileError>;

namespace info {
struct FileReadError {
	using Info = std::filesystem::path;

	static std::string format(const Info &info) {
		return fmt::format("File {} could not be read", info.string());
	}
};
} // namespace info

using FileReadError = Exception_<info::FileReadError, FileError>;

namespace info {
struct FileWriteError {
	using Info = std::filesystem::path;

	static std::string format(const Info &info) {
		return fmt::format("File {} could not be written", info.string());
	}
};
} // namespace info

using FileWriteError = Exception_<info::FileWriteError, FileError>;

namespace info {
struct FileNotFound {
	using Info = std::filesystem::path;

	static std::string format(const Info &info) {
		return fmt::format("File {} not found", info.string());
	}
};
} // namespace info

using FileNotFound = Exception_<info::FileNotFound, FileError>;

namespace info {
struct AedatFileError {
	using Info = std::filesystem::path;
};
} // namespace info

using AedatFileError = Exception_<info::AedatFileError, FileError>;

namespace info {
struct AedatVersionError {
	using Info = int32_t;

	static std::string format(const Info &info) {
		return fmt::format("AEDAT version: {:d}", info);
	}
};
} // namespace info

using AedatVersionError = Exception_<info::AedatVersionError, AedatFileError>;

namespace info {
struct AedatFileParseError {
	using Info = std::filesystem::path;

	static std::string format(const Info &info) {
		return fmt::format("Error while parsing AEDAT file: {}", info.string());
	}
};
} // namespace info

using AedatFileParseError = Exception_<info::AedatFileParseError, AedatFileError>;

namespace info {
struct EndOfFile {
	using Info = std::filesystem::path;

	static std::string format(const Info &info) {
		return fmt::format("File {} End-Of-File reached", info.string());
	}
};
} // namespace info

using EndOfFile = Exception_<info::EndOfFile>;

} // namespace dv::exceptions
