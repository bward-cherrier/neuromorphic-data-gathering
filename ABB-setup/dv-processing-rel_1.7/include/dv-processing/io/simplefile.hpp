#pragma once

#include "../core/utils.hpp"
#include "../data/cstring.hpp"
#include "../data/cvector.hpp"
#include "../exception/exceptions/file_exceptions.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"

#include <boost/nowide/cstdio.hpp>

#include <cstdio>
#include <filesystem>
#include <limits>

namespace dv::io {

enum class ModeFlags : uint8_t {
	READ  = (1 << 0), // File must exist for read-only mode.
	WRITE = (1 << 1), // File will be created if missing in write modes.
};

inline ModeFlags operator|(const ModeFlags lhs, const ModeFlags rhs) {
	return dv::IntegerAsEnum<ModeFlags>(dv::EnumAsInteger(lhs) | dv::EnumAsInteger(rhs));
}

inline ModeFlags &operator|=(ModeFlags &lhs, const ModeFlags rhs) {
	lhs = (lhs | rhs); // Call | overload above.
	return lhs;
}

inline bool operator&(const ModeFlags lhs, const ModeFlags rhs) {
	return ((dv::EnumAsInteger(lhs) & dv::EnumAsInteger(rhs)) != 0);
}

enum class WriteFlags : uint8_t {
	NONE     = (1 << 0), // Normal writes, no truncation.
	TRUNCATE = (1 << 1), // Truncate file for writing.
	APPEND   = (1 << 2), // Append only at end of file for writing.
};

inline WriteFlags operator|(const WriteFlags lhs, const WriteFlags rhs) {
	return dv::IntegerAsEnum<WriteFlags>(dv::EnumAsInteger(lhs) | dv::EnumAsInteger(rhs));
}

inline WriteFlags &operator|=(WriteFlags &lhs, const WriteFlags rhs) {
	lhs = (lhs | rhs); // Call | overload above.
	return lhs;
}

inline bool operator&(const WriteFlags lhs, const WriteFlags rhs) {
	return ((dv::EnumAsInteger(lhs) & dv::EnumAsInteger(rhs)) != 0);
}

enum class SeekFlags : int {
	START   = SEEK_SET,
	CURRENT = SEEK_CUR,
	END     = SEEK_END,
};

class SimpleFile {
public:
	constexpr SimpleFile() = default;

	/**
	 * Open a file for reading and/or writing, supporting extra modes for writing and buffer control.
	 * Will always do what you expect and throw an exception if there's any issue.
	 *
	 * @param filePath file path to open.
	 * @param modeFlags Open file for reading, writing or both.
	 * @param writeFlags If opening for writing, extra flags for truncation and append modes.
	 * @param bufferSize Size of user-space buffer for file operations. Default 64KB, use 0 to disable buffering
	 * entirely.
	 */
	explicit SimpleFile(const std::filesystem::path &filePath, const ModeFlags modeFlags,
		const WriteFlags writeFlags = WriteFlags::NONE, const size_t bufferSize = 65536) {
		// Basic path checks.
		if (filePath.empty()) {
			throw dv::exceptions::InvalidArgument<std::filesystem::path>("File path not specified.", filePath);
		}

		if (filePath.filename().empty()) {
			throw dv::exceptions::InvalidArgument<std::filesystem::path>(
				"File name is not present in file path.", filePath);
		}

		// Flag checks.
		if ((writeFlags & WriteFlags::TRUNCATE) && (writeFlags & WriteFlags::APPEND)) {
			throw dv::exceptions::FileError(
				"TRUNCATE and APPEND flags cannot be specified at the same time.", filePath);
		}
		if ((!(modeFlags & ModeFlags::WRITE))
			&& ((writeFlags & WriteFlags::TRUNCATE) || (writeFlags & WriteFlags::APPEND))) {
			throw dv::exceptions::FileError(
				"TRUNCATE and APPEND flags can only be specified together with WRITE mode.", filePath);
		}

		std::string mode;

		if ((modeFlags & ModeFlags::READ) && (!(modeFlags & ModeFlags::WRITE))) {
			// Read only mode:
			mode = "r";
			// All other flags don't matter as they make no sense for a read-only file.
		}
		else if ((!(modeFlags & ModeFlags::READ)) && (modeFlags & ModeFlags::WRITE)) {
			// Write only mode:
			if (writeFlags & WriteFlags::TRUNCATE) {
				// Always operate on an empty file: create it empty or truncate if exists.
				mode = "w";
			}
			else if (writeFlags & WriteFlags::APPEND) {
				// Append to file, so don't truncate if exists.
				mode = "a";
			}
			else {
				// Create file if not exists, but don't truncate if it does, nor
				// open in append mode as we want random writes. This doesn't
				// really exist in POSIX, big fail, so we have to make it ourselves.
				if (std::filesystem::exists(filePath)) {
					// Open for writes without truncate or append.
					// If the file disappears between this exists() check and the fopen(),
					// open will fail and give an error, this is safe.
					mode = "r+";
				}
				else {
					// Doesn't exist, so we can open with truncate which implies creation.
					// To make sure it really didn't exist we can use the "x" flag since
					// C++17, which will make it fail if the file was created in the meantime
					// by some other call. Failing with an error is the safe option here.
					mode = "wx";
				}
			}
		}
		else if ((modeFlags & ModeFlags::READ) && (modeFlags & ModeFlags::WRITE)) {
			// Read-Write mode:
			if (writeFlags & WriteFlags::TRUNCATE) {
				// Always operate on an empty file: create it empty or truncate if exists.
				mode = "w+";
			}
			else if (writeFlags & WriteFlags::APPEND) {
				// Append to file, so don't truncate if exists.
				mode = "a+";
			}
			else {
				// Create file if not exists, but don't truncate if it does, nor
				// open in append mode as we want random writes. This doesn't
				// really exist in POSIX, big fail, so we have to make it ourselves.
				if (std::filesystem::exists(filePath)) {
					// Open for writes without truncate or append.
					// If the file disappears between this exists() check and the fopen(),
					// open will fail and give an error, this is safe.
					mode = "r+";
				}
				else {
					// Doesn't exist, so we can open with truncate which implies creation.
					// To make sure it really didn't exist we can use the "x" flag since
					// C++17, which will make it fail if the file was created in the meantime
					// by some other call. Failing with an error is the safe option here.
					mode = "w+x";
				}
			}
		}
		else {
			throw dv::exceptions::FileError("Select at least one of READ, WRITE modes.", filePath);
		}

		// Always treat files as binary.
		mode.push_back('b');

		f = boost::nowide::fopen(filePath.string().c_str(), mode.c_str());
		if (f == nullptr) {
			throw dv::exceptions::FileError(
				fmt::format("Failed to open file with error: '{}'.", dv::errnoToString(errno)), filePath);
		}

		// Support unbuffered file writes.
		if (bufferSize == 0) {
			if (std::setvbuf(f, nullptr, _IONBF, 0) != 0) {
				close();

				throw dv::exceptions::FileError("Failed to set unbuffered mode (bufferSize=0).", filePath);
			}
		}
		else {
			// Allocate buffer for data.
			fBuffer = static_cast<char *>(malloc(bufferSize));
			if (fBuffer == nullptr) {
				close();

				throw dv::exceptions::FileError(
					fmt::format("Failed to allocate memory for buffered mode with size={}.", bufferSize), filePath);
			}

			if (std::setvbuf(f, fBuffer, _IOFBF, bufferSize) != 0) {
				close();

				throw dv::exceptions::FileError(
					fmt::format("Failed to set buffered mode with size={}.", bufferSize), filePath);
			}
		}

		// Remember file path.
		fPath = filePath;
	}

	~SimpleFile() noexcept {
		close();
	}

	// File resource should be movable but not copyable.
	SimpleFile(const SimpleFile &file)           = delete;
	SimpleFile &operator=(const SimpleFile &rhs) = delete;

	SimpleFile(SimpleFile &&file) noexcept {
		// Steal file and info.
		f       = file.f;
		fBuffer = file.fBuffer;
		fPath   = std::move(file.fPath);

		file.f       = nullptr;
		file.fBuffer = nullptr;
		file.fPath.clear();
	}

	SimpleFile &operator=(SimpleFile &&rhs) noexcept {
		dv::runtime_assert(this != &rhs, "cannot move-assign into self");

		// Close current file if open.
		close();

		// Steal file and info.
		f       = rhs.f;
		fBuffer = rhs.fBuffer;
		fPath   = std::move(rhs.fPath);

		rhs.f       = nullptr;
		rhs.fBuffer = nullptr;
		rhs.fPath.clear();

		return *this;
	}

	bool isOpen() const {
		return (f != nullptr);
	}

	void flush() {
		if (std::fflush(f) != 0) {
			throw dv::exceptions::FileError(
				fmt::format("Failed to flush file with error: '{}'.", dv::errnoToString(errno)), fPath);
		}
	}

	void write(const std::string_view data) {
		write(data.data(), data.size());
	}

	template<typename T>
	void write(const std::vector<T> &data) {
		write(data.data(), data.size());
	}

	template<typename T>
	void write(const dv::cvector<T> &data) {
		write(data.data(), data.size());
	}

	template<typename T>
	void write(const T *elem, size_t num) {
		while (num > 0) {
			const auto result = std::fwrite(elem, sizeof(T), num, f);
			if ((result != num) && (std::ferror(f) != 0)) {
				// TODO (llongi): Segfault on Apple clang with xcode 12 when using libfmt, try again with xcode 13
				throw dv::exceptions::FileWriteError("Failed to write with error: " + dv::errnoToString(errno), fPath);
			}

			num  -= result;
			elem += result;
		}
	}

	template<typename S, typename... Args>
	void format(const S &format, Args &&...args) {
		write(fmt::format(format, std::forward<Args>(args)...));
	}

	void read(std::string &data) const {
		read(data.data(), data.size());
	}

	template<typename T>
	void read(std::vector<T> &data) const {
		read(data.data(), data.size());
	}

	void read(dv::cstring &data) const {
		read(data.data(), data.size());
	}

	template<typename T>
	void read(dv::cvector<T> &data) const {
		read(data.data(), data.size());
	}

	template<typename T>
	void read(T *elem, size_t num) const {
		while (num > 0) {
			const auto result = std::fread(elem, sizeof(T), num, f);
			if (result != num) {
				if (std::feof(f) != 0) {
					throw dv::exceptions::EndOfFile(fPath);
				}

				if (std::ferror(f) != 0) {
					// TODO (llongi): Segfault on Apple clang with xcode 12 when using libfmt, try again with xcode 13
					throw dv::exceptions::FileReadError(
						"Failed to read with error: " + dv::errnoToString(errno), fPath);
				}
			}

			num  -= result;
			elem += result;
		}
	}

	void readAll(std::string &data) const {
		const auto fs   = fileSize();
		const auto curr = tell();
		data.resize(fs - curr);

		read(data);
	}

	void readAll(std::vector<uint8_t> &data) const {
		const auto fs   = fileSize();
		const auto curr = tell();
		data.resize(fs - curr);

		read(data);
	}

	void readAll(dv::cstring &data) const {
		const auto fs   = fileSize();
		const auto curr = tell();
		data.resize(fs - curr);

		read(data);
	}

	void readAll(dv::cvector<uint8_t> &data) const {
		const auto fs   = fileSize();
		const auto curr = tell();
		data.resize(fs - curr);

		read(data);
	}

	[[nodiscard]] uint64_t tell() const {
#if defined(_WIN32) || defined(_WIN64) || defined(__CYGWIN__)
		const auto result = _ftelli64(f);
#else
		const auto result = ftello(f);
#endif

		if (result < 0) {
			throw dv::exceptions::FileError(
				fmt::format("Failed to get position with error: '{}'.", dv::errnoToString(errno)), fPath);
		}

		return static_cast<uint64_t>(result);
	}

	void seek(const uint64_t offset, const SeekFlags flags = SeekFlags::START) const {
		if (offset > std::numeric_limits<int64_t>::max()) {
			throw dv::exceptions::FileError("Offset value is too big.", fPath);
		}

#if defined(_WIN32) || defined(_WIN64) || defined(__CYGWIN__)
		const auto result = _fseeki64(f, static_cast<int64_t>(offset), dv::EnumAsInteger(flags));
#else
		const auto result = fseeko(f, static_cast<int64_t>(offset), dv::EnumAsInteger(flags));
#endif

		if (result != 0) {
			throw dv::exceptions::FileError(
				fmt::format("Failed to set position with error: '{}'.", dv::errnoToString(errno)), fPath);
		}
	}

	void rewind() const {
		std::rewind(f);
	}

	[[nodiscard]] uint64_t fileSize() const {
		const auto curr = tell();
		seek(0, SeekFlags::END);
		const auto fileSize = tell();
		seek(curr, SeekFlags::START);

		return fileSize;
	}

	[[nodiscard]] std::filesystem::path path() const {
		return fPath;
	}

private:
	std::FILE *f{nullptr};
	char *fBuffer{nullptr};
	std::filesystem::path fPath{};

	void close() noexcept {
		if (f != nullptr) {
			std::fclose(f);
			f = nullptr;

			free(fBuffer);
			fBuffer = nullptr;

			fPath.clear();
		}
	}
};

class SimpleReadOnlyFile : private SimpleFile {
public:
	constexpr SimpleReadOnlyFile() = default;

	explicit SimpleReadOnlyFile(const std::filesystem::path &filePath, const size_t bufferSize = 65536) :
		SimpleFile(filePath, ModeFlags::READ, WriteFlags::NONE, bufferSize) {
	}

	using SimpleFile::fileSize;
	using SimpleFile::isOpen;
	using SimpleFile::path;
	using SimpleFile::read;
	using SimpleFile::readAll;
	using SimpleFile::rewind;
	using SimpleFile::seek;
	using SimpleFile::tell;
};

class SimpleWriteOnlyFile : private SimpleFile {
public:
	constexpr SimpleWriteOnlyFile() = default;

	explicit SimpleWriteOnlyFile(const std::filesystem::path &filePath, const WriteFlags writeFlags = WriteFlags::NONE,
		const size_t bufferSize = 65536) :
		SimpleFile(filePath, ModeFlags::WRITE, writeFlags, bufferSize) {
	}

	using SimpleFile::fileSize;
	using SimpleFile::flush;
	using SimpleFile::format;
	using SimpleFile::isOpen;
	using SimpleFile::path;
	using SimpleFile::rewind;
	using SimpleFile::seek;
	using SimpleFile::tell;
	using SimpleFile::write;
};

} // namespace dv::io
