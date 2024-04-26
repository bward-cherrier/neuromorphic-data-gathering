#pragma once

#include "../../data/cstring.hpp"
#include "../exception_base.hpp"

namespace dv::exceptions {

namespace info {
struct IOError : EmptyException {};
} // namespace info

using IOError = Exception_<info::IOError>;

namespace info {
struct InputError {
	struct ErrorInfo {
		dv::cstring mName;
		dv::cstring mTypeIdentifier;
	};

	using Info = ErrorInfo;

	static std::string format(const Info &info) {
		return fmt::format("Input: (name: {:s}, type: {:s})", info.mName, info.mTypeIdentifier);
	}
};
} // namespace info

using InputError = Exception_<info::InputError, IOError>;

namespace info {
struct OutputError {
	struct ErrorInfo {
		dv::cstring mName;
		dv::cstring mTypeIdentifier;
	};

	using Info = ErrorInfo;

	static std::string format(const Info &info) {
		return fmt::format("Output: (name: {:s}, type: {:s})", info.mName, info.mTypeIdentifier);
	}
};
} // namespace info

using OutputError = Exception_<info::OutputError, IOError>;

} // namespace dv::exceptions
