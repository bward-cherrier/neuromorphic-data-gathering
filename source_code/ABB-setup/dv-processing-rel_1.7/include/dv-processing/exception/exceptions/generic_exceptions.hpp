#pragma once

#include "../exception_base.hpp"

namespace dv::exceptions {

namespace info {
struct RuntimeError : EmptyException {};
} // namespace info

using RuntimeError = Exception_<info::RuntimeError>;

namespace info {
struct BadAlloc : EmptyException {};
} // namespace info

using BadAlloc = Exception_<info::BadAlloc>;

namespace info {
struct OutOfRange : EmptyException {};
} // namespace info

using OutOfRange = Exception_<info::OutOfRange>;

namespace info {
struct LengthError : EmptyException {};
} // namespace info

using LengthError = Exception_<info::LengthError>;

namespace info {
template<class TYPE>
struct InvalidArgument {
	using Info = TYPE;

	static std::string format(const Info &info) {
		return fmt::format("Value: {}", info);
	}
};
} // namespace info
template<class TYPE>
using InvalidArgument = Exception_<info::InvalidArgument<TYPE>>;

namespace info {
struct NullPointer : EmptyException {};
} // namespace info

using NullPointer = Exception_<info::NullPointer>;

} // namespace dv::exceptions
