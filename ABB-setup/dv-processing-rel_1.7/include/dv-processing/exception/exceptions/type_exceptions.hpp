#pragma once

#include "../../data/cstring.hpp"
#include "../exception_base.hpp"

namespace dv::exceptions {

namespace info {
struct TypeError {
	using Info = dv::cstring;

	static std::string format(const Info &info) {
		return fmt::format("Type: {:s})", info);
	}
};
} // namespace info

using TypeError = Exception_<info::TypeError>;

} // namespace dv::exceptions
