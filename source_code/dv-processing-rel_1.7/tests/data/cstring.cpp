#include "../../include/dv-processing/data/cstring.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;

	auto cStr = "12";
	std::string str{"1234"};
	std::string_view strView{"123456"};

	"construct"_test = [&] {
		dv::cstring empty{};
		expect(eq(empty.size(), 0));
		expect(neq(empty.data(), nullptr));

		dv::cstring fromLit{"abc"};
		expect(eq(fromLit.size(), 3));

		dv::cstring fromCStr{cStr};
		expect(eq(fromCStr.size(), 2));

		dv::cstring fromStr{str};
		expect(eq(fromStr.size(), 4));

		dv::cstring fromStrView{strView};
		expect(eq(fromStrView.size(), 6));

		dv::cstring copyCtor{fromStr};
		expect(eq(copyCtor.size(), 4));

		dv::cstring moveCtor{std::move(fromStrView)};
		expect(eq(moveCtor.size(), 6));
	};

	"assign"_test = [&] {
		dv::cstring empty;
		empty.assign("");
		expect(eq(empty.size(), 0));

		dv::cstring fromLit;
		fromLit.assign("abc");
		expect(eq(fromLit.size(), 3));

		dv::cstring fromCStr;
		fromCStr.assign(cStr);
		expect(eq(fromCStr.size(), 2));

		dv::cstring fromStr;
		fromStr.assign(str);
		expect(eq(fromStr.size(), 4));

		dv::cstring fromStrView;
		fromStrView.assign(strView);
		expect(eq(fromStrView.size(), 6));

		dv::cstring copyAssign;
		copyAssign.assign(fromStr);
		expect(eq(copyAssign.size(), 4));

		dv::cstring moveAssign;
		moveAssign.assign(std::move(fromStrView));
		expect(eq(moveAssign.size(), 6));
	};

	"operator="_test = [&] {
		dv::cstring empty;
		empty = "";
		expect(eq(empty.size(), 0));

		dv::cstring fromLit;
		fromLit = "abc";
		expect(eq(fromLit.size(), 3));

		dv::cstring fromCStr;
		fromCStr = cStr;
		expect(eq(fromCStr.size(), 2));

		dv::cstring fromStr;
		fromStr = str;
		expect(eq(fromStr.size(), 4));

		dv::cstring fromStrView;
		fromStrView = strView;
		expect(eq(fromStrView.size(), 6));

		dv::cstring copyAssign;
		copyAssign = fromStr;
		expect(eq(copyAssign.size(), 4));

		dv::cstring moveAssign;
		moveAssign = std::move(fromStrView);
		expect(eq(moveAssign.size(), 6));
	};

	"append"_test = [&] {
		dv::cstring empty{""};
		empty.append("");
		expect(eq(empty.size(), 0));

		dv::cstring fromLit{"abc"};
		fromLit.append("abc");
		expect(eq(fromLit.size(), 6));

		dv::cstring fromCStr{cStr};
		fromCStr.append(cStr);
		expect(eq(fromCStr.size(), 4));

		dv::cstring fromStr{str};
		fromStr.append(str);
		expect(eq(fromStr.size(), 8));

		dv::cstring fromStrView{strView};
		fromStrView.append(strView);
		expect(eq(fromStrView.size(), 12));

		dv::cstring copy{"12"};
		copy.append(fromStr);
		expect(eq(copy.size(), 10));

		dv::cstring move{"1234"};
		move.append(std::move(fromStrView));
		expect(eq(move.size(), 16));
	};

	"operator+="_test = [&] {
		dv::cstring empty{""};
		empty += "";
		expect(eq(empty.size(), 0));

		dv::cstring fromLit{"abc"};
		fromLit += "abc";
		expect(eq(fromLit.size(), 6));

		dv::cstring fromCStr{cStr};
		fromCStr += cStr;
		expect(eq(fromCStr.size(), 4));

		dv::cstring fromStr{str};
		fromStr += str;
		expect(eq(fromStr.size(), 8));

		dv::cstring fromStrView{strView};
		fromStrView += strView;
		expect(eq(fromStrView.size(), 12));

		dv::cstring copy{"12"};
		copy += fromStr;
		expect(eq(copy.size(), 10));

		dv::cstring move{"1234"};
		move += std::move(fromStrView);
		expect(eq(move.size(), 16));
	};

	"operator+"_test = [&] {
		dv::cstring start{"1234"};

		expect(eq((start + "").size(), 4));
		expect(eq(("" + start).size(), 4));

		expect(eq((start + "abc").size(), 7));
		expect(eq(("abc" + start).size(), 7));

		expect(eq((start + cStr).size(), 6));
		expect(eq((cStr + start).size(), 6));

		expect(eq((start + str).size(), 8));
		expect(eq((str + start).size(), 8));

		expect(eq((start + strView).size(), 10));
		expect(eq((strView + start).size(), 10));

		expect(eq((start + start).size(), 8));
	};

	"operator=="_test = [&] {
		dv::cstring left{"1234"};
		dv::cstring right{"5678"};

		expect(eq(left == left, true));
		expect(eq(left == right, false));
		expect(eq(left != left, false));
		expect(eq(left != right, true));

		expect(eq(left == "1234", true));
		expect(eq(left == "5678", false));
		expect(eq(left != "1234", false));
		expect(eq(left != "5678", true));

		expect(eq(left == std::string{"1234"}, true));
		expect(eq(left == std::string{"5678"}, false));
		expect(eq(left != std::string{"1234"}, false));
		expect(eq(left != std::string{"5678"}, true));

		expect(eq(left == std::string_view{"1234"}, true));
		expect(eq(left == std::string_view{"5678"}, false));
		expect(eq(left != std::string_view{"1234"}, false));
		expect(eq(left != std::string_view{"5678"}, true));
	};

	return EXIT_SUCCESS;
}
