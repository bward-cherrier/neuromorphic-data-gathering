#include "../../include/dv-processing/core/event_color.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;

	"basic_colors"_test = [] {
		dv::Event red(0, 0, 0, true);
		dv::Event green1(0, 0, 1, true);
		dv::Event green2(0, 1, 0, true);
		dv::Event blue(0, 1, 1, true);

		expect(eq(dv::colorForEvent(red), dv::EventColor::RED));
		expect(eq(dv::colorForEvent(green1), dv::EventColor::GREEN));
		expect(eq(dv::colorForEvent(blue), dv::EventColor::BLUE));
		expect(eq(dv::colorForEvent(green2), dv::EventColor::GREEN));
	};

	return 0;
}
