//
// Created by rokas on 11.08.21.
//

#include "../../include/dv-processing/data/depth_event_base.hpp"
#include "../../include/dv-processing/data/event_base.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;

	"depth_event_size_match"_test = [] {
		expect(eq(sizeof(dv::DepthEvent), sizeof(dv::Event)));
	};

	return EXIT_SUCCESS;
}
