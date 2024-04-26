#include "../../include/dv-processing/core/frame.hpp"
#include "../../include/dv-processing/data/generate.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;

	"pixel_accumulator"_test = [&] {
		cv::Size resolution(100, 100);
		dv::EdgeMapAccumulator accumulator(resolution);
		accumulator.accumulate(dv::data::generate::eventTestSet(0, resolution));
		dv::Frame frame = accumulator.generateFrame();
		expect(lt(cv::mean(frame.image)[0] - 64., 0.01));
		expect(eq(frame.timestamp, 0));
		expect(eq(frame.exposure.count(), 0));
		accumulator.setEventContribution(0.1f);
		accumulator.accumulate(dv::data::generate::eventTestSet(1'000, resolution));
		frame = accumulator.generateFrame();
		expect(eq(frame.timestamp, 1'000));
		expect(eq(frame.exposure.count(), 0));
		expect(lt(cv::mean(frame.image)[0] - 26., 0.01));
		accumulator.setEventContribution(0.5f);
		accumulator.accumulate(dv::data::generate::eventTestSet(2'000, resolution));
		frame = accumulator.generateFrame();
		expect(eq(frame.timestamp, 2'000));
		expect(eq(frame.exposure.count(), 0));
		expect(lt(cv::mean(frame.image)[0] - 128., 0.01));
		// Now we have 3 events in each pixel, let's test overflow
		const dv::EventStore multipleEvents = dv::data::generate::eventTestSet(0, resolution)
											+ dv::data::generate::eventTestSet(1'000, resolution)
											+ dv::data::generate::eventTestSet(2'000, resolution);
		accumulator.setEventContribution(0.5f);
		expect(eq(accumulator.getEventContribution(), 0.5f));
		accumulator.accumulate(multipleEvents);
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(multipleEvents[0].y(), multipleEvents[0].x()), 255));
		expect(eq(frame.timestamp, 0));
		expect(eq(frame.exposure.count(), 2'000));
	};

	"pixel_accumulator_bad_config"_test = [] {
		dv::EdgeMapAccumulator accumulator(cv::Size(100, 100), 0.1f);
		expect(throws([&] {
			accumulator.setEventContribution(1.1f);
		}));
		expect(throws([&] {
			accumulator.setEventContribution(-0.1f);
		}));
	};

	"pixel_accumulator_overflow_bug"_test = [] {
		dv::EdgeMapAccumulator accumulator(cv::Size(100, 100), 0.3f);

		dv::EventStore singlePixelEvents;
		singlePixelEvents.emplace_back(0LL, static_cast<int16_t>(0), static_cast<int16_t>(0), true);

		accumulator.accumulate(singlePixelEvents);
		dv::Frame frame = accumulator.generateFrame();
		expect(lt(static_cast<int>(frame.image.at<uint8_t>(0LL, 0)), 255));

		singlePixelEvents.emplace_back(1LL, static_cast<int16_t>(0), static_cast<int16_t>(0), true);
		accumulator.accumulate(singlePixelEvents);
		frame = accumulator.generateFrame();
		expect(lt(static_cast<int>(frame.image.at<uint8_t>(0, 0)), 255));

		singlePixelEvents.emplace_back(2LL, static_cast<int16_t>(0), static_cast<int16_t>(0), true);
		accumulator.accumulate(singlePixelEvents);
		frame = accumulator.generateFrame();
		expect(lt(static_cast<int>(frame.image.at<uint8_t>(0, 0)), 255));

		singlePixelEvents.emplace_back(3LL, static_cast<int16_t>(0), static_cast<int16_t>(0), true);
		accumulator.accumulate(singlePixelEvents);
		frame = accumulator.generateFrame();
		expect(eq(static_cast<int>(frame.image.at<uint8_t>(0, 0)), 255));
	};

	"pixel_accumulator_polarity"_test = [] {
		cv::Size resolution(3, 2);
		// Contribution is specifically higher than possible values, to test the range clamping
		dv::EdgeMapAccumulator accumulator(resolution, 0.66f, false, 0.5f);

		dv::EventStore events;
		events.push_back(dv::Event(1, 0, 0, true));
		events.push_back(dv::Event(1, 0, 1, true));
		events.push_back(dv::Event(1, 2, 0, false));
		events.push_back(dv::Event(1, 2, 1, false));
		accumulator.accumulate(events);

		dv::Frame frame = accumulator.generateFrame();

		expect(eq(frame.image.at<uint8_t>(0, 0), 255));
		expect(eq(frame.image.at<uint8_t>(1, 0), 255));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 0));
		expect(eq(frame.image.at<uint8_t>(1, 2), 0));
		expect(eq(frame.timestamp, 1));
		expect(eq(frame.exposure.count(), 0));

		accumulator.setEventContribution(0.25);
		accumulator.accumulate(events);

		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 191));
		expect(eq(frame.image.at<uint8_t>(1, 0), 191));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 63));
		expect(eq(frame.image.at<uint8_t>(1, 2), 63));

		accumulator.setNeutralPotential(0.f);
		accumulator.accumulate(events);

		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 64));
		expect(eq(frame.image.at<uint8_t>(1, 0), 64));
		expect(eq(frame.image.at<uint8_t>(0, 1), 0));
		expect(eq(frame.image.at<uint8_t>(1, 1), 0));
		expect(eq(frame.image.at<uint8_t>(0, 2), 0));
		expect(eq(frame.image.at<uint8_t>(1, 2), 0));

		accumulator.setNeutralPotential(1.f);
		accumulator.accumulate(events);

		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 255));
		expect(eq(frame.image.at<uint8_t>(1, 0), 255));
		expect(eq(frame.image.at<uint8_t>(0, 1), 255));
		expect(eq(frame.image.at<uint8_t>(1, 1), 255));
		expect(eq(frame.image.at<uint8_t>(0, 2), 191));
		expect(eq(frame.image.at<uint8_t>(1, 2), 191));

		accumulator.setNeutralPotential(0.5);
		// Double the events
		events.add(events);
		accumulator.accumulate(events);

		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 255));
		expect(eq(frame.image.at<uint8_t>(1, 0), 255));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 0));
		expect(eq(frame.image.at<uint8_t>(1, 2), 0));
	};

	"pixel_accumulator_decay"_test = [] {
		cv::Size resolution(3, 2);
		// zero decay first
		dv::EdgeMapAccumulator accumulator(resolution, 0.25f, false, 0.5f, 0.0f);

		dv::EventStore events;
		events.push_back(dv::Event(1, 0, 0, true));
		events.push_back(dv::Event(1, 0, 1, true));
		events.push_back(dv::Event(1, 2, 0, false));
		events.push_back(dv::Event(1, 2, 1, false));
		accumulator.accumulate(events);

		auto frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 191));
		expect(eq(frame.image.at<uint8_t>(1, 0), 191));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 63));
		expect(eq(frame.image.at<uint8_t>(1, 2), 63));

		accumulator.accumulate(dv::EventStore());

		// This should not affect the values, since decay is 0
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 191));
		expect(eq(frame.image.at<uint8_t>(1, 0), 191));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 63));
		expect(eq(frame.image.at<uint8_t>(1, 2), 63));

		// Decay byte value should be 12
		accumulator.setDecay(0.05f);

		// This should force everything back to neutral
		accumulator.reset();
		accumulator.accumulate(dv::EventStore());
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 127));
		expect(eq(frame.image.at<uint8_t>(1, 0), 127));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 127));
		expect(eq(frame.image.at<uint8_t>(1, 2), 127));

		accumulator.accumulate(events);
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 191));
		expect(eq(frame.image.at<uint8_t>(1, 0), 191));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 63));
		expect(eq(frame.image.at<uint8_t>(1, 2), 63));

		// No events, only decay applies
		accumulator.accumulate(dv::EventStore());
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 179));
		expect(eq(frame.image.at<uint8_t>(1, 0), 179));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 75));
		expect(eq(frame.image.at<uint8_t>(1, 2), 75));

		accumulator.accumulate(dv::EventStore());
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 167));
		expect(eq(frame.image.at<uint8_t>(1, 0), 167));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 87));
		expect(eq(frame.image.at<uint8_t>(1, 2), 87));

		accumulator.accumulate(dv::EventStore());
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 155));
		expect(eq(frame.image.at<uint8_t>(1, 0), 155));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 99));
		expect(eq(frame.image.at<uint8_t>(1, 2), 99));

		accumulator.accumulate(dv::EventStore());
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 143));
		expect(eq(frame.image.at<uint8_t>(1, 0), 143));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 111));
		expect(eq(frame.image.at<uint8_t>(1, 2), 111));

		accumulator.accumulate(dv::EventStore());
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 131));
		expect(eq(frame.image.at<uint8_t>(1, 0), 131));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 123));
		expect(eq(frame.image.at<uint8_t>(1, 2), 123));

		accumulator.accumulate(dv::EventStore());
		frame = accumulator.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 127));
		expect(eq(frame.image.at<uint8_t>(1, 0), 127));
		expect(eq(frame.image.at<uint8_t>(0, 1), 127));
		expect(eq(frame.image.at<uint8_t>(1, 1), 127));
		expect(eq(frame.image.at<uint8_t>(0, 2), 127));
		expect(eq(frame.image.at<uint8_t>(1, 2), 127));
	};

	"pixel_accumulator_decay_greater_than_contribution"_test = [] {
		cv::Size resolution(3, 2);
		// setting up Decay > Contribution
		dv::EdgeMapAccumulator accumulator(resolution, 0.01f, true, 0.0f, 0.05f);

		dv::EventStore events;
		events.push_back(dv::Event(1, 0, 0, true));
		events.push_back(dv::Event(1, 0, 1, true));
		events.push_back(dv::Event(1, 2, 0, false));
		events.push_back(dv::Event(1, 2, 1, false));
		accumulator.accumulate(events);
		auto frame = accumulator.generateFrame();

		// The values should have some contribution
		expect(eq(frame.image.at<uint8_t>(0, 0), 3));
		expect(eq(frame.image.at<uint8_t>(1, 0), 3));
		expect(eq(frame.image.at<uint8_t>(0, 1), 0));
		expect(eq(frame.image.at<uint8_t>(1, 1), 0));
		expect(eq(frame.image.at<uint8_t>(0, 2), 3));
		expect(eq(frame.image.at<uint8_t>(1, 2), 3));

		frame = accumulator.generateFrame();

		// Whatever the previous value, everything should be zero because we have greater decay than contribution
		expect(eq(frame.image.at<uint8_t>(0, 0), 0));
		expect(eq(frame.image.at<uint8_t>(1, 0), 0));
		expect(eq(frame.image.at<uint8_t>(0, 1), 0));
		expect(eq(frame.image.at<uint8_t>(1, 1), 0));
		expect(eq(frame.image.at<uint8_t>(0, 2), 0));
		expect(eq(frame.image.at<uint8_t>(1, 2), 0));
	};

	"frame_timestamping"_test = [] {
		dv::EdgeMapAccumulator accumulator(cv::Size(1, 1));

		dv::EventStore events;
		events.push_back(dv::Event(1000, 0, 0, true));

		dv::Frame frame = accumulator.generateFrame();
		expect(eq(frame.timestamp, -1));

		accumulator.accept(events);
		frame = accumulator.generateFrame();
		expect(eq(frame.timestamp, 1000));
		frame = accumulator.generateFrame();
		expect(eq(frame.timestamp, -1));
		frame = accumulator.generateFrame();
		expect(eq(frame.timestamp, -1));
		accumulator.accept(dv::EventStore());
		frame = accumulator.generateFrame();
		expect(eq(frame.timestamp, -1));
		expect(eq(accumulator.generateFrame().timestamp, -1));
		accumulator.accept(events);
		expect(eq(accumulator.generateFrame().timestamp, 1000));
	};

	return EXIT_SUCCESS;
}
