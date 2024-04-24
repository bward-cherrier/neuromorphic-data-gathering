#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/core/frame.hpp"
#include "../../include/dv-processing/data/generate.hpp"

#include "boost/ut.hpp"

#include <chrono>

int main() {
	using namespace boost::ut;

	// Generate some data
	int16_t dimension = 100;
	// Setup sample data
	int64_t timestamp = 1e+6;
	dv::EventStore store;
	for (int16_t y = 0; y < dimension; y++) {
		for (int16_t x = 0; x < dimension; x++) {
			store.emplace_back(timestamp, x, y, false);
		}
	}

	static constexpr float epsilon = std::numeric_limits<float>::epsilon();

	"no_decay"_test = [&] {
		dv::Accumulator accumulator(
			cv::Size(100, 100), dv::Accumulator::Decay::NONE, 0.0, true, 0.1f, 1.0f, 0.5f, 0.0f, false);
		dv::Frame frame = accumulator.generateFrame();
		// Mean should be equal to neutral potential
		expect(eq(cv::mean(frame.image)[0], 128._d));

		for (int i = 1; i < 8; i++) {
			accumulator.accumulate(store);
			frame = accumulator.generateFrame();

			// The expected value should not exceed 1.0f
			const float expectedValue = std::max(0.5f - (0.1f * static_cast<float>(i)), 0.f) * 255.f;
			const float meanValue     = static_cast<float>(cv::mean(frame.image)[0]);
			expect(eq(meanValue - expectedValue, 0._f));
		}

		// Reverse polarity
		dv::EventStore positive;
		for (const auto &event : store) {
			positive.emplace_back(event.timestamp(), event.x(), event.y(), true);
		}

		for (int i = 1; i < 15; i++) {
			accumulator.accumulate(positive);
			frame = accumulator.generateFrame();

			// The expected value should not exceed 1.0f
			const float expectedValue = std::min(0.1f * static_cast<float>(i), 1.0f) * 255.f;
			const float meanValue     = static_cast<float>(cv::mean(frame.image)[0]);
			expect(lt(std::abs(meanValue - expectedValue), 1._f));
			expect(eq(frame.timestamp, positive.getLowestTime()));
			expect(eq(frame.timestamp + frame.exposure.count(), positive.getHighestTime()));
		}

		// No decay should happen here
		frame = accumulator.generateFrame();
		expect(eq(cv::mean(frame.image)[0], 255._d));
		frame = accumulator.generateFrame();
		expect(eq(cv::mean(frame.image)[0], 255._d));
	};

	"exponential_decay"_test = [&] {
		dv::Accumulator accumulator(
			cv::Size(100, 100), dv::Accumulator::Decay::EXPONENTIAL, 1.0e+6, true, 1.0f, 1.0f, 0.5f, 0.0f, false);

		dv::Frame frame = accumulator.generateFrame();
		// Mean should be equal to neutral potential

		expect(eq(static_cast<float>(cv::mean(frame.image)[0]), 128._f));
		accumulator.accumulate(store);
		frame            = accumulator.generateFrame();
		double lastValue = cv::mean(frame.image)[0];

		for (int i = 1; i < 10; i++) {
			// Reverse polarity
			dv::EventStore positive;
			// We need at least one event with next timestamp to trigger decay
			positive.emplace_back(timestamp + (i * 1000000), static_cast<int16_t>(0), static_cast<int16_t>(0), false);
			accumulator.accumulate(positive);
			frame = accumulator.generateFrame();

			// The expected value should not exceed 1.0f
			double currentValue = cv::mean(frame.image)[0];
			expect(gt(currentValue, lastValue)) << "Value increases to the neutral potential";
			expect(eq(frame.timestamp, positive.getLowestTime()));
			expect(eq(frame.timestamp + frame.exposure.count(), positive.getHighestTime()));
		}
		// The mean value should be quite close to 0.5 neutral value, which is 128 in 8-bit unsigned representation
		const double meanValue = cv::mean(frame.image)[0];
		expect(ge(meanValue, 126._d));
		expect(le(meanValue, 129._d));
	};

	"linear_decay"_test = [&] {
		dv::Accumulator accumulator(
			cv::Size(100, 100), dv::Accumulator::Decay::LINEAR, 1.0e-6, true, 1.0f, 1.0f, 0.5f, 0.0f, false);

		dv::Frame frame = accumulator.generateFrame();
		// Mean should be equal to neutral potential

		expect(eq(cv::mean(frame.image)[0], 128._f));
		accumulator.accumulate(store);
		frame            = accumulator.generateFrame();
		double lastValue = cv::mean(frame.image)[0];

		for (int i = 1; i < 10; i++) {
			// Reverse polarity
			dv::EventStore positive;
			// We need at least one event with next timestamp to trigger decay
			positive.emplace_back(timestamp + (i * 1000000), static_cast<int16_t>(0), static_cast<int16_t>(0), false);
			accumulator.accumulate(positive);
			frame = accumulator.generateFrame();

			// The expected value should not exceed 1.0f
			double currentValue = cv::mean(frame.image)[0];
			expect(gt(currentValue, lastValue)) << "Value increases to the neutral potential";
			expect(eq(frame.timestamp, positive.getLowestTime()));
			expect(eq(frame.timestamp + frame.exposure.count(), positive.getHighestTime()));
		}
		// The mean value should be quite close to 0.5 neutral value
		const double meanValue = cv::mean(frame.image)[0];
		expect(ge(meanValue, 126._d));
		expect(le(meanValue, 129._d));
	};

	"accumulator_neutral_potential_setting"_test = [&] {
		dv::Accumulator acc(cv::Size(1, 1));
		acc.setNeutralPotential(0.5f);
		acc.setMaxPotential(1.f);
		acc.setEventContribution(0.f);

		auto frame = acc.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 128));

		dv::EventStore events;
		events.emplace_back(1000, 0, 0, true);

		// With zero contribution the image should remain the same
		acc.accept(events);
		frame = acc.generateFrame();
		expect(eq(frame.image.at<uint8_t>(0, 0), 128));
	};

	"accumulator_regeneration"_test = [] {
		cv::Size resolution(100, 100);
		dv::Accumulator acc(resolution);

		auto frame = acc.generateFrame();
		expect(eq(frame.timestamp, -1));

		dv::EventStore events = dv::data::generate::eventTestSet(1000, resolution);

		acc.accept(events);
		frame = acc.generateFrame();
		// Generated frame has correct timestamp
		expect(eq(frame.timestamp, 1000));
		// Subsequent calls doesn't change anything, since generated frame is exactly the same
		frame = acc.generateFrame();
		expect(eq(frame.timestamp, 1000));
		// Accept empty frame should not affect anything
		acc.accept(dv::EventStore());
		frame = acc.generateFrame();
		expect(eq(frame.timestamp, 1000));
	};

	"accumulator_step_regenerate"_test = [] {
		cv::Size resolution(100, 100);
		dv::Accumulator acc(resolution, dv::Accumulator::Decay::STEP);

		auto frame = acc.generateFrame();
		expect(eq(frame.timestamp, -1));

		dv::EventStore events = dv::data::generate::eventTestSet(1000, resolution);

		// Pass events
		acc.accept(events);
		frame = acc.generateFrame();
		// Generated image contains according timestamp
		expect(eq(frame.timestamp, 1000));
		frame = acc.generateFrame();
		// Subsequent calls to generate frame results in erroneous timestamp
		expect(eq(frame.timestamp, -1));
		// Empty packet shouldn't affect
		acc.accept(dv::EventStore());
		frame = acc.generateFrame();
		expect(eq(frame.timestamp, -1));
		acc.accept(events);
		frame = acc.generateFrame();
		// Passing new events, timestamp is now correct again
		expect(eq(frame.timestamp, 1000));
	};

	return EXIT_SUCCESS;
}
