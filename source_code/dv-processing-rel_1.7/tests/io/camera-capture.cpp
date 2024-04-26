#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/io/camera_capture.hpp"

#include "boost/ut.hpp"

#include <chrono>

int main() {
	using namespace boost::ut;
	using namespace std::chrono_literals;
	using CameraCapture = dv::io::CameraCapture;

	"simple_open"_test = [] {
		expect(throws([] {
			// Impossible config: dvxplorer name with DAVIS, this combination should always throw exception
			CameraCapture capture("DVXplorer_DXA00080", CameraCapture::CameraType::DAVIS);
		}));
	};

	"davis_tests"_test = [] {
		try {
			// This should open up any DAVIS camera connected to the system
			CameraCapture capture("", CameraCapture::CameraType::DAVIS);

			// Beyond this point, no function call should throw any exceptions
			expect(nothrow([&capture] {
				// Force a short exposure duration so we can expect APS triggers to be received.
				capture.setDavisExposureDuration(1ms);

				// These should safely fail
				expect(eq(capture.setDVSBiasSensitivity(CameraCapture::BiasSensitivity::VeryHigh), false));
				expect(eq(capture.setDVSGlobalHold(false), false));
				expect(eq(capture.setDVXplorerGlobalReset(true), false));
				expect(eq(capture.setDVXplorerEFPS(CameraCapture::DVXeFPS::EFPS_VARIABLE_2000), false));

				// These should work fine
				expect(eq(capture.setDavisReadoutMode(CameraCapture::DavisReadoutMode::EventsAndFrames), true));
				expect(eq(capture.setDavisExposureDuration(dv::Duration(10'000)), true));

				expect(capture.isFrameStreamAvailable());
				expect(capture.isImuStreamAvailable());
				expect(capture.isTriggerStreamAvailable());

				std::this_thread::sleep_for(500ms);

				expect(capture.getNextFrame().has_value());

				expect(nothrow([&capture] {
					expect(!capture.getNextTriggerBatch().value().empty());
				}));
				expect(capture.getNextEventBatch().has_value());
				expect(capture.getNextImuBatch().has_value());

				const auto pitch = capture.getPixelPitch();
				expect(pitch.has_value());
				expect(eq(pitch.value(), 0.0000185_f));

				// Disable auto exposure to avoid any interference with that
				capture.setDavisExposureDuration(10ms);
				expect(eq(capture.getDavisExposureDuration()->count(), 10'000));

				// Set a high frame interval
				capture.setDavisFrameInterval(100ms);
				expect(eq(capture.getDavisFrameInterval()->count(), 100'000));

				// Let everything settle in a bit
				std::this_thread::sleep_for(200ms);

				// Read any frames from within the buffer to drop them
				while (const auto frame = capture.getNextFrame()) {
				}

				// Let's collect 3 frames, these must be guaranteed to be fresh
				std::vector<dv::Frame> frames;
				while (frames.size() < 3) {
					if (const auto nextFrame = capture.getNextFrame(); nextFrame.has_value()) {
						frames.push_back(*nextFrame);
					}
				}

				// Timestamp differences should be very close to the frame interval
				expect(eq(frames.size(), 3));
				expect(gt(frames[1].timestamp - frames[0].timestamp, 98'000));
				expect(lt(frames[1].timestamp - frames[0].timestamp, 102'000));
				expect(gt(frames[2].timestamp - frames[1].timestamp, 98'000));
				expect(lt(frames[2].timestamp - frames[1].timestamp, 102'000));
			}));
		}
		catch (std::runtime_error &err) {
			// Camera is not connected, we skip any tests
		}
	};

	"dvxplorer_tests"_test = [] {
		try {
			// This should open up any DAVIS camera connected to the system
			CameraCapture capture("", CameraCapture::CameraType::DVS);

			// Beyond this point, no function call should throw any exceptions
			expect(nothrow([&capture] {
				// These should work fine
				expect(eq(capture.setDVSBiasSensitivity(CameraCapture::BiasSensitivity::VeryHigh), true));
				expect(eq(capture.setDVSGlobalHold(false), true));
				expect(eq(capture.setDVXplorerGlobalReset(true), true));
				expect(eq(capture.setDVXplorerEFPS(CameraCapture::DVXeFPS::EFPS_VARIABLE_2000), true));

				// These should safely fail
				expect(eq(capture.setDavisReadoutMode(CameraCapture::DavisReadoutMode::EventsAndFrames), false));
				expect(eq(capture.setDavisExposureDuration(dv::Duration(10'000)), false));
				expect(eq(capture.enableDavisAutoExposure(), false));

				expect(!capture.isFrameStreamAvailable());
				expect(capture.isImuStreamAvailable());
				expect(capture.isTriggerStreamAvailable());

				std::this_thread::sleep_for(1s);

				expect(!capture.getNextFrame().has_value());
				expect(capture.getNextEventBatch().has_value());
				expect(capture.getNextImuBatch().has_value());
				const auto pitch = capture.getPixelPitch();
				expect(pitch.has_value());
				expect(eq(pitch.value(), 0.000009_f));
			}));
		}
		catch (std::runtime_error &err) {
			// Camera is not connected, we skip any tests
		}
	};

	"handler"_test = [] {
		try {
			using namespace std::chrono_literals;
			CameraCapture capture;

			// Beyond this point, no function call should throw any exceptions
			expect(nothrow([&capture] {
				capture.setDavisExposureDuration(10ms);

				// These should safely fail
				dv::io::DataReadHandler handler;

				bool imuDataReceived   = false;
				bool eventDataReceived = false;
				bool frameDataReceived = false;

				handler.mImuHandler = [&imuDataReceived](const auto &) {
					imuDataReceived = true;
				};

				handler.mEventHandler = [&eventDataReceived](const auto &) {
					eventDataReceived = true;
				};

				if (capture.isFrameStreamAvailable()) {
					handler.mFrameHandler = [&frameDataReceived](const auto &) {
						frameDataReceived = true;
					};
				}
				else {
					// Cannot test this one
					frameDataReceived = true;
				}

				int64_t start = dv::now();

				// Read until imu was received, or it takes more than 100ms
				while (!imuDataReceived || !frameDataReceived || !eventDataReceived) {
					if (!capture.handleNext(handler)) {
						break;
					}
					// There should be at least one imu measurement over the period
					if (dv::Duration(dv::now() - start) > 5000ms) {
						break;
					}
				}
				expect(imuDataReceived);
				expect(frameDataReceived);
				expect(eventDataReceived);
			}));
		}
		catch (std::runtime_error &err) {
			// Camera is not connected, we skip any tests
		}
	};

	"timestamp_offset_settings"_test = [] {
		try {
			// This should open up any camera connected to the system
			CameraCapture capture;
			expect(nothrow([&capture] {
				// We should be able to read some sane data
				dv::EventStore store;
				do {
					const auto events = capture.getNextEventBatch();
					if (events.has_value()) {
						store = events.value();
					}
				}
				while (store.isEmpty());

				// A timestamp offset should be non-zero after receiving first data packet
				expect(gt(capture.getTimestampOffset(), 0));

				// Some sleep to increase the possibility of data in the buffers
				std::this_thread::sleep_for(100ms);

				// Set the offset to a zero value, this should cause any buffered unread data to be cleared
				capture.setTimestampOffset(0);
				expect(eq(capture.getTimestampOffset(), 0));

				// Read some more events
				dv::EventStore noOffsetEvents;
				do {
					const auto events = capture.getNextEventBatch();
					if (events.has_value()) {
						noOffsetEvents = events.value();
					}
				}
				while (noOffsetEvents.isEmpty());

				// What we expect now that the no-offset events timestamp should be counting from 0, where 0 should
				// be the time when the camera was opened, so the values should be above zero, but strictly
				// below the timestamp in the first received packet.
				expect(gt(noOffsetEvents.getLowestTime(), 0));
				expect(lt(noOffsetEvents.getHighestTime(), store.getLowestTime()));
			}));
		}
		catch (std::runtime_error &err) {
			// Camera is not connected, we skip any tests
		}
	};

	return EXIT_SUCCESS;
}
