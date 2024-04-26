#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/io/network_reader.hpp"
#include "../../include/dv-processing/io/network_writer.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

int main() {
	using namespace std::chrono_literals;
	namespace fs = std::filesystem;

	"basic_startup_shutdown"_test = [] {
		expect(nothrow([] {
			const dv::io::NetworkWriter writer(
				"127.0.0.1", 10001, dv::io::Stream::EventStream(0, "events", "DVXplorer_123", cv::Size(640, 480)));

			std::this_thread::sleep_for(100ms);
		}));

		std::this_thread::sleep_for(200ms);
	};

	"single_client_frames"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution] {
			expect(nothrow([&shutdown, &resolution] {
				dv::io::NetworkWriter writer(
					"127.0.0.1", 10001, dv::io::Stream::FrameStream(0, "frames", "DVXplorer_123", resolution));

				while (!shutdown) {
					writer.writeFrame(dv::Frame(0, cv::Mat(resolution, CV_8UC3, cv::Scalar(255, 255, 255))));
					std::this_thread::sleep_for(50ms);
				}
			}));
		});

		std::this_thread::sleep_for(200ms);

		expect(nothrow([] {
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(reader.isFrameStreamAvailable());
		}));

		shutdown = true;
		writerThread.join();
	};

	"single_client_events"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution] {
			expect(nothrow([&shutdown, &resolution] {
				dv::io::NetworkWriter writer(
					"127.0.0.1", 10001, dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution));

				int64_t timestamp = 10'000;
				while (!shutdown) {
					dv::EventPacket events;
					events.elements.emplace_back(timestamp, 1, 2, false);
					writer.writePacket(std::move(events));
					std::this_thread::sleep_for(50ms);
				}
			}));
		});

		std::this_thread::sleep_for(200ms);

		{
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, resolution.width));
			expect(eq(eventResolution.height, resolution.height));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				std::this_thread::sleep_for(50ms);
				retries++;
			}

			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		}

		shutdown = true;
		writerThread.join();
	};

	"multi_client_events"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution] {
			expect(nothrow([&shutdown, &resolution] {
				dv::io::NetworkWriter writer(
					"127.0.0.1", 10001, dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution));

				int64_t timestamp = 10'000;
				while (!shutdown) {
					dv::EventStore events;
					events.emplace_back(timestamp, 1, 2, false);
					writer.writeEvents(events);
					std::this_thread::sleep_for(50ms);
				}
			}));
		});

		std::this_thread::sleep_for(200ms);

		std::function readerCallback = [] {
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, 100));
			expect(eq(eventResolution.height, 100));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				std::this_thread::sleep_for(50ms);
				retries++;
			}

			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		};

		std::thread thread1(readerCallback);
		std::thread thread2(readerCallback);
		std::thread thread3(readerCallback);

		thread3.join();
		thread2.join();
		thread1.join();

		shutdown = true;
		writerThread.join();
	};

	"single_client_frames"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution] {
			expect(nothrow([&shutdown, &resolution] {
				dv::io::NetworkWriter writer(
					"127.0.0.1", 10001, dv::io::Stream::FrameStream(0, "frames", "DVXplorer_123", resolution));

				while (!shutdown) {
					writer.writeFrame(dv::Frame(dv::now(), cv::Mat::ones(resolution, CV_8UC1)));
					std::this_thread::sleep_for(50ms);
				}
			}));
		});

		std::this_thread::sleep_for(200ms);

		expect(nothrow([] {
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(!reader.isEventStreamAvailable());
			expect(reader.isFrameStreamAvailable());
			const auto streamResolution = reader.getFrameResolution().value();

			expect(eq(streamResolution.width, 100));
			expect(eq(streamResolution.height, 100));

			dv::Frame receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto frame = reader.getNextFrame()) {
					receivedData = *frame;
					break;
				}
				std::this_thread::sleep_for(50ms);
				retries++;
			}

			expect(eq(receivedData.image.rows, 100));
			expect(eq(receivedData.image.cols, 100));
			expect(eq(cv::sum(receivedData.image)[0], 100.0 * 100.0));
		}));

		shutdown = true;
		writerThread.join();
	};

	"single_client_imu"_test = [] {
		std::atomic<bool> shutdown{false};

		std::thread writerThread([&shutdown] {
			expect(nothrow([&shutdown] {
				dv::io::NetworkWriter writer("127.0.0.1", 10001, dv::io::Stream::IMUStream(0, "imu", "DVXplorer_123"));

				dv::cvector<dv::IMU> imu;
				imu.emplace_back(1'000, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 10.f);
				while (!shutdown) {
					writer.writeIMU(imu);
					std::this_thread::sleep_for(50ms);
				}
			}));
		});

		std::this_thread::sleep_for(200ms);

		expect(nothrow([] {
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(!reader.isEventStreamAvailable());
			expect(!reader.isFrameStreamAvailable());
			expect(reader.isImuStreamAvailable());
			expect(!reader.isTriggerStreamAvailable());
			expect(!reader.getFrameResolution().has_value());
			expect(!reader.getEventResolution().has_value());

			dv::cvector<dv::IMU> receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto imu = reader.getNextImuBatch()) {
					receivedData = *imu;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}

			expect(!receivedData.empty());
			expect(eq(receivedData.size(), 1));
			if (receivedData.size() == 1) {
				expect(eq(receivedData[0].timestamp, 1'000LL));

				expect(eq(receivedData.front().temperature, 1.0_f));
				expect(eq(receivedData.front().accelerometerX, 2.0_f));
				expect(eq(receivedData.front().accelerometerY, 3.0_f));
				expect(eq(receivedData.front().accelerometerZ, 4.0_f));
				expect(eq(receivedData.front().gyroscopeX, 5.0_f));
				expect(eq(receivedData.front().gyroscopeY, 6.0_f));
				expect(eq(receivedData.front().gyroscopeZ, 7.0_f));
				expect(eq(receivedData.front().magnetometerX, 8.0_f));
				expect(eq(receivedData.front().magnetometerY, 9.0_f));
				expect(eq(receivedData.front().magnetometerZ, 10.0_f));
			}
		}));

		shutdown = true;
		writerThread.join();
	};

	"single_client_triggers"_test = [] {
		std::atomic<bool> shutdown{false};

		std::thread writerThread([&shutdown] {
			expect(nothrow([&shutdown] {
				dv::io::NetworkWriter writer(
					"127.0.0.1", 10001, dv::io::Stream::TriggerStream(0, "frames", "DVXplorer_123"));

				dv::cvector<dv::Trigger> triggers;
				triggers.emplace_back(0, dv::TriggerType::APS_EXPOSURE_START);
				triggers.emplace_back(0, dv::TriggerType::EXTERNAL_SIGNAL_PULSE);
				triggers.emplace_back(0, dv::TriggerType::EXTERNAL_SIGNAL_FALLING_EDGE);
				while (!shutdown) {
					writer.writeTriggers(triggers);
					std::this_thread::sleep_for(50ms);
				}
			}));
		});

		std::this_thread::sleep_for(200ms);

		expect(nothrow([] {
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(!reader.isEventStreamAvailable());
			expect(!reader.isFrameStreamAvailable());
			expect(!reader.isImuStreamAvailable());
			expect(reader.isTriggerStreamAvailable());
			expect(!reader.getFrameResolution().has_value());
			expect(!reader.getEventResolution().has_value());

			dv::cvector<dv::Trigger> receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto triggers = reader.getNextTriggerBatch()) {
					receivedData = *triggers;
					break;
				}
				std::this_thread::sleep_for(50ms);
				retries++;
			}

			expect(eq(receivedData.size(), 3));
			if (receivedData.size() == 3) {
				expect(eq(receivedData[0].timestamp, 0));
				expect(eq(receivedData[1].timestamp, 0));
				expect(eq(receivedData[2].timestamp, 0));

				expect(receivedData[0].type == dv::TriggerType::APS_EXPOSURE_START);
				expect(receivedData[1].type == dv::TriggerType::EXTERNAL_SIGNAL_PULSE);
				expect(receivedData[2].type == dv::TriggerType::EXTERNAL_SIGNAL_FALLING_EDGE);
			}
		}));

		shutdown = true;
		writerThread.join();
	};

	"single_client_custom_type"_test = [] {
		std::atomic<bool> shutdown{false};

		std::thread writerThread([&shutdown] {
			expect(nothrow([&shutdown] {
				dv::io::NetworkWriter writer("127.0.0.1", 10001,
					dv::io::Stream::TypedStream<dv::TimedKeyPointPacket>(0, "keypoints", "DVXplorer_123"));

				dv::TimedKeyPointPacket keypoints;
				keypoints.elements.emplace_back(dv::Point2f(1.f, 2.f), 1.f, 2.f, 3.f, 4, 5, 1'000);
				keypoints.elements.emplace_back(dv::Point2f(2.f, 4.f), 2.f, 4.f, 6.f, 8, 10, 10'000);
				while (!shutdown) {
					writer.writePacket(keypoints);
					std::this_thread::sleep_for(50ms);
				}
			}));
		});

		std::this_thread::sleep_for(200ms);

		expect(nothrow([&shutdown] {
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(!reader.isEventStreamAvailable());
			expect(!reader.isFrameStreamAvailable());
			expect(!reader.isImuStreamAvailable());
			expect(!reader.isTriggerStreamAvailable());
			expect(!reader.getFrameResolution().has_value());
			expect(!reader.getEventResolution().has_value());
			expect(reader.isStreamAvailable<dv::TimedKeyPointPacket>());

			dv::TimedKeyPointPacket receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto keypoints = reader.getNextPacket<dv::TimedKeyPointPacket>()) {
					receivedData = *keypoints;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}

			expect(!receivedData.elements.empty());
			expect(eq(receivedData.elements.size(), 2));
			if (receivedData.elements.size() == 2) {
				expect(eq(receivedData.elements[0].timestamp, 1'000LL));
				expect(eq(receivedData.elements[0].pt.x(), 1.0_f));
				expect(eq(receivedData.elements[0].pt.y(), 2.0_f));
				expect(eq(receivedData.elements[0].size, 1.0_f));
				expect(eq(receivedData.elements[0].angle, 2.0_f));
				expect(eq(receivedData.elements[0].response, 3.0_f));
				expect(eq(receivedData.elements[0].octave, 4));
				expect(eq(receivedData.elements[0].class_id, 5));

				expect(eq(receivedData.elements[1].timestamp, 10'000LL));
				expect(eq(receivedData.elements[1].pt.x(), 2.0_f));
				expect(eq(receivedData.elements[1].pt.y(), 4.0_f));
				expect(eq(receivedData.elements[1].size, 2.0_f));
				expect(eq(receivedData.elements[1].angle, 4.0_f));
				expect(eq(receivedData.elements[1].response, 6.0_f));
				expect(eq(receivedData.elements[1].octave, 8));
				expect(eq(receivedData.elements[1].class_id, 10));
			}
		}));

		shutdown = true;
		writerThread.join();
	};

	"unix_client_events"_test = [] {
		std::atomic<bool> shutdown{false};

		const fs::path socketPath = fs::temp_directory_path() / "socket-0";

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution, &socketPath] {
			dv::io::NetworkWriter writer(
				socketPath, dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution));

			int64_t timestamp = 10'000;
			while (!shutdown) {
				dv::EventStore events;
				events.emplace_back(timestamp, 1, 2, false);
				writer.writeEvents(events);
				std::this_thread::sleep_for(50ms);
			}
		});

		std::this_thread::sleep_for(200ms);
		expect(fs::exists(socketPath));

		{
			dv::io::NetworkReader reader(socketPath);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, resolution.width));
			expect(eq(eventResolution.height, resolution.height));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}

			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		}

		shutdown = true;
		if (writerThread.joinable()) {
			writerThread.join();
		}
		expect(!fs::exists(socketPath));
	};

	"multi_unix_clients"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);
		const fs::path socketPath = fs::temp_directory_path() / "socket-0";

		std::thread writerThread([&shutdown, &resolution, &socketPath] {
			dv::io::NetworkWriter writer(
				socketPath, dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution));

			int64_t timestamp = 10'000;
			while (!shutdown) {
				dv::EventStore events;
				events.emplace_back(timestamp, 1, 2, false);
				writer.writeEvents(events);
				std::this_thread::sleep_for(50ms);
			}
		});

		std::this_thread::sleep_for(200ms);
		expect(fs::exists(socketPath));

		std::function readerCallback = [&socketPath] {
			dv::io::NetworkReader reader(socketPath);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, 100));
			expect(eq(eventResolution.height, 100));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}

			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		};

		std::thread thread1(readerCallback);
		std::thread thread2(readerCallback);
		std::thread thread3(readerCallback);

		thread3.join();
		thread2.join();
		thread1.join();

		shutdown = true;
		if (writerThread.joinable()) {
			writerThread.join();
		}
		expect(!fs::exists(socketPath));
	};

	"multi_unix_server"_test = [] {
		std::atomic<bool> shutdown{false};
		const fs::path socketPath = fs::temp_directory_path() / "socket-0";

		std::thread writerThread1([&shutdown, &socketPath] {
			dv::io::NetworkWriter writer(socketPath, dv::io::Stream::TriggerStream(0, "events", "DVXplorer_123"));

			int64_t timestamp = 10'000;
			while (!shutdown) {
				dv::EventStore events;
				events.emplace_back(timestamp, 1, 2, false);
				writer.writeEvents(events);
				std::this_thread::sleep_for(50ms);
			}
		});

		std::this_thread::sleep_for(100ms);
		expect(fs::exists(socketPath));

		std::thread writerThread2([&socketPath] {
			expect(throws([&socketPath] {
				dv::io::NetworkWriter writer(socketPath, dv::io::Stream::TriggerStream(0, "events", "DVXplorer_123"));
			}));
		});

		std::this_thread::sleep_for(50ms);

		shutdown = true;

		writerThread1.join();
		writerThread2.join();

		expect(!fs::exists(socketPath));
	};

	"single_client_lz4"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution] {
			dv::io::Stream stream = dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution);
			stream.setCompression(dv::CompressionType::LZ4);
			dv::io::NetworkWriter writer("127.0.0.1", 10001, stream);

			int64_t timestamp = 10'000;
			while (!shutdown) {
				dv::EventStore events;
				events.emplace_back(timestamp, 1, 2, false);
				writer.writeEvents(events);
				std::this_thread::sleep_for(50ms);
			}
		});

		std::this_thread::sleep_for(200ms);

		{
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, resolution.width));
			expect(eq(eventResolution.height, resolution.height));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}

			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		}

		shutdown = true;
		if (writerThread.joinable()) {
			writerThread.join();
		}
	};

	"single_client_lz4_high"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution] {
			dv::io::Stream stream = dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution);
			stream.setCompression(dv::CompressionType::LZ4_HIGH);
			dv::io::NetworkWriter writer("127.0.0.1", 10001, stream);

			int64_t timestamp = 10'000;
			while (!shutdown) {
				dv::EventStore events;
				events.emplace_back(timestamp, 1, 2, false);
				writer.writeEvents(events);
				std::this_thread::sleep_for(50ms);
			}
		});

		std::this_thread::sleep_for(200ms);

		{
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, resolution.width));
			expect(eq(eventResolution.height, resolution.height));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}

			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		}

		shutdown = true;
		if (writerThread.joinable()) {
			writerThread.join();
		}
	};

	"single_client_zstd"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution] {
			dv::io::Stream stream = dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution);
			stream.setCompression(dv::CompressionType::ZSTD);
			dv::io::NetworkWriter writer("127.0.0.1", 10001, stream);

			int64_t timestamp = 10'000;
			while (!shutdown) {
				dv::EventStore events;
				events.emplace_back(timestamp, 1, 2, false);
				writer.writeEvents(events);
				std::this_thread::sleep_for(50ms);
			}
		});

		std::this_thread::sleep_for(200ms);

		{
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, resolution.width));
			expect(eq(eventResolution.height, resolution.height));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}

			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		}

		shutdown = true;
		if (writerThread.joinable()) {
			writerThread.join();
		}
	};

	"single_client_zstd_high"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution] {
			dv::io::Stream stream = dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution);
			stream.setCompression(dv::CompressionType::ZSTD_HIGH);
			dv::io::NetworkWriter writer("127.0.0.1", 10001, stream);

			int64_t timestamp = 10'000;
			while (!shutdown) {
				dv::EventStore events;
				events.emplace_back(timestamp, 1, 2, false);
				writer.writeEvents(events);
				std::this_thread::sleep_for(50ms);
			}
		});

		std::this_thread::sleep_for(200ms);

		{
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, resolution.width));
			expect(eq(eventResolution.height, resolution.height));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}

			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		}

		shutdown = true;
		if (writerThread.joinable()) {
			writerThread.join();
		}
	};

	"single_client_lz4_custom"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);

		std::thread writerThread([&shutdown, &resolution] {
			dv::io::Stream stream = dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution);
			stream.setCompression(dv::CompressionType::ZSTD_HIGH);
			dv::io::NetworkWriter writer("127.0.0.1", 10001, stream);

			int64_t timestamp = 10'000;
			while (!shutdown) {
				dv::EventStore events;
				events.emplace_back(timestamp, 1, 2, false);
				writer.writeEvents(events);
				std::this_thread::sleep_for(50ms);
			}
		});

		std::this_thread::sleep_for(200ms);

		expect(nothrow([&resolution] {
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, resolution.width));
			expect(eq(eventResolution.height, resolution.height));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				std::this_thread::sleep_for(50ms);
				retries++;
			}

			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		}));

		shutdown = true;
		writerThread.join();
	};

	"encrypted_connection"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);
		std::atomic<size_t> connectedClients = 0;

		std::thread writerThread([&shutdown, &resolution, &connectedClients] {
			// Encrypted server should not throw, especially if an incompatible client connects
			expect(nothrow([&shutdown, &resolution, &connectedClients] {
				dv::io::Stream stream = dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution);
				stream.setCompression(dv::CompressionType::NONE);
				auto encryption = dv::io::encrypt::defaultEncryptionServer(fs::path("test_files/certs-1/server-cert"),
					fs::path("test_files/certs-1/server-key"), fs::path("test_files/certs-1/ca-cert"));
				dv::io::NetworkWriter writer("127.0.0.1", 10001, stream, std::move(encryption));

				int64_t timestamp = 10'000;
				while (!shutdown) {
					dv::EventStore events;
					events.emplace_back(timestamp, 1, 2, false);
					writer.writeEvents(events);
					connectedClients = writer.getClientCount();
					std::this_thread::sleep_for(50ms);
				}
			}));
		});

		std::this_thread::sleep_for(200ms);
		expect(eq(connectedClients.load(), 0ULL));

		// Correct encryption
		expect(nothrow([&resolution, &connectedClients] {
			auto encryption = dv::io::encrypt::defaultEncryptionClient(
				fs::path("test_files/certs-1/client-cert"), fs::path("test_files/certs-1/client-key"));

			dv::io::NetworkReader reader("127.0.0.1", 10001, std::move(encryption));
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, resolution.width));
			expect(eq(eventResolution.height, resolution.height));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}
			expect(eq(connectedClients.load(), 1ULL));
			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}
		}));

		// Wait for client disconnection
		std::this_thread::sleep_for(1s);
		expect(eq(connectedClients.load(), 0ULL));

		// Certificates from a different system, should throw an exception while handshaking
		expect(throws([] {
			auto encryption = dv::io::encrypt::defaultEncryptionClient(
				fs::path("test_files/certs-2/client-cert"), fs::path("test_files/certs-2/client-key"));

			dv::io::NetworkReader reader("127.0.0.1", 10001, std::move(encryption));
		}));

		expect(eq(connectedClients.load(), 0ULL));

		shutdown = true;
		writerThread.join();
	};

	"disconnect_writer_early"_test = [] {
		std::atomic<bool> shutdown{false};

		const cv::Size resolution(100, 100);
		std::atomic<size_t> connectedClients = 0;

		std::thread writerThread([&shutdown, &resolution, &connectedClients] {
			// Encrypted server should not throw, especially if an incompatible client connects
			expect(nothrow([&shutdown, &resolution, &connectedClients] {
				dv::io::Stream stream = dv::io::Stream::EventStream(0, "events", "DVXplorer_123", resolution);
				stream.setCompression(dv::CompressionType::NONE);
				dv::io::NetworkWriter writer("127.0.0.1", 10001, stream);

				int64_t timestamp = 10'000;
				while (!shutdown) {
					dv::EventStore events;
					events.emplace_back(timestamp, 1, 2, false);
					writer.writeEvents(events);
					connectedClients = writer.getClientCount();
					std::this_thread::sleep_for(50ms);
				}
			}));
		});

		std::this_thread::sleep_for(200ms);
		expect(eq(connectedClients.load(), 0ULL));

		// Correct encryption
		expect(nothrow([&resolution, &connectedClients, &shutdown, &writerThread] {
			dv::io::NetworkReader reader("127.0.0.1", 10001);
			expect(reader.isEventStreamAvailable());
			const auto eventResolution = reader.getEventResolution().value();

			expect(eq(eventResolution.width, resolution.width));
			expect(eq(eventResolution.height, resolution.height));

			dv::EventStore receivedData;
			int retries = 0;
			while (10 > retries) {
				if (const auto events = reader.getNextEventBatch()) {
					receivedData = *events;
					break;
				}
				else {
					std::this_thread::sleep_for(50ms);
				}
				retries++;
			}
			expect(eq(connectedClients.load(), 1ULL));
			expect(eq(receivedData.size(), 1));
			if (!receivedData.isEmpty()) {
				expect(eq(receivedData.front().timestamp(), 10'000LL));
				expect(eq(receivedData.front().x(), 1));
				expect(eq(receivedData.front().y(), 2));
				expect(eq(receivedData.front().polarity(), false));
			}

			shutdown = true;
			writerThread.join();
			std::this_thread::sleep_for(500ms);

			// The server should have gone away, the isRunning flag should be false
			expect(!reader.isRunning());
		}));
	};

	return EXIT_SUCCESS;
}
