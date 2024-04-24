#include "../../include/dv-processing/core/core.hpp"
#include "../../include/dv-processing/io/network_reader.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

bool portInUse(const uint16_t port);

int main() {
	using namespace std::chrono_literals;
	namespace fs = std::filesystem;

	"failed_connection"_test = [] {
		expect(throws([] {
			// No server available at that address
			dv::io::NetworkReader capture("127.0.0.1", 50101);
		}));
	};

	// Test availability for the dv-runtime, which is a requirement for this test suite.
	const int32_t status = system("dv-runtime --version");
	if (status != 0) {
		// dv-runtime is unavailable, this test is not performed, it's not expected for all testing machines
		// to contain a dv-runtime.
		return EXIT_SUCCESS;
	}

	std::thread dvRuntime([] {
		system("dv-runtime -c test_files/net_streaming.xml -p 4040");
	});

	size_t retries = 0;
	while (!portInUse(4040)) {
		std::this_thread::sleep_for(100ms);
		retries++;
		// Wait for 5 seconds
		if (retries > 50) {
			dvRuntime.join();
			std::cerr << "Failed to start dv-runtime; skipping the test" << std::endl;
			return EXIT_FAILURE;
		}
	}

	std::this_thread::sleep_for(5s);

	retries = 0;
	while (system("dv-control -s set /mainloop/capture/ running true") != 0) {
		std::this_thread::sleep_for(250ms);
		retries++;
		if (retries > 10) {
			dvRuntime.join();
			std::cout << "Failed to execute dv-control; skipping the test" << std::endl;
			return EXIT_SUCCESS;
		}
	}

	std::this_thread::sleep_for(2s);

	"basic_connection_events"_test = [] {
		if (!portInUse(50101)) {
			return;
		}

		dv::io::NetworkReader capture("127.0.0.1", 50101);

		expect(eq(capture.getCameraName(), std::string("DAVIS346_00000668")));
		expect(capture.isRunning());
		expect(capture.isEventStreamAvailable());
		expect(!capture.isFrameStreamAvailable());
		expect(!capture.isImuStreamAvailable());
		expect(!capture.isTriggerStreamAvailable());

		expect(capture.getEventResolution().has_value());
		expect(!capture.getFrameResolution().has_value());

		std::this_thread::sleep_for(100ms);

		auto events = capture.getNextEventBatch();
		expect(events.has_value());
		auto frame = capture.getNextFrame();
		expect(!frame.has_value());
		auto imu = capture.getNextImuBatch();
		expect(!imu.has_value());
		auto triggers = capture.getNextTriggerBatch();
		expect(!triggers.has_value());
	};

	"basic_connection_frames"_test = [] {
		if (!portInUse(50102)) {
			return;
		}

		dv::io::NetworkReader capture("127.0.0.1", 50102);

		expect(eq(capture.getCameraName(), std::string("DAVIS346_00000668")));
		expect(capture.isRunning());
		expect(!capture.isEventStreamAvailable());
		expect(capture.isFrameStreamAvailable());
		expect(!capture.isImuStreamAvailable());
		expect(!capture.isTriggerStreamAvailable());

		expect(!capture.getEventResolution().has_value());
		expect(capture.getFrameResolution().has_value());

		std::this_thread::sleep_for(100ms);

		auto events = capture.getNextEventBatch();
		expect(!events.has_value());
		auto frame = capture.getNextFrame();
		expect(frame.has_value());
		auto imu = capture.getNextImuBatch();
		expect(!imu.has_value());
		auto triggers = capture.getNextTriggerBatch();
		expect(!triggers.has_value());
	};

	"basic_connection_imu"_test = [] {
		if (!portInUse(50103)) {
			return;
		}

		dv::io::NetworkReader capture("127.0.0.1", 50103);

		expect(eq(capture.getCameraName(), std::string("DAVIS346_00000668")));
		expect(capture.isRunning());
		expect(!capture.isEventStreamAvailable());
		expect(!capture.isFrameStreamAvailable());
		expect(capture.isImuStreamAvailable());
		expect(!capture.isTriggerStreamAvailable());

		expect(!capture.getEventResolution().has_value());
		expect(!capture.getFrameResolution().has_value());

		std::this_thread::sleep_for(100ms);

		auto events = capture.getNextEventBatch();
		expect(!events.has_value());
		auto frame = capture.getNextFrame();
		expect(!frame.has_value());
		auto imu = capture.getNextImuBatch();
		expect(imu.has_value());
		auto triggers = capture.getNextTriggerBatch();
		expect(!triggers.has_value());
	};

	"basic_connection_triggers"_test = [] {
		if (!portInUse(50105)) {
			return;
		}

		dv::io::NetworkReader capture("127.0.0.1", 50104);

		expect(eq(capture.getCameraName(), std::string("DAVIS346_00000668")));
		expect(capture.isRunning());
		expect(!capture.isEventStreamAvailable());
		expect(!capture.isFrameStreamAvailable());
		expect(!capture.isImuStreamAvailable());
		expect(capture.isTriggerStreamAvailable());

		expect(!capture.getEventResolution().has_value());
		expect(!capture.getFrameResolution().has_value());
	};

	"basic_unix_socket"_test = [] {
		const fs::path socketPath("/tmp/dv-runtime-events.sock");

		if (!fs::exists(socketPath)) {
			return;
		}

		dv::io::NetworkReader capture(socketPath);

		expect(eq(capture.getCameraName(), std::string("DAVIS346_00000668")));
		expect(capture.isRunning());
		expect(capture.isEventStreamAvailable());
		expect(!capture.isFrameStreamAvailable());
		expect(!capture.isImuStreamAvailable());
		expect(!capture.isTriggerStreamAvailable());
		expect(capture.getEventResolution().has_value());
		expect(!capture.getFrameResolution().has_value());

		std::this_thread::sleep_for(100ms);

		auto events = capture.getNextEventBatch();
		expect(events.has_value());
		auto frame = capture.getNextFrame();
		expect(!frame.has_value());
		auto imu = capture.getNextImuBatch();
		expect(!imu.has_value());
		auto triggers = capture.getNextTriggerBatch();
		expect(!triggers.has_value());
	};

	"basic_connection_frames"_test = [] {
		const fs::path socketPath("/tmp/dv-runtime-frames.sock");

		if (!fs::exists(socketPath)) {
			return;
		}

		dv::io::NetworkReader capture(socketPath);

		expect(eq(capture.getCameraName(), std::string("DAVIS346_00000668")));
		expect(capture.isRunning());
		expect(!capture.isEventStreamAvailable());
		expect(capture.isFrameStreamAvailable());
		expect(!capture.isImuStreamAvailable());
		expect(!capture.isTriggerStreamAvailable());

		expect(!capture.getEventResolution().has_value());
		expect(capture.getFrameResolution().has_value());

		std::this_thread::sleep_for(100ms);

		auto events = capture.getNextEventBatch();
		expect(!events.has_value());
		auto frame = capture.getNextFrame();
		expect(frame.has_value());
		auto imu = capture.getNextImuBatch();
		expect(!imu.has_value());
		auto triggers = capture.getNextTriggerBatch();
		expect(!triggers.has_value());
	};

	"basic_connection_imu"_test = [] {
		const fs::path socketPath("/tmp/dv-runtime-imu.sock");

		if (!fs::exists(socketPath)) {
			return;
		}

		dv::io::NetworkReader capture(socketPath);

		expect(eq(capture.getCameraName(), std::string("DAVIS346_00000668")));
		expect(capture.isRunning());
		expect(!capture.isEventStreamAvailable());
		expect(!capture.isFrameStreamAvailable());
		expect(capture.isImuStreamAvailable());
		expect(!capture.isTriggerStreamAvailable());

		expect(!capture.getEventResolution().has_value());
		expect(!capture.getFrameResolution().has_value());

		std::this_thread::sleep_for(100ms);

		auto events = capture.getNextEventBatch();
		expect(!events.has_value());
		auto frame = capture.getNextFrame();
		expect(!frame.has_value());
		auto imu = capture.getNextImuBatch();
		expect(imu.has_value());
		auto triggers = capture.getNextTriggerBatch();
		expect(!triggers.has_value());
	};

	"basic_connection_triggers"_test = [] {
		const fs::path socketPath("/tmp/dv-runtime-triggers.sock");

		if (!fs::exists(socketPath)) {
			return;
		}

		dv::io::NetworkReader capture(socketPath);

		expect(eq(capture.getCameraName(), std::string("DAVIS346_00000668")));
		expect(capture.isRunning());
		expect(!capture.isEventStreamAvailable());
		expect(!capture.isFrameStreamAvailable());
		expect(!capture.isImuStreamAvailable());
		expect(capture.isTriggerStreamAvailable());

		expect(!capture.getEventResolution().has_value());
		expect(!capture.getFrameResolution().has_value());
	};

	"wrong_port"_test = [] {
		expect(throws([] {
			dv::io::NetworkReader capture("127.0.0.1", 50105);
		}));
	};

	std::this_thread::sleep_for(5s);

	"multiple_connections"_test = [] {
		std::vector<std::thread> threads;
		for (int i = 0; i < 5; i++) {
			threads.emplace_back([] {
				dv::io::NetworkReader capture("127.0.0.1", 50101);

				expect(eq(capture.getCameraName(), std::string("DAVIS346_00000668")));
				expect(capture.isRunning());
				expect(capture.isEventStreamAvailable());
				expect(!capture.isFrameStreamAvailable());
				expect(!capture.isImuStreamAvailable());
				expect(!capture.isTriggerStreamAvailable());

				expect(capture.getEventResolution().has_value());
				expect(!capture.getFrameResolution().has_value());

				std::this_thread::sleep_for(100ms);

				auto events = capture.getNextEventBatch();
				expect(events.has_value());
				auto frame = capture.getNextFrame();
				expect(!frame.has_value());
				auto imu = capture.getNextImuBatch();
				expect(!imu.has_value());
				auto triggers = capture.getNextTriggerBatch();
				expect(!triggers.has_value());
			});
		}

		for (auto &thread : threads) {
			thread.join();
		}

		threads.clear();
	};

	system("dv-control -s set /system/ running false");
	dvRuntime.join();

	return EXIT_SUCCESS;
}

bool portInUse(const uint16_t port) {
	using namespace boost::asio;
	using ip::tcp;

	io_service svc;
	tcp::acceptor a(svc);

	boost::system::error_code ec;
	a.open(tcp::v4(), ec) || a.bind({tcp::v4(), port}, ec);

	return ec == error::address_in_use;
}
