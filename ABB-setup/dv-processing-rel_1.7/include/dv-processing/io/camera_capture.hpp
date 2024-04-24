#pragma once

#include "../core/core.hpp"
#include "../core/utils.hpp"
#include "../data/event_base.hpp"
#include "../data/frame_base.hpp"
#include "../data/imu_base.hpp"
#include "../data/trigger_base.hpp"
#include "../exception/exception.hpp"
#include "camera_input_base.hpp"
#include "data_read_handler.hpp"
#include "discovery.hpp"

#include <boost/lockfree/spsc_queue.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <functional>
#include <future>
#include <thread>

namespace dv::io {

class StereoCapture;

class CameraCapture : public CameraInputBase {
public:
	enum class BiasSensitivity {
		VeryLow  = DVX_DVS_CHIP_BIAS_SIMPLE_VERY_LOW,
		Low      = DVX_DVS_CHIP_BIAS_SIMPLE_LOW,
		Default  = DVX_DVS_CHIP_BIAS_SIMPLE_DEFAULT,
		High     = DVX_DVS_CHIP_BIAS_SIMPLE_HIGH,
		VeryHigh = DVX_DVS_CHIP_BIAS_SIMPLE_VERY_HIGH
	};

	enum class DavisReadoutMode {
		EventsAndFrames,
		EventsOnly,
		FramesOnly
	};

	enum class DavisColorMode {
		// Force grascale
		Grayscale = caer_davis_aps_frame_modes::APS_FRAME_GRAYSCALE,
		// Resort to default: color for color, grayscale to mono
		Color = caer_davis_aps_frame_modes::APS_FRAME_DEFAULT
	};

	enum class DVXeFPS {
		EFPS_CONSTANT_100,
		EFPS_CONSTANT_200,
		EFPS_CONSTANT_500,
		EFPS_CONSTANT_1000,
		EFPS_CONSTANT_LOSSY_2000,
		EFPS_CONSTANT_LOSSY_5000,
		EFPS_CONSTANT_LOSSY_10000,
		EFPS_VARIABLE_2000,
		EFPS_VARIABLE_5000,
		EFPS_VARIABLE_10000,
		EFPS_VARIABLE_15000,
	};

	friend std::ostream &operator<<(std::ostream &os, const DVXeFPS &var) {
		switch (var) {
			case DVXeFPS::EFPS_CONSTANT_100:
				return os << "100 fps constant";

			case DVXeFPS::EFPS_CONSTANT_200:
				return os << "200 fps constant";

			case DVXeFPS::EFPS_CONSTANT_500:
				return os << "500 fps constant";

			case DVXeFPS::EFPS_CONSTANT_1000:
				return os << "1000 fps constant";

			case DVXeFPS::EFPS_CONSTANT_LOSSY_2000:
				return os << "2000 fps constant, lossy";

			case DVXeFPS::EFPS_CONSTANT_LOSSY_5000:
				return os << "5000 fps constant, lossy";

			case DVXeFPS::EFPS_CONSTANT_LOSSY_10000:
				return os << "10000 fps constant, lossy";

			case DVXeFPS::EFPS_VARIABLE_2000:
				return os << "2000 fps variable";

			case DVXeFPS::EFPS_VARIABLE_5000:
				return os << "5000 fps variable";

			case DVXeFPS::EFPS_VARIABLE_10000:
				return os << "10000 fps variable";

			case DVXeFPS::EFPS_VARIABLE_15000:
				return os << "15000 fps variable";
		}
	}

	enum class CameraType {
		Any,
		DAVIS,
		DVS
	};

private:
	enum class InitialState {
		DISCARD_DATA,
		WAIT_FOR_RESET,
		DO_MANUAL_RESET,
		RUNNING,
	};

	std::atomic<bool> keepRunning{true};
	std::atomic<InitialState> initState{InitialState::DISCARD_DATA};
	std::atomic<int64_t> mTimestampOffset{-1};

	caer_device_discovery_result discoveryResult{};

	// The device handle
	std::unique_ptr<libcaer::devices::device> device{nullptr};

	using EventPacketPair = std::pair<size_t, std::shared_ptr<libcaer::events::EventPacket>>;

	struct SortedPacketBuffers {
		size_t packetCount = 0;

		boost::lockfree::spsc_queue<EventPacketPair, boost::lockfree::capacity<10000>> events;
		boost::lockfree::spsc_queue<EventPacketPair, boost::lockfree::capacity<10000>> imu;
		boost::lockfree::spsc_queue<EventPacketPair, boost::lockfree::capacity<10000>> triggers;
		boost::lockfree::spsc_queue<EventPacketPair, boost::lockfree::capacity<1000>> frames;

		int64_t eventStreamSeek   = -1;
		int64_t imuStreamSeek     = -1;
		int64_t triggerStreamSeek = -1;
		int64_t framesStreamSeek  = -1;

		void acceptPacket(const std::shared_ptr<libcaer::events::EventPacket> &packet) {
			const auto eventType = static_cast<caer_default_event_types>(packet->getEventType());
			switch (eventType) {
				case SPECIAL_EVENT:
					triggers.push(std::make_pair(packetCount++, packet));
					break;
				case POLARITY_EVENT:
					events.push(std::make_pair(packetCount++, packet));
					break;
				case FRAME_EVENT:
					frames.push(std::make_pair(packetCount++, packet));
					break;
				case IMU6_EVENT:
					imu.push(std::make_pair(packetCount++, packet));
					break;
				default:
					break;
			}
		}

		void clearBuffers() {
			events.reset();
			imu.reset();
			triggers.reset();
			frames.reset();
		}

		[[nodiscard]] std::optional<dv::EventStore> popEvents(const int64_t timeOffset) {
			EventPacketPair packet;
			if (!events.pop(packet)) {
				return std::nullopt;
			}

			const dv::EventStore output = convertEventsPacket(packet.second, timeOffset);
			eventStreamSeek             = output.getHighestTime();
			return output;
		}

		[[nodiscard]] std::optional<dv::Frame> popFrame(const int64_t timeOffset) {
			EventPacketPair packet;
			if (!frames.pop(packet)) {
				return std::nullopt;
			}

			const dv::Frame output = convertFramePacket(packet.second, timeOffset);
			framesStreamSeek       = output.timestamp;
			return output;
		}

		[[nodiscard]] std::optional<dv::cvector<dv::IMU>> popImu(const int64_t timeOffset) {
			EventPacketPair packet;
			if (!imu.pop(packet)) {
				return std::nullopt;
			}

			const dv::cvector<dv::IMU> output = convertImuPacket(packet.second, timeOffset);
			imuStreamSeek                     = output.back().timestamp;
			return output;
		}

		[[nodiscard]] std::optional<dv::cvector<dv::Trigger>> popTriggers(const int64_t timeOffset) {
			EventPacketPair packet;
			if (!triggers.pop(packet)) {
				return std::nullopt;
			}

			// NOTE: seek timestamp is retrieved differently, conversion method filters out
			// certain types of triggers, which can be used to retrieve the seek timestamp.
			// Further, due to this filtering, empty vectors are possible, which we do not forward.
			const dv::cvector<dv::Trigger> output = convertTriggerPacket(packet.second, timeOffset, triggerStreamSeek);
			if (output.empty()) {
				return std::nullopt;
			}
			return output;
		}
	};

	static float boschAccRateToFreq(const uint32_t value) {
		switch (value) {
			case BOSCH_ACCEL_12_5HZ:
				return 12.5f;
			case BOSCH_ACCEL_25HZ:
				return 25.f;
			case BOSCH_ACCEL_50HZ:
				return 50.f;
			case BOSCH_ACCEL_100HZ:
				return 100.f;
			case BOSCH_ACCEL_200HZ:
				return 200.f;
			case BOSCH_ACCEL_400HZ:
				return 400.f;
			case BOSCH_ACCEL_800HZ:
				return 800.f;
			case BOSCH_ACCEL_1600HZ:
				return 1600.f;
			default:
				throw dv::exceptions::InvalidArgument<uint32_t>("Invalid accelerometer measurement rate value", value);
		}
	}

	static float boschGyroRateToFreq(const uint32_t value) {
		switch (value) {
			case BOSCH_GYRO_25HZ:
				return 25.f;
			case BOSCH_GYRO_50HZ:
				return 50.f;
			case BOSCH_GYRO_100HZ:
				return 100.f;
			case BOSCH_GYRO_200HZ:
				return 200.f;
			case BOSCH_GYRO_400HZ:
				return 400.f;
			case BOSCH_GYRO_800HZ:
				return 800.f;
			case BOSCH_GYRO_1600HZ:
				return 1600.f;
			case BOSCH_GYRO_3200HZ:
				return 3200.f;
			default:
				throw dv::exceptions::InvalidArgument<uint32_t>("Invalid gyroscope measurement rate value", value);
		}
	}

	SortedPacketBuffers buffers;

	[[nodiscard]] static inline dv::Frame convertFramePacket(
		const std::shared_ptr<libcaer::events::EventPacket> &packet, const int64_t timestampOffset) {
		const libcaer::events::FrameEventPacket oldPacketFrame(
			const_cast<caerEventPacketHeader>(packet->getHeaderPointer()), false);

		dv::Frame newFrame;

		// We return the first valid frame, ignoring subsequent ones.
		for (const auto &evt : oldPacketFrame) {
			if (!evt.isValid()) {
				continue;
			}

			newFrame.timestamp = timestampOffset + evt.getTSStartOfExposure64(oldPacketFrame);
			newFrame.exposure  = dv::Duration{evt.getExposureLength()};
			newFrame.source    = dv::FrameSource::SENSOR;
			newFrame.positionX = static_cast<int16_t>(evt.getPositionX());
			newFrame.positionY = static_cast<int16_t>(evt.getPositionY());

			// New frame format specification.
			if (evt.getChannelNumber() == libcaer::events::FrameEvent::colorChannels::RGB) {
				// 16 bits, 3 channels, RGB to 8 bits BGR.
				cv::Mat oldFrame{
					evt.getLengthY(), evt.getLengthX(), CV_16UC3, const_cast<uint16_t *>(evt.getPixelArrayUnsafe())};
				cv::Mat newFrameMat{evt.getLengthY(), evt.getLengthX(), CV_8UC3};

				oldFrame.convertTo(newFrameMat, CV_8U, (1.0 / 256.0), 0);
				cv::cvtColor(newFrameMat, newFrameMat, cv::COLOR_RGB2BGR);

				newFrame.image = newFrameMat;
			}
			else if (evt.getChannelNumber() == libcaer::events::FrameEvent::colorChannels::RGBA) {
				// 16 bits, 4 channels, RGBA to 8 bits BGRA.
				cv::Mat oldFrame{
					evt.getLengthY(), evt.getLengthX(), CV_16UC4, const_cast<uint16_t *>(evt.getPixelArrayUnsafe())};
				cv::Mat newFrameMat{evt.getLengthY(), evt.getLengthX(), CV_8UC4};

				oldFrame.convertTo(newFrameMat, CV_8U, (1.0 / 256.0), 0);
				cv::cvtColor(newFrameMat, newFrameMat, cv::COLOR_RGBA2BGRA);

				newFrame.image = newFrameMat;
			}
			else {
				// Default: 16 bits, 1 channel, grayscale to 8 bits grayscale.
				cv::Mat oldFrame{
					evt.getLengthY(), evt.getLengthX(), CV_16UC1, const_cast<uint16_t *>(evt.getPixelArrayUnsafe())};
				cv::Mat newFrameMat{evt.getLengthY(), evt.getLengthX(), CV_8UC1};

				oldFrame.convertTo(newFrameMat, CV_8U, (1.0 / 256.0), 0);
				// No color format conversion needed here.

				newFrame.image = newFrameMat;
			}

			break;
		}

		return newFrame;
	}

	[[nodiscard]] static inline dv::cvector<dv::IMU> convertImuPacket(
		const std::shared_ptr<libcaer::events::EventPacket> &packet, const int64_t timestampOffset) {
		const libcaer::events::IMU6EventPacket oldPacketIMU(
			const_cast<caerEventPacketHeader>(packet->getHeaderPointer()), false);

		dv::cvector<dv::IMU> output;
		output.reserve(static_cast<size_t>(oldPacketIMU.getEventValid()));

		for (const auto &evt : oldPacketIMU) {
			if (!evt.isValid()) {
				continue;
			}

			output.emplace_back(timestampOffset + evt.getTimestamp64(oldPacketIMU), evt.getTemp(), evt.getAccelX(),
				evt.getAccelY(), evt.getAccelZ(), evt.getGyroX(), evt.getGyroY(), evt.getGyroZ(), 0.f, 0.f, 0.f);
		}

		output.shrink_to_fit();
		return output;
	}

	[[nodiscard]] static inline dv::cvector<dv::Trigger> convertTriggerPacket(
		const std::shared_ptr<libcaer::events::EventPacket> &packet, const int64_t timestampOffset,
		int64_t &maxTimestamp) {
		const libcaer::events::SpecialEventPacket oldPacketSpecial(
			const_cast<caerEventPacketHeader>(packet->getHeaderPointer()), false);

		dv::cvector<dv::Trigger> output;
		output.reserve(static_cast<size_t>(oldPacketSpecial.getEventValid()));

		for (const auto &evt : oldPacketSpecial) {
			if (!evt.isValid()) {
				continue;
			}

			dv::TriggerType type;

			switch (static_cast<caer_special_event_types>(evt.getType())) {
				case TIMESTAMP_RESET:
					type = dv::TriggerType::TIMESTAMP_RESET;
					break;
				case EXTERNAL_INPUT_RISING_EDGE:
					type = dv::TriggerType::EXTERNAL_SIGNAL_RISING_EDGE;
					break;
				case EXTERNAL_INPUT_FALLING_EDGE:
					type = dv::TriggerType::EXTERNAL_SIGNAL_FALLING_EDGE;
					break;
				case EXTERNAL_INPUT_PULSE:
					type = dv::TriggerType::EXTERNAL_SIGNAL_PULSE;
					break;
				case EXTERNAL_GENERATOR_RISING_EDGE:
					type = dv::TriggerType::EXTERNAL_GENERATOR_RISING_EDGE;
					break;
				case EXTERNAL_GENERATOR_FALLING_EDGE:
					type = dv::TriggerType::EXTERNAL_GENERATOR_FALLING_EDGE;
					break;
				case APS_FRAME_START:
					type = dv::TriggerType::APS_FRAME_START;
					break;
				case APS_FRAME_END:
					type = dv::TriggerType::APS_FRAME_END;
					break;
				case APS_EXPOSURE_START:
					type = dv::TriggerType::APS_EXPOSURE_START;
					break;
				case APS_EXPOSURE_END:
					type = dv::TriggerType::APS_EXPOSURE_END;
					break;
				default:
					// Ignoring the packet
					continue;
			}

			output.emplace_back(timestampOffset + evt.getTimestamp64(oldPacketSpecial), type);
		}

		output.shrink_to_fit();

		// Find last valid packet; this is needed because previous loop skip non-convertible event packets, while
		// they can contain relevant timestamp information.
		auto iter = oldPacketSpecial.rbegin();
		while (iter != oldPacketSpecial.rend()) {
			if (iter->isValid()) {
				maxTimestamp = timestampOffset + iter->getTimestamp64(oldPacketSpecial);
				break;
			}
			iter++;
		}

		return output;
	}

	[[nodiscard]] static dv::EventStore convertEventsPacket(
		const std::shared_ptr<libcaer::events::EventPacket> &packet, const int64_t timestampOffset) {
		const libcaer::events::PolarityEventPacket oldPacketPolarity(
			const_cast<caerEventPacketHeader>(packet->getHeaderPointer()), false);

		auto newEventPacket = std::make_shared<dv::EventPacket>();
		newEventPacket->elements.reserve(static_cast<size_t>(oldPacketPolarity.getEventValid()));

		for (const auto &evt : oldPacketPolarity) {
			if (!evt.isValid()) {
				continue;
			}

			newEventPacket->elements.emplace_back(
				timestampOffset + evt.getTimestamp64(oldPacketPolarity), evt.getX(), evt.getY(), evt.getPolarity());
		}

		newEventPacket->elements.shrink_to_fit();

		return dv::EventStore(newEventPacket);
	}

	[[nodiscard]] static inline bool containsResetEvent(const std::shared_ptr<libcaer::events::EventPacket> &packet) {
		if (packet->getEventType() != SPECIAL_EVENT) {
			return false;
		}

		const libcaer::events::SpecialEventPacket oldPacketSpecial(
			const_cast<caerEventPacketHeader>(packet->getHeaderPointer()), false);

		return std::any_of(oldPacketSpecial.begin(), oldPacketSpecial.end(), [](const auto &evt) {
			return (evt.isValid() && (evt.getType() == TIMESTAMP_RESET));
		});
	}

	void discoverMatchingCamera(const std::string &cameraName, const CameraType type) {
		const auto allCameras = libcaer::devices::discover::all();
		auto iter             = std::find_if(
            allCameras.begin(), allCameras.end(), [&cameraName, &type](const caer_device_discovery_result &camera) {
                bool typeMatches
                    = type == CameraType::Any
                   || (type == CameraType::DVS
                       && (camera.deviceType == CAER_DEVICE_DVXPLORER || camera.deviceType == CAER_DEVICE_DVS128
                           || camera.deviceType == CAER_DEVICE_EDVS || camera.deviceType == CAER_DEVICE_DVS132S
                           || camera.deviceType == CAER_DEVICE_SAMSUNG_EVK))
                   || (type == CameraType::DAVIS && camera.deviceType == CAER_DEVICE_DAVIS);

                bool nameMatches = cameraName.empty() || cameraName == internal::getDiscoveredCameraName(camera);
                return nameMatches && typeMatches;
            });

		if (iter == allCameras.end()) {
			throw std::runtime_error("No compatible device discovered");
		}
		else {
			discoveryResult = *iter;
		}
	}

	void sendTimestampReset() {
		initState = InitialState::WAIT_FOR_RESET;

		switch (discoveryResult.deviceType) {
			case CAER_DEVICE_DAVIS:
				deviceConfigSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, true);
				break;
			case CAER_DEVICE_DVS128:
				deviceConfigSet(DVS128_CONFIG_DVS, DVS128_CONFIG_DVS_TIMESTAMP_RESET, true);
				break;
			case CAER_DEVICE_EDVS:
				deviceConfigSet(EDVS_CONFIG_DVS, EDVS_CONFIG_DVS_TIMESTAMP_RESET, true);
				break;
			case CAER_DEVICE_DVS132S:
				deviceConfigSet(DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_TIMESTAMP_RESET, true);
				break;
			case CAER_DEVICE_DVXPLORER:
				// Mini does not currently support TS reset explicitly.
				if (isDeviceDVXplorerMini()) {
					initState = InitialState::DO_MANUAL_RESET;
				}
				else {
					deviceConfigSet(DVX_MUX, DVX_MUX_TIMESTAMP_RESET, true);
				}
				break;
			default:
				// Device does not support timestamp reset.
				initState = InitialState::DO_MANUAL_RESET;
				break;
		}
	}

	/**
	 * Checks whether connected device is a DVXplorer Mini model.
	 * @return 	True if the device is a mini, false otherwise
	 */
	[[nodiscard]] bool isDeviceDVXplorerMini() const {
		if (discoveryResult.deviceType != CAER_DEVICE_DVXPLORER) {
			return false;
		}
		const auto &devInfo = discoveryResult.deviceInfo.dvXplorerInfo;
		return (!devInfo.muxHasStatistics && !devInfo.dvsHasStatistics && !devInfo.extInputHasGenerator);
	}

	friend class dv::io::StereoCapture;

	/**
	 * Create a camera capture class which opens a camera according to given parameters.
	 * @param cameraName 	Camera name, an empty string will match any name.
	 * @param type 			Type of camera, one of: any, DVS, or DAVIS.
	 * @param doTimestampReset Reset this camera's timestamps on startup. Required for stereo capture.
	 */
	explicit CameraCapture(const std::string &cameraName, const CameraType type, const bool doTimestampReset) {
		discoverMatchingCamera(cameraName, type);

		device = libcaer::devices::discover::open(0, discoveryResult);
		device->sendDefaultConfig();

		// DAVIS autoexposure not enabled by default in libcaer, but so useful DV and here enable it.
		// No deviation from defaults for DVXplorer cameras.
		enableDavisAutoExposure();

		device->dataStart(
			[](void *ptr) {
				auto *self = reinterpret_cast<CameraCapture *>(ptr);
				if (self->keepRunning.load(std::memory_order_relaxed)) {
					// Read and sort packets
					if (auto packetContainer = self->device->dataGet(); packetContainer != nullptr) {
						// Discard initial data before we've sent an explicit timestamp reset.
						if (self->initState.load(std::memory_order_relaxed) == InitialState::DISCARD_DATA) {
							return;
						}

						for (auto &packet : *packetContainer) {
							if ((packet == nullptr) || packet->empty() || (packet->getEventValid() <= 0)) {
								continue;
							}

							// Always be on the lookout for resets, could happen multiple times.
							if (containsResetEvent(packet)) {
								self->setTimestampOffset(dv::now());
								self->initState = InitialState::RUNNING;

								// Skip the current packet container if a reset is found.
								// Valid data would only be present starting with the next one.
								return;
							}

							// For cameras with no timestamp reset support, like DVX Mini.
							if (self->initState.load(std::memory_order_relaxed) == InitialState::DO_MANUAL_RESET) {
								// Remove lowest timestamp, since it won't be zero due to no reset.
								self->setTimestampOffset(dv::now() - packetContainer->getLowestEventTimestamp());
								self->initState = InitialState::RUNNING;

								// Skip the current packet container.
								return;
							}

							// Wait for at least first reset to come in.
							if (self->initState.load(std::memory_order_relaxed) == InitialState::RUNNING) {
								self->buffers.acceptPacket(packet);
							}
						}
					}
				}
			},
			nullptr, reinterpret_cast<void *>(this),
			[](void *ptr) {
				auto *self        = reinterpret_cast<CameraCapture *>(ptr);
				self->keepRunning = false;
			},
			reinterpret_cast<void *>(this));

		if (doTimestampReset) {
			// Wait a bit before resetting timestamp to allow old data to be discarded.
			std::this_thread::sleep_for(std::chrono::milliseconds{500});

			sendTimestampReset();
		}
	}

public:
	/**
	 * Create a camera capture class which opens first discovered camera of any type.
	 */
	CameraCapture() : CameraCapture("", CameraType::Any) {
	}

	/**
	 * Create a camera capture class which opens a camera according to given parameters.
	 * @param cameraName 	Camera name, an empty string will match any name.
	 * @param type 			Type of camera, one of: any, DVS, or DAVIS.
	 */
	explicit CameraCapture(const std::string &cameraName, const CameraType type = CameraType::Any) :
		CameraCapture(cameraName, type, true) {
	}

	/**
	 * Parse and retrieve next event batch.
	 * @return 		Event batch or `std::nullopt` if no events were received since last read.
	 */
	std::optional<dv::EventStore> getNextEventBatch() override {
		return buffers.popEvents(mTimestampOffset.load(std::memory_order_relaxed));
	}

	/**
	 * Parse and retrieve next frame.
	 * @return 		Frame or `std::nullopt` if no frames were received since last read.
	 */
	std::optional<dv::Frame> getNextFrame() override {
		return buffers.popFrame(mTimestampOffset.load(std::memory_order_relaxed));
	}

	/**
	 * Parse and retrieve next IMU data batch.
	 * @return 		IMU data batch or `std::nullopt` if no IMU data was received since last read.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::IMU>> getNextImuBatch() override {
		return buffers.popImu(mTimestampOffset.load(std::memory_order_relaxed));
	}

	/**
	 * Parse and retrieve next trigger data batch.
	 * @return 		Trigger data batch or `std::nullopt` if no triggers were received since last read.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::Trigger>> getNextTriggerBatch() override {
		return buffers.popTriggers(mTimestampOffset.load(std::memory_order_relaxed));
	}

	/**
	 * Get event stream resolution.
	 * @return 		Event stream resolution, `std::nullopt` if event stream is unavailable.
	 */
	[[nodiscard]] std::optional<cv::Size> getEventResolution() const override {
		cv::Size resolution;
		switch (discoveryResult.deviceType) {
			case CAER_DEVICE_DAVIS:
				resolution.width  = discoveryResult.deviceInfo.davisInfo.dvsSizeX;
				resolution.height = discoveryResult.deviceInfo.davisInfo.dvsSizeY;
				break;
			case CAER_DEVICE_DVS128:
				resolution.width  = discoveryResult.deviceInfo.dvs128Info.dvsSizeX;
				resolution.height = discoveryResult.deviceInfo.dvs128Info.dvsSizeY;
				break;
			case CAER_DEVICE_EDVS:
				resolution.width  = discoveryResult.deviceInfo.edvsInfo.dvsSizeX;
				resolution.height = discoveryResult.deviceInfo.edvsInfo.dvsSizeY;
				break;
			case CAER_DEVICE_DVS132S:
				resolution.width  = discoveryResult.deviceInfo.dvs132sInfo.dvsSizeX;
				resolution.height = discoveryResult.deviceInfo.dvs132sInfo.dvsSizeY;
				break;
			case CAER_DEVICE_DVXPLORER:
				resolution.width  = discoveryResult.deviceInfo.dvXplorerInfo.dvsSizeX;
				resolution.height = discoveryResult.deviceInfo.dvXplorerInfo.dvsSizeY;
				break;
			case CAER_DEVICE_SAMSUNG_EVK:
				resolution.width  = discoveryResult.deviceInfo.samsungEVKInfo.dvsSizeX;
				resolution.height = discoveryResult.deviceInfo.samsungEVKInfo.dvsSizeY;
				break;
			default:
				throw std::runtime_error("Unsupported device type");
		}
		return resolution;
	}

	/**
	 * Retrieve frame stream resolution.
	 * @return 		Frame stream resolution or `std::nullopt` if the frame stream is not available.
	 */
	[[nodiscard]] std::optional<cv::Size> getFrameResolution() const override {
		cv::Size resolution;
		switch (discoveryResult.deviceType) {
			case CAER_DEVICE_DAVIS: {
				resolution.width  = discoveryResult.deviceInfo.davisInfo.apsSizeX;
				resolution.height = discoveryResult.deviceInfo.davisInfo.apsSizeY;
				break;
			}
			default:
				return std::nullopt;
		}
		return resolution;
	}

	/**
	 * Check whether frame stream is available.
	 * @return 		True if frame stream is available, false otherwise.
	 */
	[[nodiscard]] bool isFrameStreamAvailable() const override {
		// DAVIS models support frame stream output
		return discoveryResult.deviceType == CAER_DEVICE_DAVIS;
	}

	/**
	 * Check whether event stream is available.
	 * @return 		True if event stream is available, false otherwise.
	 */
	[[nodiscard]] bool isEventStreamAvailable() const override {
		// All supported cameras have event output right now
		return true;
	}

	/**
	 * Check whether device outputs IMU data.
	 * @return True if device outputs IMU data, false otherwise.
	 */
	[[nodiscard]] bool isImuStreamAvailable() const override {
		switch (discoveryResult.deviceType) {
			case CAER_DEVICE_DAVIS:
				return discoveryResult.deviceInfo.davisInfo.imuType != caer_imu_types::IMU_NONE;
			case CAER_DEVICE_DVS132S:
				return discoveryResult.deviceInfo.dvs132sInfo.imuType != caer_imu_types::IMU_NONE;
			case CAER_DEVICE_DVXPLORER:
				return discoveryResult.deviceInfo.dvXplorerInfo.imuType != caer_imu_types::IMU_NONE;
			case CAER_DEVICE_DVS128:
			case CAER_DEVICE_EDVS:
			case CAER_DEVICE_SAMSUNG_EVK:
				return false;
			default:
				throw std::runtime_error("Unsupported device type");
		}
	}

	/**
	 * Check whether device outputs trigger data.
	 * @return True if device outputs trigger data, false otherwise.
	 */
	[[nodiscard]] bool isTriggerStreamAvailable() const override {
		switch (discoveryResult.deviceType) {
			case CAER_DEVICE_DAVIS:
			case CAER_DEVICE_DVS128:
			case CAER_DEVICE_DVS132S:
				return true;
			case CAER_DEVICE_EDVS:
			case CAER_DEVICE_SAMSUNG_EVK:
				return false;
			case CAER_DEVICE_DVXPLORER:
				// DVXplorer Mini does not support trigger data, while full DVXplorer does support triggers.
				return !isDeviceDVXplorerMini();
			default:
				throw std::runtime_error("Unsupported device type");
		}
	}

	/**
	 * Destructor: stops the readout thread.
	 */
	~CameraCapture() {
		device->dataStop();
	}

	/**
	 * Get camera name, which is a combination of the camera model and the serial number.
	 * @return 		String containing the camera model and serial number separated by an underscore character.
	 */
	[[nodiscard]] std::string getCameraName() const override {
		return internal::getDiscoveredCameraName(discoveryResult);
	}

	/**
	 * Enable auto-exposure. To disable the auto-exposure, use the manual set exposure function.
	 * @return 			True if configuration was successful, false otherwise.
	 */
	bool enableDavisAutoExposure() {
		if (discoveryResult.deviceType != CAER_DEVICE_DAVIS) {
			return false;
		}

		deviceConfigSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, 1);
		return deviceConfigGet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE) == 1;
	}

	/**
	 * Disable auto-exposure and set a new fixed exposure value.
	 * @param exposure 	Exposure duration.
	 * @return 			True if configuration was successful, false otherwise.
	 */
	bool setDavisExposureDuration(const dv::Duration &exposure) {
		if (discoveryResult.deviceType != CAER_DEVICE_DAVIS) {
			return false;
		}

		auto exposureValue = static_cast<uint32_t>(exposure.count());
		deviceConfigSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, 0);
		deviceConfigSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, exposureValue);
		return deviceConfigGet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE) == exposureValue;
	}

	/**
	 * Get the current exposure duration.
	 * @return 			An optional containing the exposure duration, return `std::nullopt` in case exposure duration
	 * 					setting is not available for the device.
	 */
	[[nodiscard]] std::optional<dv::Duration> getDavisExposureDuration() const {
		if (discoveryResult.deviceType != CAER_DEVICE_DAVIS) {
			return std::nullopt;
		}

		return dv::Duration(deviceConfigGet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE));
	}

	/**
	 * Set a new frame interval value. This interval defines the framerate output of the camera. The frames
	 * will be produced at the given interval, the interval can be reduced in case exposure time is longer
	 * than the frame interval.
	 * @param interval 	Output frame interval.
	 * @return 			True if configuration was successful, false otherwise.
	 */
	bool setDavisFrameInterval(const dv::Duration &interval) {
		if (discoveryResult.deviceType != CAER_DEVICE_DAVIS) {
			return false;
		}

		auto intervalValue = static_cast<uint32_t>(interval.count());
		deviceConfigSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL, intervalValue);
		return deviceConfigGet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL) == intervalValue;
	}

	/**
	 * Get the configured frame interval.
	 * @return 			An optional containing the frame interval value, return `std::nullopt` in case frame
	 * 					interval setting is not available for the device.
	 */
	[[nodiscard]] std::optional<dv::Duration> getDavisFrameInterval() const {
		if (discoveryResult.deviceType != CAER_DEVICE_DAVIS) {
			return std::nullopt;
		}

		return dv::Duration(deviceConfigGet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL));
	}

	/**
	 * Get a configuration setting value from the connected device.
	 * @param moduleAddress		Module address. An integer number that represents a group of settings.
	 * @param parameterAddress	Parameter address. An integer number that specifies a parameter within a parameter
	 * 							module group.
	 * @return					Configured value of the parameter.
	 * @throws runtime_error	Exception is thrown if parameter is not available for the device.
	 */
	[[nodiscard]] uint32_t deviceConfigGet(const int8_t moduleAddress, const uint8_t parameterAddress) const {
		return device->configGet(moduleAddress, parameterAddress);
	}

	/**
	 * Set a configuration setting to a given value.
	 * @param moduleAddress		Module address. An integer number that represents a group of settings.
	 * @param parameterAddress	Parameter address. An integer number that specifies a parameter within a parameter
	 * 							module group.
	 * @param value				New value for the configuration.
	 * @throws runtime_error	Exception is thrown if parameter is not available for the device.
	 */
	void deviceConfigSet(const int8_t moduleAddress, const uint8_t parameterAddress, const uint32_t value) {
		device->configSet(moduleAddress, parameterAddress, value);
	}

	/**
	 * Set DVS chip bias sensitivity preset.
	 * @param sensitivity 	DVS sensitivity preset.
	 * @return 				True if configuration was successful, false otherwise.
	 */
	bool setDVSBiasSensitivity(const BiasSensitivity sensitivity) {
		if (discoveryResult.deviceType != CAER_DEVICE_DVXPLORER) {
			return false;
		}

		// Sensitivity settings are bit-exact with the enum
		deviceConfigSet(DVX_DVS_CHIP_BIAS, DVX_DVS_CHIP_BIAS_SIMPLE, static_cast<uint32_t>(sensitivity));
		return true;
	}

	/**
	 * Enable or disable DVXplorer global hold setting.
	 * @param state		True to enable global hold, false to disable.
	 * @return 			True if configuration was successful, false otherwise.
	 */
	bool setDVSGlobalHold(const bool state) {
		if (discoveryResult.deviceType != CAER_DEVICE_DVXPLORER) {
			return false;
		}

		deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_GLOBAL_HOLD_ENABLE, state);
		return deviceConfigGet(DVX_DVS_CHIP, DVX_DVS_CHIP_GLOBAL_HOLD_ENABLE) == state;
	}

	/**
	 * Enable or disable DVXplorer global reset setting.
	 * @param state		True to enable global reset, false to disable.
	 * @return 			True if configuration was successful, false otherwise.
	 */
	bool setDVXplorerGlobalReset(const bool state) {
		if (discoveryResult.deviceType != CAER_DEVICE_DVXPLORER) {
			return false;
		}

		deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_GLOBAL_RESET_ENABLE, state);
		return deviceConfigGet(DVX_DVS_CHIP, DVX_DVS_CHIP_GLOBAL_RESET_ENABLE) == state;
	}

	/**
	 * Set davis data readout mode. The configuration will be performed if the connected camera is a DAVIS camera.
	 * @param mode 		New readout mode
	 * @return 			True if configuration was successful, false otherwise.
	 */
	bool setDavisReadoutMode(const DavisReadoutMode mode) {
		if (discoveryResult.deviceType != CAER_DEVICE_DAVIS) {
			return false;
		}

		const bool runDVS = mode == DavisReadoutMode::EventsOnly || mode == DavisReadoutMode::EventsAndFrames;
		deviceConfigSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, runDVS);
		const bool runAPS = mode == DavisReadoutMode::FramesOnly || mode == DavisReadoutMode::EventsAndFrames;
		deviceConfigSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, runAPS);

		return (deviceConfigGet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN) == runDVS)
			&& (deviceConfigGet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN) == runAPS);
	}

	/**
	 * Set davis color mode. The configuration will be performed if the connected camera is a DAVIS camera.
	 * @param colorMode Color mode, either grayscale or color (if supported).
	 * @return True if configuration was successful, false otherwise.
	 */
	bool setDavisColorMode(const DavisColorMode colorMode) {
		if (discoveryResult.deviceType != CAER_DEVICE_DAVIS) {
			return false;
		}

		const auto value = static_cast<uint32_t>(colorMode);
		deviceConfigSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE, value);
		return deviceConfigGet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE) == value;
	}

	/**
	 * Set DVXplorer event FPS value. The configuration will be performed if the connected camera is a DVXplorer camera.
	 * @param eFPS number of event frames per second in readout (if supported).
	 * @return True if configuration was successful, false otherwise.
	 */
	bool setDVXplorerEFPS(const DVXeFPS eFPS) {
		if (discoveryResult.deviceType != CAER_DEVICE_DVXPLORER) {
			return false;
		}

		// Changes to readout timings need to happen while chip is not reading out data, to avoid lock-up.
		deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_DTAG_CONTROL, DVX_DVS_CHIP_DTAG_CONTROL_STOP);

		if ((eFPS == DVXeFPS::EFPS_CONSTANT_100) || (eFPS == DVXeFPS::EFPS_CONSTANT_200)
			|| (eFPS == DVXeFPS::EFPS_CONSTANT_500) || (eFPS == DVXeFPS::EFPS_CONSTANT_1000)) {
			// Worst case readout time is around 0.85ms, so if we want 1k FPS or slower we need to increase T_ED
			// to compensate, and at this point it makes sense to also make the timing constant, using the
			// fixed read time readout mode. Default setting is 45000 cycles @ 50 MHz: 900ms.
			deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_READ_FIXED, 45000);
			deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_FIXED_READ_TIME_ENABLE, true);

			deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_NEXT_SEL, 15); // Safe default.

			if (eFPS == DVXeFPS::EFPS_CONSTANT_100) {
				// 10000ms per frame: 10000-900=9100
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 9100);
			}
			else if (eFPS == DVXeFPS::EFPS_CONSTANT_200) {
				// 5000ms per frame: 5000-900=4100
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 4100);
			}
			else if (eFPS == DVXeFPS::EFPS_CONSTANT_500) {
				// 2000ms per frame: 2000-900=1100
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 1100);
			}
			else if (eFPS == DVXeFPS::EFPS_CONSTANT_1000) {
				// 1000ms per frame: 1000-900=100
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 100);
			}
		}
		else if ((eFPS == DVXeFPS::EFPS_CONSTANT_LOSSY_2000) || (eFPS == DVXeFPS::EFPS_CONSTANT_LOSSY_5000)
				 || (eFPS == DVXeFPS::EFPS_CONSTANT_LOSSY_10000)) {
			// Constant readout time can also be used at values below the maximum readout time, you then simply
			// hit possible data loss if the readout cannot complete in time. It's still a useful option to have.
			// Start by minimizing T_ED and readout time as much as possible, to maximize the chance of it finishing
			// in the allotted timeframe.
			deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_FIXED_READ_TIME_ENABLE, true);
			deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 1);       // 1µs T_ED, minimum safe value.
			deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_NEXT_SEL, 5); // Smallest NEXT_SEL value.

			if (eFPS == DVXeFPS::EFPS_CONSTANT_LOSSY_2000) {
				// 500 µs per frame: 50 x 500 = 25000
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_READ_FIXED, 25000);
			}
			else if (eFPS == DVXeFPS::EFPS_CONSTANT_LOSSY_5000) {
				// 200 µs per frame: 50 x 200 = 10000
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_READ_FIXED, 10000);
			}
			else if (eFPS == DVXeFPS::EFPS_CONSTANT_LOSSY_10000) {
				// 100 µs per frame: 50 x 100 = 5000
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_READ_FIXED, 5000);
			}
		}
		else {
			// For high-speed, variable time readout mode is used (less events, more speed).
			deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_FIXED_READ_TIME_ENABLE, false);

			if (eFPS == DVXeFPS::EFPS_VARIABLE_2000) {
				// 307+192=499µs min readout time.
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 307);      // 307µs T_ED.
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_NEXT_SEL, 15); // Safe default.
			}
			else if (eFPS == DVXeFPS::EFPS_VARIABLE_5000) {
				// 7+192=199µs min readout time.
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 7);        // 7µs T_ED.
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_NEXT_SEL, 15); // Safe default.
			}
			else if (eFPS == DVXeFPS::EFPS_VARIABLE_10000) {
				// 10+89=99µs min readout time.
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 10);      // 10µs T_ED.
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_NEXT_SEL, 7); // Second smallest NEXT_SEL value.
			}
			else if (eFPS == DVXeFPS::EFPS_VARIABLE_15000) {
				// 1+64=65µs min readout time.
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 1);       // 1µs T_ED, minimum safe value.
				deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_NEXT_SEL, 5); // Smallest NEXT_SEL value.
			}
		}

		deviceConfigSet(DVX_DVS_CHIP, DVX_DVS_CHIP_DTAG_CONTROL, DVX_DVS_CHIP_DTAG_CONTROL_START);

		return true;
	}

	/**
	 * Read a packet from the camera and return a variant of any packet. You can use std::visit
	 * with `dv::io::DataReadHandler` to handle each type of packet using callback methods. This method
	 * might not maintain timestamp monotonicity between different stream types.
	 *
	 * @return			A variant containing data packet from the camera.
	 */
	[[nodiscard]] DataReadVariant readNext() {
		if (!keepRunning) {
			return DataReadHandler::OutputFlag::EndOfFile;
		}

		std::array<size_t, 4> indices{std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max(),
			std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()};

		// Find the packet with lowest packet index
		if (!buffers.events.empty()) {
			indices[0] = buffers.events.front().first;
		}
		if (!buffers.triggers.empty()) {
			indices[1] = buffers.triggers.front().first;
		}
		if (!buffers.imu.empty()) {
			indices[2] = buffers.imu.front().first;
		}
		if (!buffers.frames.empty()) {
			indices[3] = buffers.frames.front().first;
		}

		const auto min = std::min_element(indices.begin(), indices.end());
		if (min == indices.end() || *min == std::numeric_limits<size_t>::max()) {
			return DataReadHandler::OutputFlag::Continue;
		}
		// Read the specified packet from the according packet buffer
		switch (std::distance(indices.begin(), min)) {
			case 0: {
				if (const auto events = buffers.popEvents(mTimestampOffset.load(std::memory_order_relaxed));
					events.has_value()) {
					return *events;
				}
				return DataReadHandler::OutputFlag::Continue;
			}
			case 1: {
				if (const auto triggers = buffers.popTriggers(mTimestampOffset.load(std::memory_order_relaxed));
					triggers.has_value()) {
					return *triggers;
				}
				return DataReadHandler::OutputFlag::Continue;
			}
			case 2: {
				if (const auto imuData = buffers.popImu(mTimestampOffset.load(std::memory_order_relaxed));
					imuData.has_value()) {
					return *imuData;
				}
				return DataReadHandler::OutputFlag::Continue;
			}
			case 3: {
				if (const auto frame = buffers.popFrame(mTimestampOffset.load(std::memory_order_relaxed));
					frame.has_value()) {
					return *frame;
				}
				return DataReadHandler::OutputFlag::Continue;
			}
			default:
				return DataReadHandler::OutputFlag::Continue;
		}
	}

	/**
	 * Read next packet from the camera and use a handler object to handle all types of packets. The function returns
	 * a true if end-of-file was not reached, so this function call can be used in a while loop like so:
	 * ```
	 * while (camera.handleNext(handler)) {
	 * 		// While-loop executes after each packet
	 * }
	 * ```
	 * @param handler	Handler instance that contains callback functions to handle different packets.
	 * @return			False to indicate end of data stream, true to continue.
	 */
	[[nodiscard]] bool handleNext(DataReadHandler &handler) {
		std::visit(handler, readNext());
		return !handler.eof;
	}

	/**
	 * Check whether camera is still connected.
	 * @return 		False if camera is disconnected, true if it is still connected and running.
	 * @deprecated 	Please use `isRunning()` method instead.
	 */
	[[deprecated("Please use `isRunning()` method instead.")]] [[nodiscard]] bool isConnected() const {
		return isRunning();
	}

	/**
	 * Check whether camera is connected and active.
	 * @return 		True if it is still connected and running, false if camera is disconnected.
	 */
	[[nodiscard]] bool isRunning() const override {
		return keepRunning;
	}

	/**
	 * Checks whether the camera is a master camera in multiple camera setups. If camera does not have synchronization
	 * cable connected, it will identified as master camera.
	 * @return 		True if camera is master camera, false otherwise.
	 */
	[[nodiscard]] bool isMasterCamera() const {
		switch (discoveryResult.deviceType) {
			case CAER_DEVICE_DAVIS:
				return discoveryResult.deviceInfo.davisInfo.deviceIsMaster;
			case CAER_DEVICE_DVS128:
				return discoveryResult.deviceInfo.dvs128Info.deviceIsMaster;
			case CAER_DEVICE_EDVS:
				return discoveryResult.deviceInfo.edvsInfo.deviceIsMaster;
			case CAER_DEVICE_DVS132S:
				return discoveryResult.deviceInfo.dvs132sInfo.deviceIsMaster;
			case CAER_DEVICE_DVXPLORER:
				return discoveryResult.deviceInfo.dvXplorerInfo.deviceIsMaster;
			case CAER_DEVICE_SAMSUNG_EVK:
				return true;
			default:
				throw dv::exceptions::InvalidArgument<int>(
					"Device does not support master camera mode", discoveryResult.deviceType);
		}
	}

	/**
	 * Get the configured IMU measurement rate. DVXplorer cameras support individual rates for accelerometer and
	 * gyroscope, in the case camera configured to have different rates, this function return the lowest value.
	 * @return IMU rate in Hz.
	 */
	[[nodiscard]] float getImuRate() const {
		switch (discoveryResult.deviceType) {
			case CAER_DEVICE_DVXPLORER: {
				auto accelRate = boschAccRateToFreq(deviceConfigGet(DVX_IMU, DVX_IMU_ACCEL_DATA_RATE));
				auto gyroRate  = boschGyroRateToFreq(deviceConfigGet(DVX_IMU, DVX_IMU_GYRO_DATA_RATE));
				return std::min(accelRate, gyroRate);
			}
			case CAER_DEVICE_DAVIS: {
				auto value = deviceConfigGet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER);
				return 1000.f / (1.f + static_cast<float>(value));
			}
			default:
				throw dv::exceptions::InvalidArgument<int>("Unsupported device type", discoveryResult.deviceType);
		}
	}

	/**
	 * Get IMU production model name.
	 * @return String containing production model name of the camera on-board IMU.
	 */
	[[nodiscard]] std::string getImuName() const {
		switch (discoveryResult.deviceType) {
			case CAER_DEVICE_DVXPLORER:
				return dv::io::internal::imuModelString(discoveryResult.deviceInfo.dvXplorerInfo.imuType);
			case CAER_DEVICE_DAVIS:
				return dv::io::internal::imuModelString(discoveryResult.deviceInfo.davisInfo.imuType);
			default:
				return "None";
		}
	}

	/**
	 * Return pixel pitch distance for the connected camera model. The value is returned in meters, it is:
	 * - DVXplorer Lite - 18 micrometers (1.8e-5)
	 * - DVXplorer and DVXplorer Mini - 9 micrometers (9e-6)
	 * - DAVIS346 and DAVIS240 - 18.5 micrometers (1.85e-5)
	 * @return 	Pixel pitch distance in meters according to the connected device, returns `std::nullopt` if device
	 * 			can't be reliably identified.
	 */
	[[nodiscard]] std::optional<float> getPixelPitch() const noexcept {
		switch (discoveryResult.deviceType) {
			case CAER_DEVICE_DVXPLORER: {
				if (discoveryResult.deviceInfo.dvXplorerInfo.chipID == DVXPLORER_LITE_CHIP_ID) {
					return 18e-6f;
				}
				else if (discoveryResult.deviceInfo.dvXplorerInfo.chipID == DVXPLORER_CHIP_ID) {
					return 9e-6f;
				}
				else {
					return std::nullopt;
				}
			}
			case CAER_DEVICE_DAVIS: {
				return 18.5e-6f;
			}
			default:
				return std::nullopt;
		}
	}

	/**
	 * Get the timestamp offset.
	 * @return      Absolute timestamp offset value.
	 */
	[[nodiscard]] int64_t getTimestampOffset() const {
		return mTimestampOffset;
	}

	/**
	 * Set a new timestamp offset value for the camera. This will cause to drop any buffered data captured before
	 * calling this method.
	 * @param timestampOffset   New timestamp offset value in microseconds.
	 */
	void setTimestampOffset(const int64_t timestampOffset) {
		mTimestampOffset = timestampOffset;
		buffers.clearBuffers();
	}

	/**
	 * Get latest timestamp of event data stream that has been read from the capture class.
	 * @return Latest processed event timestamp; returns -1 if no data was processed or stream is unavailable.
	 */
	[[nodiscard]] int64_t getEventSeekTime() const {
		return buffers.eventStreamSeek;
	}

	/**
	 * Get latest timestamp of frames stream that has been read from the capture class.
	 * @return Latest processed frame timestamp; returns -1 if no data was processed or stream is unavailable.
	 */
	[[nodiscard]] int64_t getFrameSeekTime() const {
		return buffers.framesStreamSeek;
	}

	/**
	 * Get latest timestamp of imu data that has been read from the capture class.
	 * @return Latest processed imu data timestamp; returns -1 if no data was processed or stream is unavailable.
	 */
	[[nodiscard]] int64_t getImuSeekTime() const {
		return buffers.imuStreamSeek;
	}

	/**
	 * Get latest timestamp of trigger data stream that has been read from the capture class.
	 * @return Latest processed trigger timestamp; returns -1 if no data was processed or stream is unavailable.
	 */
	[[nodiscard]] int64_t getTriggerSeekTime() const {
		return buffers.triggerStreamSeek;
	}
};

} // namespace dv::io

// fmt compatibility for enum class printing.
template<>
struct fmt::formatter<dv::io::CameraCapture::DVXeFPS> : fmt::ostream_formatter {};
