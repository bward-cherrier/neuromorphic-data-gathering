#pragma once

#include "../core/core.hpp"
#include "../core/frame.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"
#include "camera_capture.hpp"
#include "reader.hpp"
#include "support/utils.hpp"
#include "support/xml_config_io.hpp"
#include "write_only_file.hpp"

namespace dv::io {

namespace fs = std::filesystem;

class StereoCameraWriter;

class MonoCameraWriter {
public:
	/**
	 * A configuration structure for the MonoCameraWriter.
	 */
	class Config {
	private:
		std::map<std::string, std::string> customDataStreams;

		std::map<std::string, std::map<std::string, dv::io::support::VariantValueOwning>> customDataStreamsMetadata;

	public:
		/**
		 * Compression type for this file.
		 */
		dv::CompressionType compression;

		/**
		 * Camera name that produces the data, usually contains production serial number.
		 */
		std::string cameraName;

		/**
		 * Add a metadata entry for a data type stream.
		 * @param name 				Name of the stream.
		 * @param metadataEntry 	Metadata entry consisting of a pair, where first element is the key name of the
		 * 							stream and second element is the value.
		 */
		void addStreamMetadata(
			const std::string &name, const std::pair<std::string, dv::io::support::VariantValueOwning> &metadataEntry) {
			auto metadata = customDataStreamsMetadata.find(name);
			if (metadata != customDataStreamsMetadata.end()) {
				metadata->second.insert(metadataEntry);
			}
			else {
				std::map<std::string, dv::io::support::VariantValueOwning> metadataList;
				metadataList.insert(metadataEntry);
				customDataStreamsMetadata.insert(std::make_pair(name, metadataList));
			}
		}

		/**
		 * Add an event stream with a given resolution.
		 * @param resolution 	Resolution of the event sensor.
		 * @param name 			Name of the stream
		 * @param source 		Name of the source camera.
		 */
		void addEventStream(const cv::Size &resolution, const std::string &name = "events",
			const std::optional<std::string> &source = std::nullopt) {
			addStream<dv::EventPacket>(name, source);
			addStreamMetadata(name, std::make_pair("sizeX", resolution.width));
			addStreamMetadata(name, std::make_pair("sizeY", resolution.height));
		}

		/**
		 * Add a frame stream with a given resolution.
		 * @param resolution 	Resolution of the frame sensor.
		 * @param name 			Name of the stream
		 * @param source 		Name of the source camera.
		 */
		void addFrameStream(const cv::Size &resolution, const std::string &name = "frames",
			const std::optional<std::string> &source = std::nullopt) {
			if (resolution.area() <= 0) {
				throw std::invalid_argument("Please provide a valid frame stream resolution for the MonoCameraWriter!");
			}

			addStream<dv::Frame>(name, source);
			addStreamMetadata(name, std::make_pair("sizeX", resolution.width));
			addStreamMetadata(name, std::make_pair("sizeY", resolution.height));
		}

		/**
		 * Add an imu data stream.
		 * @param name Stream name, with a default value of "imu".
		 */
		void addImuStream(const std::string &name = "imu", const std::optional<std::string> &source = std::nullopt) {
			addStream<dv::IMUPacket>(name, source);
		}

		/**
		 * Add a trigger stream.
		 * @param name Stream name, with a default value of "triggers".
		 */
		void addTriggerStream(
			const std::string &name = "triggers", const std::optional<std::string> &source = std::nullopt) {
			addStream<dv::TriggerPacket>(name, source);
		}

		/**
		 * Add a stream of given data type.
		 * @tparam PacketType 	Stream data packet type.
		 * @param name 			Name for the stream.
		 * @param source		Camera name for the source of the data, usually a concatenation of "MODEL_SERIAL",
		 * 						e.g. "DVXplorer_DXA000000"
		 */
		template<class PacketType>
		void addStream(const std::string &name, const std::optional<std::string> &source = std::nullopt) {
			if (name.empty()) {
				throw dv::exceptions::InvalidArgument<std::string>("Stream name cannot be empty", name);
			}

			if (customDataStreams.contains(name)) {
				throw dv::exceptions::InvalidArgument<std::string>(
					"Writer already contains a stream with the given name", name);
			}

			customDataStreams.insert(std::make_pair(name, PacketType::TableType::identifier));
			if (source.has_value()) {
				if (source->empty()) {
					throw dv::exceptions::InvalidArgument<std::string>("Source camera name cannot be empty", *source);
				}

				addStreamMetadata(name, std::make_pair("source", *source));
			}
			else {
				// Use the preconfigured camera name
				addStreamMetadata(name, std::make_pair("source", cameraName));
			}
		}

		/**
		 * Parse resolution of the stream from metadata of the stream. Resolution should be
		 * set as two metadata parameters: "sizeX" and "sizeY" parameters.
		 * @param name 	Stream name.
		 * @return 		Configured resolution. `std::nullopt` if unavailable or incorrectly configured.
		 */
		[[nodiscard]] std::optional<cv::Size> findStreamResolution(const std::string &name) const {
			const auto metadata = customDataStreamsMetadata.find(name);
			if (metadata == customDataStreamsMetadata.end()) {
				return std::nullopt;
			}

			const auto sizeX = metadata->second.find("sizeX");
			const auto sizeY = metadata->second.find("sizeY");

			if (sizeX == metadata->second.end() || sizeY == metadata->second.end()) {
				return std::nullopt;
			}

			const int *width  = std::get_if<int>(&sizeX->second);
			const int *height = std::get_if<int>(&sizeY->second);

			if (!width || !height) {
				return std::nullopt;
			}

			return cv::Size(*width, *height);
		}

		/**
		 * Create a config instance
		 * @param cameraName
		 * @param compression
		 */
		explicit Config(const std::string &cameraName, CompressionType compression = CompressionType::LZ4) :
			compression(compression),
			cameraName(cameraName) {
			if (cameraName.empty()) {
				throw dv::exceptions::InvalidArgument<std::string>("Camera name can't be an empty string.", cameraName);
			}
		}

		friend class dv::io::MonoCameraWriter;
		friend class dv::io::StereoCameraWriter;
	};

	/**
	 * Generate a config for a writer that will expect a stream of events only.
	 * @param cameraName 		Name of the camera.
	 * @param resolution 		Camera sensor resolution.
	 * @param compression 		Compression type.
	 * @return 					A config template for MonoCameraWriter.
	 */
	static Config EventOnlyConfig(const std::string &cameraName, const cv::Size &resolution,
		dv::CompressionType compression = dv::CompressionType::LZ4) {
		auto config = Config(cameraName, compression);
		config.addEventStream(resolution);
		return config;
	}

	/**
	 * Generate a config for a writer that will expect a stream of frames only.
	 * @param cameraName 		Name of the camera.
	 * @param resolution 		Camera sensor resolution.
	 * @param compression 		Compression type.
	 * @return 					A config template for MonoCameraWriter.
	 */
	static Config FrameOnlyConfig(const std::string &cameraName, const cv::Size &resolution,
		dv::CompressionType compression = dv::CompressionType::LZ4) {
		auto config = Config(cameraName, compression);
		config.addFrameStream(resolution);
		return config;
	}

	/**
	 * Generate a config for a writer that will expect data from a DVS camera - events, IMU, triggers.
	 * @param cameraName 		Name of the camera.
	 * @param resolution 		Camera sensor resolution.
	 * @param compression 		Compression type.
	 * @return 					A config template for MonoCameraWriter.
	 */
	static Config DVSConfig(const std::string &cameraName, const cv::Size &resolution,
		dv::CompressionType compression = dv::CompressionType::LZ4) {
		auto config = Config(cameraName, compression);
		config.addEventStream(resolution);
		config.addImuStream();
		config.addTriggerStream();
		return config;
	}

	/**
	 * Generate a config for a writer that will expect data from a DAVIS camera - frames, events, IMU, triggers.
	 * @param cameraName 		Name of the camera.
	 * @param resolution 		Camera sensor resolution.
	 * @param compression 		Compression type.
	 * @return 					A config template for MonoCameraWriter.
	 */
	static Config DAVISConfig(const std::string &cameraName, const cv::Size &resolution,
		dv::CompressionType compression = dv::CompressionType::LZ4) {
		auto config = Config(cameraName, compression);
		config.addEventStream(resolution);
		config.addFrameStream(resolution);
		config.addImuStream();
		config.addTriggerStream();
		return config;
	}

	/**
	 * Generate a config from a camera capture instance, this only checks whether camera provides
	 * frame data stream or not and enables all available streams to be recorded.
	 * @param capture 			Camera capture class instance.
	 * @param compression 		Compression type.
	 * @return 					A config template for MonoCameraWriter.
	 */
	static Config CaptureConfig(
		const dv::io::CameraCapture &capture, dv::CompressionType compression = dv::CompressionType::LZ4) {
		if (capture.isFrameStreamAvailable()) {
			return DAVISConfig(capture.getCameraName(), *capture.getEventResolution(), compression);
		}
		else {
			return DVSConfig(capture.getCameraName(), *capture.getEventResolution(), compression);
		}
	}

private:
	// Pack up imu and trigger in batches of 20 measurements
	size_t mPackagingCount = 20;

	MonoCameraWriter::Config inputConfig;

	struct StreamDescriptor {
		uint32_t id;
		const dv::types::Type *type;
		int64_t lastTimestamp;
		void *elementBuffer;

		std::function<void(void *)> freeElementBufferCall = nullptr;

		~StreamDescriptor() {
			if (freeElementBufferCall != nullptr && elementBuffer != nullptr) {
				freeElementBufferCall(elementBuffer);
			}
		}

		StreamDescriptor(uint32_t id, const types::Type *type) :
			id(id),
			type(type),
			lastTimestamp(-1),
			elementBuffer(nullptr) {
		}
	};

	typedef std::map<std::string, StreamDescriptor> StreamDescriptorMap;

	StreamDescriptorMap mOutputStreamDescriptors;

	dv::io::support::XMLTreeNode mRoot;
	std::shared_ptr<dv::io::WriteOnlyFile> mOutput;

	static void validateConfig(const MonoCameraWriter::Config &config) {
		if (config.customDataStreams.empty()) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"No output streams enabled! Output configuration must have at least one output stream enabled.",
				config.customDataStreams.size());
		}
	}

	[[nodiscard]] std::string createHeader(
		const MonoCameraWriter::Config &config, const dv::io::support::TypeResolver &resolver) {
		validateConfig(config);

		mRoot = dv::io::support::XMLTreeNode("outInfo");

		int32_t index = 0;
		// Create a new config instance so not to modify the one passed in parameters
		inputConfig = config;

		for (const auto &stream : inputConfig.customDataStreams) {
			mOutputStreamDescriptors.insert(std::make_pair(
				stream.first, StreamDescriptor(index, resolver(dv::types::IdentifierStringToId(stream.second)))));
			auto &customNode = mRoot.mChildren.emplace_back(std::to_string(index++));
			customNode.mAttributes.emplace_back("originalModuleName").mValue = "MonoCameraWriter";
			customNode.mAttributes.emplace_back("originalOutputName").mValue = stream.first;
			customNode.mAttributes.emplace_back("compression").mValue = dv::EnumNameCompressionType(config.compression);
			customNode.mAttributes.emplace_back("typeDescription").mValue = "Custom data type output.";
			customNode.mAttributes.emplace_back("typeIdentifier").mValue  = stream.second;

			auto &info = customNode.mChildren.emplace_back("info");

			info.mAttributes.emplace_back("source").mValue = inputConfig.cameraName;

			auto metadata = inputConfig.customDataStreamsMetadata.find(stream.first);
			if (stream.second == dv::EventPacketIdentifier() || stream.second == dv::FrameIdentifier()) {
				const auto resolution = inputConfig.findStreamResolution(stream.first);
				if (!resolution.has_value() || resolution->area() <= 0) {
					throw dv::exceptions::RuntimeError(
						fmt::format("Stream [{}] does not have properly configured resolution", stream.first));
				}
			}

			if (metadata != inputConfig.customDataStreamsMetadata.end()) {
				for (const auto &meta : metadata->second) {
					info.mAttributes.emplace_back(meta.first).mValue = meta.second;
				}
			}
		}

		dv::io::support::XMLConfigWriter xml(mRoot);
		return xml.getXMLContent();
	}

	template<class PacketType>
	[[nodiscard]] StreamDescriptorMap::iterator findStreamDescriptor(const std::string &streamName) {
		auto iter = mOutputStreamDescriptors.find(streamName);
		if (iter == mOutputStreamDescriptors.end()) {
			throw dv::exceptions::InvalidArgument<std::string>("No such configured stream!", streamName);
		}

		// Validate the stream type
		constexpr uint32_t typeId = dv::types::IdentifierStringToId(PacketType::TableType::identifier);
		if (iter->second.type->id != typeId) {
			const auto idString = dv::types::IdToIdentifierString(iter->second.type->id);
			throw dv::exceptions::InvalidArgument<std::string>(
				fmt::format("Wrong data type for stream [{}]; stream requires a type identified by [{}]", streamName,
					std::string(idString.begin(), idString.end()), PacketType::TableType::identifier));
		}

		return iter;
	}

	template<class PacketType>
	[[nodiscard]] StreamDescriptorMap::const_iterator findStreamDescriptor(const std::string &streamName) const {
		auto iter = mOutputStreamDescriptors.find(streamName);
		if (iter == mOutputStreamDescriptors.end()) {
			throw dv::exceptions::InvalidArgument<std::string>("No such configured stream!", streamName);
		}

		// Validate the stream type
		constexpr uint32_t typeId = dv::types::IdentifierStringToId(PacketType::TableType::identifier);
		if (iter->second.type->id != typeId) {
			const auto idString = dv::types::IdToIdentifierString(iter->second.type->id);
			throw dv::exceptions::InvalidArgument<std::string>(
				fmt::format("Wrong data type for stream [{}]; stream requires a type identified by [{}]", streamName,
					std::string(idString.begin(), idString.end()), PacketType::TableType::identifier));
		}

		return iter;
	}

	/**
	 * Preconfigured output file constructor. Internal use only, used for multi-camera
	 * recording.
	 * @param outputFile 	WriteOnlyFile instance to write data.
	 * @param config 		Output stream configuration.
	 * @param resolver 		Type resolver for the output file.
	 */
	explicit MonoCameraWriter(const std::shared_ptr<dv::io::WriteOnlyFile> &outputFile,
		const dv::io::MonoCameraWriter::Config &config,
		const dv::io::support::TypeResolver &resolver = dv::io::support::defaultTypeResolver) :
		mOutput(outputFile),
		inputConfig(config) {
		// Call the setter, it will reserve memory requirements according to the default value
		setPackagingCount(mPackagingCount);
	}

	friend class StereoCameraWriter;

public:
	/**
	 * Create an aedat4 file writer with simplified API.
	 * @param aedat4Path 	Path to the output file. The file is going to be overwritten.
	 * @param config 		Writer config. Defines expected output streams and recording metadata.
	 * @param resolver 		Type resolver for the output file.
	 */
	MonoCameraWriter(const fs::path &aedat4Path, const MonoCameraWriter::Config &config,
		const dv::io::support::TypeResolver &resolver = dv::io::support::defaultTypeResolver) :
		mOutput(
			std::make_shared<dv::io::WriteOnlyFile>(aedat4Path, createHeader(config, resolver), config.compression)),
		inputConfig(config.cameraName, config.compression) {
		// Call the setter, it will reserve memory requirements according to the default value
		setPackagingCount(mPackagingCount);
	}

	/**
	 * Create an aedat4 file writer that inspects the capabilities and configuration from a `dv::io::CameraCapture`
	 * class. This will enable all available data streams present from the camera capture.
	 * @param aedat4Path 	Path to the output file. The file is going to be overwritten.
	 * @param capture 		Direct camera capture instance. This is used to inspect the available data streams
	 * 						and metadata of the camera.
	 * @param compression 	Compression to be used for the output file.
	 * @param resolver 		Type resolver for the output file.
	 */
	MonoCameraWriter(const fs::path &aedat4Path, const CameraCapture &capture,
		const CompressionType compression             = CompressionType::LZ4,
		const dv::io::support::TypeResolver &resolver = dv::io::support::defaultTypeResolver) :
		mOutput(std::make_shared<dv::io::WriteOnlyFile>(
			aedat4Path, createHeader(CaptureConfig(capture), resolver), compression)),
		inputConfig(capture.getCameraName(), compression) {
		// Call the setter, it will reserve memory requirements according to the default value
		setPackagingCount(mPackagingCount);
	}

	/**
	 * Write an event packet into the output file.
	 *
	 * The data is passed directly into the serialization procedure without performing copies. Data
	 * is serialized and the actual file IO is performed on a separate thread.
	 * @param events 			Packet of events.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @throws invalid_argument Invalid argument exception is thrown if function is called and compatible output stream
	 * 							was not added during construction.
	 */
	void writeEventPacket(const dv::EventPacket &events, const std::string &streamName = "events") {
		if (events.elements.empty()) {
			return;
		}

		const auto resolution = inputConfig.findStreamResolution(streamName);

		if (!resolution.has_value()) {
			throw dv::exceptions::RuntimeError(
				fmt::format("Output stream [{}] is not enabled or does not have configured resolution", streamName));
		}

		// Event coordinate range-check
		dv::runtime_assert(std::all_of(events.elements.cbegin(), events.elements.cend(),
							   [&resolution](const auto &event) {
								   return event.x() >= 0 && event.y() >= 0 && event.x() < resolution->width
									   && event.y() < resolution->height;
							   }),
			fmt::format("Trying to write an event with out of bounds coordinates for "
						"the configured resolution [{}x{}] of event stream [{}]",
				resolution->width, resolution->height, streamName));

		writePacket<dv::EventPacket>(events, streamName);
	}

	/**
	 * Write an event store into the output file. The store is written by maintaining internal data partial
	 * ordering and fragmentation.
	 *
	 * The data is passed directly into the serialization procedure without performing copies. Data
	 * is serialized and the actual file IO is performed on a separate thread.
	 * @param events 			Store of events.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @throws invalid_argument Invalid argument exception is thrown if function is called and compatible output stream
	 * 							was not added during construction.
	 */
	void writeEvents(const dv::EventStore &events, const std::string &streamName = "events") {
		if (events.isEmpty()) {
			return;
		}

		const auto resolution = inputConfig.findStreamResolution(streamName);

		if (!resolution.has_value()) {
			throw dv::exceptions::RuntimeError(
				fmt::format("Output stream [{}] is not enabled or does not have configured resolution", streamName));
		}

		// Event coordinate range-check
		dv::runtime_assert(std::all_of(events.begin(), events.end(),
							   [&resolution](const auto &event) {
								   return event.x() >= 0 && event.y() >= 0 && event.x() < resolution->width
									   && event.y() < resolution->height;
							   }),
			fmt::format("Trying to write an event with out of bounds coordinates for "
						"the configured resolution [{}x{}] of event stream [{}]",
				resolution->width, resolution->height, streamName));

		for (const auto &partial : events.dataPartials_) {
			writePacket<dv::EventPacket>(*partial.data_, streamName);
		}
	}

	/**
	 * Write a frame image into the file.
	 *
	 * The data is passed directly into the serialization procedure without performing copies. Data
	 * is serialized and the actual file IO is performed on a separate thread.
	 *
	 * NOTE: if the frame contains an empty image, it will be ignored and not recorded.
	 * @param frame 			A frame to be written.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @throws invalid_argument Invalid argument exception is thrown if function is called and compatible output stream
	 * 							was not added during construction.
	 */
	void writeFrame(const dv::Frame &frame, const std::string &streamName = "frames") {
		if (frame.image.empty()) {
			return;
		}

		const cv::Size &resolution = frame.image.size();

		const auto configuredResolution = inputConfig.findStreamResolution(streamName);

		if (!configuredResolution.has_value()) {
			throw dv::exceptions::RuntimeError(
				fmt::format("Output stream [{}] is not enabled or does not have configured resolution", streamName));
		}

		if (resolution.width > configuredResolution->width || resolution.height > configuredResolution->height) {
			throw dv::exceptions::InvalidArgument<std::string>(
				fmt::format("Trying to write a frame with larger dimension than configured for the stream, expected "
							"dimensions are not larger then [{}x{}].",
					configuredResolution->width, configuredResolution->height),
				fmt::format("{}x{}", resolution.width, resolution.height));
		}

		writePacket<dv::Frame>(frame, streamName);
	}

	/**
	 * Write a packet of imu data into the file.
	 *
	 * The data is passed directly into the serialization procedure without performing copies. Data
	 * is serialized and the actual file IO is performed on a separate thread.
	 * @param packet 			IMU measurement packet.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @throws invalid_argument Invalid argument exception is thrown if function is called and compatible output stream
	 * 							was not added during construction.
	 */
	void writeImuPacket(const dv::IMUPacket &packet, const std::string &streamName = "imu") {
		writePacket<dv::IMUPacket>(packet, streamName);
	}

	/**
	 * Write an IMU measurement.
	 *
	 * This function is not immediate, it batches the measurements until a configured amount is reached, only then the
	 * data is passed to the serialization step. Only then the data will be passed to the file write IO thread. If the
	 * file is closed (the object gets destroyed), destructor will dump the rest of the buffered measurements to the
	 * serialization step.
	 * @param imu 				A single IMU measurement.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @throws invalid_argument Invalid argument exception is thrown if function is called and compatible output stream
	 * 							was added enabled during construction.
	 * @sa setPackagingCount
	 */
	void writeImu(const dv::IMU &imu, const std::string &streamName = "imu") {
		writePacketElement<dv::IMUPacket>(imu, streamName);
	}

	/**
	 * Write a packet of trigger data into the file.
	 *
	 * The data is passed directly into the serialization procedure without performing copies. Data
	 * is serialized and the actual file IO is performed on a separate thread.
	 * @param packet 			Trigger data packet.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @throws invalid_argument Invalid argument exception is thrown if function is called and compatible output stream
	 * 							was added enabled during construction.
	 */
	void writeTriggerPacket(const dv::TriggerPacket &packet, const std::string &streamName = "triggers") {
		writePacket<dv::TriggerPacket>(packet, streamName);
	}

	/**
	 * Write a packet into a named stream.
	 * @tparam PacketType 	Type of data packet.
	 * @param stream 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @param packet 		Data packet
	 * @throws InvalidArgument	If a stream with given name is not configured.
	 * @throws InvalidArgument	If a stream with given name is configured for a different type of data packet.
	 * @throws invalid_argument Invalid argument exception is thrown if function is called and compatible output stream
	 * 							was added enabled during construction.
	 */
	template<class PacketType>
	void writePacket(const PacketType &packet, const std::string &stream) {
		const auto streamDesc = findStreamDescriptor<PacketType>(stream);

		// Validate timestamp monotonicity, find the timestamp in a complex way!
		int64_t timestamp;
		if constexpr (dv::concepts::TimestampedByMember<PacketType>) {
			timestamp = packet.timestamp;
		}
		else if constexpr (dv::concepts::TimestampedByAccessor<PacketType>) {
			timestamp = packet.timestamp();
		}
		else if constexpr (dv::concepts::HasElementsVector<PacketType>) {
			if (packet.elements.empty()) {
				return;
			}

			if constexpr (dv::concepts::HasTimestampedElementsVectorByMember<PacketType>) {
				// The timestamps within the packet is in wrong order
				if (packet.elements.back().timestamp < packet.elements.front().timestamp) {
					throw dv::exceptions::InvalidArgument<int64_t>(
						"Passing data with out-of-order timestamps!", packet.elements.back().timestamp);
				}
				timestamp = packet.elements.back().timestamp;
			}
			else if constexpr (dv::concepts::HasTimestampedElementsVectorByAccessor<PacketType>) {
				// The timestamps within the packet is in wrong order
				if (packet.elements.back().timestamp() < packet.elements.front().timestamp()) {
					throw dv::exceptions::InvalidArgument<int64_t>(
						"Passing data with out-of-order timestamps!", packet.elements.back().timestamp());
				}
				timestamp = packet.elements.back().timestamp();
			}
		}
		if (timestamp < streamDesc->second.lastTimestamp) {
			throw dv::exceptions::InvalidArgument<int64_t>(
				"Writing data into stream with out-of-order timestamp!", timestamp);
		}

		// Write and update the stream timestamp
		mOutput->write(reinterpret_cast<const void *>(&packet), *streamDesc->second.type, streamDesc->second.id);
		streamDesc->second.lastTimestamp = timestamp;
	}

	/**
	 * Write a Trigger measurement.
	 *
	 * This function is not immediate, it batches the measurements until a configured amount is reached, only then the
	 * data is passed to the serialization step. Only then the data will be passed to the file write IO thread. If the
	 * file is closed (the object gets destroyed), destructor will dump the rest of the buffered measurements to the
	 * serialization step.
	 * @param imu 				A single Trigger measurement.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @throws invalid_argument Invalid argument exception is thrown if function is called and compatible output stream
	 * 							was not added during construction.
	 * @sa setPackagingCount
	 */
	void writeTrigger(const dv::Trigger &trigger, const std::string &streamName = "triggers") {
		writePacketElement<dv::TriggerPacket>(trigger, streamName);
	}

	/**
	 * Write a single element into packet. A packet will be created per stream and element will be
	 * added until packaging count is reached, at that point the packet will be written do disk.
	 * @tparam PacketType 	Type of the packet to hold the elements.
	 * @tparam ElementType 	Type of an element.
	 * @param element 		Element to be saved.
	 * @param streamName 	Name of the stream, an empty string will match first stream with compatible data type.
	 */
	template<class PacketType, class ElementType>
	requires dv::concepts::HasElementsVector<PacketType>
	void writePacketElement(const ElementType &element, const std::string &streamName) {
		const auto streamDesc = findStreamDescriptor<PacketType>(streamName);

		// The packet timestamp is not monotonically increasing compared to last timestamp
		if (element.timestamp < streamDesc->second.lastTimestamp) {
			throw dv::exceptions::InvalidArgument<int64_t>(
				"Passing data element with out-of-order timestamps!", element.timestamp);
		}

		if (streamDesc->second.elementBuffer == nullptr) {
			auto packet = new PacketType;
			packet->elements.reserve(mPackagingCount);
			streamDesc->second.elementBuffer         = reinterpret_cast<void *>(packet);
			streamDesc->second.freeElementBufferCall = [](void *ptr) {
				delete reinterpret_cast<PacketType *>(ptr);
			};
		}
		auto buffer = reinterpret_cast<PacketType *>(streamDesc->second.elementBuffer);
		buffer->elements.push_back(element);
		if (buffer->elements.size() >= mPackagingCount) {
			writePacket<PacketType>(*buffer, streamName);
			delete buffer;
			streamDesc->second.elementBuffer = nullptr;
		}

		streamDesc->second.lastTimestamp = element.timestamp;
	}

	/**
	 * Set the size batch size for trigger and imu buffering. The single measurements passed into `writeTrigger` and
	 * `writeImu` functions will packed into batches of the given size before writing to the file.
	 *
	 * A packaging value of 0 or 1 will cause each measurement to be serialized immediately.
	 *
	 * @param packagingCount	Trigger and IMU measurement packet size that is batched up using the `writeImu`
	 * 							and `writeTrigger` functions.
	 * @sa writeTrigger
	 * @sa writeImu
	 */
	void setPackagingCount(size_t packagingCount) {
		mPackagingCount = packagingCount;
	}

	/**
	 * Check if the event stream is configured for this writer.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @return 	True if event stream is configured, false otherwise.
	 */
	[[nodiscard]] bool isEventStreamConfigured(const std::string &streamName = "events") const {
		return isStreamConfigured<dv::EventPacket>(streamName);
	}

	/**
	 * Check if the frame stream is configured for this writer.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @return 	True if frame stream is configured, false otherwise.
	 */
	[[nodiscard]] bool isFrameStreamConfigured(const std::string &streamName = "frames") const {
		return isStreamConfigured<dv::Frame>(streamName);
	}

	/**
	 * Check if the IMU stream is configured for this writer.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @return 	True if IMU stream is configured, false otherwise.
	 */
	[[nodiscard]] bool isImuStreamConfigured(const std::string &streamName = "imu") const {
		return isStreamConfigured<dv::IMUPacket>(streamName);
	}

	/**
	 * Check if the trigger stream is configured for this writer.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @return 	True if trigger stream is configured, false otherwise.
	 */
	[[nodiscard]] bool isTriggerStreamConfigured(const std::string &streamName = "triggers") const {
		return isStreamConfigured<dv::TriggerPacket>(streamName);
	}

	/**
	 * Check whether a stream with given name and compatible data type is configured.
	 * @tparam PacketType 	Type of the packet to hold the elements.
	 * @param streamName 		Name of the stream, an empty string will match first stream with compatible data type.
	 * @return
	 */
	template<class PacketType>
	[[nodiscard]] bool isStreamConfigured(const std::string &streamName) const {
		try {
			return findStreamDescriptor<PacketType>(streamName) != mOutputStreamDescriptors.end();
		}
		catch (std::exception &e) {
			return false;
		}
	}

	~MonoCameraWriter() {
		for (auto &[_, stream] : mOutputStreamDescriptors) {
			if (stream.elementBuffer != nullptr) {
				mOutput->write(stream.elementBuffer, *stream.type, stream.id);
			}
		}
	}
};

} // namespace dv::io
