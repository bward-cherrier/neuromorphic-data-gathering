#pragma once

#include "../core/frame.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"
#include "camera_input_base.hpp"
#include "data_read_handler.hpp"
#include "read_only_file.hpp"

#include <functional>
#include <optional>

namespace dv::io {

namespace fs = std::filesystem;

/**
 * A convenience class for reading recordings containing data captured from a single camera.
 * Looks for an event, frame, imu, and trigger streams within the supplied aedat4 file.
 */
class MonoCameraRecording : public CameraInputBase {
private:
	std::shared_ptr<ReadOnlyFile> mReader = nullptr;
	FileInfo mInfo;

	std::string mCameraName;
	dv::cvector<FileDataDefinition>::const_iterator mPacketIter;
	bool eofReached = false;

	struct StreamDescriptor {
		size_t mSeekIndex = 0;
		dv::io::Stream mStream;
		std::map<std::string, std::string> mMetadata;

		explicit StreamDescriptor(const Stream &stream) : mStream(stream) {
			const auto &children = mStream.mXMLNode.mChildren;
			const auto infoNode  = std::find(children.begin(), children.end(), "info");
			if (infoNode == children.end()) {
				return;
			}

			for (const auto &attribute : infoNode->mAttributes) {
				const auto value = std::get_if<std::string>(&attribute.mValue);
				if (value != nullptr) {
					mMetadata.insert(std::make_pair(attribute.mName, *value));
				}
			}
		}
	};

	typedef std::map<std::string, StreamDescriptor> StreamInfoMap;

	StreamInfoMap mStreamInfo;

	[[nodiscard]] const dv::io::Stream *getStream(const int streamId) const {
		auto iter
			= std::find_if(mInfo.mStreams.begin(), mInfo.mStreams.end(), [streamId](const dv::io::Stream &stream) {
				  return streamId == stream.mId;
			  });

		if (iter == mInfo.mStreams.end()) {
			return nullptr;
		}
		else {
			return &*iter;
		}
	}

	void parseStreamIds() {
		for (const auto &stream : mInfo.mStreams) {
			if (stream.getSource() == mCameraName) {
				mStreamInfo.insert(std::make_pair(stream.mName, StreamDescriptor(stream)));
			}
		}
	}

	/**
	 * Trim a vector containing elements with a timestamp. Retains only the data within [start; end).
	 * @tparam VectorClass  The class of the vector
	 * @param vector        The vector of data
	 * @param start         Start timestamp (inclusive start of range)
	 * @param end           End timestamp (exclusive end of range)
	 */
	template<class VectorClass>
	requires dv::concepts::MutableIterable<VectorClass>
	static void trimVector(VectorClass &vector, int64_t start, int64_t end) {
		const auto frontCut = std::find_if(vector.begin(), vector.end(), [start](const auto &dataPoint) {
			return dataPoint.timestamp >= start;
		});
		vector.erase(vector.begin(), frontCut);
		const auto backCut = std::find_if(vector.begin(), vector.end(), [end](const auto &dataPoint) {
			return dataPoint.timestamp >= end;
		});
		vector.erase(backCut, vector.end());
	}

	template<class DataType>
	requires dv::concepts::FlatbufferPacket<DataType>
	StreamInfoMap::iterator getStreamInfo(const std::string &streamName) {
		StreamInfoMap::iterator iter;

		static constexpr int32_t typeId = dv::types::IdentifierStringToId(DataType::TableType::identifier);
		if (streamName.empty()) {
			iter = std::find_if(mStreamInfo.begin(), mStreamInfo.end(), [](const auto &m) {
				return m.second.mStream.mType.id == typeId;
			});
			if (iter == mStreamInfo.end()) {
				throw dv::exceptions::InvalidArgument<std::string>(
					"Stream of a requested type is not available", DataType::TableType::identifier);
			}
		}
		else {
			iter = mStreamInfo.find(streamName);
			if (iter == mStreamInfo.end()) {
				throw dv::exceptions::InvalidArgument<std::string>(
					"Stream with a given name is not available", streamName);
			}
			if (iter->second.mStream.mType.id != typeId) {
				throw dv::exceptions::InvalidArgument<std::string>(
					fmt::format("Wrong data type for stream [{}]; stream requires a type identified by [{}]",
						streamName, iter->second.mStream.mTypeIdentifier),
					DataType::TableType::identifier);
			}
		}

		return iter;
	}

	template<class DataType>
	requires dv::concepts::FlatbufferPacket<DataType>
	StreamInfoMap::const_iterator getStreamInfo(const std::string &streamName) const {
		StreamInfoMap::const_iterator iter;

		static constexpr int32_t typeId = dv::types::IdentifierStringToId(DataType::TableType::identifier);
		if (streamName.empty()) {
			iter = std::find_if(mStreamInfo.begin(), mStreamInfo.end(), [](const auto &m) {
				return m.second.mStream.mType.id == typeId;
			});
			if (iter == mStreamInfo.end()) {
				throw dv::exceptions::InvalidArgument<std::string>(
					"Stream of a requested type is not available", DataType::TableType::identifier);
			}
		}
		else {
			iter = mStreamInfo.find(streamName);
			if (iter == mStreamInfo.end()) {
				throw dv::exceptions::InvalidArgument<std::string>(
					"Stream with a given name is not available", streamName);
			}
			if (iter->second.mStream.mType.id != typeId) {
				throw dv::exceptions::InvalidArgument<std::string>(
					fmt::format("Wrong data type for stream [{}]; stream requires a type identified by [{}]",
						streamName, iter->second.mStream.mTypeIdentifier),
					DataType::TableType::identifier);
			}
		}

		return iter;
	}

	template<class DataType>
	requires dv::concepts::FlatbufferPacket<DataType>
	[[nodiscard]] std::shared_ptr<DataType> getNextPacket(StreamDescriptor &streamInfo) {
		static constexpr int32_t typeId = dv::types::IdentifierStringToId(DataType::TableType::identifier);
		if (streamInfo.mStream.mType.id != typeId) {
			throw dv::exceptions::InvalidArgument<std::string>(
				fmt::format("Wrong data type for stream [{}]; stream requires a type identified by [{}]",
					streamInfo.mStream.mName, streamInfo.mStream.mTypeIdentifier),
				DataType::TableType::identifier);
		}

		const auto &table = mInfo.mPerStreamDataTables[streamInfo.mStream.mId].Table;
		if (streamInfo.mSeekIndex >= table.size()) {
			eofReached = true;
			return nullptr;
		}

		auto [packet, _] = mReader->read(table[streamInfo.mSeekIndex++]);
		return packet->moveToSharedPtr<DataType>();
	}

public:
	/**
	 * Create a reader that reads single camera data recording from a pre-constructed file reader.
	 * @param fileReader 	A pointer for pre-constructed file reader.
	 * @param cameraName	Name of the camera in the recording. If an empty string is passed (the default value),
	 * reader will try detect the name of the camera. In case recording contains more than one camera, it will choose
	 * the first encountered name and ignore streams that were recorded by a different camera.
	 */
	explicit MonoCameraRecording(const std::shared_ptr<ReadOnlyFile> &fileReader, const std::string &cameraName = "") :
		mReader(fileReader) {
		mInfo       = mReader->getFileInfo();
		mPacketIter = mInfo.mDataTable.Table.cbegin();

		if (cameraName.empty()) {
			// Try to detect camera name, just the first found name is fine
			for (const auto &stream : mInfo.mStreams) {
				if (const auto name = stream.getSource(); name.has_value()) {
					mCameraName = *name;
					break;
				}
			}
		}
		else {
			mCameraName = cameraName;
		}

		parseStreamIds();

		if (mStreamInfo.empty()) {
			throw dv::exceptions::FileError(
				"Provided file reader does not contain any supported data streams for MonoCameraRecording");
		}
	}

	/**
	 * Create a reader that reads single camera data recording from an aedat4 file.
	 * @param aedat4Path 	Path to the aedat4 file.
	 * @param cameraName	Name of the camera in the recording. If an empty string is passed (the default value),
	 * reader will try detect the name of the camera. In case recording contains more than one camera, it will choose
	 * the first encountered name and ignore streams that were recorded by a different camera.
	 */
	explicit MonoCameraRecording(const fs::path &aedat4Path, const std::string &cameraName = "") :
		MonoCameraRecording(
			std::make_shared<ReadOnlyFile>(aedat4Path, dv::io::support::defaultTypeResolver), cameraName) {
	}

	/**
	 * Sequential read of a frame, tries reading from stream named "frames". This function increments an internal seek
	 * counter which will return the next frame at each call.
	 * @return	A `dv::Frame` or `std::nullopt` if the frame stream is not available or the end-of-stream was reached.
	 */
	[[nodiscard]] std::optional<dv::Frame> getNextFrame() override {
		return getNextFrame("frames");
	}

	/**
	 * Sequential read of a frame. This function increments an internal seek counter which
	 * will return the next frame at each call.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						frame data type.
	 * @return		A dv::Frame, `std::nullopt` if the frame stream is not available or
	 * 				the end-of-stream was reached.
	 */
	[[nodiscard]] std::optional<dv::Frame> getNextFrame(const std::string &streamName) {
		return getNextStreamPacket<dv::Frame>(streamName);
	}

	/**
	 * Check whether a given stream name is available.
	 *
	 * @param streamName 		Name of the stream.
	 * @return 					True if this stream is available, false otherwise.
	 */
	[[nodiscard]] bool isStreamAvailable(const std::string &streamName) {
		return mStreamInfo.find(streamName) != mStreamInfo.end();
	}

	/**
	 * Return a vector containing all available stream names.
	 * @return 	A list of custom data type stream names.
	 */
	[[nodiscard]] std::vector<std::string> getStreamNames() const {
		std::vector<std::string> names;
		for (const auto &[name, _] : mStreamInfo) {
			names.push_back(name);
		}
		return names;
	}

	/**
	 * Read a custom data type packet sequentially.
	 *
	 * Custom data types are any flatbuffer generated types that are not the following: `dv::EventPacket`,
	 * `dv::TriggerPacket`, `dv::IMUPacket`, `dv::Frame`.
	 * @tparam DataType 	Custom data packet class.
	 * @param streamName 	Name of the stream.
	 * @return 				Next packet within given stream or `std::nullopt` in case of end-of-stream.
	 * @throws InvalidArgument	An exception is thrown if a stream with given name is not found in the file.
	 * @throws InvalidArgument	An exception is thrown if given type does not match the type identifier of the
	 * 							given stream.
	 */
	template<class DataType>
	requires dv::concepts::FlatbufferPacket<DataType>
	[[nodiscard]] std::optional<DataType> getNextStreamPacket(const std::string &streamName) {
		auto packetPtr = getNextPacket<DataType>(getStreamInfo<DataType>(streamName)->second);

		if (packetPtr == nullptr) {
			return std::nullopt;
		}

		return std::move(*packetPtr);
	}

	/**
	 * Sequential read of events, tries reading from stream named "events". This function increments an internal seek
	 * counter which will return the next event batch at each call.
	 * @return	A `dv::EventStore` or `std::nullopt` if the frame stream is not available or the end-of-stream was
	 * reached.
	 */
	[[nodiscard]] std::optional<dv::EventStore> getNextEventBatch() override {
		return getNextEventBatch("events");
	}

	/**
	 * Sequentially read a batch of recorded events. This function increments an internal seek counter which
	 * will return the next batch at each call.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						event data type.
	 * @return		A vector containing events, `std::nullopt` if the event stream is not available or
	 * 				the end-of-stream was reached.
	 */
	[[nodiscard]] std::optional<dv::EventStore> getNextEventBatch(const std::string &streamName) {
		if (auto data = getNextPacket<const dv::EventPacket>(getStreamInfo<dv::EventPacket>(streamName)->second)) {
			return dv::EventStore(data);
		}
		return std::nullopt;
	}

	/**
	 * Sequential read of imu data, tries reading from stream named "imu". This function increments an internal seek
	 * counter which will return the next imu data batch at each call.
	 * @return	A vector or IMU measurements or `std::nullopt` if the imu data stream is not available or the
	 * end-of-stream was reached.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::IMU>> getNextImuBatch() override {
		return getNextImuBatch("imu");
	}

	/**
	 * Sequentially read a batch of recorded imu data. This function increments an internal seek counter which
	 * will return the next batch at each call.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						imu data type.
	 * @return		A vector containing imu data, `std::nullopt` if the imu data stream is not available or
	 * 				the end-of-stream was reached.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::IMU>> getNextImuBatch(const std::string &streamName) {
		auto data = getNextStreamPacket<dv::IMUPacket>(streamName);
		if (data.has_value()) {
			return data->elements;
		}
		return std::nullopt;
	}

	/**
	 * Sequential read of trigger data, tries reading from stream names "triggers". This function increments an internal
	 * seek counter which will return the next trigger data batch at each call.
	 * @return	A vector of trigger data or `std::nullopt` if the frame stream is not available or the end-of-stream was
	 * reached.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::Trigger>> getNextTriggerBatch() override {
		return getNextTriggerBatch("triggers");
	}

	/**
	 * Sequentially read a batch of recorded triggers. This function increments an internal seek counter which
	 * will return the next batch at each call.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						trigger data type.
	 * @return		A vector containing triggers, `std::nullopt` if the trigger stream is not available or
	 * 				the end-of-stream was reached.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::Trigger>> getNextTriggerBatch(const std::string &streamName) {
		std::optional<dv::TriggerPacket> data = getNextStreamPacket<dv::TriggerPacket>(streamName);
		if (data.has_value()) {
			return data->elements;
		}
		return std::nullopt;
	}

	/**
	 * Reset the sequential read function to start from the beginning of the file.
	 */
	void resetSequentialRead() {
		for (auto &[_, streamInfo] : mStreamInfo) {
			streamInfo.mSeekIndex = 0;
		}
		eofReached = false;
	}

	/**
	 * Check whether sequential read functions has not yet reached end-of-stream.
	 * @return 	True if at least one of the streams has reached end-of-stream, false otherwise.
	 */
	[[nodiscard]] bool isRunning() const override {
		return !eofReached;
	}

	/**
	 * Get events within given time range [startTime; endTime).
	 * @param startTime 	Start timestamp of the time range.
	 * @param endTime 		End timestamp of the time range.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						event data type.
	 * @return 				`dv::EventStore` with events in the time range if the event stream is available,
	 * `std::nullopt` otherwise.
	 */
	[[nodiscard]] std::optional<dv::EventStore> getEventsTimeRange(
		const int64_t startTime, const int64_t endTime, const std::string &streamName = "events") {
		if (startTime > endTime) {
			throw dv::exceptions::InvalidArgument<int64_t>(
				fmt::format("Start time [{}] is larger than the end time [{}]", startTime, endTime), startTime);
		}

		auto iter = getStreamInfo<dv::EventPacket>(streamName);
		dv::EventStore store;
		// We subtract 1 from endTime here to get [start; end) range, since read() returns [start; end].
		auto packets = mReader->read(startTime, endTime - 1, iter->second.mStream.mId);
		for (auto &packet : packets) {
			store.add(dv::EventStore(packet.first->moveToSharedPtr<const dv::EventPacket>()));
		}
		return store.sliceTime(startTime, endTime);
	}

	/**
	 * Get frames within given time range [startTime; endTime).
	 * @param startTime 	Start timestamp of the time range.
	 * @param endTime 		End timestamp of the time range.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						frame data type.
	 * @return 				Vector containing frames and timestamps.
	 * @throws InvalidArgument	If frame stream doesn't exists or a stream with given name doesn't exist.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::Frame>> getFramesTimeRange(
		const int64_t startTime, const int64_t endTime, const std::string &streamName = "frames") {
		return getStreamTimeRange<dv::Frame>(startTime, endTime, streamName);
	}

	/**
	 * Get packets from a stream within given period of time. Returns a vector of packets. If a packet contains
	 * elements that are outside of given time range, the internal elements will be cut to match exactly the
	 * [startTime; endTime). If stream does not contain any packets within requested time range, the function
	 * returns an empty vector.
	 * @tparam DataType 	Packet type
	 * @param startTime 	Period start timestamp.
	 * @param endTime 		Period end timestamp.
	 * @param streamName 	Name of the stream, empty string will pick a first stream with matching type.
	 * @return 				A vector of packets containing the data only within [startTime; endTime) period.
	 * @throws InvalidArgument	An exception is thrown if a stream with given name is not found in the file.
	 * @throws InvalidArgument	An exception is thrown if given type does not match the type identifier of the
	 * 							given stream.
	 */
	template<class DataType>
	[[nodiscard]] std::optional<dv::cvector<DataType>> getStreamTimeRange(
		const int64_t startTime, const int64_t endTime, const std::string &streamName) {
		if (startTime > endTime) {
			throw dv::exceptions::InvalidArgument<int64_t>(
				fmt::format("Start time [{}] is larger than the end time [{}]", startTime, endTime));
		}

		auto &streamInfo = getStreamInfo<DataType>(streamName)->second;

		dv::cvector<DataType> data;
		// We subtract 1 from endTime here to get [start; end) range, since read() returns [start; end].
		auto packets = mReader->read(startTime, endTime - 1, streamInfo.mStream.mId);
		if constexpr (dv::concepts::HasElementsVector<DataType>) {
			for (auto iter = packets.begin(); iter < packets.end(); iter++) {
				std::shared_ptr<DataType> dataPacket = iter->first->template moveToSharedPtr<DataType>();
				if (iter == packets.begin() || iter == std::prev(packets.end())) {
					trimVector(dataPacket->elements, startTime, endTime);
				}
				data.push_back(*dataPacket);
			}
		}
		else {
			for (auto &[packet, _] : packets) {
				data.push_back(*(packet->template moveToSharedPtr<DataType>()));
			}
		}
		return data;
	}

	/**
	 * Get IMU data within given time range [startTime; endTime).
	 * @param startTime 	Start timestamp of the time range.
	 * @param endTime 		End timestamp of the time range.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						imu data type.
	 * @return 				Vector containing IMU data if the IMU stream is available, `std::nullopt` otherwise.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::IMU>> getImuTimeRange(
		const int64_t startTime, const int64_t endTime, const std::string &streamName = "imu") {
		auto data = getStreamTimeRange<dv::IMUPacket>(startTime, endTime, streamName);

		if (data.has_value()) {
			dv::cvector<dv::IMU> allData;
			for (auto &packet : *data) {
				std::move(packet.elements.begin(), packet.elements.end(), std::back_inserter(allData));
			}
			// data container is invalidated beyond this point
			return allData;
		}

		return std::nullopt;
	}

	/**
	 * Get trigger data within given time range [startTime; endTime).
	 * @param startTime 	Start timestamp of the time range.
	 * @param endTime 		End timestamp of the time range.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						trigger data type.
	 * @return 				Vector containing triggers if the trigger stream is available, `std::nullopt` otherwise.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::Trigger>> getTriggersTimeRange(
		const int64_t startTime, const int64_t endTime, const std::string &streamName = "triggers") {
		auto data = getStreamTimeRange<dv::TriggerPacket>(startTime, endTime, streamName);
		if (data.has_value()) {
			dv::cvector<dv::Trigger> allData;
			for (auto &packet : *data) {
				std::move(packet.elements.begin(), packet.elements.end(), std::back_inserter(allData));
			}
			// data container is invalidated beyond this point
			return allData;
		}
		return std::nullopt;
	}

	/**
	 * Check whether frame stream is available. Specifically checks whether a stream named "frames" is available since
	 * it's the default stream name for frames.
	 * @return		True if the frame stream is available.
	 */
	[[nodiscard]] inline bool isFrameStreamAvailable() const override {
		return isFrameStreamAvailable("frames");
	}

	/**
	 * Checks whether a frame data stream is present in the file.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						frame data type.
	 * @return 		True if the frames are available, false otherwise.
	 */
	[[nodiscard]] inline bool isFrameStreamAvailable(const std::string &streamName) const {
		static constexpr int32_t typeId = dv::types::IdentifierStringToId(dv::Frame::TableType::identifier);
		return std::any_of(mStreamInfo.begin(), mStreamInfo.end(), [&streamName](const auto &m) {
			return m.second.mStream.mType.id == typeId && (streamName.empty() || streamName == m.first);
		});
	}

	/**
	 * Check whether event stream is available. Specifically checks whether a stream named "events" is available since
	 * it's the default stream name for events.
	 * @return 		True if the event stream is available, false otherwise.
	 */
	[[nodiscard]] inline bool isEventStreamAvailable() const override {
		return isEventStreamAvailable("events");
	}

	/**
	 * Checks whether an event data stream is present in the file.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						event data type.
	 * @return 		True if the events are available, false otherwise.
	 */
	[[nodiscard]] inline bool isEventStreamAvailable(const std::string &streamName) const {
		static constexpr int32_t typeId = dv::types::IdentifierStringToId(dv::EventPacket::TableType::identifier);
		return std::any_of(mStreamInfo.begin(), mStreamInfo.end(), [&streamName](const auto &m) {
			return m.second.mStream.mType.id == typeId && (streamName.empty() || streamName == m.first);
		});
	}

	/**
	 * Check whether imu data stream is available. Specifically checks whether a stream named "imu" is available since
	 * it's the default stream name for imu data.
	 * @return 		True if the imu stream is available, false otherwise.
	 */
	[[nodiscard]] inline bool isImuStreamAvailable() const override {
		return isImuStreamAvailable("imu");
	}

	/**
	 * Checks whether an imu data stream is present in the file.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						IMU data type.
	 * @return 		True if the imu data is available, false otherwise.
	 */
	[[nodiscard]] inline bool isImuStreamAvailable(const std::string &streamName) const {
		static constexpr int32_t typeId = dv::types::IdentifierStringToId(dv::IMUPacket::TableType::identifier);
		return std::any_of(mStreamInfo.begin(), mStreamInfo.end(), [&streamName](const auto &m) {
			return m.second.mStream.mType.id == typeId && (streamName.empty() || streamName == m.first);
		});
	}

	/**
	 * Check whether trigger stream is available. Specifically checks whether a stream named "triggers" is available
	 * since it's the default stream name for trigger data.
	 * @return 		True if the trigger stream are available, false otherwise.
	 */
	[[nodiscard]] inline bool isTriggerStreamAvailable() const override {
		return isTriggerStreamAvailable("triggers");
	}

	/**
	 * Checks whether a trigger data stream is present in the file.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						trigger data type.
	 * @return 		True if the triggers are available, false otherwise.
	 */
	[[nodiscard]] inline bool isTriggerStreamAvailable(const std::string &streamName) const {
		static constexpr int32_t typeId = dv::types::IdentifierStringToId(dv::TriggerPacket::TableType::identifier);
		return std::any_of(mStreamInfo.begin(), mStreamInfo.end(), [&streamName](const auto &m) {
			return m.second.mStream.mType.id == typeId && (streamName.empty() || streamName == m.first);
		});
	}

	/**
	 * Return a pair containing start (first) and end (second) time of the recording file.
	 * @return 		A pair containing start and end timestamps for the recording.
	 */
	[[nodiscard]] std::pair<int64_t, int64_t> getTimeRange() const {
		return std::make_pair(mInfo.mTimeLowest, mInfo.mTimeHighest);
	}

	/**
	 * Return the duration of the recording.
	 * @return 		Duration value holding the total playback time of the recording.
	 */
	[[nodiscard]] dv::Duration getDuration() const {
		return dv::Duration(mInfo.mTimeHighest - mInfo.mTimeLowest);
	}

	/**
	 * Return the camera name that is detected in the recording.
	 * @return 		String containing camera name.
	 */
	[[nodiscard]] std::string getCameraName() const override {
		return mCameraName;
	}

	/**
	 * Read next packet in the recorded stream, the function returns a `std::variant` containing
	 * one of the following types:
	 * - dv::EventStore
	 * - dv::Frame
	 * - dv::cvector<dv::IMU>
	 * - dv::cvector<dv::Trigger>
	 * - dv::io::MonoCameraRecording::OutputFlag
	 * The `OutputFlag` is used to determine when the end of file is reached.
	 * If the reader encounters an unsupported type, the data will be skipped and will seek until a packet
	 * containing a supported type is reached.
	 *
	 * @return 		`std::variant` containing a packet with data of one of the supported types.
	 */
	[[nodiscard]] DataReadVariant readNext() {
		static constexpr int32_t eventTypeId = dv::types::IdentifierStringToId(dv::EventPacket::TableType::identifier);
		static constexpr int32_t frameTypeId = dv::types::IdentifierStringToId(dv::Frame::TableType::identifier);
		static constexpr int32_t imuTypeId   = dv::types::IdentifierStringToId(dv::IMUPacket::TableType::identifier);
		static constexpr int32_t triggerTypeId
			= dv::types::IdentifierStringToId(dv::TriggerPacket::TableType::identifier);

		if (mPacketIter == mInfo.mDataTable.Table.cend()) {
			return DataReadHandler::OutputFlag::EndOfFile;
		}
		else {
			auto [packet, _] = mReader->read(*mPacketIter);
			mPacketIter++;

			switch (packet->type.id) {
				case eventTypeId:
					return dv::EventStore(packet->moveToSharedPtr<const dv::EventPacket>());
				case frameTypeId:
					return std::move(*packet->moveToSharedPtr<dv::Frame>());
				case imuTypeId:
					return std::move(packet->moveToSharedPtr<dv::IMUPacket>()->elements);
				case triggerTypeId:
					return std::move(packet->moveToSharedPtr<dv::TriggerPacket>()->elements);
				default:
					return readNext();
			}
		}
	}

	/**
	 * Read next packet from the recording and use a handler object to handle all types of packets. The function returns
	 * a true if end-of-file was not reached, so this function call can be used in a while loop like so:
	 * ```
	 * while (recording.handleNext(handler)) {
	 * 		// While-loop executes after each packet
	 * }
	 * ```
	 * @param handler
	 * @return
	 */
	[[nodiscard]] bool handleNext(DataReadHandler &handler) {
		std::visit(handler, readNext());
		return !handler.eof;
	}

	/**
	 * Sequentially read all packets from the recording and apply handler to each packet. This is a blocking
	 * call.
	 * @param handler 		Handler class containing lambda functions for each supported packet type.
	 */
	void run(DataReadHandler &handler) {
		while (handleNext(handler)) {
			// no-op
		}
	}

	/**
	 * Get event stream resolution for the "events" stream.
	 * @return 		Resolution of the "events" stream.
	 */
	[[nodiscard]] std::optional<cv::Size> getEventResolution() const override {
		return getEventResolution("events");
	}

	/**
	 * Get the resolution of the event data stream if it is available.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						event data type.
	 * @return 		Returns the resolution of the event data if available, `std::nullopt` otherwise.
	 */
	[[nodiscard]] std::optional<cv::Size> getEventResolution(const std::string &streamName) const {
		try {
			return getStreamInfo<dv::EventPacket>(streamName)->second.mStream.getResolution();
		}
		catch (const dv::exceptions::Exception &e) {
			return std::nullopt;
		}
	}

	/**
	 * Get frame stream resolution for the "frames" stream.
	 * @return 		Resolution of the "frames" stream.
	 */
	[[nodiscard]] std::optional<cv::Size> getFrameResolution() const override {
		return getFrameResolution("frames");
	}

	/**
	 * Get the resolution of the frame data stream if it is available.
	 * @param streamName 	Name of the stream, if an empty name is passed, it will select any one stream with
	 * 						frame data type.
	 * @return 		Returns the resolution of the frames if available, `std::nullopt` otherwise.
	 */
	[[nodiscard]] std::optional<cv::Size> getFrameResolution(const std::string &streamName) const {
		try {
			return getStreamInfo<dv::Frame>(streamName)->second.mStream.getResolution();
		}
		catch (const dv::exceptions::Exception &e) {
			return std::nullopt;
		}
	}

	/**
	 * Get all metadata of a stream.
	 * @param streamName 	Name of the stream.
	 * @return 				A map containing key-value strings of each available metadata of a requested stream.
	 * @throws out_of_range	Out of range exception is thrown if a stream with given name is not available.
	 */
	[[nodiscard]] const std::map<std::string, std::string> &getStreamMetadata(const std::string &streamName) {
		// Use .at method to "safely" cause exception on wrong streamName
		return mStreamInfo.at(streamName).mMetadata;
	}

	/**
	 * Get a value of a given metadata key. Throws an exception if given stream doesn't exist and returns std::nullopt
	 * if a metadata entry with given key is not found for the stream.
	 * @param streamName 	Name of the stream.
	 * @param key 			Key string of the metadata.
	 * @return 				Metadata entry with given key is found for the stream, `std::nullopt` otherwise.
	 * @throws out_of_range	Out of range exception is thrown if a stream with given name is not available.
	 */
	[[nodiscard]] std::optional<std::string> getStreamMetadataValue(
		const std::string &streamName, const std::string &key) {
		const auto &stream = mStreamInfo.at(streamName);
		auto iter          = stream.mMetadata.find(key);
		if (iter == stream.mMetadata.end()) {
			return std::nullopt;
		}
		else {
			return iter->second;
		}
	}

	/**
	 * Check whether a stream is of a given data type.
	 * @tparam DataType 	Data type to be checked.
	 * @param streamName 	Name of the stream.
	 * @return 				True if the given stream contains DataType data.
	 * @throws out_of_bounds	Out of bounds exception is thrown if stream of a given name is not found.
	 */
	template<class DataType>
	[[nodiscard]] bool isStreamOfDataType(const std::string &streamName) const {
		if constexpr (std::is_same<DataType, dv::EventStore>()) {
			// Specialization for event store.
			return mStreamInfo.at(streamName).mStream.mTypeIdentifier == dv::EventPacket::TableType::identifier;
		}
		else {
			return mStreamInfo.at(streamName).mStream.mTypeIdentifier == DataType::TableType::identifier;
		}
	}
};

} // namespace dv::io
