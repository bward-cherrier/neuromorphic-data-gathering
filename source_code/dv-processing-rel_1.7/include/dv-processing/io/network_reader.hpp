#pragma once

#include "camera_input_base.hpp"
#include "network/encrypt.hpp"
#include "network/tcp_tls_socket.hpp"
#include "network/unix_socket.hpp"
#include "reader.hpp"

#include <boost/lockfree/spsc_queue.hpp>

namespace dv::io {

using namespace dv::io::network;

/**
 * Network capture class. Connect to a TCP or a local socket server providing a data stream. The class
 * provides a single data stream per network capture.
 */
class NetworkReader : public CameraInputBase {
public:
	/**
	 * Initialize a network capture object, it will connect to a given TCP port with given IP address.
	 * @param ipAddress IP address of the target TCP server.
	 * @param port TCP port number.
	 */
	NetworkReader(const std::string_view ipAddress, const uint16_t port) : mTLSEnabled(false) {
		connectTCP(ipAddress, port, mTLSEnabled);
		initializeReader();
	}

	/**
	 * Initialize an encrypted network capture object, it will connect to a given TCP port with given IP address.
	 * Provide an encryption context that is preconfigured, prefer using existing
	 * `dv::io::encrypt::defaultEncryptionClient()` method for configuring the encryption context.
	 * @param ipAddress IP address of the target TCP server.
	 * @param port TCP port number.
	 * @param encryptionContext Preconfigured encryption context.
	 */
	NetworkReader(
		const std::string_view ipAddress, const uint16_t port, boost::asio::ssl::context &&encryptionContext) :
		mTLSContext(std::move(encryptionContext)),
		mTLSEnabled(true) {
		connectTCP(ipAddress, port, mTLSEnabled);
		initializeReader();
	}

	/**
	 * Initialize a network capture object, it will connect to a given UNIX socket with a given file system path.
	 * @param socketPath Path to the UNIX socket.
	 */
	explicit NetworkReader(const std::filesystem::path &socketPath) : mTLSEnabled(false) {
		connectUNIX(socketPath);
		initializeReader();
	}

	/**
	 * Destructor - disconnects from network resource, stops threads and frees any buffered data.
	 */
	virtual ~NetworkReader() {
		mKeepReading = false;

		close();

		mReadingThread.join();

		// Release anything that is in the queue
		mPacketQueue.consume_all([](dv::types::TypedObject *objPtr) {
			delete objPtr;
		});
	}

	/**
	 * Read next event batch. This is a non-blocking method, if there is no data to read, it will return a
	 * `std::nullopt`.
	 *
	 * @return Next batch of events, `std::nullopt` if no data received from last read or the event stream is not
	 * available.
	 */
	[[nodiscard]] std::optional<dv::EventStore> getNextEventBatch() override {
		auto packetPtr = getNextPacket<const dv::EventPacket>();

		if (!packetPtr) {
			return std::nullopt;
		}

		return dv::EventStore(packetPtr);
	}

	/**
	 * Read next frame. This is a non-blocking method, if there is no data to read, it will return a `std::nullopt`.
	 *
	 * @return Next frame, `std::nullopt` if no data received from last read or the event stream is not available.
	 */
	[[nodiscard]] std::optional<dv::Frame> getNextFrame() override {
		auto packetPtr = getNextPacket<dv::Frame>();

		if (!packetPtr) {
			return std::nullopt;
		}

		return std::move(*packetPtr);
	}

	/**
	 * Read next imu measurement batch. This is a non-blocking method, if there is no data to read, it will return a
	 * `std::nullopt`.
	 *
	 * @return Next batch of imu measurements, `std::nullopt` if no data received from last read or the event stream is
	 * not available.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::IMU>> getNextImuBatch() override {
		auto packetPtr = getNextPacket<dv::IMUPacket>();

		if (!packetPtr) {
			return std::nullopt;
		}

		return std::move(packetPtr->elements);
	}

	/**
	 * Read next trigger batch. This is a non-blocking method, if there is no data to read, it will return a
	 * `std::nullopt`.
	 *
	 * @return Next batch of triggers, `std::nullopt` if no data received from last read or the event stream is not
	 * available.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::Trigger>> getNextTriggerBatch() override {
		auto packetPtr = getNextPacket<dv::TriggerPacket>();

		if (!packetPtr) {
			return std::nullopt;
		}

		return std::move(packetPtr->elements);
	}

	/**
	 * Retrieve the event sensor resolution. The method returns `std::nullopt` if event stream is not available
	 * or the metadata does not contain resolution.
	 * @return Event sensor resolution or `std::nullopt` if not available.
	 */
	[[nodiscard]] std::optional<cv::Size> getEventResolution() const override {
		if (!isEventStreamAvailable()) {
			return std::nullopt;
		}

		return mStream.getResolution();
	}

	/**
	 * Retrieve the frame sensor resolution. The method returns `std::nullopt` if frame stream is not available
	 * or the metadata does not contain resolution.
	 * @return Frame sensor resolution or `std::nullopt` if not available.
	 */
	[[nodiscard]] std::optional<cv::Size> getFrameResolution() const override {
		if (!isFrameStreamAvailable()) {
			return std::nullopt;
		}

		return mStream.getResolution();
	}

	/**
	 * Read next packet, given it's type.
	 *
	 * The given type must match the stream type exactly (it must be a flatbuffer generated type). Returns `nullptr`
	 * if no data is available for reading or stream of such type is not available.
	 * @tparam PacketType Stream packet type, must be a flatbuffer type and must match stream type exactly.
	 * @return Shared pointer to a packet of data, or nullptr if unavailable.
	 */
	template<class PacketType>
	requires dv::concepts::FlatbufferPacket<PacketType>
	[[nodiscard]] std::shared_ptr<PacketType> getNextPacket() {
		if (mExceptionThrown.load(std::memory_order_relaxed)) {
			std::rethrow_exception(mException);
		}

		if (!isRunning()) {
			return nullptr;
		}

		if (!isStreamAvailable<PacketType>()) {
			return nullptr;
		}

		dv::types::TypedObject *object;
		if (!mPacketQueue.pop(object)) {
			return nullptr;
		}

		auto returnPtr = object->template moveToSharedPtr<PacketType>();

		// Manually destroy the object, since it was released from unique_ptr before putting it into a queue.
		delete object;

		return returnPtr;
	}

	/**
	 * Check whether an event stream is available in this capture class.
	 * @return True if an event stream is available; false otherwise.
	 */
	[[nodiscard]] bool isEventStreamAvailable() const override {
		return isStreamAvailable<dv::EventPacket>();
	}

	/**
	 * Check whether a frame stream is available in this capture class.
	 * @return True if a frame stream is available; false otherwise.
	 */
	[[nodiscard]] bool isFrameStreamAvailable() const override {
		return isStreamAvailable<dv::Frame>();
	}

	/**
	 * Check whether an IMU data stream is available in this capture class.
	 * @return True if an IMU data stream is available; false otherwise.
	 */
	[[nodiscard]] bool isImuStreamAvailable() const override {
		return isStreamAvailable<dv::IMUPacket>();
	}

	/**
	 * Check whether a trigger stream is available in this capture class.
	 * @return True if a trigger stream is available; false otherwise.
	 */
	[[nodiscard]] bool isTriggerStreamAvailable() const override {
		return isStreamAvailable<dv::TriggerPacket>();
	}

	/**
	 * Get camera name, which is a combination of the camera model and the serial number.
	 * @return 		String containing the camera model and serial number separated by an underscore character.
	 */
	[[nodiscard]] std::string getCameraName() const override {
		return mCameraName;
	}

	/**
	 * Check whether the network stream is still connected.
	 * @return True if network stream is running and available.
	 */
	[[nodiscard]] bool isRunning() const override {
		return (mSocket && mSocket->isOpen());
	}

	/**
	 * Check whether a stream of given type is available.
	 *
	 * The given type must match the stream type exactly (it must be a flatbuffer generated type). Returns `nullptr`
	 * if no data is available for reading or stream of such type is not available.
	 * @tparam PacketType Stream packet type, must be a flatbuffer type and must match stream type exactly.
	 * @return True if stream of a given type is available, false otherwise.
	 */
	template<class PacketType>
	requires dv::concepts::FlatbufferPacket<PacketType>
	[[nodiscard]] bool isStreamAvailable() const {
		static constexpr int32_t typeId = dv::types::IdentifierStringToId(PacketType::TableType::identifier);
		return mStream.mType.id == typeId;
	}

	/**
	 * Explicitly close the communication socket, receiving data is not going to possible after this method call.
	 */
	void close() {
		if (isRunning()) {
			mSocket->close();
		}
	}

	/**
	 * Get the stream definition object, which describe the available data stream by this reader.
	 * @return Data stream definition object.
	 */
	[[nodiscard]] const dv::io::Stream &getStreamDefinition() const {
		return mStream;
	}

private:
	// Usage of better types like unique_ptr was considered, but it is currently impossible due to spsc_queue not
	// supporting move semantics, more info here:
	// https://github.com/boostorg/lockfree/pull/31
	using PacketQueue = boost::lockfree::spsc_queue<dv::types::TypedObject *>;

	/// Callback method that calls read method of the socket.
	std::function<void(std::vector<std::byte> &, const int64_t)> mReadHandler
		= std::bind_front(&NetworkReader::readClbk, this);

	/**
	 * Read block of data from the network socket.
	 * @param data Container for data that is going to be read.
	 */
	void readClbk(std::vector<std::byte> &data, const int64_t) {
		// The pointer to socket should always exist throughout the instance lifetime
		dv::runtime_assert(static_cast<bool>(mSocket), "Socket has gone away.");

		mSocket->syncRead(boost::asio::buffer(data));
	}

	/**
	 * Initiate connection to the given IP address and port.
	 * @param ipAddress	Ip address, dot separated (in format "0.0.0.0")
	 * @param port TCP port number
	 * @param tlsEnabled Enable TLS encryption
	 */
	void connectTCP(const std::string_view ipAddress, const uint16_t port, const bool tlsEnabled = false) {
		const auto endpoint = asioTCP::endpoint(asioIP::address::from_string(ipAddress.data()), port);
		auto socket         = asioTCP::socket(mIOService);

		socket.connect(endpoint);

		mSocket
			= std::make_unique<TCPTLSSocket>(std::move(socket), tlsEnabled, asioSSL::stream_base::client, mTLSContext);
	}

	/**
	 * Initiate a connection to UNIX socket under given filesystem path.
	 * @param socketPath Path to a socket.
	 */
	void connectUNIX(const std::filesystem::path &socketPath) {
		const auto endpoint = asioUNIX::endpoint(socketPath.string());
		auto socket         = asioUNIX::socket(mIOService);

		socket.connect(endpoint);

		mSocket = std::make_unique<UNIXSocket>(std::move(socket));
	}

	void readThread() {
		while (mKeepReading) {
			try {
				auto [packetHeader, packet, sizes] = mAedat4Reader.readPacket(mReadHandler);
				mPacketQueue.push(packet.release());
			}
			catch (const boost::system::system_error &exc) {
				mKeepReading = false;

				// Ensure socket is closed, so isRunning() will return false.
				close();

				// Handle EOF cleanly, it can happen when closing the socket
				if (exc.code().value() == boost::asio::error::eof) {
					break;
				}

				mExceptionThrown = true;
				mException       = std::current_exception();
				break;
			}
		}
	}

	void initializeReader() {
		// Parse streams from header
		const auto header = mAedat4Reader.readHeader(mReadHandler);

		const auto streams = mAedat4Reader.getStreams();

		if (streams.empty()) {
			throw dv::exceptions::RuntimeError("No streams available in the network source!");
		}

		mStream = streams.front();

		if (const auto name = mStream.getSource(); name.has_value()) {
			mCameraName = *name;
		}

		if (mCameraName.empty()) {
			throw dv::exceptions::RuntimeError("Invalid network stream metadata: camera name can't be empty!");
		}

		mReadingThread = std::thread([this] {
			readThread();
		});
	}

	/// IO service context.
	boost::asio::io_service mIOService;

	/// Socket to contain the connection instance.
	std::unique_ptr<network::SocketBase> mSocket = nullptr;

	/// Decryption context.
	asioSSL::context mTLSContext = asioSSL::context(asioSSL::context::method::tlsv12_client);

	/// Whether TLS encryption is enabled
	bool mTLSEnabled;

	/// AEDAT4 reader.
	dv::io::Reader mAedat4Reader;

	/// Data stream container - one per capture.
	dv::io::Stream mStream;

	/// Name of the camera producing the stream.
	std::string mCameraName;

	/// Incoming packet queue.
	PacketQueue mPacketQueue = PacketQueue(1000);

	/// Reading thread.
	std::thread mReadingThread;

	/// Atomic bool used to stop the reading thread.
	std::atomic<bool> mKeepReading = true;

	/// Boolean value that indicated whether an exception was thrown on reading thread
	std::atomic<bool> mExceptionThrown = false;

	/// Pointer that holds thrown exception, mExceptionThrown contains thread-safe flag indicating an exception was
	/// thrown
	std::exception_ptr mException = nullptr;
};

} // namespace dv::io
