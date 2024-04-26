#pragma once

#include "camera_output_base.hpp"
#include "network/socket_base.hpp"
#include "network/tcp_tls_socket.hpp"
#include "network/unix_socket.hpp"
#include "network/write_ordered_socket.hpp"
#include "stream.hpp"
#include "support/utils.hpp"
#include "writer.hpp"

#include <boost/lockfree/spsc_queue.hpp>

#include <utility>

namespace dv::io {

using namespace dv::io::network;

/**
 * Network server class for streaming AEDAT4 serialized data types.
 */
class NetworkWriter : public CameraOutputBase {
public:
	using ErrorMessageCallback = std::function<void(const boost::system::error_code &, const std::string_view)>;

	/**
	 * Create a non-encrypted server that listens for connections on a given IP address. Supports multiple clients.
	 * @param ipAddress IP address to bind the server.
	 * @param port Port number.
	 * @param stream AEDAT4 stream definition.
	 * @param maxClientConnections Maximum number of client connections supported by this instance.
	 * @param messageCallback Callback to handle any error messages received by the client connections.
	 */
	NetworkWriter(
		const std::string_view ipAddress, const uint16_t port, const dv::io::Stream &stream,
		const size_t maxClientConnections = 10,
		ErrorMessageCallback messageCallback =
			[](const boost::system::error_code &, const std::string_view) {
			}) :
		mCameraName(stream.getSource().value()),
		mMaxConnections(maxClientConnections),
		mAcceptorTcp(std::make_unique<asioTCP::acceptor>(mIoService)),
		mAcceptorTcpSocket(std::make_unique<asioTCP::socket>(mIoService)),
		mTLSEnabled(false),
		mAedat4Writer(stream.getCompression().value()),
		mStreamId(stream.mId),
		mErrorMessageHandler(std::move(messageCallback)) {
		connectTCP(ipAddress, port);

		generateHeaderContent(stream);

		mIOThread = std::thread([this] {
			acceptStart<dv::io::network::TCPTLSSocket>();
			ioThread();
		});
	}

	/**
	 * Create an encrypted server that listens for connections on a given IP address. Supports multiple clients.
	 * @param ipAddress IP address to bind the server.
	 * @param port Port number.
	 * @param stream AEDAT4 stream definition.
	 * @param encryptionContext Preconfigured encryption context, use either
	 * `dv::io::encrypt::defaultEncryptionServer()` to create the context or configure custom encryption context.
	 * When a client connects to the server, it will run handshake, during which client certificates will be
	 * validated, if the handshake fails, connection is terminated.
	 * @param maxClientConnections Maximum number of client connections supported by this instance.
	 * @param messageCallback Callback to handle any error messages received by the client connections.
	 */
	NetworkWriter(
		const std::string_view ipAddress, const uint16_t port, const dv::io::Stream &stream,
		boost::asio::ssl::context &&encryptionContext, const size_t maxClientConnections = 10,
		ErrorMessageCallback messageCallback =
			[](const boost::system::error_code &, const std::string_view) {
			}) :
		mCameraName(stream.getSource().value()),
		mMaxConnections(maxClientConnections),
		mAcceptorTcp(std::make_unique<asioTCP::acceptor>(mIoService)),
		mAcceptorTcpSocket(std::make_unique<asioTCP::socket>(mIoService)),
		mTLSContext(std::move(encryptionContext)),
		mTLSEnabled(true),
		mAedat4Writer(stream.getCompression().value()),
		mStreamId(stream.mId),
		mErrorMessageHandler(std::move(messageCallback)) {
		connectTCP(ipAddress, port);

		generateHeaderContent(stream);

		mIOThread = std::thread([this] {
			acceptStart<dv::io::network::TCPTLSSocket>();
			ioThread();
		});
	}

	/**
	 * Create a local socket server. Provide a path to the socket, if a file already exists on a given path, the
	 * connection will fail by throwing an exception. It is required that the given socket path does not point to
	 * an existing socket file. If the file *can* exist, it is up to the user of this class to decide whether it is
	 * safe to remove any existing socket files or the class should not bind to the path.
	 * @param socketPath Path to a socket file, must be a non-existent path.
	 * @param stream AEDAT4 stream definition.
	 * @param maxClientConnections Maximum number of client connections supported by this instance.
	 * @param messageCallback Callback to handle any error messages received by the client connections.
	 */
	NetworkWriter(
		const std::filesystem::path &socketPath, const dv::io::Stream &stream, const size_t maxClientConnections = 10,
		ErrorMessageCallback messageCallback =
			[](const boost::system::error_code &, const std::string_view) {
			}) :
		mCameraName(stream.getSource().value()),
		mMaxConnections(maxClientConnections),
		mAcceptorUnix(std::make_unique<asioUNIX::acceptor>(mIoService)),
		mAcceptorUnixSocket(std::make_unique<asioUNIX::socket>(mIoService)),
		mTLSEnabled(false),
		mAedat4Writer(stream.getCompression().value()),
		mStreamId(stream.mId),
		mSocketPath(socketPath),
		mErrorMessageHandler(messageCallback) {
		connectUNIX(socketPath);

		generateHeaderContent(stream);

		mIOThread = std::thread([this] {
			acceptStart<dv::io::network::UNIXSocket>();
			ioThread();
		});
	}

	/**
	 * Closes the socket, frees allocated memory, and removes any queued packets from write queue.
	 */
	virtual ~NetworkWriter() {
		try {
			// Post to IO thread to avoid any possible race-condition.
			mIoService.post([this] {
				if (mAcceptorTcp) {
					mAcceptorTcp->close();
				}
				if (mAcceptorUnix) {
					mAcceptorUnix->close();
				}
			});
		}
		catch (const std::exception &exc) {
			// NOOP, can be ignored
		}

		try {
			// Post 'close all connections' to end of async queue,
			// so that any other callbacks, such as pending accepts,
			// are executed first, and we really close all sockets.
			mIoService.post([this] {
				const std::scoped_lock lock(mClientsMutex);

				// Close all open connections, hard.
				for (auto &client : mClients) {
					client->close();
				}

				mClients.clear();
			});
		}
		catch (const std::exception &exc) {
			// NOOP, can be ignored
		}

		// Wait for all clients to go away.
		{
			std::unique_lock lock(mClientsMutex);

			while (!mClients.empty()) {
				lock.unlock();
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				lock.lock();
			}
		}

		mShutdownRequested = true;

		mIOThread.join();

		if (std::filesystem::exists(mSocketPath)) {
			std::filesystem::remove(mSocketPath);
		}
	}

	/**
	 * Write an event store to the network stream.
	 * @param events Data to be sent out.
	 */
	void writeEvents(const EventStore &events) override {
		for (const auto &partial : events.dataPartials_) {
			// We can use full partial for writing
			if (partial.getLength() == partial.data_->elements.size()) {
				writePacket(*partial.data_);
			}
			else {
				dv::EventPacket packet;
				packet.elements = dv::cvector<dv::Event>(partial.begin(), partial.end());
				writePacket(std::move(packet));
			}
		}
	}

	/**
	 * Write a frame image to the network stream.
	 * @param frame Data to be sent out.
	 */
	void writeFrame(const dv::Frame &frame) override {
		writePacket(frame);
	}

	/**
	 * Write IMU data to the socket.
	 * @param imu Data to be sent out.
	 */
	void writeIMU(const cvector<dv::IMU> &imu) override {
		writePacket(dv::IMUPacket(imu));
	}

	/**
	 * Write trigger data to the network stream.
	 * @param triggers Data to be sent out.
	 */
	void writeTriggers(const cvector<dv::Trigger> &triggers) override {
		writePacket(dv::TriggerPacket(triggers));
	}

	/**
	 * Write a flatbuffer packet to the network stream.
	 * @tparam PacketType Type of the packet, must satisfy the `dv::concepts::FlatbufferPacket` concept.
	 * @param packet Data to write.
	 */
	template<class PacketType>
	requires dv::concepts::FlatbufferPacket<PacketType>
	void writePacket(PacketType &&packet) {
		mWriteQueue.push(dv::io::support::packetToObject(std::forward<PacketType>(packet)));
		mQueuedPackets++;
	}

	/**
	 * Get camera name. It is looked up from the stream definition during construction.
	 * @return
	 */
	[[nodiscard]] std::string getCameraName() const override {
		return mCameraName;
	}

	/**
	 * Get number of packets in the write queue.
	 * @return Number of packets in the write queue.
	 */
	[[nodiscard]] size_t getQueuedPacketCount() const {
		return mQueuedPackets;
	}

	/**
	 * Get number of active connected clients.
	 * @return Number of active connected clients.
	 */
	[[nodiscard]] size_t getClientCount() {
		const std::scoped_lock<std::mutex> lock(mClientsMutex);
		return mClients.size();
	}

private:
	class Connection;
	using WriteQueue = boost::lockfree::spsc_queue<std::shared_ptr<dv::types::TypedObject>>;

	template<class SocketType>
	requires dv::concepts::is_type_one_of<SocketType, dv::io::network::TCPTLSSocket, dv::io::network::UNIXSocket>
	void acceptStart() {
		const auto acceptCallback = [this](const boost::system::error_code &error) {
			// Ignore cancel error, normal on shutdown.
			if (error && (error != asio::error::operation_aborted)) {
				return;
			}

			const std::scoped_lock lock(mClientsMutex);

			if (mClients.size() >= mMaxConnections) {
				if constexpr (std::same_as<SocketType, dv::io::network::TCPTLSSocket>) {
					mAcceptorTcpSocket->close();
				}
				else {
					mAcceptorUnixSocket->close();
				}
			}
			else {
				try {
					std::shared_ptr<Connection> client;

					if constexpr (std::same_as<SocketType, dv::io::network::TCPTLSSocket>) {
						client = std::make_shared<Connection>(
							WriteOrderedSocket(
								std::make_unique<dv::io::network::TCPTLSSocket>(std::move(*mAcceptorTcpSocket),
									mTLSEnabled, asioSSL::stream_base::server, mTLSContext)),
							this);
					}
					else {
						client = std::make_shared<Connection>(
							WriteOrderedSocket(
								std::make_unique<dv::io::network::UNIXSocket>(std::move(*mAcceptorUnixSocket))),
							this);
					}

					mClients.push_back(client.get());

					client->start();
				}
				catch (std::exception &exc) {
					// Failed to initialize a client, close the connection
					if constexpr (std::same_as<SocketType, dv::io::network::TCPTLSSocket>) {
						mAcceptorTcpSocket->close();
					}
					else {
						mAcceptorUnixSocket->close();
					}
				}
			}

			acceptStart<SocketType>();
		};

		if constexpr (std::same_as<SocketType, dv::io::network::TCPTLSSocket>) {
			mAcceptorTcp->async_accept(*mAcceptorTcpSocket, acceptCallback);
		}
		else {
			mAcceptorUnix->async_accept(*mAcceptorUnixSocket, acceptCallback);
		}
	}

	void writePacketToClients(const std::shared_ptr<dv::types::TypedObject> &packet) {
		mIoService.post([this, packet] {
			mAedat4Writer.writePacket(
				packet.get(), mStreamId, [this](const std::shared_ptr<const dv::io::support::IODataBuffer> packet) {
					const std::scoped_lock lock(mClientsMutex);
					for (auto client : mClients) {
						if (client->isOpen()) {
							client->writePacket(packet);
						}
					}
				});
		});
	}

	void ioThread() {
		while (!mShutdownRequested.load()) {
			mWriteQueue.consume_all([this](const auto &packet) {
				writePacketToClients(packet);
				mQueuedPackets--;
			});

			mIoService.poll();
			mIoService.restart();
		}
	}

	void connectTCP(const std::string_view ipAddress, const uint16_t port) {
		const auto endpoint = asioTCP::endpoint(asioIP::address::from_string(ipAddress.data()), port);

		try {
			mAcceptorTcp->open(endpoint.protocol());
			mAcceptorTcp->set_option(asioTCP::socket::reuse_address(true));
			mAcceptorTcp->bind(endpoint);
			mAcceptorTcp->listen(static_cast<int32_t>(mMaxConnections));
		}
		catch (const std::exception &e) {
			mAcceptorTcp->close();
			throw e;
		}
	}

	void connectUNIX(const std::filesystem::path &socketPath) {
		const asioUNIX::endpoint endpoint(socketPath.string());

		try {
			mAcceptorUnix->open(endpoint.protocol());
			mAcceptorUnix->bind(endpoint);
			mAcceptorUnix->listen(static_cast<int32_t>(mMaxConnections));
		}
		catch (const std::exception &e) {
			mAcceptorUnix->close();
			throw e;
		}
	}

	void generateHeaderContent(const dv::io::Stream &stream) {
		dv::io::support::XMLTreeNode root("outInfo");

		root.mChildren.emplace_back(stream.mXMLNode);

		dv::io::support::XMLConfigWriter xml(root);
		mInfoNode = xml.getXMLContent();
	}

	void removeClient(const Connection *const client) {
		std::scoped_lock<std::mutex> lock(mClientsMutex);
		std::erase(mClients, client);
	}

	std::string mCameraName;
	size_t mMaxConnections;

	asio::io_service mIoService;

	std::unique_ptr<asioTCP::acceptor> mAcceptorTcp     = nullptr;
	std::unique_ptr<asioTCP::socket> mAcceptorTcpSocket = nullptr;

	std::unique_ptr<asioUNIX::acceptor> mAcceptorUnix     = nullptr;
	std::unique_ptr<asioUNIX::socket> mAcceptorUnixSocket = nullptr;

	asioSSL::context mTLSContext = asioSSL::context(asioSSL::context::method::tlsv12_server);
	bool mTLSEnabled;

	std::mutex mClientsMutex;

	/// The client list is raw point, that is self-owned, read Connection class documentation for more details.
	std::vector<Connection *> mClients;
	std::atomic<size_t> mQueuedPackets = 0;

	dv::io::Writer mAedat4Writer;
	dv::cstring mInfoNode;

	std::atomic<bool> mShutdownRequested = false;
	std::thread mIOThread;
	int32_t mStreamId = 0;

	std::filesystem::path mSocketPath;

	WriteQueue mWriteQueue = WriteQueue(1024);

	/// Error message handler, by default: NOOP
	ErrorMessageCallback mErrorMessageHandler;

	/**
	 * Connection helper class that maintains shared pointer to itself when called on the public API methods.
	 *
	 * This class should be wrapped in a shared pointer and start method should be called. This will intrinsically
	 * increment the reference count to maintain the pointer to itself even if the wrapper shared_ptr goes out-of-scope
	 * until the instance gets API calls to write data into the buffer. During destruction, the instance will remove
	 * it's own pointer from a connection list in the top-level class.
	 *
	 * (Personal comment by Rokas): this seems over-engineered and unnecessary, but it's the way ASIO works and,
	 * although there are other ways to implement it, it just doesn't work with other approaches leading to undefined
	 * behaviors.
	 */
	class Connection : public std::enable_shared_from_this<Connection> {
	public:
		Connection(WriteOrderedSocket &&socket, NetworkWriter *const server) :
			mParent(server),
			mSocket(std::move(socket)) {
			if (server == nullptr) {
				throw dv::exceptions::RuntimeError("Invalid server instance while creating a network connection.");
			}
		}

		~Connection() {
			mParent->removeClient(this);
		}

		void start() {
			mParent->mAedat4Writer.writeHeader(
				-1, mParent->mInfoNode, [this](const std::shared_ptr<const dv::io::support::IODataBuffer> header) {
					writeIOHeader(header);
				});

			keepAliveByReading();
		}

		void close() {
			mSocket.close();
		}

		void writePacket(const std::shared_ptr<const dv::io::support::IODataBuffer> &packet) {
			const auto self(shared_from_this());

			// Write packet header first.
			mSocket.write(
				asio::buffer(reinterpret_cast<const std::byte *>(packet->getHeader()), sizeof(dv::PacketHeader)),
				[this, self, packet = packet](const boost::system::error_code &error, const size_t /*length*/) {
					if (error) {
						handleError(error, "Failed to write message header");
					}
				});

			// Then write packet content.
			mSocket.write(asio::buffer(packet->getData(), packet->getDataSize()),
				[this, self, packet](const boost::system::error_code &error, const size_t /*length*/) {
					if (error) {
						handleError(error, "Failed to write message data");
					}
				});
		}

		[[nodiscard]] bool isOpen() const {
			return mSocket.isOpen();
		}

	private:
		void writeIOHeader(const std::shared_ptr<const dv::io::support::IODataBuffer> &ioHeader) {
			const auto self(shared_from_this());

			mSocket.write(asio::buffer(ioHeader->getData(), ioHeader->getDataSize()),
				[this, self, ioHeader = ioHeader](const boost::system::error_code &error, const size_t /*length*/) {
					if (error) {
						handleError(error, "Failed to write IOHeader data");
					}
				});
		}

		void keepAliveByReading() {
			const auto self(shared_from_this());

			mSocket.read(asio::buffer(&mKeepAliveReadSpace, sizeof(mKeepAliveReadSpace)),
				[this, self](const boost::system::error_code &error, const size_t /*length*/) {
					if (error) {
						handleError(error, "Read keep-alive failure");
					}
					else {
						handleError(error, "Detected illegal incoming data");
					}
				});
		}

		void handleError(const boost::system::error_code &error, const std::string_view message) {
			// Let's fail fast by closing the socket.
			close();

			if (mParent->mErrorMessageHandler) {
				mParent->mErrorMessageHandler(error, message);
			}
		}

		NetworkWriter *mParent;
		WriteOrderedSocket mSocket;
		uint8_t mKeepAliveReadSpace{0};
	};
};

} // namespace dv::io
