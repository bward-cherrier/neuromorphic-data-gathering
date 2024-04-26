#pragma once

#include <boost/asio.hpp>

namespace dv::io::network {

namespace asio      = boost::asio;
namespace asioLocal = boost::asio::local;
using asioUNIX      = asioLocal::stream_protocol;

namespace asioIP = asio::ip;
using asioTCP    = asioIP::tcp;

/**
 * Interface class to define a socket API.
 */
class SocketBase {
public:
	virtual ~SocketBase() = default;

	/// Callback alias that is used to handle a completed IO operation.
	using CompletionHandler = std::function<void(const boost::system::error_code &, const size_t)>;

	/**
	 * Check whether a socket is open and active.
	 * @return True if socket is open, false otherwise.
	 */
	[[nodiscard]] virtual bool isOpen() const = 0;

	/**
	 * Close the underlying socket communication. Async reads/writes can be aborted during this function call.
	 */
	virtual void close() = 0;

	/**
	 * Write a data buffer to the socket asynchronously. Completion handler is called when write to the socket is
	 * complete.
	 * @param buffer Data buffer to written to the socket.
	 * @param handler Completion handler, that is called when write is complete.
	 */
	virtual void write(const asio::const_buffer &buffer, CompletionHandler &&handler) = 0;

	/**
	 * Read a data buffer from the socket asynchronously. Completion handler is called when read from the socket is
	 * complete.
	 * @param buffer Output buffer to place data from the socket.
	 * @param wrHandler Completion handler, that is called when write is complete.
	 */
	virtual void read(const asio::mutable_buffer &buffer, CompletionHandler &&handler) = 0;

	/**
	 * Write data into the socket synchronously, this method is a blocking call which returns when writing data is
	 * complete.
	 * @param buffer Data to be written into the socket.
	 */
	virtual void syncWrite(const asio::const_buffer &buffer) = 0;

	/**
	 * Read data from the socket synchronously, this method is a blocking call which returns when reading data is
	 * complete.
	 * @param buffer Output buffer to place data from the socket.
	 */
	virtual void syncRead(const asio::mutable_buffer &buffer) = 0;
};

} // namespace dv::io::network
