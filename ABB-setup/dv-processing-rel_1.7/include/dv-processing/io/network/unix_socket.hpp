#pragma once

#include "socket_base.hpp"

#include <deque>
#include <mutex>
#include <utility>

namespace dv::io::network {

/**
 * Minimal wrapper of UNIX socket. It follows RAII principle, the socket will closed and released when this object
 * is released.
 */
class UNIXSocket : public SocketBase {
public:
	/**
	 * Initial a socket wrapper by taking ownership of a connected socket.
	 * @param s
	 */
	explicit UNIXSocket(asioUNIX::socket &&s) : socket(std::move(s)) {
	}

	~UNIXSocket() override {
		UNIXSocket::close();
	}

	/**
	 * Check whether socket is open and active.
	 * @return True if socket is open, false otherwise.
	 */
	[[nodiscard]] inline bool isOpen() const override {
		return !socketClosed;
	}

	/**
	 * Close underlying UNIX socket cleanly.
	 */
	void close() override {
		if (!socketClosed) {
			boost::system::error_code ec;
			socket.shutdown(asioUNIX::socket::shutdown_both, ec);
			socket.close(ec);

			socketClosed = true;
		}
	}

	/**
	 * Write handler needs following signature:
	 * void (const boost::system::error_code &, size_t)
	 */
	void write(const asio::const_buffer &buf, CompletionHandler &&wrHandler) override {
		const asio::const_buffers_1 buf2(buf);

		asio::async_write(socket, buf2, wrHandler);
	}

	/**
	 * Read handler needs following signature:
	 * void (const boost::system::error_code &, size_t)
	 */
	void read(const asio::mutable_buffer &buf, CompletionHandler &&rdHandler) override {
		const asio::mutable_buffers_1 buf2(buf);

		asio::async_read(socket, buf2, rdHandler);
	}

	/**
	 * Blocking write data to the socket.
	 * @param buf Data to write.
	 */
	void syncWrite(const asio::const_buffer &buf) override {
		const asio::const_buffers_1 buf2(buf);

		asio::write(socket, buf2);
	}

	/**
	 * Blocking read from socket.
	 * @param buf Buffer for data to be read into.
	 */
	void syncRead(const asio::mutable_buffer &buf) override {
		const asio::mutable_buffers_1 buf2(buf);

		asio::read(socket, buf2);
	}

private:
	asioUNIX::socket socket;
	bool socketClosed = false;
};

} // namespace dv::io::network
