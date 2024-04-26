#pragma once

#include "encrypt.hpp"
#include "socket_base.hpp"

#include <deque>
#include <mutex>
#include <utility>

namespace dv::io::network {

/**
 * Minimal wrapper of TCP socket with optional TLS encryption.
 */
class TCPTLSSocket : public SocketBase {
public:
	/**
	 * Create a TCP socket with optional TLS encryption.
	 * @param socket A connected TCP socket instance.
	 * @param tlsEnabled Whether TLS encryption is enabled, if true, TLS handshake will be immediately performed during
	 * construction.
	 * @param tlsHandshake Type of TLS handshake, this is ignored if TLS is disabled.
	 * @param tlsContext Pre-configured TLS context for encryption.
	 */
	TCPTLSSocket(asioTCP::socket &&socket, const bool tlsEnabled,
		const asioSSL::stream_base::handshake_type tlsHandshake, asioSSL::context &tlsContext) :
		mLocalEndpoint(socket.local_endpoint()),
		mRemoteEndpoint(socket.remote_endpoint()),
		mSocket(std::move(socket), tlsContext),
		mSecureConnection(tlsEnabled) {
		if (mSecureConnection) {
			mSocket.handshake(tlsHandshake);
		}
	}

	~TCPTLSSocket() override {
		TCPTLSSocket::close();
	}

	/**
	 * Check whether socket is open and active.
	 * @return True if socket is open, false otherwise.
	 */
	[[nodiscard]] bool isOpen() const override {
		return !mSocketClosed;
	}

	/**
	 * Check whether socket has encryption enabled.
	 * @return True if socket has encryption enabled, false otherwise.
	 */
	[[nodiscard]] bool isSecured() const {
		return mSecureConnection;
	}

	/**
	 * Close underlying TCP socket cleanly.
	 */
	void close() override {
		// Close underlying TCP socket cleanly.
		// TCP shutdown() should be called for portability.
		// Note: no TLS shutdown, as the ASIO implementation does not
		// easily allow to just send a close_notify and close the socket.
		// It waits on reply from the other side, which we can't and don't
		// want to guarantee. There is a workaround, but it makes the
		// whole thing much more complex. Since shutdown only really
		// protects against a truncation attack, and it is not a problem
		// for our protocol, we can safely ignore it.
		if (!mSocketClosed) {
			boost::system::error_code ec;
			baseSocket().shutdown(asioTCP::socket::shutdown_both, ec);
			baseSocket().close(ec);

			mSocketClosed = true;
		}
	}

	/**
	 * Write handler needs following signature:
	 * void (const boost::system::error_code &, size_t)
	 */
	void write(const asio::const_buffer &buf, SocketBase::CompletionHandler &&wrHandler) override {
		const asio::const_buffers_1 buf2(buf);

		if (mSecureConnection) {
			asio::async_write(mSocket, buf2, wrHandler);
		}
		else {
			asio::async_write(baseSocket(), buf2, wrHandler);
		}
	}

	/**
	 * Read handler needs following signature:
	 * void (const boost::system::error_code &, size_t)
	 */
	void read(const asio::mutable_buffer &buf, SocketBase::CompletionHandler &&rdHandler) override {
		const asio::mutable_buffers_1 buf2(buf);

		if (mSecureConnection) {
			asio::async_read(mSocket, buf2, rdHandler);
		}
		else {
			asio::async_read(baseSocket(), buf2, rdHandler);
		}
	}

	/**
	 * Blocking write data to the socket.
	 * @param buf Data to write.
	 */
	void syncWrite(const asio::const_buffer &buf) override {
		const asio::const_buffers_1 buf2(buf);

		if (mSecureConnection) {
			asio::write(mSocket, buf2);
		}
		else {
			asio::write(baseSocket(), buf2);
		}
	}

	/**
	 * Blocking read from socket.
	 * @param buf Buffer for data to be read into.
	 */
	void syncRead(const asio::mutable_buffer &buf) override {
		const asio::mutable_buffers_1 buf2(buf);

		if (mSecureConnection) {
			asio::read(mSocket, buf2);
		}
		else {
			asio::read(baseSocket(), buf2);
		}
	}

	/**
	 * Retrieve local endpoint.
	 * @return Local endpoint.
	 */
	[[nodiscard]] asioTCP::endpoint local_endpoint() const {
		return mLocalEndpoint;
	}

	/**
	 * Get the local address of the current endpoint.
	 * @return IP address of the local connection.
	 */
	[[nodiscard]] asioIP::address local_address() const {
		return local_endpoint().address();
	}

	/**
	 * Get local port number.
	 * @return Local port number.
	 */
	[[nodiscard]] uint16_t local_port() const {
		return local_endpoint().port();
	}

	[[nodiscard]] asioTCP::endpoint remote_endpoint() const {
		return mRemoteEndpoint;
	}

	/**
	 * Remote endpoint IP address.
	 * @return Remote endpoint IP address.
	 */
	[[nodiscard]] asioIP::address remote_address() const {
		return remote_endpoint().address();
	}

	/**
	 * Get remote endpoint port number.
	 * @return Remote endpoint port number.
	 */
	[[nodiscard]] uint16_t remote_port() const {
		return remote_endpoint().port();
	}

private:
	asioTCP::socket &baseSocket() {
		return mSocket.next_layer();
	}

	asioTCP::endpoint mLocalEndpoint;
	asioTCP::endpoint mRemoteEndpoint;
	asioSSL::stream<asioTCP::socket> mSocket;
	bool mSocketClosed     = false;
	bool mSecureConnection = false;
};

} // namespace dv::io::network
