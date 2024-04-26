#pragma once

#include "socket_base.hpp"

#include <deque>
#include <functional>
#include <utility>

namespace dv::io::network {

/**
 * Write ordered socket. Implemented because in asio simultaneous async_writes are not allowed.
 * \see https://stackoverflow.com/questions/45813835/boostasio-ordering-of-data-sent-to-a-socket
 */
class WriteOrderedSocket {
private:
	struct WriteJob {
		WriteJob(const asio::const_buffer &buffer, SocketBase::CompletionHandler handler) :
			mBuffer(buffer),
			mHandler(std::move(handler)) {
		}

		asio::const_buffer mBuffer;
		SocketBase::CompletionHandler mHandler;
	};

public:
	explicit WriteOrderedSocket(std::unique_ptr<SocketBase> &&socket) : mSocket(std::move(socket)) {
	}

	/**
	 * Add a buffer to be written out to the socket. This call adds the buffer to a ordered queue that
	 * guarantees that will chain multiple write_async calls to the socket so no simultaneous calls would
	 * happen.
	 * @param buf Buffers to be written into the socket.
	 * @param wrHandler Write handler that is called when buffer write is completed.
	 */
	void write(const asio::const_buffer &buf, SocketBase::CompletionHandler &&wrHandler) {
		SocketBase::CompletionHandler orderedHandler
			= [this, wrHandler](const boost::system::error_code &error, size_t length) {
				  // Execute bound handler.
				  wrHandler(error, length);

				  // Remove currently executing handler from queue (placeholder only).
				  mWriteQueue.pop_front();

				  // On error, clear pending writes and do nothing.
				  if (error) {
					  mWriteQueue.clear();
				  }
				  else {
					  // Start new writes.
					  if (!mWriteQueue.empty()) {
						  auto &writeJob = mWriteQueue.front();
						  mSocket->write(writeJob.mBuffer, std::move(writeJob.mHandler));
					  }
				  }
			  };

		// Check current status.
		const auto noWrites = mWriteQueue.empty();

		// Enqueue all writes.
		mWriteQueue.emplace_back(buf, orderedHandler);

		if (noWrites) {
			// Start first write.
			auto &writeJob = mWriteQueue.front();
			mSocket->write(writeJob.mBuffer, std::move(writeJob.mHandler));
		}
	}

	/**
	 * Close the underlying socket.
	 */
	void close() {
		mSocket->close();
	}

	/**
	 * Check whether underlying socket is open
	 * @return
	 */
	[[nodiscard]] bool isOpen() const {
		return mSocket->isOpen();
	}

	/**
	 * Read data from the socket. This only wraps the read call of the underlying socket.
	 * @param buf
	 * @param rdHandler
	 */
	void read(const asio::mutable_buffer &buf, SocketBase::CompletionHandler &&rdHandler) {
		mSocket->read(buf, std::move(rdHandler));
	}

private:
	/// No locking for writeQueue because all changes are posted to io_service thread.
	std::deque<WriteJob> mWriteQueue;

	/// Underlying socket
	std::unique_ptr<dv::io::network::SocketBase> mSocket;
};

} // namespace dv::io::network
