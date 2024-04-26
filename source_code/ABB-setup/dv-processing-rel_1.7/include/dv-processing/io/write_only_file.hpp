#pragma once

#include "simplefile.hpp"
#include "writer.hpp"

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>

namespace dv::io {

class WriteOnlyFile : private dv::io::SimpleWriteOnlyFile {
public:
	WriteOnlyFile() = delete;

	WriteOnlyFile(const std::filesystem::path &filePath, const std::string_view outputInfo,
		std::unique_ptr<dv::io::compression::CompressionSupport> compression,
		std::unique_ptr<dv::io::support::IOStatistics> stats = nullptr) :
		dv::io::SimpleWriteOnlyFile(filePath, dv::io::WriteFlags::TRUNCATE),
		mOutputInfo(outputInfo),
		mWriter(std::move(compression), std::move(stats), std::make_unique<dv::FileDataTable>()),
		mWriteThread([this]() {
			writeThread();
		}) {
		if ((filePath.extension().string() != dv::io::support::AEDAT4_FILE_EXTENSION)) {
			throw dv::exceptions::FileNotFound(fmt::format("File extension '{}' is not the required '{}'.",
												   filePath.extension(), dv::io::support::AEDAT4_FILE_EXTENSION),
				filePath);
		}

		mWriter.writeAedatVersion([this](const std::shared_ptr<const dv::io::support::IODataBuffer> version) {
			pushVersion(version);
		});

		// First header write, data-table position is not set.
		mWriter.writeHeader(-1, mOutputInfo, [this](const std::shared_ptr<const dv::io::support::IODataBuffer> header) {
			pushHeader(header);
		});
	}

	WriteOnlyFile(const std::filesystem::path &filePath, const std::string_view outputInfo,
		const CompressionType compression                    = CompressionType::NONE,
		std::unique_ptr<dv::io::support::IOStatistics> stats = nullptr) :
		WriteOnlyFile(
			filePath, outputInfo, dv::io::compression::createCompressionSupport((compression)), std::move(stats)) {
	}

	~WriteOnlyFile() {
		const auto dataTablePosition
			= mWriter.writeFileDataTable([this](const std::shared_ptr<const dv::io::support::IODataBuffer> fdt) {
				  pushFileDataTable(fdt);
			  });

		// Second header write with data-table position.
		mWriter.writeHeader(
			dataTablePosition, mOutputInfo, [this](const std::shared_ptr<const dv::io::support::IODataBuffer> header) {
				pushHeader(header);
			});

		stop();
	}

	void write(const dv::types::TypedObject *const packet, const int32_t streamId) {
		mWriter.writePacket(packet, streamId, [this](const std::shared_ptr<const dv::io::support::IODataBuffer> pkt) {
			pushPacket(pkt);
		});
	}

	void write(const void *ptr, const dv::types::Type &type, const int32_t streamId) {
		mWriter.writePacket(
			ptr, type, streamId, [this](const std::shared_ptr<const dv::io::support::IODataBuffer> pkt) {
				pushPacket(pkt);
			});
	}

private:
	std::string mOutputInfo; // Remember for second header write.
	dv::io::Writer mWriter;
	std::mutex mMutex;
	std::queue<std::function<void(void)>> mWriteBuffer;
	std::atomic<bool> mStopRequested{false};
	std::thread mWriteThread;

	void pushVersion(const std::shared_ptr<const dv::io::support::IODataBuffer> version) {
		const std::scoped_lock lock(mMutex);

		mWriteBuffer.emplace([this, version]() {
			writeVersion(version);
		});
	}

	void pushHeader(const std::shared_ptr<const dv::io::support::IODataBuffer> header) {
		const std::scoped_lock lock(mMutex);

		mWriteBuffer.emplace([this, header]() {
			writeHeader(header);
		});
	}

	void pushPacket(const std::shared_ptr<const dv::io::support::IODataBuffer> packet) {
		const std::scoped_lock lock(mMutex);

		mWriteBuffer.emplace([this, packet]() {
			writePacket(packet);
		});
	}

	void pushFileDataTable(const std::shared_ptr<const dv::io::support::IODataBuffer> fileDataTable) {
		const std::scoped_lock lock(mMutex);

		mWriteBuffer.emplace([this, fileDataTable]() {
			writeFileDataTable(fileDataTable);
		});
	}

	void writeThread() {
		while (!mStopRequested.load()) {
			emptyWriteBuffer();
		}

		// One last call to ensure that everything is written in case buffer is empty, write thread is idle while more
		// data gets added, mStopRequested is set to true, and then mStopRequested is checked before buffer is emptied.
		emptyWriteBuffer();
		flush();
	}

	void stop() {
		mStopRequested.store(true);
		mWriteThread.join();
	}

	void emptyWriteBuffer() {
		std::unique_lock lock(mMutex);

		auto empty = mWriteBuffer.empty();

		while (!empty) {
			const auto writeOp = mWriteBuffer.front();
			mWriteBuffer.pop();

			lock.unlock();
			writeOp();
			lock.lock();

			empty = mWriteBuffer.empty();
		}
	}

	void writeVersion(const std::shared_ptr<const dv::io::support::IODataBuffer> packet) {
		seek(0);
		SimpleWriteOnlyFile::write(packet->getData(), packet->getDataSize());
	}

	void writeHeader(const std::shared_ptr<const dv::io::support::IODataBuffer> packet) {
		seek(static_cast<uint64_t>(dv::Constants::AEDAT_VERSION_LENGTH));
		SimpleWriteOnlyFile::write(packet->getData(), packet->getDataSize());
		flush();
	}

	void writePacket(const std::shared_ptr<const dv::io::support::IODataBuffer> packet) {
		SimpleWriteOnlyFile::write(reinterpret_cast<const std::byte *>(packet->getHeader()), sizeof(dv::PacketHeader));
		SimpleWriteOnlyFile::write(packet->getData(), packet->getDataSize());
	}

	void writeFileDataTable(const std::shared_ptr<const dv::io::support::IODataBuffer> packet) {
		flush(); // Ensure all packet data is committed to disk.

		SimpleWriteOnlyFile::write(packet->getData(), packet->getDataSize());
		flush(); // Ensure table is committed to disk.
	}
};

} // namespace dv::io
