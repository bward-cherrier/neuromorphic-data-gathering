#pragma once

#include "../../external/fmt_compat.hpp"

#include "../../core/utils.hpp"
#include "../data/IOHeader.hpp"
#include "../support/io_data_buffer.hpp"

#include <lz4.h>
#include <lz4frame.h>
#include <lz4hc.h>
#include <memory>
#include <vector>
#include <zstd.h>

namespace dv::io::compression {

#ifndef LZ4F_HEADER_SIZE_MAX
#	define LZ4F_HEADER_SIZE_MAX 19
#endif

#ifndef ZSTD_CLEVEL_DEFAULT
#	define ZSTD_CLEVEL_DEFAULT 3
#endif

class CompressionSupport {
public:
	explicit CompressionSupport(const CompressionType type) : mType(type) {
	}

	virtual ~CompressionSupport() = default;

	virtual void compress(dv::io::support::IODataBuffer &packet) = 0;

	[[nodiscard]] CompressionType getCompressionType() const {
		return mType;
	}

private:
	CompressionType mType;
};

class ZstdCompressionSupport : public CompressionSupport {
public:
	explicit ZstdCompressionSupport(const CompressionType type) : CompressionSupport(type) {
		if ((type != CompressionType::ZSTD_HIGH) && (type != CompressionType::ZSTD)) {
			throw std::runtime_error(fmt::format(
				"Compression type {} not supported in ZstdCompressionSupport", dv::EnumNameCompressionType(type)));
		}

		// Create Zstd compression context.
		ZSTD_CCtx_s *const ctx = ZSTD_createCCtx();
		if (ctx == nullptr) {
			throw std::bad_alloc();
		}

		mContext = std::shared_ptr<ZSTD_CCtx_s>(ctx, [](ZSTD_CCtx_s *const c) {
			ZSTD_freeCCtx(c);
		});

		// Select appropriate compression flags.
		if (getCompressionType() == CompressionType::ZSTD_HIGH) {
			mLevel = ZSTD_maxCLevel();
		}
	}

	/**
	 * Create a Zstd compression support class with custom compression.  Internally sets compression type to
	 * `CompressionType::ZSTD`.
	 * @param compressionLevel Compression level, recommended range is [1, 22].
	 * @sa For more info on compression level values see here: https://facebook.github.io/zstd/zstd_manual.html
	 */
	explicit ZstdCompressionSupport(const int compressionLevel) :
		CompressionSupport(CompressionType::ZSTD),
		mLevel(compressionLevel) {
		// Create Zstd compression context.
		ZSTD_CCtx_s *const ctx = ZSTD_createCCtx();
		if (ctx == nullptr) {
			throw std::bad_alloc();
		}

		mContext = std::shared_ptr<ZSTD_CCtx_s>(ctx, [](ZSTD_CCtx_s *const c) {
			ZSTD_freeCCtx(c);
		});
	}

	void compress(dv::io::support::IODataBuffer &packet) override {
		// Flatbuffer encoded packet.
		const auto *dataPtr            = packet.getData();
		const auto dataSize            = packet.getDataSize();
		std::vector<std::byte> &target = *packet.getBuffer();

		// Allocate maximum needed memory for compressed data block.
		const auto maxCompressedSize = ZSTD_compressBound(dataSize);
		target.resize(maxCompressedSize);

		// Compress data using Zstd algorithm.
		const auto ret = ZSTD_compressCCtx(mContext.get(), target.data(), maxCompressedSize, dataPtr, dataSize, mLevel);
		if (ZSTD_isError(ret) != 0) {
			// Compression error.
			throw std::runtime_error(fmt::format("Zstd compression error: {}", ZSTD_getErrorName(ret)));
		}

		// Update size.
		target.resize(ret);

		// Switch to vector instead of flatbuffer.
		// The vector will contain the compressed packet.
		packet.switchToBuffer();
	}

private:
	std::shared_ptr<ZSTD_CCtx_s> mContext;
	int mLevel{ZSTD_CLEVEL_DEFAULT};
};

class Lz4CompressionSupport : public CompressionSupport {
public:
	explicit Lz4CompressionSupport(const CompressionType type) :
		CompressionSupport(type),
		mPrefs(type == CompressionType::LZ4 ? lz4CompressionPreferences : lz4HighCompressionPreferences) {
		if ((type != CompressionType::LZ4_HIGH) && (type != CompressionType::LZ4)) {
			throw std::runtime_error(fmt::format(
				"Compression type {} not supported in Lz4CompressionSupport", dv::EnumNameCompressionType(type)));
		}

		// Create LZ4 compression context.
		LZ4F_cctx_s *ctx = nullptr;
		const auto ret   = LZ4F_createCompressionContext(&ctx, LZ4F_VERSION);
		if (ret != 0) {
			throw std::bad_alloc();
		}

		mContext = std::shared_ptr<LZ4F_cctx_s>(ctx, [](LZ4F_cctx_s *const c) {
			LZ4F_freeCompressionContext(c);
		});

		mChunkSize = LZ4F_compressBound(LZ4_COMPRESSION_CHUNK_SIZE, &mPrefs);
		mEndSize   = LZ4F_compressBound(0, &mPrefs);
	}

	/**
	 * LZ4 compression support with custom compression settings. Internally sets compression type to
	 * `CompressionType::LZ4`.
	 * @param preferences LZ4 compression settings.
	 */
	explicit Lz4CompressionSupport(const LZ4F_preferences_t &preferences) :
		CompressionSupport(CompressionType::LZ4),
		mPrefs(preferences) {
		// Create LZ4 compression context.
		LZ4F_cctx_s *ctx = nullptr;
		const auto ret   = LZ4F_createCompressionContext(&ctx, LZ4F_VERSION);
		if (ret != 0) {
			throw std::bad_alloc();
		}

		mContext = std::shared_ptr<LZ4F_cctx_s>(ctx, [](LZ4F_cctx_s *const c) {
			LZ4F_freeCompressionContext(c);
		});

		mChunkSize = LZ4F_compressBound(LZ4_COMPRESSION_CHUNK_SIZE, &mPrefs);
		mEndSize   = LZ4F_compressBound(0, &mPrefs);
	}

	void compress(dv::io::support::IODataBuffer &packet) override {
		// Flatbuffer encoded packet.
		const auto *dataPtr            = packet.getData();
		auto dataSize                  = packet.getDataSize();
		std::vector<std::byte> &target = *packet.getBuffer();

		// Write out header of compressed frame.
		target.resize(LZ4F_HEADER_SIZE_MAX);

		auto written = LZ4F_compressBegin(mContext.get(), target.data(), LZ4F_HEADER_SIZE_MAX, &mPrefs);
		if (LZ4F_isError(written) != 0) {
			// Compression error.
			throw std::runtime_error(fmt::format("LZ4 compression error: {}", LZ4F_getErrorName(written)));
		}

		while (dataSize > 0) {
			// Write out compressed data in chunks.
			auto chunkSize         = LZ4_COMPRESSION_CHUNK_SIZE;
			auto maxCompressedSize = mChunkSize;

			if (dataSize < LZ4_COMPRESSION_CHUNK_SIZE) {
				chunkSize         = dataSize;
				maxCompressedSize = LZ4F_compressBound(chunkSize, &mPrefs);
			}

			target.resize(written + maxCompressedSize);

			const auto ret = LZ4F_compressUpdate(
				mContext.get(), target.data() + written, maxCompressedSize, dataPtr, chunkSize, nullptr);
			if (LZ4F_isError(ret) != 0) {
				// Compression error.
				throw std::runtime_error(fmt::format("LZ4 compression error: {}", LZ4F_getErrorName(ret)));
			}
			else {
				written += ret;
			}

			// Update counters.
			dataPtr  += chunkSize;
			dataSize -= chunkSize;
		}

		// Write out end of compressed frame.
		target.resize(written + mEndSize);

		const auto ret = LZ4F_compressEnd(mContext.get(), target.data() + written, mEndSize, nullptr);
		if (LZ4F_isError(ret) != 0) {
			// Compression error.
			throw std::runtime_error(fmt::format("LZ4 compression error: {}", LZ4F_getErrorName(ret)));
		}
		else {
			written += ret;
		}

		// Set final size.
		target.resize(written);

		// Switch to vector instead of flatbuffer.
		// The vector will contain the compressed packet.
		packet.switchToBuffer();
	}

private:
	static constexpr size_t LZ4_COMPRESSION_CHUNK_SIZE{64 * 1024};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
	static constexpr LZ4F_preferences_t lz4CompressionPreferences = {
		{LZ4F_max64KB, LZ4F_blockLinked, LZ4F_noContentChecksum, LZ4F_frame},
		0, /* compression level; 0 == default fast level */
		0, /* autoflush */
	};

	static constexpr LZ4F_preferences_t lz4HighCompressionPreferences = {
		{LZ4F_max64KB, LZ4F_blockLinked, LZ4F_noContentChecksum, LZ4F_frame},
		9, /* compression level; 9 == default HC level */
		0, /* autoflush */
	};
#pragma GCC diagnostic pop

	std::shared_ptr<LZ4F_cctx_s> mContext;
	const LZ4F_preferences_t mPrefs;
	size_t mChunkSize;
	size_t mEndSize;
};

class NoneCompressionSupport : public CompressionSupport {
public:
	explicit NoneCompressionSupport(const CompressionType type) : CompressionSupport(type) {
		if (type != CompressionType::NONE) {
			throw std::runtime_error(fmt::format(
				"Compression type {} not supported in NoneCompressionSupport", dv::EnumNameCompressionType(type)));
		}
	}

	void compress([[maybe_unused]] dv::io::support::IODataBuffer &packet) override {
		// By design, this method does nothing.
	}
};

static std::unique_ptr<CompressionSupport> createCompressionSupport(const CompressionType type) {
	if ((type == CompressionType::LZ4_HIGH) || (type == CompressionType::LZ4)) {
		return std::make_unique<Lz4CompressionSupport>(type);
	}
	else if ((type == CompressionType::ZSTD_HIGH) || (type == CompressionType::ZSTD)) {
		return std::make_unique<ZstdCompressionSupport>(type);
	}
	else if (type == CompressionType::NONE) {
		return std::make_unique<NoneCompressionSupport>(type);
	}
	else {
		throw std::runtime_error(fmt::format("Unsupported compression type: {}", dv::EnumAsInteger(type)));
	}
}

} // namespace dv::io::compression
