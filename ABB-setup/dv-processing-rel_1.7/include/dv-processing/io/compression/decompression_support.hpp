#pragma once

#include "../../external/fmt_compat.hpp"

#include "../../core/utils.hpp"
#include "../data/IOHeader.hpp"

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

class DecompressionSupport {
public:
	explicit DecompressionSupport(const CompressionType type) : mType(type) {
	}

	virtual ~DecompressionSupport() = default;

	virtual void decompress(std::vector<std::byte> &source, std::vector<std::byte> &target) = 0;

	[[nodiscard]] CompressionType getCompressionType() const {
		return mType;
	}

private:
	CompressionType mType;
};

class ZstdDecompressionSupport : public DecompressionSupport {
public:
	explicit ZstdDecompressionSupport(const CompressionType type) : DecompressionSupport(type) {
		if ((type != CompressionType::ZSTD_HIGH) && (type != CompressionType::ZSTD)) {
			throw std::runtime_error(fmt::format(
				"Compression type {} not supported in ZstdDecompressionSupport", dv::EnumNameCompressionType(type)));
		}

		initDecompressionContext();
	}

	void decompress(std::vector<std::byte> &src, std::vector<std::byte> &target) override {
		const std::vector<std::byte> &source = src;

		const auto decompressedSize = ZSTD_getFrameContentSize(source.data(), source.size());
		if (decompressedSize == ZSTD_CONTENTSIZE_UNKNOWN) {
			throw std::runtime_error("Zstd decompression error: unknown content size");
		}
		if (decompressedSize == ZSTD_CONTENTSIZE_ERROR) {
			throw std::runtime_error("Zstd decompression error: content size error");
		}

		target.resize(decompressedSize);

		const auto ret
			= ZSTD_decompressDCtx(mContext.get(), target.data(), decompressedSize, source.data(), source.size());

		if (ZSTD_isError(ret) != 0) {
			// Decompression error, ignore this packet.
#if defined(ZSTD_VERSION_NUMBER) && ZSTD_VERSION_NUMBER >= 10400
			ZSTD_DCtx_reset(mContext.get(), ZSTD_reset_session_only);
#else
			mContext.reset();
			initDecompressionContext();
#endif
		}
		else {
			target.resize(ret);
		}
	}

private:
	// Support Zstd compression.
	std::shared_ptr<ZSTD_DCtx_s> mContext;

	void initDecompressionContext() {
		// Create Zstd decompression context.
		ZSTD_DCtx_s *const ctx = ZSTD_createDCtx();
		if (ctx == nullptr) {
			throw std::bad_alloc();
		}

		mContext = (std::shared_ptr<ZSTD_DCtx_s>(ctx, [](ZSTD_DCtx_s *const c) {
			ZSTD_freeDCtx(c);
		}));
	}
};

class Lz4DecompressionSupport : public DecompressionSupport {
public:
	explicit Lz4DecompressionSupport(const CompressionType type) : DecompressionSupport(type) {
		if ((type != CompressionType::LZ4_HIGH) && (type != CompressionType::LZ4)) {
			throw std::runtime_error(fmt::format(
				"Compression type {} not supported in Lz4DecompressionSupport", dv::EnumNameCompressionType(type)));
		}

		initDecompressionContext();
	}

	void decompress(std::vector<std::byte> &src, std::vector<std::byte> &target) override {
		const std::vector<std::byte> &source = src;

		size_t decompressedSize = 0;
		size_t ret              = 1;
		size_t dataSize         = source.size();
		const auto *dataPtr     = source.data();

		while ((dataSize > 0) && (ret != 0)) {
			target.resize(decompressedSize + LZ4_DECOMPRESSION_CHUNK_SIZE);
			size_t dstSize = LZ4_DECOMPRESSION_CHUNK_SIZE;

			size_t srcSize = dataSize;

			ret = LZ4F_decompress(
				mContext.get(), target.data() + decompressedSize, &dstSize, dataPtr, &srcSize, nullptr);

			if (LZ4F_isError(ret) != 0) {
#if defined(LZ4_VERSION_NUMBER) && LZ4_VERSION_NUMBER >= 10800
				LZ4F_resetDecompressionContext(mContext.get());
#else
				mContext.reset();
				initDecompressionContext();
#endif
			}

			decompressedSize += dstSize;

			dataSize -= srcSize;
			dataPtr  += srcSize;
		}

		target.resize(decompressedSize);
	}

private:
	static constexpr size_t LZ4_DECOMPRESSION_CHUNK_SIZE{64 * 1024};

	std::shared_ptr<LZ4F_dctx_s> mContext;

	void initDecompressionContext() {
		// Create LZ4 decompression context.
		LZ4F_dctx_s *ctx = nullptr;
		const auto ret   = LZ4F_createDecompressionContext(&ctx, LZ4F_VERSION);
		if (ret != 0) {
			throw std::bad_alloc();
		}

		mContext = (std::shared_ptr<LZ4F_dctx_s>(ctx, [](LZ4F_dctx_s *const c) {
			LZ4F_freeDecompressionContext(c);
		}));
	}
};

class NoneDecompressionSupport : public DecompressionSupport {
public:
	explicit NoneDecompressionSupport(const CompressionType type) : DecompressionSupport(type) {
		if (type != CompressionType::NONE) {
			throw std::runtime_error(fmt::format(
				"Compression type {} not supported in NoneDecompressionSupport", dv::EnumNameCompressionType(type)));
		}
	}

	void decompress(std::vector<std::byte> &source, std::vector<std::byte> &target) override {
		// By design, this method does nothing other than swapping over the contents.
		std::swap(source, target);
		source.clear();
	}
};

static std::unique_ptr<DecompressionSupport> createDecompressionSupport(const CompressionType type) {
	if ((type == CompressionType::LZ4_HIGH) || (type == CompressionType::LZ4)) {
		return std::make_unique<Lz4DecompressionSupport>(type);
	}
	else if ((type == CompressionType::ZSTD_HIGH) || (type == CompressionType::ZSTD)) {
		return std::make_unique<ZstdDecompressionSupport>(type);
	}
	else if (type == CompressionType::NONE) {
		return std::make_unique<NoneDecompressionSupport>(type);
	}
	else {
		throw std::runtime_error(fmt::format("Unsupported compression type: {}", dv::EnumAsInteger(type)));
	}
}

} // namespace dv::io::compression
