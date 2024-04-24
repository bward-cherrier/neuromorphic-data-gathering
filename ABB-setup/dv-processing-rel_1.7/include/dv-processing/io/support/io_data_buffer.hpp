#pragma once

#include "../data/FileDataTable.hpp"

#include <vector>

namespace dv::io::support {

class IODataBuffer {
public:
	IODataBuffer() = default;

	[[nodiscard]] dv::PacketHeader *getHeader() {
		return (&mHeader);
	}

	[[nodiscard]] const dv::PacketHeader *getHeader() const {
		return (&mHeader);
	}

	[[nodiscard]] flatbuffers::FlatBufferBuilder *getBuilder() {
		return (&mBuilder);
	}

	[[nodiscard]] std::vector<std::byte> *getBuffer() {
		return (&mBuffer);
	}

	[[nodiscard]] const std::byte *getData() const {
		if (mIsFlatBuffer) {
			return reinterpret_cast<const std::byte *>(mBuilder.GetBufferPointer());
		}
		else {
			return mBuffer.data();
		}
	}

	[[nodiscard]] size_t getDataSize() const {
		return (mIsFlatBuffer) ? (mBuilder.GetSize()) : (mBuffer.size());
	}

	void switchToBuffer() {
		mIsFlatBuffer = false;
	}

private:
	static constexpr size_t INITIAL_SIZE{64 * 1024};

	dv::PacketHeader mHeader;
	std::vector<std::byte> mBuffer;
	flatbuffers::FlatBufferBuilder mBuilder{INITIAL_SIZE};
	bool mIsFlatBuffer{true};
};

} // namespace dv::io::support
