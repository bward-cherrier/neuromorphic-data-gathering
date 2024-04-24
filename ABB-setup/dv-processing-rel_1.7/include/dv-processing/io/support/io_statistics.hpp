#pragma once

#include <cstdint>

namespace dv::io::support {

class IOStatistics {
public:
	IOStatistics() = default;

	virtual ~IOStatistics() = default;

	IOStatistics(const IOStatistics &other)            = delete;
	IOStatistics &operator=(const IOStatistics &other) = delete;
	IOStatistics(IOStatistics &&other) noexcept        = default;
	IOStatistics &operator=(IOStatistics &&other)      = default;

	virtual void publish() = 0;

	void addBytes(const uint64_t bytes) {
		mDataSize += bytes;
	}

	void update(const uint64_t addedDataSize, const uint64_t addedPacketsNumber, const uint64_t addedPacketsElements,
		const uint64_t addedPacketsSize) {
		mDataSize        += addedDataSize;
		mPacketsNumber   += addedPacketsNumber;
		mPacketsElements += addedPacketsElements;
		mPacketsSize     += addedPacketsSize;

		publish();
	}

protected:
	uint64_t mPacketsNumber{0};
	uint64_t mPacketsElements{0};
	uint64_t mPacketsSize{0};
	uint64_t mDataSize{0};
};

} // namespace dv::io::support
