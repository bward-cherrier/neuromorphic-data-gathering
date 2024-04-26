#pragma once

#include "../../include/dv-processing/core/core.hpp"

#include <fstream>
#include <optional>

namespace test {

class CsvEventReader {
protected:
	std::ifstream fileInput;

public:
	CsvEventReader(const std::string &file) : fileInput(file) {
	}

	[[nodiscard]] std::optional<dv::Event> readEvent() {
		int16_t x, y, polarity;
		double timestamp;
		if (fileInput >> timestamp >> x >> y >> polarity) {
			return dv::Event(static_cast<int64_t>(timestamp * 1e+6), x, y, polarity);
		}
		else {
			return std::nullopt;
		}
	}

	[[nodiscard]] std::optional<dv::EventStore> readBatch(size_t batchSize) {
		dv::EventStore batch;
		for (size_t i = 0; i < batchSize; i++) {
			if (auto event = readEvent()) {
				batch.push_back(*event);
			}
			else if (batch.isEmpty()) {
				return std::nullopt;
			}
			else {
				break;
			}
		}
		return batch;
	}
};

} // namespace test
