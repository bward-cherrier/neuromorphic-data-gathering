#pragma once

#include "../../include/dv-processing/core/event.hpp"

#include <filesystem>
#include <fstream>
#include <optional>

namespace fs = std::filesystem;

namespace test {

class EventWriter {
private:
	const fs::path outPath;
	std::ofstream outStream;

public:
	explicit EventWriter(const fs::path &path) : outPath(path) {
		if (fs::exists(path)) {
			std::stringstream ss;
			ss << "File exists: " << outPath << std::endl;
			throw std::runtime_error(ss.str());
		}

		outStream = std::ofstream(path, std::ios::out | std::ios::binary);

		if (!outStream.is_open()) {
			std::stringstream ss;
			ss << "Failed to open file stream at path: " << outPath << std::endl;
			throw std::runtime_error(ss.str());
		}
	}

	virtual ~EventWriter() {
		if (outStream.is_open()) {
			outStream.close();
		}
	}

	/**
	 * Write a vector of events to a file.
	 *
	 * @param events
	 */
	void writeData(const std::vector<dv::Event> &events) {
		for (const auto &event : events) {
			outStream.write(reinterpret_cast<const char *>(&event), sizeof(event));
		}
	}
};

class EventReader {
private:
	const fs::path inPath;
	std::ifstream inStream;

	bool end() {
		return inStream.tellg() == -1;
	}

public:
	explicit EventReader(const fs::path &path) : inPath(path) {
		if (!fs::exists(path)) {
			std::stringstream ss;
			ss << "File does not exist: " << inPath << std::endl;
			throw std::runtime_error(ss.str());
		}

		inStream = std::ifstream(path, std::ios::in | std::ios::binary);

		if (!inStream.is_open()) {
			std::stringstream ss;
			ss << "Failed to open file stream at path: " << inPath << std::endl;
			throw std::runtime_error(ss.str());
		}
	}

	virtual ~EventReader() {
		if (inStream.is_open()) {
			inStream.close();
		}
	}

	/**
	 * Read a single event.
	 *
	 * @param event output event struct
	 * @return false when the end of file is reached
	 */
	bool readData(dv::Event *event) {
		dv::runtime_assert(event != nullptr, "event cannot be NULL");

		if (end()) {
			return false;
		}

		inStream.read(reinterpret_cast<char *>(event), sizeof(*event));
		return !end();
	}

	[[nodiscard]] std::optional<dv::EventStore> readBatch(size_t batchSize) {
		dv::EventStore batch;
		for (size_t i = 0; i < batchSize; i++) {
			dv::Event ev;
			if (readData(&ev)) {
				batch.add(ev);
			}
			else if (batch.isEmpty()) {
				return std::nullopt;
			}
		}
		return batch;
	}
};

} // namespace test
