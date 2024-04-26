#pragma once

#include "../../include/dv-processing/kinematics/linear_transformer.hpp"

#include "CsvEventReader.hpp"

#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include <optional>

namespace test {

namespace fs = std::filesystem;

class EventDatasetReader {
protected:
	std::unique_ptr<CsvEventReader> reader = nullptr;
	fs::path root;
	std::map<int64_t, fs::path> images;
	std::map<int64_t, fs::path>::const_iterator imageIterator;
	size_t batchSize = 200;

public:
	static cv::Mat imread(const std::string &filename, int flags) {
		std::ifstream file(filename, std::ios::binary | std::ios::ate);
		std::streamsize size = file.tellg();
		file.seekg(0, std::ios::beg);

		std::vector<char> buffer(static_cast<size_t>(size));
		if (file.read(buffer.data(), size)) {
			return cv::imdecode(buffer, flags);
		}
		else {
			throw std::invalid_argument("Failed to read input image file [" + filename + "]");
		}
	}

	explicit EventDatasetReader(const std::string &directory) {
		root                = fs::path(directory);
		fs::path imageList  = root / "images.txt";
		fs::path eventsFile = root / "events.txt";

		if (!fs::is_directory(root) || !fs::is_directory(root / "images") || !fs::is_regular_file(imageList)
			|| !fs::is_regular_file(eventsFile)) {
			throw std::invalid_argument("Given dataset path does not contain compatible dataset");
		}

		reader = std::make_unique<CsvEventReader>(eventsFile.string());
		std::ifstream infile(imageList);
		double seconds;
		std::string path;
		while (infile >> seconds >> path) {
			images.insert(std::make_pair(static_cast<int64_t>(seconds * 1000000.), root / path));
		}
		imageIterator = images.cbegin();
	}

	[[nodiscard]] std::optional<std::pair<int64_t, cv::Mat>> readNextImage() {
		imageIterator++;
		if (imageIterator == images.end()) {
			return std::nullopt;
		}
		cv::Mat image = EventDatasetReader::imread(imageIterator->second.string(), cv::IMREAD_GRAYSCALE);
		return std::make_pair(imageIterator->first, image);
	}

	[[nodiscard]] std::optional<dv::EventStore> readEventBatch(const size_t length) {
		return reader->readBatch(length);
	}

	[[nodiscard]] dv::EventStore readEventsUntil(const int64_t timestamp) {
		dv::EventStore store;
		while (auto batch = reader->readBatch(batchSize)) {
			store.add(*batch);
			if (timestamp < batch->getHighestTime()) {
				break;
			}
		}
		return store;
	}

	[[nodiscard]] std::string getCalibrationPath() const {
		return (root / "calib.txt").string();
	}

	[[nodiscard]] dv::kinematics::LinearTransformerf getGroundTruthTrajectory() const {
		auto gt           = dv::kinematics::LinearTransformerf(20000);
		fs::path filepath = root / "groundtruth.txt";
		if (!fs::exists(filepath)) {
			throw std::runtime_error("File does not exist: " + filepath.string());
		}

		std::ifstream infile(filepath.string());
		float ts, t_x, t_y, t_z, q_x, q_y, q_z, q_w;
		while (infile >> ts >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w) {
			Eigen::Vector3f translation(t_x, t_y, t_z);
			Eigen::Quaternionf rotation(q_w, q_x, q_y, q_z);
			dv::kinematics::Transformationf transform(static_cast<int64_t>(ts * 1e+6f), translation, rotation);
			gt.pushTransformation(transform);
		}

		return gt;
	}
};

} // namespace test
