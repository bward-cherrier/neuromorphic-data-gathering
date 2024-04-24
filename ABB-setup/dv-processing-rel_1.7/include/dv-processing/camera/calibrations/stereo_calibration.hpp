#pragma once

#include "camera_calibration.hpp"

namespace dv::camera::calibrations {

struct StereoCalibration {
	/**
	 * Metadata for the stereo calibration.
	 */
	struct Metadata {
		/// Average epipolar error
		std::optional<float> epipolarError = std::nullopt;

		/// Any additional information
		std::string comment;

		Metadata() = default;

		explicit Metadata(const std::optional<float> &epipolarError, const std::string_view comment = "") :
			epipolarError(epipolarError),
			comment(comment) {
		}

		explicit Metadata(const pt::ptree &tree) :
			Metadata(std::nullopt, tree.get_child("comment").get_value<std::string>("")) {
			auto errorVal = tree.get_child_optional("epipolarError");
			if (errorVal.has_value()) {
				epipolarError = errorVal->get_value<float>();
			}
		}

		/**
		 * Serialize into a property tree.
		 * @return
		 */
		[[nodiscard]] pt::ptree toPropertyTree() const {
			pt::ptree tree;
			if (epipolarError.has_value()) {
				tree.put("epipolarError", *epipolarError);
			}
			tree.put("comment", comment);
			return tree;
		}

		bool operator==(const Metadata &rhs) const {
			return epipolarError == rhs.epipolarError && comment == rhs.comment;
		}
	};

	/// Name of the left camera
	std::string leftCameraName;

	/// Name of the right camera
	std::string rightCameraName;

	/// Stereo calibration Fundamental Matrix
	std::vector<float> fundamentalMatrix;

	/// Stereo calibration Essential Matrix
	std::vector<float> essentialMatrix;

	/// Metadata
	std::optional<Metadata> metadata;

	StereoCalibration() = default;

	StereoCalibration(const std::string &leftName, const std::string &rightName,
		const std::vector<float> &fundamentalMatrix_, const std::vector<float> &essentialMatrix_,
		const std::optional<Metadata> &metadata_) :
		leftCameraName(leftName),
		rightCameraName(rightName),
		fundamentalMatrix(fundamentalMatrix_),
		essentialMatrix(essentialMatrix_),
		metadata(metadata_) {
		if (fundamentalMatrix.size() != 9) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"Expected number of elements of fundamentalMatrix is 9", fundamentalMatrix.size());
		}
		if (essentialMatrix.size() != 9) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"Expected number of elements of essentialMatrix is 9", essentialMatrix.size());
		}
	}

	explicit StereoCalibration(const pt::ptree &tree) :
		StereoCalibration(tree.get_child("leftCameraName").get_value<std::string>(""),
			tree.get_child("rightCameraName").get_value<std::string>(""),
			CameraCalibration::getVectorFromTree<float>("fundamentalMatrix", tree),
			CameraCalibration::getVectorFromTree<float>("essentialMatrix", tree),
			CameraCalibration::getOptionalMetadata<Metadata>(tree, "metadata")) {
	}

	[[nodiscard]] pt::ptree toPropertyTree() const {
		pt::ptree tree;
		tree.put("leftCameraName", leftCameraName);
		tree.put("rightCameraName", rightCameraName);
		CameraCalibration::pushVectorToTree("fundamentalMatrix", fundamentalMatrix, tree);
		CameraCalibration::pushVectorToTree("essentialMatrix", essentialMatrix, tree);
		if (metadata.has_value()) {
			tree.put_child("metadata", metadata->toPropertyTree());
		}
		return tree;
	}

	bool operator==(const StereoCalibration &rhs) const {
		return leftCameraName == rhs.leftCameraName && rightCameraName == rhs.rightCameraName
			&& fundamentalMatrix == rhs.fundamentalMatrix && essentialMatrix == rhs.essentialMatrix
			&& metadata == rhs.metadata;
	}

	/**
	 * Retrieve the fundamental matrix as Eigen::Matrix3f.
	 * @return 		Fundamental matrix.
	 */
	[[nodiscard]] Eigen::Matrix3f getFundamentalMatrix() const {
		return Eigen::Matrix3f(fundamentalMatrix.data());
	}

	/**
	 * Retrieve the essential matrix as Eigen::Matrix3f.
	 * @return 		Essential matrix.
	 */
	[[nodiscard]] Eigen::Matrix3f getEssentialMatrix() const {
		return Eigen::Matrix3f(essentialMatrix.data());
	}
};

} // namespace dv::camera::calibrations
