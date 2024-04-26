#pragma once

#include "../../external/fmt_compat.hpp"

#include "../../exception/exception.hpp"
#include "../camera_geometry.hpp"

#include <Eigen/Core>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/core.hpp>

#include <optional>
#include <span>

namespace dv::camera::calibrations {

namespace pt = boost::property_tree;

class CameraCalibration {
public:
	class Metadata {
	public:
		/// Shape of the calibration pattern
		cv::Size patternShape;

		/// Shape of the calibration pattern in terms of internal intersections
		cv::Size internalPatternShape;

		/// Type of the calibration pattern used (e.g. apriltag)
		std::string patternType;

		/// Size of the calibration pattern in [m]
		float patternSize = -1.f;

		/// Ratio between tags to patternSize (apriltag only)
		float patternSpacing = -1.f;

		/// Calibration reprojection error
		std::optional<float> calibrationError = std::nullopt;

		/// Timestamp when the calibration was conducted
		std::string calibrationTime;

		/// Description of the calibration quality (excellent/good/bad etc)
		std::string quality;

		/// Any additional information
		std::string comment;

		/// Pixel pitch in meters
		std::optional<float> pixelPitch = std::nullopt;

		Metadata() = default;

		Metadata(const cv::Size &patternShape_, const cv::Size &internalPatternShape_, const std::string &patternType_,
			float patternSize_, float patternSpacing_, const std::optional<float> &calibrationError_,
			const std::string &calibrationTime_, const std::string &quality_, const std::string &comment_,
			const std::optional<float> &pixelPitch_) :
			patternShape(patternShape_),
			internalPatternShape(internalPatternShape_),
			patternType(patternType_),
			patternSize(patternSize_),
			patternSpacing(patternSpacing_),
			calibrationError(calibrationError_),
			calibrationTime(calibrationTime_),
			quality(quality_),
			comment(comment_),
			pixelPitch(pixelPitch_) {
		}

		/**
		 * Create an instance of metadata from a property tree structure.
		 * @param tree 		Property tree to be parsed.
		 * @return 			Constructed Metadata instance.
		 */
		explicit Metadata(const pt::ptree &tree) :
			Metadata(parsePair<cv::Size, int>(tree, "patternShape", -1),
				parsePair<cv::Size, int>(tree, "internalPatternShape", -1),
				tree.get_child("patternType").get_value<std::string>(""),
				tree.get_child("patternSize").get_value<float>(-1.f),
				tree.get_child("patternSpacing").get_value<float>(-1.f), std::nullopt,
				tree.get_child("calibrationTime").get_value<std::string>(""),
				tree.get_child("quality").get_value<std::string>(""),
				tree.get_child("comment").get_value<std::string>(""), std::nullopt) {
			auto errorVal = tree.get_child_optional("calibrationError");
			if (errorVal.has_value()) {
				calibrationError = errorVal->get_value<float>();
			}
			auto pitch = tree.get_child_optional("pixelPitch");
			if (pitch.has_value()) {
				pixelPitch = pitch->get_value<float>();
			}
		}

		/**
		 * Serialize the metadata structure into a property tree.
		 * @return		Serialized property tree.
		 */
		[[nodiscard]] pt::ptree toPropertyTree() const {
			pt::ptree tree;
			pushVectorToTree<int>("patternShape", {patternShape.width, patternShape.height}, tree);
			pushVectorToTree<int>(
				"internalPatternShape", {internalPatternShape.width, internalPatternShape.height}, tree);
			tree.put("patternType", patternType);
			tree.put("patternSize", patternSize);
			tree.put("patternSpacing", patternSpacing);
			if (calibrationError.has_value()) {
				tree.put("calibrationError", *calibrationError);
			}
			tree.put("calibrationTime", calibrationTime);
			tree.put("quality", quality);
			tree.put("comment", comment);
			if (pixelPitch.has_value()) {
				tree.put("pixelPitch", *pixelPitch);
			}
			return tree;
		}

		/**
		 * Equality operator.
		 * @param rhs
		 * @return
		 */
		bool operator==(const Metadata &rhs) const {
			return patternShape == rhs.patternShape && internalPatternShape == rhs.internalPatternShape
				&& patternType == rhs.patternType && patternSize == rhs.patternSize
				&& patternSpacing == rhs.patternSpacing && calibrationError == rhs.calibrationError
				&& calibrationTime == rhs.calibrationTime && quality == rhs.quality && comment == rhs.comment
				&& pixelPitch == rhs.pixelPitch;
		}
	};

	/// Camera name (e.g. "DVXplorer_DXA02137")
	std::string name;

	/// Description of the location of the camera in the camera rig (e.g. "left")
	std::string position;

	/// Indicate whether it is the master camera in a multi-camera rig
	bool master = false;

	/// Camera resolution width
	cv::Size resolution;

	/// Intersection of optical axis and image plane
	cv::Point2f principalPoint;

	/// Focal length
	cv::Point2f focalLength;

	/// Distortion coefficients
	std::vector<float> distortion;

	/// Distortion model used
	DistortionModel distortionModel = DistortionModel::RadTan;

	/// Transformation from camera zero to this camera
	std::vector<float> transformationToC0;

	/// Metadata
	std::optional<Metadata> metadata = std::nullopt;

	CameraCalibration() = default;

	/**
	 * Parse a property tree and initialize camera calibration out of it.
	 * @param tree 		Serialized property tree containing camera intrinsics calibration.
	 */
	explicit CameraCalibration(const pt::ptree &tree) :
		CameraCalibration(tree.get_child("name").get_value<std::string>(""),
			tree.get_child("position").get_value<std::string>(""), tree.get_child("master").get_value<bool>(false),
			parsePair<cv::Size, int>(tree, "resolution"), parsePair<cv::Point2f, float>(tree, "principalPoint"),
			parsePair<cv::Point2f, float>(tree, "focalLength"),
			getVectorFromTree<float>("distortionCoefficients", tree),
			stringToDistortionModel(
				tree.get_child("distortionModel").get_value<std::string>(std::string{internal::NoneModelString})),
			getVectorFromTree<float>("transformationToC0", tree), getOptionalMetadata<Metadata>(tree, "metadata")) {
		validateTransformation(transformationToC0);
	}

	/**
	 * Construct the camera calibration
	 * @param name_ 				Camera name (e.g. "DVXplorer_DXA02137")
	 * @param position_ 			Description of the location of the camera in the camera rig (e.g. "left")
	 * @param master_				Whether camera was a master camera during calibration
	 * @param resolution_ 			Camera resolution
	 * @param principalPoint_		Principal point
	 * @param focalLength_			Focal length
	 * @param distortion_ 			Distortion coefficients
	 * @param distortionModel_ 		Distortion model used (can be empty string or "radialTangential")
	 * @param transformationToC0_ 	Transformation from camera zero to this camera
	 * @param metadata_ 			Metadata
	 */
	CameraCalibration(const std::string_view name_, const std::string_view position_, const bool master_,
		const cv::Size &resolution_, const cv::Point2f &principalPoint_, const cv::Point2f &focalLength_,
		const std::vector<float> &distortion_, const DistortionModel &distortionModel_,
		std::span<const float> transformationToC0View, const std::optional<Metadata> &metadata_) :
		name(name_),
		position(position_),
		master(master_),
		resolution(resolution_),
		principalPoint(principalPoint_),
		focalLength(focalLength_),
		distortion(distortion_),
		distortionModel(distortionModel_),
		transformationToC0(transformationToC0View.begin(), transformationToC0View.end()),
		metadata(metadata_) {
		validateTransformation(transformationToC0);

		if ((distortionModel == DistortionModel::RadTan) && ((distortion.size() < 4) || (distortion.size() > 5))) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"Number of distortion coefficients for \"radialTangential\" model must be 4 or 5.", distortion.size());
		}
		else if ((distortionModel == DistortionModel::Equidistant) && (distortion.size() != 4)) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"Number of distortion coefficients for \"equidistant\" model must be 4.", distortion.size());
		}
		else if ((distortionModel == DistortionModel::None) && (!distortion.empty())) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"Number of distortion coefficients for \"none\" model must be 0.", distortion.size());
		}
	}

	/**
	 * Serialize the CameraCalibration structure into a property tree.
	 * @return		Serialized property tree.
	 */
	[[nodiscard]] pt::ptree toPropertyTree() const {
		pt::ptree tree;
		tree.put("name", name);
		tree.put("position", position);
		tree.put("master", master);
		pushVectorToTree<int>("resolution", {resolution.width, resolution.height}, tree);
		pushVectorToTree<float>("principalPoint", {principalPoint.x, principalPoint.y}, tree);
		pushVectorToTree<float>("focalLength", {focalLength.x, focalLength.y}, tree);
		pushVectorToTree("distortionCoefficients", distortion, tree);
		tree.put("distortionModel", distortionModelToString(distortionModel));
		pushVectorToTree("transformationToC0", transformationToC0, tree);
		if (metadata.has_value()) {
			tree.put_child("metadata", metadata->toPropertyTree());
		}
		return tree;
	}

	/**
	 * Return the transformation matrix to C0 as a Eigen matrix.
	 * @return 	Eigen matrix containing transformation to camera "C0".
	 */
	[[nodiscard]] Eigen::Matrix4f getTransformMatrix() const {
		return Eigen::Matrix<float, 4, 4, Eigen::RowMajor>(transformationToC0.data());
	}

	/**
	 * Get camera matrix in the format:
	 * | mFx  0 mCx |
	 * |  0 mFy mCy |
	 * |  0  0  1 |
	 * for direct OpenCV compatibility.
	 * @return      3x3 Camera matrix with pixel length values
	 */
	[[nodiscard]] cv::Matx33f getCameraMatrix() const {
		cv::Matx33f camMat;
		camMat << focalLength.x, 0.f, principalPoint.x, 0.f, focalLength.y, principalPoint.y, 0.f, 0.f, 1.f;
		return camMat;
	}

	/**
	 * Equality operator for the class, compares each member of the class.
	 * @param rhs 	Other instance of this class
	 * @return
	 */
	[[nodiscard]] bool operator==(const CameraCalibration &rhs) const {
		return master == rhs.master && name == rhs.name && position == rhs.position && resolution == rhs.resolution
			&& principalPoint == rhs.principalPoint && focalLength == rhs.focalLength && distortion == rhs.distortion
			&& distortionModel == rhs.distortionModel && transformationToC0 == rhs.transformationToC0
			&& metadata == rhs.metadata;
	}

	/**
	 * Retrieve camera geometry instance from this calibration instance. Distortion model is going to be
	 * ignored if the `CameraGeometry` class doesn't support the distortion model.
	 *
	 * CameraGeometry class only supports "radialTangential" distortion model.
	 * @return 		Camera geometry class that implements geometrical transformations of pixel coordinates.
	 */
	[[nodiscard]] dv::camera::CameraGeometry getCameraGeometry() const {
		if (distortion.empty()) {
			return {focalLength.x, focalLength.y, principalPoint.x, principalPoint.y, resolution};
		}

		return {
			distortion, focalLength.x, focalLength.y, principalPoint.x, principalPoint.y, resolution, distortionModel};
	}

	/**
	 * Serialize the object into a stream.
	 * @param os
	 * @param calibration
	 * @return
	 */
	friend std::ostream &operator<<(std::ostream &os, const CameraCalibration &calibration) {
		os << std::boolalpha;
		os << "name: " << calibration.name << std::endl;
		os << "position: " << calibration.position << std::endl;
		os << "master: " << calibration.master << std::endl;
		os << "resolution: " << calibration.resolution << std::endl;
		os << "principalPoint: " << calibration.principalPoint << std::endl;
		os << "focalLength: " << calibration.focalLength << std::endl;
		os << "distortion: " << fmt::format("[{}]", fmt::join(calibration.distortion, ", ")) << std::endl;
		os << "distortionModel: " << calibration.distortionModel << std::endl;
		os << "transformationToC0: " << fmt::format("[{}]", fmt::join(calibration.transformationToC0, ", "))
		   << std::endl;
		os << std::noboolalpha;
		return os;
	}

	/**
	 * Get distortion model name as a string.
	 * @return Distortion model name.
	 */
	[[nodiscard]] std::string getDistortionModelString() const {
		return distortionModelToString(distortionModel);
	}

protected:
	/**
	 * Push a vector of the given type to the property tree at the given key.
	 */
	template<typename T>
	static void pushVectorToTree(const std::string &key, const std::vector<T> &vals, pt::ptree &tree) {
		pt::ptree row;
		for (const auto &val : vals) {
			pt::ptree cell;
			cell.put_value(val);
			row.push_back(std::make_pair("", cell));
		}
		tree.add_child(key, row);
	}

	/**
	 * Retrieve a vector of the given type from the property tree from the given key.
	 * @return		A sequence value in a std::vector container.
	 */
	template<typename T>
	static std::vector<T> getVectorFromTree(const std::string &key, const pt::ptree &tree) {
		std::vector<T> vec;

		for (const auto &cell : tree.get_child(key)) {
			vec.push_back(cell.second.get_value<T>());
		}

		return vec;
	}

	template<class Container, typename Scalar>
	static Container parsePair(
		const pt::ptree &child, const std::string &name, std::optional<Scalar> defaults = std::nullopt) {
		// If default value is provided and tree doesn't exist, return defaults
		if (defaults.has_value() && !child.get_child_optional(name).has_value()) {
			return {defaults.value(), defaults.value()};
		}

		const auto vec = getVectorFromTree<Scalar>(name, child);
		if (vec.size() != 2) {
			throw dv::exceptions::InvalidArgument<size_t>(
				fmt::format("Expected number of items in {} is 2", name), vec.size());
		}
		return {vec[0], vec[1]};
	}

	template<class Container, typename Scalar>
	static Container parseTripple(
		const pt::ptree &child, const std::string &name, std::optional<Scalar> defaults = std::nullopt) {
		// If default value is provided and tree doesn't exist, return defaults
		if (defaults.has_value() && !child.get_child_optional(name).has_value()) {
			return {defaults.value(), defaults.value(), defaults.value()};
		}

		const auto vec = CameraCalibration::getVectorFromTree<Scalar>(name, child);
		if (vec.size() != 3) {
			throw dv::exceptions::InvalidArgument<size_t>(
				fmt::format("Expected number of items in {} is 3", name), vec.size());
		}
		return {vec[0], vec[1], vec[2]};
	}

	template<class MetadataClass>
	static std::optional<MetadataClass> getOptionalMetadata(
		const boost::property_tree::ptree &tree, const std::string &path) {
		const auto child = tree.get_child_optional(path);
		if (child.has_value()) {
			return MetadataClass(*child);
		}
		else {
			return std::nullopt;
		}
	}

	static bool homogeneityCheck(const std::vector<float> &transformation) {
		return std::abs(transformation[12]) <= std::numeric_limits<float>::epsilon()
			&& std::abs(transformation[13]) <= std::numeric_limits<float>::epsilon()
			&& std::abs(transformation[14]) <= std::numeric_limits<float>::epsilon()
			&& std::abs(transformation[15] - 1.f) <= std::numeric_limits<float>::epsilon();
	}

	static void validateTransformation(const std::vector<float> &transformation) {
		if (transformation.size() != 16) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"Expected size of transformationToC0 is 16", transformation.size());
		}
		if (!homogeneityCheck(transformation)) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"The provided transformation does not satisfy homogenous representation check, please make "
				"sure the transformation matrix is provided correctly.",
				fmt::format("[{}]", fmt::join(transformation, ", ")));
		}
	}

	friend struct IMUCalibration;
	friend struct StereoCalibration;
};

} // namespace dv::camera::calibrations
