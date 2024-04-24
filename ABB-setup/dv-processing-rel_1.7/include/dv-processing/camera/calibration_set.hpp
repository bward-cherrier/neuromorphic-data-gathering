#pragma once

#include "../core/utils.hpp"
#include "../exception/exception.hpp"
#include "calibrations/camera_calibration.hpp"
#include "calibrations/imu_calibration.hpp"
#include "calibrations/stereo_calibration.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <map>
#include <regex>
#include <vector>

namespace fs = std::filesystem;
namespace pt = boost::property_tree;

namespace dv::camera {

/**
 * CalibrationSet class is used to store, serialize and deserialize various camera related calibrations - intrinsic,
 * extrinsic, IMU calibrations. Supports multi-camera and multi sensor setups.
 *
 * Each calibration for each sensor received a designation string which consist of a letter determining the type
 * of sensor and a numeric index automatically generated for each sensor. Designation string look like this:
 * "C0" - camera with index 0
 * "S0" - IMU sensor with index 0
 * "C0C1" - stereo calibration where C0 is the left camera and C1 is the right camera in the camera rig setup.
 *
 * Designation indexes are automatically incremented by the order they are added to the calibration set.
 */
class CalibrationSet {
public:
	static constexpr std::array<float, 16> identity{
		1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};

	CalibrationSet() = default;

	/**
	 * Create a calibration file representation from a persistent file. Supports legacy ".xml"
	 * calibration files produced by DV as well as JSON files containing calibration of a new format.
	 *
	 * The file format is distinguished using the file path extension.
	 * @param path 		Path to calibration file.
	 * @return 			CalibrationFile instanced containing parsed calibration values.
	 */
	static CalibrationSet LoadFromFile(const fs::path &path) {
		if (!fs::exists(path)) {
			throw dv::exceptions::FileNotFound("Calibration file doesn't exist, please check the path!", path.string());
		}
		if (!fs::is_regular_file(path)) {
			throw dv::exceptions::FileError(
				"Calibration file is not a regular file, please check the path!", path.string());
		}

		if (path.extension() == ".json") {
			return cameraRigCalibrationFromJsonFile(path);
		}
		else if (path.extension() == ".xml") {
			return cameraRigCalibrationFromXmlFile(path);
		}
		else {
			throw dv::exceptions::InvalidArgument<std::string>(
				"Unsupported calibration file extension", path.extension().string());
		}
	}

	/**
	 * Serialize calibration data into a property tree that can be saved into a file
	 * using `boost::property_tree::write_json` or other `property_tree` serialization method.
	 * @return		Property tree containing calibration data.
	 */
	[[nodiscard]] inline pt::ptree toPropertyTree() const {
		pt::ptree tree;
		tree.put("version", "2.0");

		for (const auto &[key, cal] : cameras) {
			tree.put_child("cameras." + key, cal.toPropertyTree());
		}

		for (const auto &[key, cal] : imus) {
			tree.put_child("imus." + key, cal.toPropertyTree());
		}

		for (const auto &[key, cal] : stereo) {
			tree.put_child("stereo." + key, cal.toPropertyTree());
		}

		return tree;
	}

	/**
	 * Get a list of cameras available by their designation.
	 * @return 		Vector of available camera designations.
	 */
	[[nodiscard]] std::vector<std::string> getCameraList() const {
		std::vector<std::string> cameraNames;
		for (const auto &[name, _] : cameras) {
			cameraNames.push_back(name);
		}
		return cameraNames;
	}

	/**
	 * Get a list camera designations which have imu calibrations available in
	 * this calibration set.
	 * @return 		Vector of available imu designations.
	 */
	[[nodiscard]] std::vector<std::string> getImuList() const {
		std::vector<std::string> imuNames;
		for (const auto &[name, _] : imus) {
			imuNames.push_back(name);
		}
		return imuNames;
	}

	/**
	 * Get a list of designations of stereo calibrations available here.
	 * @return 		Vector of available stereo calibrations designations.
	 */
	[[nodiscard]] std::vector<std::string> getStereoList() const {
		std::vector<std::string> stereoNames;
		for (const auto &[name, _] : stereo) {
			stereoNames.push_back(name);
		}
		return stereoNames;
	}

	/**
	 * Retrieve a camera calibration by designation (e.g. "C0").
	 *
	 * Designation string consists of a letter determining the type of sensor and a numeric index automatically
	 * generated for each sensor. Designation string look like this:
	 * "C0" - camera with index 0
	 * "S0" - IMU sensor with index 0
	 * "C0C1" - stereo calibration where C0 is the left camera and C1 is the right camera in the camera rig setup.
	 *
	 * @param designation 		Camera designation string.
	 * @return 					Camera instrinsics calibration, `std::nullopt` if given designation is not found.
	 */
	[[nodiscard]] std::optional<calibrations::CameraCalibration> getCameraCalibration(
		const std::string &designation) const {
		const auto cameraCalibration = cameras.find(designation);
		if (cameraCalibration == cameras.end()) {
			return std::nullopt;
		}
		else {
			return cameraCalibration->second;
		}
	}

	/**
	 * Get IMU calibration by IMU sensor designation (e.g. "S0").
	 *
	 * Designation string consists of a letter determining the type of sensor and a numeric index automatically
	 * generated for each sensor. Designation string look like this:
	 * "C0" - camera with index 0
	 * "S0" - IMU sensor with index 0
	 * "C0C1" - stereo calibration where C0 is the left camera and C1 is the right camera in the camera rig setup.
	 *
	 * @param designation		IMU designation string.
	 * @return 					IMU extrinsic calibration, `std::nullopt` if given designation is not found.
	 */
	[[nodiscard]] std::optional<calibrations::IMUCalibration> getImuCalibration(const std::string &designation) const {
		const auto imuCalibration = imus.find(designation);
		if (imuCalibration == imus.end()) {
			return std::nullopt;
		}
		else {
			return imuCalibration->second;
		}
	}

	/**
	 * Get stereo calibration by stereo rig designation (e.g. "C0C1").Retrieve the full list of IMU extrinsic
	 * calibrations.
	 *
	 * Designation string consists of a letter determining the type of sensor and a numeric index automatically
	 * generated for each sensor. Designation string look like this:
	 * "C0" - camera with index 0
	 * "S0" - IMU sensor with index 0
	 * "C0C1" - stereo calibration where C0 is the left camera and C1 is the right camera in the camera rig setup.
	 *
	 * @param designation		Stereo rig designation string.
	 * @return 					Stereo extrinsic calibration, `std::nullopt` if given designation is not found.
	 */
	[[nodiscard]] std::optional<calibrations::StereoCalibration> getStereoCalibration(
		const std::string &designation) const {
		const auto stereoCalibration = stereo.find(designation);
		if (stereoCalibration == stereo.end()) {
			return std::nullopt;
		}
		else {
			return stereoCalibration->second;
		}
	}

	/**
	 * Retrieve a camera calibration by camera name, which consist of model and serial number
	 * concatenation with an underscore separator (e.g. "DVXplorer_DXA00000").
	 *
	 * Camera name is usually available in recording files and when connected directly to a camera.
	 * @param camera	Name of the camera.
	 * @return			Camera intrinsic calibration, `std::nullopt` if given camera name is not found.
	 */
	[[nodiscard]] std::optional<calibrations::CameraCalibration> getCameraCalibrationByName(
		const std::string &camera) const {
		auto cameraCalibration = std::find_if(cameras.begin(), cameras.end(),
			[&camera](const std::pair<std::string, calibrations::CameraCalibration> &calibration) -> bool {
				return calibration.second.name == camera;
			});
		if (cameraCalibration == cameras.end()) {
			return std::nullopt;
		}
		else {
			return cameraCalibration->second;
		}
	}

	/**
	 * Retrieve an IMU calibration by camera name, which consist of model and serial number
	 * concatenation with an underscore separator (e.g. "DVXplorer_DXA00000").
	 *
	 * Camera name is usually available in recording files and when connected directly to a camera.
	 * @param camera	Name of the camera.
	 * @return			IMU extrinsics calibration, `std::nullopt` if given camera name is not found.
	 */
	[[nodiscard]] std::optional<calibrations::IMUCalibration> getImuCalibrationByName(const std::string &camera) const {
		auto imuCalibration = std::find_if(imus.begin(), imus.end(),
			[&camera](const std::pair<std::string, calibrations::IMUCalibration> &calibration) -> bool {
				return calibration.second.name == camera;
			});
		if (imuCalibration == imus.end()) {
			return std::nullopt;
		}
		else {
			return imuCalibration->second;
		}
	}

	/**
	 * Retrieve a stereo calibration by matching camera name to left camera name in the stereo calibrations.
	 * Camera name consist of model and serial number concatenation with an underscore separator
	 * (e.g. "DVXplorer_DXA00000").
	 *
	 * Camera name is usually available in recording files and when connected directly to a camera.
	 * @param camera	Name of the camera.
	 * @return			Stereo extrinsic calibration, `std::nullopt` if given camera name is not found.
	 */
	[[nodiscard]] std::optional<calibrations::StereoCalibration> getStereoCalibrationByLeftCameraName(
		const std::string &camera) const {
		auto stereoCalibration = std::find_if(stereo.begin(), stereo.end(),
			[&camera](const std::pair<std::string, calibrations::StereoCalibration> &calibration) -> bool {
				return calibration.second.leftCameraName == camera;
			});

		if (stereoCalibration == stereo.end()) {
			return std::nullopt;
		}
		else {
			return stereoCalibration->second;
		}
	}

	/**
	 * Retrieve a stereo calibration by matching camera name to right camera name in the stereo calibrations.
	 * Camera name consist of model and serial number concatenation with an underscore separator
	 * (e.g. "DVXplorer_DXA00000").
	 *
	 * Camera name is usually available in recording files and when connected directly to a camera.
	 * @param camera	Name of the camera.
	 * @return			Stereo extrinsic calibration, `std::nullopt` if given camera name is not found.
	 */
	[[nodiscard]] std::optional<calibrations::StereoCalibration> getStereoCalibrationByRightCameraName(
		const std::string &camera) const {
		auto stereoCalibration = std::find_if(stereo.begin(), stereo.end(),
			[&camera](const std::pair<std::string, calibrations::StereoCalibration> &calibration) -> bool {
				return calibration.second.rightCameraName == camera;
			});

		if (stereoCalibration == stereo.end()) {
			return std::nullopt;
		}
		else {
			return stereoCalibration->second;
		}
	}

	/**
	 * Update IMU calibration for the camera name.
	 * @param calibration IMU calibration instance.
	 */
	void updateImuCalibration(const calibrations::IMUCalibration &calibration) {
		const auto camera   = calibration.name;
		auto imuCalibration = std::find_if(imus.begin(), imus.end(),
			[&camera](const std::pair<std::string, calibrations::IMUCalibration> &calibration) -> bool {
				return calibration.second.name == camera;
			});
		if (imuCalibration == imus.end()) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"IMU data not found in the IMU calibration list.", camera);
		}
		else {
			imuCalibration->second = calibration;
		}
	}

	/**
	 * Update Camera calibration for the given camera name.
	 * @param calibration Camera calibration instance.
	 */
	void updateCameraCalibration(const calibrations::CameraCalibration &calibration) {
		const auto camera      = calibration.name;
		auto cameraCalibration = std::find_if(cameras.begin(), cameras.end(),
			[&camera](const std::pair<std::string, calibrations::CameraCalibration> &calibration) -> bool {
				return calibration.second.name == camera;
			});
		if (cameraCalibration == cameras.end()) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"Camera not found in the camera calibration list.", camera);
		}
		else {
			cameraCalibration->second = calibration;
		}
	}

	/**
	 * Update Stereo Camera calibration for the given camera name.
	 * @param calibration Stereo calibration instance.
	 */
	void updateStereoCameraCalibration(const calibrations::StereoCalibration &calibration) {
		const auto leftCamera  = calibration.leftCameraName;
		const auto rightCamera = calibration.rightCameraName;

		auto cameraCalibration = std::find_if(stereo.begin(), stereo.end(),
			[&leftCamera, &rightCamera](
				const std::pair<std::string, calibrations::StereoCalibration> &calibration) -> bool {
				return calibration.second.leftCameraName == leftCamera
					&& calibration.second.rightCameraName == rightCamera;
			});
		if (cameraCalibration == stereo.end()) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"Stereo pair not found in the stereo camera calibration.", leftCamera + "-" + rightCamera);
		}
		else {
			cameraCalibration->second = calibration;
		}
	}

	/**
	 * Add an intrinsic calibration to the camera calibration set. Camera designation is going to be
	 * generated automatically.
	 * @param calibration 		Camera intrinsics calibration.
	 */
	void addCameraCalibration(const calibrations::CameraCalibration &calibration) {
		auto camera = calibration.name;
		for (const auto &pair : cameras) {
			if (pair.second.name == camera) {
				throw dv::exceptions::InvalidArgument<std::string>(
					"Camera calibration list already contains the camera name", camera);
			}
		}
		std::string designation = "C" + std::to_string(cameraIndex++);
		// This is only a debugging test, technically this shouldn't happen
		dv::runtime_assert(
			!cameras.contains(designation), "Camera calibration list already contains camera with same designation!");
		cameras.emplace(std::make_pair(designation, calibration));
	}

	/**
	 * Add an IMU extrinsics calibration to the calibration set.
	 * @param calibration 		IMU extrinsic calibration.
	 */
	void addImuCalibration(const calibrations::IMUCalibration &calibration) {
		auto camera = calibration.name;
		for (const auto &pair : imus) {
			if (pair.second.name == camera) {
				throw dv::exceptions::InvalidArgument<std::string>(
					"IMU calibration list already contains the camera name", camera);
			}
		}
		std::string designation = "S" + std::to_string(imuIndex++);
		// This is only a debugging test, technically this shouldn't happen
		dv::runtime_assert(
			!imus.contains(designation), "IMU calibration list already contains camera with same designation!");
		imus.emplace(std::make_pair(designation, calibration));
	}

	/**
	 * Add a stereo calibration to the calibration set. Intrinsic calibrations of the sensors should
	 * already be added using `addCameraCalibration` prior to adding the stereo extrinsic calibration.
	 * @param calibration 	Stereo calibration.
	 * @throw 	Throws an invalid argument exception if the intrinsic calibration of given
	 * 			camera sensors are not available in the set or stereo calibration for the
	 * 			given cameras already exist/
	 */
	void addStereoCalibration(const calibrations::StereoCalibration &calibration) {
		const auto rightCamera = calibration.rightCameraName;
		const auto leftCamera  = calibration.leftCameraName;
		for (const auto &pair : stereo) {
			if (pair.second.rightCameraName == rightCamera && pair.second.leftCameraName == leftCamera) {
				throw dv::exceptions::InvalidArgument<std::string>(
					"Stereo camera calibration list already contains the camera pair", leftCamera + "-" + rightCamera);
			}
		}
		const auto leftcam = std::find_if(cameras.begin(), cameras.end(), [&calibration](const auto &cam) {
			return cam.second.name == calibration.leftCameraName;
		});
		if (leftcam == cameras.end()) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"Left camera intrinsics were not found in the calibrations while adding a stereo calibration."
				"Make sure to add camera intrinsic calibration before adding stereo calibration.",
				calibration.leftCameraName);
		}

		const auto rightcam = std::find_if(cameras.begin(), cameras.end(), [&calibration](const auto &cam) {
			return cam.second.name == calibration.rightCameraName;
		});
		if (rightcam == cameras.end()) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"Right camera intrinsics were not found in the calibrations while adding a stereo calibration."
				"Make sure to add camera intrinsic calibration before adding stereo calibration.",
				calibration.rightCameraName);
		}

		std::string designation = leftcam->first + "_" + rightcam->first;
		if (stereo.contains(designation)) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"Stereo calibration for the given cameras already exists.", designation);
		}

		stereo.emplace(std::make_pair(designation, calibration));
	}

	/**
	 * Retrieve the full list of camera intrinsic calibrations.
	 * @return 		`std::map` containing camera calibrations where keys are camera designation strings.
	 */
	[[nodiscard]] const std::map<std::string, calibrations::CameraCalibration> &getCameraCalibrations() const {
		return cameras;
	}

	/**
	 * Retrieve the full list of IMU extrinsic calibrations.
	 * @return 		`std::map` containing IMU calibrations where keys are IMU sensor designation strings.
	 */
	[[nodiscard]] const std::map<std::string, calibrations::IMUCalibration> &getImuCalibrations() const {
		return imus;
	}

	/**
	 * Retrieve the full list of stereo extrinsic calibrations.
	 * @return 		`std::map` containing stereo calibrations where keys are stereo rig camera designation strings.
	 */
	[[nodiscard]] const std::map<std::string, calibrations::StereoCalibration> &getStereoCalibrations() const {
		return stereo;
	}

	/**
	 * Write the contents of this calibration set into a file at given path.
	 *
	 * This function requires that supplied path contains ".json" extension.
	 * @param outputFile 	Output file path with ".json" extension to write the contents of the calibration set.
	 */
	void writeToFile(const fs::path &outputFile) const {
		if (outputFile.extension().string() != ".json") {
			throw dv::exceptions::InvalidArgument<std::string>(
				"Output file path must have \".json\" extension, otherwise it might not be interpreted correctly while "
				"reading!",
				outputFile.extension().string());
		}

		auto tree = toPropertyTree();
		pt::write_json(outputFile.string(), tree);
	}

private:
	size_t cameraIndex = 0;
	size_t imuIndex    = 0;

	using CameraCalibrationMap = std::map<std::string, calibrations::CameraCalibration>;
	using IMUCalibrationMap    = std::map<std::string, calibrations::IMUCalibration>;
	using StereoCalibrationMap = std::map<std::string, calibrations::StereoCalibration>;

	CameraCalibrationMap cameras;
	IMUCalibrationMap imus;
	StereoCalibrationMap stereo;

	explicit CalibrationSet(const pt::ptree &tree) {
		auto version = tree.get_child("version").get_value_optional<std::string>();
		if (!version.has_value()) {
			throw dv::exceptions::Exception("Could not decode the calibration version from the property tree");
		}
		if (*version != "2.0") {
			throw dv::exceptions::InvalidArgument<std::string>("Expected calibration version \"2.0\"", *version);
		}

		auto camerasData = tree.get_child_optional("cameras");
		if (camerasData.has_value()) {
			for (const auto &camera : tree.get_child("cameras")) {
				auto camData = calibrations::CameraCalibration(camera.second);
				addCameraCalibration(camData);
			}
		}

		auto imuData = tree.get_child_optional("imus");
		if (imuData.has_value()) {
			for (const auto &imu : tree.get_child("imus")) {
				auto imuParsed = calibrations::IMUCalibration(imu.second);
				addImuCalibration(imuParsed);
			}
		}

		auto stereoMeta = tree.get_child_optional("stereo");
		if (stereoMeta.has_value()) {
			for (const auto &stereoData : tree.get_child("stereo")) {
				auto sm = calibrations::StereoCalibration(stereoData.second);
				addStereoCalibration(sm);
			}
		}
	}

	static CalibrationSet cameraRigCalibrationFromJsonFile(const fs::path &path) {
		pt::ptree tree;
		pt::read_json(path.string(), tree);

		const auto version = tree.get_child("version").get_value<float>(-1.f);
		if (version == 2.0f) {
			return CalibrationSet(tree);
		}
		else {
			throw dv::exceptions::InvalidArgument<float>("Invalid calibration version", version);
		}
	}

	static calibrations::CameraCalibration oneCameraCalibrationFromXML(
		const cv::FileNode &node, const std::string_view cameraName, const bool cameraIsMaster) {
		cv::Mat camMat;
		cv::Mat distMat;

		std::vector<std::string> keys = node.keys();
		if (!dv::vectorContains(keys, "camera_matrix") || !dv::vectorContains(keys, "distortion_coefficients")
			|| !dv::vectorContains(keys, "image_width") || !dv::vectorContains(keys, "image_height")) {
			throw dv::exceptions::RuntimeError("Invalid calibration file, file format unsupported. Camera calibration "
											   "fields were not found while parsing V1 XML calibration format.");
		}

		node["camera_matrix"] >> camMat;
		node["distortion_coefficients"] >> distMat;

		distMat.convertTo(distMat, CV_32FC1);
		std::vector<float> distVector(
			reinterpret_cast<float *>(distMat.data), reinterpret_cast<float *>(distMat.data) + (distMat.total()));
		if (distVector.size() > 5) {
			distVector.resize(5);
		}

		int width, height;
		node["image_width"] >> width;
		node["image_height"] >> height;

		auto fx = static_cast<float>(camMat.at<double>(0, 0));
		auto fy = static_cast<float>(camMat.at<double>(1, 1));
		auto cx = static_cast<float>(camMat.at<double>(0, 2));
		auto cy = static_cast<float>(camMat.at<double>(1, 2));

		bool useFisheye = false;
		if (dv::vectorContains(keys, "use_fisheye_model")) {
			node["use_fisheye_model"] >> useFisheye;
		}

		return {
			cameraName, "unknown", cameraIsMaster, {width, height},
               {cx,    cy    },
               {fx,    fy    },
               distVector,
			(useFisheye ? DistortionModel::Equidistant : DistortionModel::RadTan), identity,
			calibrations::CameraCalibration::Metadata()
        };
	}

	static CalibrationSet cameraRigCalibrationFromXmlFile(const fs::path &path) {
		CalibrationSet calib;

		cv::FileStorage fs;
		fs.open(path.string(), cv::FileStorage::READ);

		if (!fs.isOpened()) {
			throw dv::exceptions::FileNotFound{"XML calibration file could not be opened for reading."} << path;
		}

		// Check for presence of nodes in OpenCV XML tree.
		auto childExists = [](const cv::FileNode &root, const std::string_view name) -> bool {
			for (const auto &node : root) {
				if (node.name() == name) {
					return true;
				}
			}
			return false;
		};

		// The old XML files existed in two versions:
		//
		// **v1**: one camera supported, no camera name, dist/coeffs were directly at the root, no "type" attribute
		// v1 has the following root level fields: "calibration_time", "image_width", "image_height",
		// "board_width", "board_height", "square_size", "use_fisheye_model", "avg_reprojection_error".
		//
		// **v2**: stereo support, camera names recorded as node names, "type" == "camera" or "stereo"
		// v2 has the following common fields: "type", "pattern_width", "pattern_height", "pattern_type",
		// "board_width", "board_height", "square_size", "calibration_error", "calibration_time".
		// Stereo calibrations have the following differences: no "use_fisheye_model", additional R,T,E,F matrices,
		// additional "epipolar_error"; all at the root level.
		//
		// So let's first disambiguate between the two types.
		bool xmlOldStyle{false};
		bool stereoCalibration{false};

		const auto root = fs.root();

		if (!childExists(root, "type") || !root["type"].isString()) {
			xmlOldStyle = true;
		}
		else {
			stereoCalibration = (root["type"].string() == "stereo");
		}

		if (xmlOldStyle) {
			// Old XML style: only one camera at root level, no name, must be master.
			calib.addCameraCalibration(oneCameraCalibrationFromXML(root, "", true));
		}
		else {
			bool firstIsMaster{true};

			// Read the camera data for each of the cameras.
			for (const auto &node : root) {
				if (childExists(node, "camera_matrix")) {
					const auto cameraName = node.name();

					// This is a camera node, name of camera is in the node name itself.
					calib.addCameraCalibration(oneCameraCalibrationFromXML(node, cameraName, firstIsMaster));

					// First found camera is always the master, all after are not.
					firstIsMaster = false;
				}
			}
		}

		// Check distortion model used, if fisheye change all the cameras' models
		// from "radialTangential" to "equidistant".
		if (childExists(root, "use_fisheye_model")) {
			bool fisheye{false};
			root["use_fisheye_model"] >> fisheye;
			if (fisheye) {
				for (auto &[name, cam] : calib.cameras) {
					cam.distortionModel = DistortionModel::Equidistant;
				}
			}
		}

		// Add metadata to all the cameras.
		for (const auto &[name, cam] : calib.cameras) {
			calibrations::CameraCalibration::Metadata meta;

			if (childExists(root, "pattern_width")) {
				root["pattern_width"] >> meta.internalPatternShape.width;
			}

			if (childExists(root, "pattern_height")) {
				root["pattern_height"] >> meta.internalPatternShape.height;
			}

			if (childExists(root, "pattern_type")) {
				root["pattern_type"] >> meta.patternType;
			}

			if (childExists(root, "board_width")) {
				root["board_width"] >> meta.patternShape.width;
			}

			if (childExists(root, "board_height")) {
				root["board_height"] >> meta.patternShape.height;
			}

			if (childExists(root, "square_size")) {
				root["square_size"] >> meta.patternSize;
			}

			// Can be N/A in some cases, so check for it being a number.
			if (childExists(root, "calibration_error") && root["calibration_error"].isReal()) {
				float value;
				root["calibration_error"] >> value;
				meta.calibrationError = value;
			}

			// Old style calibration error.
			if (childExists(root, "avg_reprojection_error") && root["avg_reprojection_error"].isReal()) {
				float value;
				root["avg_reprojection_error"] >> value;
				meta.calibrationError = value;
			}

			if (childExists(root, "calibration_time")) {
				root["calibration_time"] >> meta.calibrationTime;
			}

			calib.cameras.at(name).metadata = meta;
		}

		// In case of two cameras (stereo) we process additional fields.
		if (stereoCalibration && (calib.cameras.size() == 2)) {
			// Stereo calibration
			std::vector<float> T, R;
			cv::Mat TMat, RMat;
			root["T"] >> TMat;
			root["R"] >> RMat;
			TMat.convertTo(TMat, CV_32FC1);
			RMat.convertTo(RMat, CV_32FC1);
			T.assign(reinterpret_cast<float *>(TMat.data), reinterpret_cast<float *>(TMat.data) + TMat.total());
			R.assign(reinterpret_cast<float *>(RMat.data), reinterpret_cast<float *>(RMat.data) + RMat.total());

			// Let's reuse R to contain the full transformation. This is achieved by concatenating
			// T translation vector to the rotation matrix at the very right, the 3x3 matrix indices
			// are row-major and looks like this:
			// | 0 1 2 |
			// | 3 4 5 |
			// | 6 7 8 |
			// The translational vector values needs to be inserted at indices 3, 6, 9. Since insert a value will shift
			// indices by one, we need to insert these at indices (3, 6+1, 9+2), then the matrix will look like:
			// | 0  1   2   3 |
			// | 4  5   6   7 |
			// | 8  9  10  11 |
			// Where values at indices 3, 7, 11 is going to be the translational vector
			R.insert(R.begin() + 3, T[0]);
			R.insert(R.begin() + 7, T[1]);
			R.insert(R.begin() + 11, T[2]);
			// The transformation matrix needs to be homogenous, that means we need 4x4 representation,
			// so let's add | 0 0 0 1 | values at the bottom row of the matrix.
			R.emplace_back(0.f);
			R.emplace_back(0.f);
			R.emplace_back(0.f);
			R.emplace_back(1.f);

			// C0 is the master camera.
			for (auto &[name, cam] : calib.cameras) {
				if (cam.master) {
					continue;
				}

				cam.transformationToC0 = R;
			}

			// Stereo calibration.
			calibrations::StereoCalibration::Metadata stereoMeta;

			if (childExists(root, "epipolar_error") && root["epipolar_error"].isReal()) {
				float value;
				root["epipolar_error"] >> value;
				stereoMeta.epipolarError = value;
			}

			std::vector<float> E, F;

			if (childExists(root, "E")) {
				cv::Mat values;
				root["E"] >> values;
				values.convertTo(values, CV_32FC1);
				E.assign(
					reinterpret_cast<float *>(values.data), reinterpret_cast<float *>(values.data) + values.total());
			}

			if (childExists(root, "F")) {
				cv::Mat values;
				root["F"] >> values;
				values.convertTo(values, CV_32FC1);
				F.assign(
					reinterpret_cast<float *>(values.data), reinterpret_cast<float *>(values.data) + values.total());
			}

			calib.addStereoCalibration(calibrations::StereoCalibration(
				calib.cameras.at("C0").name, calib.cameras.at("C1").name, F, E, stereoMeta));
		}

		// In case of one camera with IMU we process additional fields.
		if (childExists(root, "time_offset_cam_imu") && childExists(root, "transformation_cam_imu")
			&& (calib.cameras.size() == 1)) {
			std::stringstream ssComment;
			ssComment << "Time offset usage: t_correct = t_imu - offset";

			if (childExists(root, "mean_reprojection_error")) {
				double val;
				root["mean_reprojection_error"] >> val;
				ssComment << " Mean reprojection error: " << val;
			}

			if (childExists(root, "mean_accelerometer_error")) {
				double val;
				root["mean_accelerometer_error"] >> val;
				ssComment << " Mean accelerometer error: " << val;
			}

			if (childExists(root, "mean_gyroscope_error")) {
				double val;
				root["mean_gyroscope_error"] >> val;
				ssComment << " Mean gyroscope error: " << val;
			}

			calibrations::IMUCalibration::Metadata meta;
			if (childExists(root, "calibration_time")) {
				root["calibration_time"] >> meta.calibrationTime;
			}

			meta.comment = ssComment.str();

			calibrations::IMUCalibration imuCal(calib.cameras.at("C0").name, -1.f, -1.f, cv::Point3f(0, 0, 0),
				cv::Point3f(0, 0, 0), -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, identity, meta);

			if (childExists(root, "time_offset_cam_imu")) {
				double seconds;
				root["time_offset_cam_imu"] >> seconds;
				imuCal.timeOffsetMicros = static_cast<int64_t>(seconds * 1e+6);
			}

			if (childExists(root, "transformation_cam_imu")) {
				cv::Mat T_C0;
				root["transformation_cam_imu"] >> T_C0;
				T_C0.convertTo(T_C0, CV_32FC1);
				imuCal.transformationToC0.assign(
					reinterpret_cast<float *>(T_C0.data), reinterpret_cast<float *>(T_C0.data) + T_C0.total());
			}

			calib.addImuCalibration(imuCal);
		}

		return calib;
	}
};

} // namespace dv::camera
