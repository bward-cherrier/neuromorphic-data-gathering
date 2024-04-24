#include "../../include/dv-processing/camera/calibration_set.hpp"

#include "boost/ut.hpp"

#include <fstream>
#include <numeric>

namespace fs = std::filesystem;

dv::camera::calibrations::CameraCalibration::Metadata getTestCameraMetadata() {
	return dv::camera::calibrations::CameraCalibration::Metadata{
		{6, 6},
        {6, 6},
        "maytag", 0.05f, 0.01f, std::nullopt, "today", "not great but not terrible", "no comment",
		9e-6
    };
}

dv::camera::calibrations::CameraCalibration getTestCameraCalibration(const std::string &name) {
	cv::Matx44f identity = cv::Matx44f::eye();
	return dv::camera::calibrations::CameraCalibration(name, "left", true, {640, 480}, {320, 240}, {10, 12},
		{0.0f, 0.01f, 0.02f, 0.03f}, dv::camera::DistortionModel::RadTan,
		std::vector<float>(identity.val, identity.val + 16), getTestCameraMetadata());
}

dv::camera::calibrations::IMUCalibration::Metadata getTestIMUMetadata() {
	return dv::camera::calibrations::IMUCalibration::Metadata{"just now", "nothing to say"};
}

dv::camera::calibrations::IMUCalibration getTestIMUCalibration(const std::string &name) {
	cv::Matx44f identity = cv::Matx44f::eye();
	return dv::camera::calibrations::IMUCalibration{name, 0.1f, 0.2f, cv::Point3f(1, 2, 3), cv::Point3f(4, 5, 6), 0.3f,
		0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 900'000, std::vector<float>(identity.val, identity.val + 16),
		getTestIMUMetadata()};
}

dv::camera::calibrations::StereoCalibration getTestStereoMetadata(
	const std::string &leftName, const std::string &rightName) {
	return dv::camera::calibrations::StereoCalibration{
		leftName, rightName, std::vector<float>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f},
		std::vector<float>{9.f, 8.f, 7.f, 6.f, 5.f, 4.f, 3.f, 2.f, 1.f},
		dv::camera::calibrations::StereoCalibration::Metadata(-1.f, "nothing to say")
    };
}

int main() {
	using namespace boost::ut;
	using namespace dv::camera;

	"non_existant_path"_test = [] {
		expect(throws([] {
			const auto _ = CalibrationSet::LoadFromFile("some(garbage-name.xml");
		}));

		expect(throws([] {
			const auto _ = CalibrationSet::LoadFromFile("some(garbage-name.json");
		}));

		expect(throws([] {
			const auto _ = CalibrationSet::LoadFromFile("some(garbage-name");
		}));
	};

	"camera_calibration_serialize"_test = [] {
		std::stringstream ss;

		// Write
		{
			auto cam  = getTestCameraCalibration("camera");
			auto tree = cam.toPropertyTree();
			pt::write_json(ss, tree);
		}

		// Read
		{
			pt::ptree tree;
			pt::read_json(ss, tree);
			auto cam = calibrations::CameraCalibration(tree);

			expect(eq(getTestCameraCalibration("camera"), cam));
		}
	};

	"camera_calibration_meta_serialize"_test = [] {
		std::stringstream ss;

		// Write
		{
			auto meta = getTestCameraMetadata();
			auto tree = meta.toPropertyTree();
			pt::write_json(ss, tree);
		}

		// Read
		{
			pt::ptree tree;
			pt::read_json(ss, tree);
			auto meta = calibrations::CameraCalibration::Metadata(tree);

			expect(meta.comment != "blah");
		}
	};

	"imu_calibration_serialize"_test = [] {
		std::stringstream ss;

		// Write
		{
			auto imu  = getTestIMUCalibration("test");
			auto tree = imu.toPropertyTree();
			pt::write_json(ss, tree);
		}

		// Read
		{
			pt::ptree tree;
			pt::read_json(ss, tree);
			auto imu = calibrations::IMUCalibration(tree);

			expect(eq(getTestIMUCalibration("test"), imu));
		}
	};

	"imu_calibration_meta_serialize"_test = [] {
		std::stringstream ss;

		// Write
		{
			auto meta = getTestIMUMetadata();
			auto tree = meta.toPropertyTree();
			pt::write_json(ss, tree);
		}

		// Read
		{
			pt::ptree tree;
			pt::read_json(ss, tree);
			auto meta = calibrations::IMUCalibration::Metadata(tree);

			expect(meta.comment != "blah");
		}
	};

	"stereo_meta_serialize"_test = [] {
		std::stringstream ss;

		// Write
		{
			auto meta = getTestStereoMetadata("a", "b");
			auto tree = meta.toPropertyTree();
			pt::write_json(ss, tree);
		}

		// Read
		{
			pt::ptree tree;
			pt::read_json(ss, tree);
			auto meta = calibrations::StereoCalibration(tree);
			expect(meta.metadata.has_value());
			expect(meta.metadata->comment != "blah");
		}
	};

	"rig_to_property_tree"_test = [] {
		std::stringstream ss;

		// Write
		CalibrationSet calibration;
		calibration.addCameraCalibration(getTestCameraCalibration("dvxplorer_1"));
		calibration.addCameraCalibration(getTestCameraCalibration("dvxplorer_2"));

		calibration.addImuCalibration(getTestIMUCalibration("dvxplorer_1"));
		calibration.addImuCalibration(getTestIMUCalibration("dvxplorer_2"));

		calibration.addStereoCalibration(getTestStereoMetadata("dvxplorer_1", "dvxplorer_2"));

		auto tree = calibration.toPropertyTree();

		fs::path filepath = "./test_v2_calibration_file.json";
		std::ofstream outfile(filepath);
		pt::write_json(outfile, calibration.toPropertyTree());

		// Read & check
		CalibrationSet calibrationRead = CalibrationSet::LoadFromFile(filepath);

		expect(eq(tree.get<std::string>("version"), std::string("2.0")));
		expect(calibrationRead.getCameraCalibrationByName("dvxplorer_1").has_value());
		expect(calibrationRead.getCameraCalibrationByName("dvxplorer_2").has_value());
		expect(calibrationRead.getImuCalibrationByName("dvxplorer_1").has_value());
		expect(calibrationRead.getImuCalibrationByName("dvxplorer_2").has_value());
		expect(calibrationRead.getStereoCalibrationByLeftCameraName("dvxplorer_1").has_value());
		expect(calibrationRead.getStereoCalibrationByRightCameraName("dvxplorer_2").has_value());
	};

	"xml_mono_old_file_to_rig"_test = [] {
		fs::path filename = "./calibration_files/calibration_camera_oldest.xml";
		auto calibration  = CalibrationSet::LoadFromFile(filename);

		expect(eq(calibration.getCameraList().size(), 1));
		expect(eq(calibration.getImuList().size(), 0));
		expect(eq(calibration.getStereoList().size(), 0));
		expect(calibration.getCameraCalibration("C0").has_value());

		auto cameras = calibration.getCameraList();
		expect(eq(cameras.size(), 1));
		expect(eq(cameras[0], std::string("C0")));

		auto cam = *calibration.getCameraCalibration("C0");
		expect(eq(cam.name, std::string("")));
		expect(eq(cam.position, std::string("unknown")));
		expect(eq(cam.distortionModel, dv::camera::DistortionModel::RadTan));
		expect(eq(cam.getDistortionModelString(), std::string("radialTangential")));
		expect(eq(cam.resolution.width, 640));
		expect(eq(cam.resolution.height, 480));
		expect(eq(cam.principalPoint.x, 278.287537_d));
		expect(eq(cam.principalPoint.y, 243.193054_d));
		expect(eq(cam.focalLength.x, 698.462463_d));
		expect(eq(cam.focalLength.y, 697.592712_d));
		expect(eq(cam.distortion.size(), 5));
		expect(eq(cam.distortion.at(0), -3.3964557788001604e-01f));
		expect(eq(cam.distortion.at(1), 4.2529450906452312e-04f));
		expect(eq(cam.distortion.at(2), 2.7961387689219583e-03f));
		expect(eq(cam.distortion.at(3), 1.4214263083290506e-03f));
		expect(eq(cam.distortion.at(4), 2.8338594947193457e-01f));
		auto transSum = std::accumulate(cam.transformationToC0.begin(), cam.transformationToC0.end(), 0.f);
		expect(eq(transSum, 4));

		dv::camera::CameraGeometry geometry = cam.getCameraGeometry();
		expect(eq(geometry.getCentralPoint().x, 278.287537_d));
		expect(eq(geometry.getCentralPoint().y, 243.193054_d));
		expect(eq(geometry.getFocalLength().x, 698.462463_d));
		expect(eq(geometry.getFocalLength().y, 697.592712_d));
		expect(geometry.isUndistortionAvailable());

		auto meta = cam.metadata;
		expect(meta.has_value());
		expect(eq(meta->patternShape.width, 10));
		expect(eq(meta->patternShape.height, 6));
		expect(eq(meta->internalPatternShape.width, 0));
		expect(eq(meta->internalPatternShape.height, 0));
		expect(eq(meta->patternType, std::string("")));
		expect(eq(meta->patternSize, 0.075_d));
		expect(eq(*meta->calibrationError, 9.8864525930686753e-01f));
		expect(eq(meta->calibrationTime, std::string("Mon Nov 1 14:32:23 2021")));

		// Check the number of lines in the output. In case this fails, please make sure that the test above does not
		// need to be updated.
		std::stringstream ss;
		auto tree = calibration.toPropertyTree();
		pt::write_json(ss, tree);
		auto text  = ss.str();
		auto lines = std::count(text.begin(), text.end(), '\n');
		expect(eq(lines, 65));
	};

	"xml_mono_file_to_rig"_test = [] {
		fs::path filename = "./calibration_files/calibration_camera_DVXplorer_DXA00080-2021_11_01_14_32_23.xml";
		auto calibration  = CalibrationSet::LoadFromFile(filename);

		expect(eq(calibration.getCameraList().size(), 1));
		expect(eq(calibration.getImuList().size(), 0));
		expect(eq(calibration.getStereoList().size(), 0));
		expect(calibration.getCameraCalibration("C0").has_value());

		auto cameras = calibration.getCameraList();
		expect(eq(cameras.size(), 1));
		expect(eq(cameras[0], std::string("C0")));

		auto cam = *calibration.getCameraCalibration("C0");
		expect(eq(cam.name, std::string("DVXplorer_DXA00080")));
		expect(eq(cam.position, std::string("unknown")));
		expect(eq(cam.distortionModel, dv::camera::DistortionModel::RadTan));
		expect(eq(cam.getDistortionModelString(), std::string("radialTangential")));
		expect(eq(cam.resolution.width, 640));
		expect(eq(cam.resolution.height, 480));
		expect(eq(cam.principalPoint.x, 278.287537_d));
		expect(eq(cam.principalPoint.y, 243.193054_d));
		expect(eq(cam.focalLength.x, 698.462463_d));
		expect(eq(cam.focalLength.y, 697.592712_d));
		expect(eq(cam.distortion.size(), 5));
		expect(eq(cam.distortion.at(0), -3.3964557788001604e-01f));
		expect(eq(cam.distortion.at(1), 4.2529450906452312e-04f));
		expect(eq(cam.distortion.at(2), 2.7961387689219583e-03f));
		expect(eq(cam.distortion.at(3), 1.4214263083290506e-03f));
		expect(eq(cam.distortion.at(4), 2.8338594947193457e-01f));
		auto transSum = std::accumulate(cam.transformationToC0.begin(), cam.transformationToC0.end(), 0.f);
		expect(eq(transSum, 4));

		dv::camera::CameraGeometry geometry = cam.getCameraGeometry();
		expect(eq(geometry.getCentralPoint().x, 278.287537_d));
		expect(eq(geometry.getCentralPoint().y, 243.193054_d));
		expect(eq(geometry.getFocalLength().x, 698.462463_d));
		expect(eq(geometry.getFocalLength().y, 697.592712_d));
		expect(geometry.isUndistortionAvailable());

		auto meta = cam.metadata;
		expect(meta.has_value());
		expect(eq(meta->patternShape.width, 10));
		expect(eq(meta->patternShape.height, 6));
		expect(eq(meta->internalPatternShape.width, 9));
		expect(eq(meta->internalPatternShape.height, 5));
		expect(eq(meta->patternType, std::string("chessboard")));
		expect(eq(meta->patternSize, 0.075_d));
		expect(eq(*meta->calibrationError, 9.8864525930686753e-01f));
		expect(eq(meta->calibrationTime, std::string("Mon Nov 1 14:32:23 2021")));

		// Check the number of lines in the output. In case this fails, please make sure that the test above does not
		// need to be updated.
		std::stringstream ss;
		auto tree = calibration.toPropertyTree();
		pt::write_json(ss, tree);
		auto text  = ss.str();
		auto lines = std::count(text.begin(), text.end(), '\n');
		expect(eq(lines, 65));
	};

	"xml_mono_imu_file_to_rig"_test = [] {
		fs::path filename = "./calibration_files/camera_imu_calibration_DVXplorer_DXA00080-2021_09_08_10_54_37.xml";
		auto calibration  = CalibrationSet::LoadFromFile(filename);

		expect(eq(calibration.getCameraList().size(), 1));
		expect(eq(calibration.getImuList().size(), 1));
		expect(eq(calibration.getStereoList().size(), 0));
		expect(calibration.getCameraCalibration("C0").has_value());
		expect(calibration.getImuCalibration("S0").has_value());

		auto cameras = calibration.getCameraList();
		expect(eq(cameras.size(), 1));
		expect(eq(cameras[0], std::string("C0")));
		auto imus = calibration.getImuList();
		expect(eq(imus.size(), 1));
		expect(eq(imus[0], std::string("S0")));
		auto stereo = calibration.getStereoList();
		expect(stereo.empty());

		{
			auto cam = *calibration.getCameraCalibration("C0");
			expect(eq(cam.name, std::string("DVXplorer_DXA00080")));
			expect(eq(cam.position, std::string("unknown")));
			expect(eq(cam.resolution.width, 640));
			expect(eq(cam.resolution.height, 480));
			expect(eq(cam.principalPoint.x, 280.937_d));
			expect(eq(cam.principalPoint.y, 251.474_d));
			expect(eq(cam.focalLength.x, 536.514_d));
			expect(eq(cam.focalLength.y, 537.617_d));
			expect(eq(cam.distortion.size(), 4));
			auto transSum = std::accumulate(cam.transformationToC0.begin(), cam.transformationToC0.end(), 0.f);
			expect(eq(transSum, 4));
			auto meta = cam.metadata;
			expect(meta.has_value());
			expect(eq(meta->patternShape.width, 6));
			expect(eq(meta->patternShape.height, 6));
			expect(eq(meta->internalPatternShape.width, 6));
			expect(eq(meta->internalPatternShape.height, 6));
			expect(eq(meta->patternType, std::string("aprilTag")));
			expect(eq(meta->patternSize, 0.03_d));
			expect(!meta->calibrationError.has_value());
			expect(eq(meta->calibrationTime, std::string("Mi 08 Sep 2021 10:54:37")));
		}

		{
			auto imu = *calibration.getImuCalibration("S0");
			expect(eq(imu.name, std::string("DVXplorer_DXA00080")));
			expect(eq(imu.timeOffsetMicros, 2897));
			auto transSum = std::accumulate(imu.transformationToC0.begin(), imu.transformationToC0.end(), 0.f);
			expect(lt(std::abs(transSum - 3.809131f), 0.000001f));
			auto meta = imu.metadata;
			expect(meta.has_value());
			expect(eq(meta->calibrationTime, std::string("Mi 08 Sep 2021 10:54:37")));
			expect(eq(meta->comment,
				std::string("Time offset usage: t_correct = t_imu - offset Mean reprojection error: "
							"3.23208 Mean accelerometer error: 0.478185 Mean gyroscope error: 0.0742152")));

			const auto imuByName = calibration.getImuCalibrationByName("DVXplorer_DXA00080");
			expect(imuByName.has_value());
			expect(eq(imuByName->name, std::string("DVXplorer_DXA00080")));
		}

		// Check the number of lines in the output. In case this fails, please make sure that the test above does not
		// need to be updated.
		std::stringstream ss;
		auto tree = calibration.toPropertyTree();
		pt::write_json(ss, tree);
		auto text  = ss.str();
		auto lines = std::count(text.begin(), text.end(), '\n');
		expect(eq(lines, 109));
	};

	"xml_stereo_file_to_rig"_test = [] {
		fs::path filename
			= "./calibration_files/calibration_stereo_DVXplorer_DXA00013_DAVIS346_00000237-2020_05_22_16_04_47.xml";
		auto calibration = CalibrationSet::LoadFromFile(filename);

		expect(eq(calibration.getCameraList().size(), 2));
		expect(eq(calibration.getImuList().size(), 0));
		expect(eq(calibration.getStereoList().size(), 1));
		expect(calibration.getCameraCalibration("C0").has_value());
		expect(calibration.getCameraCalibration("C1").has_value());
		expect(calibration.getStereoCalibration("C0_C1").has_value());

		{
			auto cam = calibration.getCameraCalibration("C0").value();
			expect(eq(cam.name, std::string("DVXplorer_DXA00013")));
			expect(eq(cam.position, std::string("unknown")));
			expect(eq(cam.resolution.width, 640));
			expect(eq(cam.resolution.height, 480));
			expect(eq(cam.principalPoint.x, 303.551_d));
			expect(eq(cam.principalPoint.y, 202.983_d));
			expect(eq(cam.focalLength.x, 621.482_d));
			expect(eq(cam.focalLength.y, 619.68_d));
			expect(eq(cam.distortion.size(), 5));
			expect(eq(cam.distortion.at(0), -3.3340488011285940e-01f));
			expect(eq(cam.distortion.at(1), 1.0723352202460688e-01f));
			expect(eq(cam.distortion.at(2), 7.8182065238027493e-04f));
			expect(eq(cam.distortion.at(3), 4.9430253970448937e-03f));
			expect(eq(cam.distortion.at(4), 2.7710169366544732e-02f));
			auto transSum = std::accumulate(cam.transformationToC0.begin(), cam.transformationToC0.end(), 0.f);
			expect(eq(transSum, 4));
			auto meta = cam.metadata;
			expect(meta.has_value());
			expect(eq(meta->patternShape.width, 10));
			expect(eq(meta->patternShape.height, 6));
			expect(eq(meta->internalPatternShape.width, 9));
			expect(eq(meta->internalPatternShape.height, 5));
			expect(eq(meta->patternType, std::string("asymmetricCirclesGrid")));
			expect(eq(meta->patternSize, 75.0f));
			expect(eq(*meta->calibrationError, 1.2488455246819389e+02f));
			expect(eq(meta->calibrationTime, std::string("05/22/20 16:04:47")));

			auto camByName = calibration.getCameraCalibrationByName("DVXplorer_DXA00013");
			expect(camByName.has_value());
			expect(eq(camByName->name, std::string("DVXplorer_DXA00013")));
		}

		{
			auto cam = *calibration.getCameraCalibration("C1");
			expect(eq(cam.name, std::string("DAVIS346_00000237")));
			expect(eq(cam.position, std::string("unknown")));
			expect(eq(cam.resolution.width, 346));
			expect(eq(cam.resolution.height, 260));
			expect(eq(cam.principalPoint.x, 161.553_d));
			expect(eq(cam.principalPoint.y, 140.308_d));
			expect(eq(cam.focalLength.x, 281.316_d));
			expect(eq(cam.focalLength.y, 281.162_d));
			expect(eq(cam.distortion.size(), 5));
			auto transSum = std::accumulate(cam.transformationToC0.begin(), cam.transformationToC0.end(), 0.f);
			expect(eq(cam.transformationToC0[0], 0.99_f));
			expect(eq(cam.transformationToC0[5], 0.99_f));
			expect(eq(cam.transformationToC0[10], 0.99_f));
			expect(eq(cam.transformationToC0[15], 1.0_f));
			expect(eq(cam.transformationToC0[3], -117.871_f));
			expect(eq(cam.transformationToC0[7], -6.22721_f));
			expect(eq(cam.transformationToC0[11], -38.9891_f));
			expect(eq(transSum, -159.092_f));
			auto meta = cam.metadata;
			expect(meta.has_value());
			expect(eq(meta->patternShape.width, 10));
			expect(eq(meta->patternShape.height, 6));
			expect(eq(meta->internalPatternShape.width, 9));
			expect(eq(meta->internalPatternShape.height, 5));
			expect(eq(meta->patternType, std::string("asymmetricCirclesGrid")));
			expect(eq(meta->patternSize, 75.0f));
			expect(eq(*meta->calibrationError, 124.88_f));
			expect(eq(meta->calibrationTime, std::string("05/22/20 16:04:47")));

			auto camByName = calibration.getCameraCalibrationByName("DAVIS346_00000237");
			expect(camByName.has_value());
			expect(eq(camByName->name, std::string("DAVIS346_00000237")));
		}

		{
			auto stereo = calibration.getStereoCalibration("C0_C1").value();
			expect(eq(stereo.leftCameraName, std::string("DVXplorer_DXA00013")));
			expect(eq(stereo.rightCameraName, std::string("DAVIS346_00000237")));
			expect(stereo.metadata.has_value());
			expect(stereo.metadata->epipolarError.has_value());
			expect(eq(*stereo.metadata->epipolarError, 0.123f));

			auto esum = std::accumulate(stereo.essentialMatrix.begin(), stereo.essentialMatrix.end(), 0.f);
			expect(eq(esum, 13.724291f));
			auto fsum = std::accumulate(stereo.fundamentalMatrix.begin(), stereo.fundamentalMatrix.end(), 0.f);
			expect(eq(fsum, 0.98516625f));

			auto stereoByLeft = calibration.getStereoCalibrationByLeftCameraName("DVXplorer_DXA00013");
			expect(stereoByLeft.has_value());
			expect(eq(stereoByLeft->leftCameraName, std::string("DVXplorer_DXA00013")));
			expect(eq(stereoByLeft->rightCameraName, std::string("DAVIS346_00000237")));

			auto stereoByRight = calibration.getStereoCalibrationByRightCameraName("DAVIS346_00000237");
			expect(stereoByRight.has_value());
			expect(eq(stereoByRight->leftCameraName, std::string("DVXplorer_DXA00013")));
			expect(eq(stereoByLeft->rightCameraName, std::string("DAVIS346_00000237")));
		}

		// Check the number of lines in the output. In case this fails, please make sure that the test above does not
		// need to be updated.
		std::stringstream ss;
		auto tree = calibration.toPropertyTree();
		pt::write_json(ss, tree);
		auto text  = ss.str();
		auto lines = std::count(text.begin(), text.end(), '\n');
		expect(eq(lines, 157));
	};

	"load_sample"_test = [] {
		fs::path filename = "./calibration_files/calibration_v2.0.json";
		auto calibration  = CalibrationSet::LoadFromFile(filename);

		expect(eq(calibration.getCameraList().size(), 2));
		expect(eq(calibration.getImuList().size(), 2));
		expect(eq(calibration.getStereoList().size(), 1));

		const auto cam1 = calibration.getCameraCalibration("C0");
		expect(cam1.has_value());
		if (cam1.has_value()) {
			expect(eq(cam1->resolution, cv::Size(640, 480)));
			expect(eq(cam1->focalLength, cv::Point2f(10.f, 12.f)));
			expect(eq(cam1->principalPoint, cv::Point2f(320.f, 240.f)));
			expect(eq(cam1->distortionModel, dv::camera::DistortionModel::RadTan));
			expect(eq(cam1->getDistortionModelString(), std::string("radialTangential")));
			expect(eq(cam1->position, std::string("left")));
			expect(eq(cam1->name, std::string("dvxplorer_1")));
			expect(cam1->master);
			expect(cam1->metadata.has_value());
			expect(eq(*cam1->metadata->calibrationError, 1000.0f));
		}
		const auto cam2 = calibration.getCameraCalibration("C1");
		expect(cam2.has_value());
		if (cam2.has_value()) {
			expect(eq(cam2->resolution, cv::Size(640, 480)));
			expect(eq(cam2->focalLength, cv::Point2f(10.f, 12.f)));
			expect(eq(cam2->principalPoint, cv::Point2f(320.f, 240.f)));
			expect(eq(cam2->distortionModel, dv::camera::DistortionModel::RadTan));
			expect(eq(cam2->getDistortionModelString(), std::string("radialTangential")));
			expect(eq(cam2->name, std::string("dvxplorer_2")));
			expect(eq(cam2->position, std::string("right")));
			expect(!cam2->master);
			expect(!cam2->metadata.has_value());
		}

		const auto imu1 = calibration.getImuCalibration("S0");
		expect(imu1.has_value());
		if (imu1.has_value()) {
			expect(eq(imu1->name, std::string("dvxplorer_1")));
			expect(eq(imu1->omegaMax, 0.1f));
			expect(eq(imu1->accMax, 0.2f));
		}
	};

	"stereo_calib_equidistant"_test = [] {
		fs::path filename = "./calibration_files/calibration_stereo_DAVIS_equidistant.json";
		auto calibration  = CalibrationSet::LoadFromFile(filename);

		expect(eq(calibration.getCameraList().size(), 2));
		expect(eq(calibration.getImuList().size(), 1));

		const auto cam1 = calibration.getCameraCalibration("C0");
		expect(cam1.has_value());
		if (cam1.has_value()) {
			expect(eq(cam1->resolution, cv::Size(346, 260)));
			expect(eq(cam1->focalLength.x, 222.395_f));
			expect(eq(cam1->focalLength.y, 222.470_f));
			expect(eq(cam1->principalPoint.x, 175.044_f));
			expect(eq(cam1->principalPoint.y, 139.301_f));
			expect(eq(cam1->distortionModel, dv::camera::DistortionModel::Equidistant));
			expect(eq(cam1->getDistortionModelString(), std::string("equidistant")));
			expect(eq(cam1->position, std::string("left")));
			expect(eq(cam1->name, std::string("DAVIS346_00000668")));
			expect(cam1->master);
			expect(cam1->metadata.has_value());
			expect(eq(*cam1->metadata->calibrationError, 0.14552_f));
		}
		const auto cam2 = calibration.getCameraCalibration("C1");
		expect(cam2.has_value());
		if (cam2.has_value()) {
			expect(eq(cam1->resolution, cv::Size(346, 260)));
			expect(eq(cam2->focalLength.x, 224.160_f));
			expect(eq(cam2->focalLength.y, 224.174_f));
			expect(eq(cam2->principalPoint.x, 167.958_f));
			expect(eq(cam2->principalPoint.y, 126.480_f));
			expect(eq(cam2->distortionModel, dv::camera::DistortionModel::Equidistant));
			expect(eq(cam2->getDistortionModelString(), std::string("equidistant")));
			expect(eq(cam2->name, std::string("DAVIS346_00000499")));
			expect(eq(cam2->position, std::string("right")));
			expect(!cam2->master);
			expect(cam2->metadata.has_value());
			expect(eq(*cam2->metadata->calibrationError, 0.14552_f));
		}

		const auto imu1 = calibration.getImuCalibration("S0");
		expect(imu1.has_value());
		if (imu1.has_value()) {
			expect(eq(imu1->name, std::string("DAVIS346_00000668")));
			expect(eq(imu1->omegaMax, 7.8_f));
			expect(eq(imu1->accMax, 176._f));
		}
	};

	"invalid_transformations"_test = [] {
		expect(throws([] {
			dv::camera::calibrations::CameraCalibration calibration("a", "left", true, {640, 480}, {320, 240}, {10, 12},
				{0.0f, 0.01f, 0.02f, 0.03f}, dv::camera::DistortionModel::RadTan, std::vector<float>(16, -1),
				getTestCameraMetadata());
		}));
		expect(throws([] {
			dv::camera::calibrations::IMUCalibration calibration("a", 0.1f, 0.2f, cv::Point3f(1, 2, 3),
				cv::Point3f(4, 5, 6), 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 900'000, std::vector<float>(16, -1),
				getTestIMUMetadata());
		}));
	};

	"update_sample"_test = [] {
		fs::path filename = "./calibration_files/calibration_v2.0.json";
		auto calibration  = CalibrationSet::LoadFromFile(filename);

		expect(eq(calibration.getCameraList().size(), 2));
		expect(eq(calibration.getImuList().size(), 2));
		expect(eq(calibration.getStereoList().size(), 1));

		auto cam1             = calibration.getCameraCalibration("C0");
		cam1->distortion      = {0.f, 0.f, 0.f, 0.f};
		cam1->resolution      = cv::Size(642, 482);
		cam1->focalLength     = cv::Point2f(100.f, 120.f);
		cam1->principalPoint  = cv::Point2f(325.f, 245.f);
		cam1->distortionModel = dv::camera::DistortionModel::Equidistant;
		cam1->position        = "right";
		cam1->metadata        = std::nullopt;
		cam1->master          = false;

		calibration.updateCameraCalibration(*cam1);
		const auto cam1New = calibration.getCameraCalibration("C0");
		expect(cam1New.has_value());
		if (cam1New.has_value()) {
			expect(eq(cam1New->resolution, cv::Size(642, 482)));
			expect(eq(cam1New->focalLength, cv::Point2f(100.f, 120.f)));
			expect(eq(cam1New->principalPoint, cv::Point2f(325.f, 245.f)));
			expect(eq(cam1New->distortionModel, dv::camera::DistortionModel::Equidistant));
			expect(eq(cam1New->getDistortionModelString(), std::string("equidistant")));
			expect(eq(cam1New->position, std::string("right")));
			expect(eq(cam1New->name, std::string("dvxplorer_1")));
			expect(!cam1New->master);
			expect(!cam1New->metadata.has_value());
		}

		auto cam2             = calibration.getCameraCalibration("C1");
		cam2->distortion      = cam1->distortion;
		cam2->resolution      = cam1->resolution;
		cam2->focalLength     = cam1->focalLength;
		cam2->principalPoint  = cam1->principalPoint;
		cam2->distortionModel = cam1->distortionModel;
		cam2->position        = cam1->position;
		cam2->metadata        = cam1->metadata;
		cam2->position        = "left";
		cam2->master          = true;
		calibration.updateCameraCalibration(*cam2);

		const auto cam2New = calibration.getCameraCalibration("C1");
		expect(cam2.has_value());
		if (cam2.has_value()) {
			expect(eq(cam2New->resolution, cv::Size(642, 482)));
			expect(eq(cam2New->focalLength, cv::Point2f(100.f, 120.f)));
			expect(eq(cam2New->principalPoint, cv::Point2f(325.f, 245.f)));
			expect(eq(cam2New->distortionModel, dv::camera::DistortionModel::Equidistant));
			expect(eq(cam2New->getDistortionModelString(), std::string("equidistant")));
			expect(eq(cam2New->position, std::string("left")));
			expect(eq(cam2New->name, std::string("dvxplorer_2")));
			expect(cam2New->master);
			expect(!cam2New->metadata.has_value());
		}

		auto imu1      = calibration.getImuCalibration("S0");
		imu1->omegaMax = 10.f;
		calibration.updateImuCalibration(*imu1);

		const auto imu1New = calibration.getImuCalibration("S0");
		expect(imu1New.has_value());
		if (imu1New.has_value()) {
			expect(eq(imu1New->omegaMax, 10.f));
		}

		auto stereo               = calibration.getStereoCalibrationByRightCameraName("dvxplorer_2");
		stereo->metadata->comment = "This is a test";
		calibration.updateStereoCameraCalibration(*stereo);

		const auto stereo_new = calibration.getStereoCalibrationByRightCameraName("dvxplorer_2");
		expect(stereo_new.has_value());
		if (stereo_new.has_value()) {
			expect(eq(stereo_new->metadata->comment, std::string("This is a test")));
		}

		// exceptions tests:
		expect(throws([&calibration, &cam1] {
			cam1->name = "This name definitely does not exist";
			// Updating camera with non-existent name must throw an exception
			calibration.updateCameraCalibration(*cam1);
		}));

		expect(throws([&calibration, &imu1] {
			imu1->name = "This name definitely does not exist";
			// Updating camera with non-existent name must throw an exception
			calibration.updateImuCalibration(*imu1);
		}));

		expect(throws([&calibration, &stereo] {
			stereo->leftCameraName  = "This name definitely does not exist";
			stereo->rightCameraName = "This name definitely does not exist 2";
			// Updating camera with non-existent name must throw an exception
			calibration.updateStereoCameraCalibration(*stereo);
		}));
	};

	"add_sample"_test = [] {
		fs::path filename = "./calibration_files/calibration_v2.0.json";
		auto calibration  = CalibrationSet::LoadFromFile(filename);

		auto cam1   = calibration.getCameraCalibration("C0");
		auto imu1   = calibration.getImuCalibration("S0");
		auto stereo = calibration.getStereoCalibrationByRightCameraName("dvxplorer_2");

		// exceptions tests:
		expect(throws([&calibration, &cam1] {
			// Adding camera with an existing name
			calibration.addCameraCalibration(*cam1);
		}));

		expect(throws([&calibration, &imu1] {
			// Adding imu with an existing camera name
			calibration.addImuCalibration(*imu1);
		}));

		expect(throws([&calibration, &stereo] {
			// Adding stereo with existing camera names
			calibration.addStereoCalibration(*stereo);
		}));
	};

	return EXIT_SUCCESS;
}
