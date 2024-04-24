#include <dv-processing/imu/rotation-integrator.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/kinematics/motion_compensator.hpp>

#include <CLI/CLI.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

std::shared_ptr<dv::camera::CameraGeometry> getCameraInfoFromXmlFile(const cv::FileStorage &fs);

int main(int ac, char **av) {
	using namespace std::chrono_literals;

	std::string xmlPath;
	std::string aedat4Path;
	std::vector<float> gyroscopeOffsetAngle = {0.f, 0.f, 0.f};
	float framerate;
	bool thresholding;

	CLI::App app{"Command-line for imu compensation script."};
	app.add_option("-c,--calibration", xmlPath, "Path to xml calibration file.")->required()->check(CLI::ExistingFile);
	app.add_option("-i,--input", aedat4Path, "Path to an input aedat4 file.")->required()->check(CLI::ExistingFile);
	app.add_option("-g,--gyro-offset", gyroscopeOffsetAngle, "Gyroscope angle offset in radians.");
	app.add_option("-f,--framerate", framerate, "Image generation framerate.")->default_val(50.f);
	app.add_option("-t,--thresholding", thresholding, "Apply thresholding to the motion compensated image.")
		->default_val(true);

	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	// camera info from xml file
	cv::FileStorage fs(xmlPath, cv::FileStorage::READ);
	auto camera = getCameraInfoFromXmlFile(fs);
	cv::Mat T_CS;
	fs["transformation_cam_imu"] >> T_CS;

	auto imuToCamTimeOffsetUs
		= static_cast<int64_t>(static_cast<float>(fs.getFirstTopLevelNode()["time_offset_cam_imu"]) * 1e+6f);

	Eigen::Matrix4f T_CS_eigen;
	cv::cv2eigen(T_CS, T_CS_eigen);
	// Compute target (camera) position wrt to sensor (imu).
	// This will be used to get integrated imu positions wrt to camera initial frame (i.e. get camera orientation in
	// time).
	const dv::kinematics::Transformationf T_S_target = dv::kinematics::Transformationf(0, T_CS_eigen).inverse();

	const float contribution = 0.1f;
	std::unique_ptr<dv::kinematics::MotionCompensator<>> mcCompensate
		= std::make_unique<dv::kinematics::MotionCompensator<>>(
			camera, std::make_unique<dv::EdgeMapAccumulator>(camera->getResolution(), contribution));
	dv::EdgeMapAccumulator accumulator(camera->getResolution(), contribution);
	Eigen::Vector3f gyroscopeOffsetAngleEig
		= {gyroscopeOffsetAngle.at(0), gyroscopeOffsetAngle.at(1), gyroscopeOffsetAngle.at(2)};
	dv::imu::RotationIntegrator rotationIntegrator(T_S_target, imuToCamTimeOffsetUs, gyroscopeOffsetAngleEig);

	cv::namedWindow("Preview", cv::WINDOW_NORMAL);
	cv::setWindowTitle("Preview", "compensated (L) vs uncompensated (R) images");

	dv::EventStore store;
	std::vector<dv::EventStore> eventSlices;
	dv::Duration interval(static_cast<int64_t>(1e+6f / framerate));
	dv::Duration imuTimeOverhead = 5ms;
	int64_t nextTimestamp        = -1;

	dv::io::DataReadHandler handler;

	handler.mEventHandler = [&store](const dv::EventStore &events) {
		store.add(events);
	};

	handler.mImuHandler = [&](const dv::cvector<dv::IMU> &imuData) {
		// Integrate the IMU measured rotation and feed it into the motion compensator
		for (const auto &imu : imuData) {
			rotationIntegrator.accept(imu);
			auto transform = rotationIntegrator.getTransformation();
			mcCompensate->accept(transform);

			int64_t generationTime = transform.getTimestamp() - imuTimeOverhead.count();

			if (nextTimestamp < 0) {
				nextTimestamp = generationTime + interval.count();
			}
			else if (generationTime >= nextTimestamp) {
				auto events = store.sliceTime(generationTime - interval.count(), generationTime);

				eventSlices.push_back(events);
				nextTimestamp = generationTime + interval.count();
			}
		}
	};

	// Construct the camera reader
	dv::io::MonoCameraRecording reader(aedat4Path);

	while (reader.handleNext(handler)) {
		for (const auto &events : eventSlices) {
			int64_t generationTime = events.getHighestTime();
			mcCompensate->accept(events);
			dv::Frame compensatedImage = mcCompensate->generateFrame();

			if (thresholding) {
				cv::threshold(compensatedImage.image, compensatedImage.image,
					(static_cast<double>(contribution) * 255.) * 2., 0, cv::THRESH_TOZERO);
			}

			accumulator.accept(events);
			dv::Frame uncompensatedImage = accumulator.generateFrame();

			cv::Mat preview;
			cv::hconcat(compensatedImage.image, uncompensatedImage.image, preview);

			nextTimestamp = generationTime + interval.count();

			cv::imshow("Preview", preview);
			cv::waitKey(std::chrono::duration_cast<std::chrono::milliseconds>(interval).count());
		}
		eventSlices.clear();
	}

	reader.run(handler);

	return EXIT_SUCCESS;
}

std::shared_ptr<dv::camera::CameraGeometry> getCameraInfoFromXmlFile(const cv::FileStorage &fs) {
	// read camera infos from xml file
	cv::FileNode davis = fs.getFirstTopLevelNode();
	cv::Mat cameraIntrinsic;
	int width  = davis["image_width"];
	int height = davis["image_height"];
	davis["camera_matrix"] >> cameraIntrinsic;
	auto fx = static_cast<float>(cameraIntrinsic.at<double>(0, 0));
	auto fy = static_cast<float>(cameraIntrinsic.at<double>(1, 1));
	auto cx = static_cast<float>(cameraIntrinsic.at<double>(0, 2));
	auto cy = static_cast<float>(cameraIntrinsic.at<double>(1, 2));
	cv::Size resolution(width, height);

	return std::make_shared<dv::camera::CameraGeometry>(fx, fy, cx, cy, resolution);
}
