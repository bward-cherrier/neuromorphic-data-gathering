#include "../../include/dv-processing/visualization/pose_visualizer.hpp"

#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

namespace dvv = dv::visualization;
namespace dvk = dv::kinematics;

std::vector<dvk::Transformationf> generateTrajectory(int64_t nPositions, int64_t maxTime);

int main(int ac, char **av) {
	using namespace boost::ut;

	bool showPreview = false;
	bool interactive = false;

	CLI::App app{"Image feature tracker test"};

	app.add_flag("-p,--preview", showPreview, "Display preview image");
	app.add_flag("-i,--interactive", interactive, "Interactive mode to test pose visualizer features.");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	if (showPreview) {
		cv::namedWindow("Preview", cv::WINDOW_NORMAL);
	}

	"simple_view"_test = [showPreview] {
		const cv::Size dims(640, 480);
		auto poses = generateTrajectory(50, 20000000);
		dvv::PoseVisualizer visualizer(10000, dims);

		visualizer.setViewMode(dvv::PoseVisualizer::Mode::VIEW_XY);

		for (const auto &pose : poses) {
			visualizer.accept(pose);
			auto img = visualizer.generateFrame();

			expect(eq(img.image.size(), dims));

			if (showPreview) {
				cv::imshow("Preview", img.image);
				cv::waitKey(33);
			}
		}

		visualizer.reset();

		auto timestampOffset = dv::now();
		for (auto &pose : poses) {
			visualizer.accept(dvk::Transformationf(timestampOffset + pose.getTimestamp(), pose.getTransform()));
			auto img = visualizer.generateFrame();

			expect(eq(img.image.size(), dims));

			if (showPreview) {
				cv::imshow("Preview", img.image);
				cv::waitKey(33);
			}
		}

		for (int thickness = 1; thickness < 10; thickness++) {
			visualizer.setLineThickness(thickness);
			auto img = visualizer.generateFrame();
			expect(eq(img.image.size(), dims));
			if (showPreview) {
				cv::imshow("Preview", img.image);
				cv::waitKey(33);
			}
		}
	};

	"markers"_test = [&showPreview] {
		const cv::Size dims(640, 480);
		dvv::PoseVisualizer visualizer(10000, dims);

		auto poses = generateTrajectory(50, 20000000);

		visualizer.setViewMode(dvv::PoseVisualizer::Mode::VIEW_XY);

		dv::LandmarksPacket landmarks;

		dv::cvector<dv::Observation> obs;
		dv::cvector<int8_t> dsc;
		dv::cvector<float> cov;
		int64_t timestamp = poses.front().getTimestamp();

		landmarks.elements.emplace_back(dv::Point3f(.1f, .1f, 1.f), 1, timestamp, dsc, "", cov, obs);
		landmarks.elements.emplace_back(dv::Point3f(-.1f, .1f, 1.f), 2, timestamp, dsc, "", cov, obs);
		landmarks.elements.emplace_back(dv::Point3f(-.1f, -.1f, 1.f), 3, timestamp, dsc, "", cov, obs);
		landmarks.elements.emplace_back(dv::Point3f(.1f, -.1f, 1.f), 4, timestamp, dsc, "", cov, obs);
		landmarks.elements.emplace_back(dv::Point3f(0.f, 0.f, 1.f), 5, timestamp, dsc, "", cov, obs);

		// Just updating the landmarks
		for (const auto &pose : poses) {
			expect(nothrow([&pose, &landmarks, &visualizer] {
				visualizer.accept(pose);
				visualizer.accept(landmarks);
			}));
			auto img = visualizer.generateFrame();
			expect(eq(img.image.size(), dims));
			expect(eq(visualizer.getLandmarkSize(), landmarks.elements.size()));

			if (showPreview) {
				cv::imshow("Preview", img.image);
				cv::waitKey(33);
			}
		}

		visualizer.setLandmarkLimit(10);
		timestamp += poses.back().getTimestamp() + 1000;
		landmarks.elements.clear();
		landmarks.elements.emplace_back(dv::Point3f(.2f, .2f, 1.f), 6, timestamp, dsc, "", cov, obs);
		landmarks.elements.emplace_back(dv::Point3f(-.2f, .2f, 1.f), 7, timestamp, dsc, "", cov, obs);
		landmarks.elements.emplace_back(dv::Point3f(-.2f, -.2f, 1.f), 8, timestamp, dsc, "", cov, obs);
		landmarks.elements.emplace_back(dv::Point3f(.2f, -.2f, 1.f), 9, timestamp, dsc, "", cov, obs);
		landmarks.elements.emplace_back(dv::Point3f(0.f, 0.f, 1.f), 10, timestamp, dsc, "", cov, obs);

		visualizer.accept(dv::kinematics::Transformationf(timestamp, poses.back().getTransform()));
		visualizer.accept(landmarks);
		expect(eq(visualizer.getLandmarkSize(), 10));
		auto img = visualizer.generateFrame();
		if (showPreview) {
			cv::imshow("Preview", img.image);
			cv::waitKey(0);
		}
		landmarks.elements.clear();
		landmarks.elements.emplace_back(dv::Point3f(.3f, .3f, 1.f), 11, timestamp, dsc, "", cov, obs);
		visualizer.accept(landmarks);
		expect(le(visualizer.getLandmarkSize(), 10));
		img = visualizer.generateFrame();
		if (showPreview) {
			cv::imshow("Preview", img.image);
			cv::waitKey(0);
		}
	};

	"invalid_parameters"_test = [] {
		const cv::Size dims(640, 480);
		dvv::PoseVisualizer visualizer(10000, dims);

		expect(throws([&visualizer] {
			visualizer.setViewMode("");
		}));
		expect(throws([&visualizer] {
			visualizer.setViewMode("some random text");
		}));
		expect(throws([&visualizer] {
			visualizer.setGridPlane("");
		}));
		expect(throws([&visualizer] {
			visualizer.setGridPlane("some random text");
		}));
	};

	"resize_image"_test = [showPreview] {
		const cv::Size dims(640, 480);
		dvk::Transformationf pose;
		dvv::PoseVisualizer visualizer(10000, dims);
		visualizer.accept(pose);

		dv::Frame output = visualizer.generateFrame();
		expect(eq(output.image.size(), dims));
		if (showPreview) {
			cv::imshow("Preview", output.image);
			cv::waitKey(33);
		}

		cv::Size smaller(320, 240);
		visualizer.setFrameSize(smaller);
		output = visualizer.generateFrame();
		expect(eq(output.image.size(), smaller));
		if (showPreview) {
			cv::imshow("Preview", output.image);
			cv::waitKey(33);
		}

		cv::Size larger(1280, 960);
		visualizer.setFrameSize(larger);
		output = visualizer.generateFrame();
		expect(eq(output.image.size(), larger));
		if (showPreview) {
			cv::imshow("Preview", output.image);
			cv::waitKey(33);
		}
	};

	"changing_colors"_test = [showPreview] {
		const cv::Size dims(640, 480);
		dvk::Transformationf pose;
		dvv::PoseVisualizer visualizer(10000, dims);
		visualizer.accept(pose);

		dv::Frame output = visualizer.generateFrame();
		expect(eq(output.image.size(), dims));
		if (showPreview) {
			cv::imshow("Preview", output.image);
			cv::waitKey(33);
		}

		cv::Vec<uint8_t, 3> newColor(255, 0, 0);
		visualizer.setBackgroundColor(newColor);
		output = visualizer.generateFrame();
		expect(eq(output.image.at<cv::Vec<uint8_t, 3>>(5, 5), newColor));
		if (showPreview) {
			cv::imshow("Preview", output.image);
			cv::waitKey(33);
		}

		newColor = cv::Vec<uint8_t, 3>(0, 255, 0);
		visualizer.setBackgroundColor(newColor);
		output = visualizer.generateFrame();
		expect(eq(output.image.at<cv::Vec<uint8_t, 3>>(5, 5), newColor));
		if (showPreview) {
			cv::imshow("Preview", output.image);
			cv::waitKey(33);
		}

		newColor = cv::Vec<uint8_t, 3>(0, 0, 255);
		visualizer.setBackgroundColor(newColor);
		output = visualizer.generateFrame();
		expect(eq(output.image.at<cv::Vec<uint8_t, 3>>(5, 5), newColor));
		if (showPreview) {
			cv::imshow("Preview", output.image);
			cv::waitKey(33);
		}

		// I'm not sure how to properly sample grid color. Maybe setting grid width
		// to some absurd size that the grid would consistently overwrite the background and then
		// it would be possible to easily sample the color?
		visualizer.setGridColor(newColor);
		output = visualizer.generateFrame();
		if (showPreview) {
			cv::imshow("Preview", output.image);
			cv::waitKey(0);
		}
	};

	if (showPreview) {
		cv::destroyWindow("Preview");
	}

	if (interactive) {
		const cv::Size dims(640, 480);

		struct VisualizerContext {
			const int distMax  = 100;
			const int angleMax = 360;
			int xSlider, ySlider, zSlider;
			int yawSlider, pitchSlider, rollSlider;
			int frameSlider;
			int mode;
			int gridPlane;
			dvv::PoseVisualizer visualizer;

			explicit VisualizerContext(const cv::Size &dimensions) :
				xSlider(distMax / 2 + 0),
				ySlider(distMax / 2 + 0),
				zSlider(distMax / 2 + -5),
				yawSlider(angleMax / 2 + 0),
				pitchSlider(angleMax / 2 + 0),
				rollSlider(angleMax / 2 + 0),
				frameSlider(1),
				mode(0),
				gridPlane(0),
				visualizer(10000, dimensions) {
				visualizer.setViewMode(dvv::PoseVisualizer::Mode(mode));
			}
		};

		VisualizerContext context(dims);

		auto poses = generateTrajectory(50, 20000000);
		for (const auto &pose : poses) {
			context.visualizer.accept(pose);
		}

		cv::TrackbarCallback on_trackbar = [](int, void *visualizerPtr) {
			auto *_visualizer = reinterpret_cast<VisualizerContext *>(visualizerPtr);

			_visualizer->visualizer.updateCameraPosition(
				Eigen::Vector3f(static_cast<const float>(_visualizer->xSlider - _visualizer->distMax / 2),
					static_cast<const float>(_visualizer->ySlider - _visualizer->distMax / 2),
					static_cast<const float>(_visualizer->zSlider - _visualizer->distMax / 2)));
			_visualizer->visualizer.updateCameraOrientation(
				static_cast<float>(_visualizer->yawSlider - _visualizer->angleMax / 2),
				static_cast<float>(_visualizer->pitchSlider - _visualizer->angleMax / 2),
				static_cast<float>(_visualizer->rollSlider - _visualizer->angleMax / 2));
			_visualizer->visualizer.setCoordinateDimensions(static_cast<float>(_visualizer->frameSlider + 1));
			_visualizer->visualizer.setViewMode(static_cast<dvv::PoseVisualizer::Mode>(_visualizer->mode));
			_visualizer->visualizer.setGridPlane(static_cast<dvv::PoseVisualizer::GridPlane>(_visualizer->gridPlane));
			dv::Frame dst = _visualizer->visualizer.generateFrame();
			cv::imshow("Trajectory", dst.image);
		};

		// Make it possible to play with the controls and see how the visualization behaves
		namedWindow("Trajectory", cv::WINDOW_AUTOSIZE); // Create Window
		cv::createTrackbar(
			"x", "Trajectory", &context.xSlider, context.distMax, on_trackbar, reinterpret_cast<void *>(&context));
		cv::createTrackbar(
			"y", "Trajectory", &context.ySlider, context.distMax, on_trackbar, reinterpret_cast<void *>(&context));
		cv::createTrackbar(
			"z", "Trajectory", &context.zSlider, context.distMax, on_trackbar, reinterpret_cast<void *>(&context));
		cv::createTrackbar("yaw [deg]", "Trajectory", &context.yawSlider, context.angleMax, on_trackbar,
			reinterpret_cast<void *>(&context));
		cv::createTrackbar("pitch [deg]", "Trajectory", &context.pitchSlider, context.angleMax, on_trackbar,
			reinterpret_cast<void *>(&context));
		cv::createTrackbar("roll [deg]", "Trajectory", &context.rollSlider, context.angleMax, on_trackbar,
			reinterpret_cast<void *>(&context));
		cv::createTrackbar("Frame size [m]", "Trajectory", &context.frameSlider, context.distMax, on_trackbar,
			reinterpret_cast<void *>(&context));
		cv::createTrackbar(
			"Mode [0-6]", "Trajectory", &context.mode, 6, on_trackbar, reinterpret_cast<void *>(&context));
		cv::createTrackbar(
			"Grid plane [0-3]", "Trajectory", &context.gridPlane, 3, on_trackbar, reinterpret_cast<void *>(&context));
		on_trackbar(context.xSlider, reinterpret_cast<void *>(&context));
		cv::waitKey(0);
	}

	return EXIT_SUCCESS;
}

std::vector<dvk::Transformationf> generateTrajectory(const int64_t nPositions, const int64_t maxTime) {
	std::vector<dvk::Transformationf> poses;
	for (int64_t i = 0; i < nPositions; ++i) {
		int64_t ts = maxTime / nPositions * i;
		float tss  = static_cast<float>(ts) / 1e6f;
		// Feed some data to the plot
		auto r = Eigen::Vector3f(sin(tss) * (tss / 5 + 1), cos(tss) * (tss / 5 + 1), tss / 8);
		auto q = Eigen::Quaternionf(1, 0, 0, 0);
		poses.emplace_back(ts, r, q);
	}
	return poses;
}
