#include <dv-processing/camera/camera_geometry.hpp>
#include <dv-processing/core/event.hpp>
#include <dv-processing/data/generate.hpp>
#include <dv-processing/kinematics/transformation.hpp>
#include <dv-processing/visualization/colors.hpp>

#include <opencv2/highgui.hpp>

int main() {
	// Use VGA resolution for this sample
	const cv::Size resolution(640, 480);

	// Initialize an ideal pinhole camera model parameters with no distortion
	dv::camera::CameraGeometry geometry(640.f, 640.f, 320.f, 240.f, resolution);

	dv::EventStore positiveEvents;

	// Generate a sample set of events and filter out only positive events for this sample
	dv::polarityFilter(dv::data::generate::dvLogoAsEvents(0, resolution), positiveEvents, true);

	// Back project the events into a set of 3D points
	const auto points = geometry.backProjectSequence<std::vector<dv::Point3f>>(positiveEvents);

	// Apply some 3D transformation
	dv::kinematics::Transformationf shift(
		0, Eigen::Vector3f(0.5f, 0.3f, 0.1f), Eigen::Quaternionf(0.24f, -0.31f, -0.89f, 0.18f));

	// Apply the transformation above to each of the back-projected points
	std::vector<dv::Point3f> shiftedPoints;
	for (const auto &point : points) {
		shiftedPoints.push_back(shift.transformPoint<dv::Point3f>(point));
	}

	// Forward project the points with transformation
	const auto rotatedPixels = geometry.projectSequence<std::vector<cv::Point2f>>(shiftedPoints);

	// Choose a color for visualization and store it in a variable that can be efficiently assigned to a pixel intensity
	const auto blue = dv::visualization::colors::iniBlue;
	const cv::Vec3b color(static_cast<uint8_t>(blue[0]), static_cast<uint8_t>(blue[1]), static_cast<uint8_t>(blue[2]));

	// Draw input events on an image for input preview
	cv::Mat input(resolution, CV_8UC3, dv::visualization::colors::white);
	for (const auto &event : positiveEvents) {
		input.at<cv::Vec3b>(event.y(), event.x()) = color;
	}

	// Draw output pixels on another image
	cv::Mat output(resolution, CV_8UC3, dv::visualization::colors::white);
	for (const auto &pixel : rotatedPixels) {
		output.at<cv::Vec3b>(pixel) = color;
	}

	// Concatenate both image for a single image preview
	cv::Mat preview;
	cv::hconcat(input, output, preview);

	// Create preview window and show the image
	cv::namedWindow("Preview", cv::WINDOW_NORMAL);
	cv::imshow("Preview", preview);
	cv::waitKey();

	return 0;
}
