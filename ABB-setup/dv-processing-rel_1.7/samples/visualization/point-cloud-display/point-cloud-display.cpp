#include "dv-processing/core/core.hpp"
#include "dv-processing/io/camera_capture.hpp"

#include <CLI/CLI.hpp>

#include <chrono>
#include <csignal>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

static std::atomic<bool> globalShutdown(false);

static void handleShutdown(int) {
	globalShutdown.store(true);
}

/**
 * Convert and event store into a point cloud
 * @param events 		Input events
 * @param resolution 	Camera resolution used for scaling
 * @param zDistance 	Z distance to draw the data, time will be scaled to this virtual distance
 * @param duration 		Duration of data for scaling, if nullopt is passed, the data is scaled from the event store
 * 						duration into the zDistance value.
 * @return 				Pointer to converted point cloud representation of event store.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr eventsToPointCloud(const dv::EventStore &events, const cv::Size &resolution,
	float zDistance = 1.f, const std::optional<dv::Duration> &duration = std::nullopt) {
	// Retrieve the optional duration or measure the event store duration
	float maxDuration;
	if (duration.has_value()) {
		maxDuration = static_cast<float>(duration->count());
	}
	else {
		maxDuration = static_cast<float>(events.duration().count());
	}

	// Initialize the pointer, at version 1.11 PCL deprecated boost smart pointers
#if PCL_VERSION_COMPARE(<, 1, 11, 0)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
#else
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
#endif

	// Filling the metadata
	cloud->width    = events.size();
	cloud->height   = 1;
	cloud->is_dense = false;

	// Corner case handling
	if (events.isEmpty()) {
		return cloud;
	}

	// Some scaling factors, the data is scaled into a cloud of dimensions [1.0, 1.0, zDistance].
	const float invWidth  = 1.0f / static_cast<float>(resolution.width);
	const float invHeight = 1.0f / static_cast<float>(resolution.height);
	const float invTime   = maxDuration == 0.f ? 0.f : (1.0f / maxDuration) * zDistance;

	// Allocate the memory and retrieve pointer to first point
	cloud->points.resize(events.size());
	auto pIter = cloud->points.begin();

	// Color presets
	uint32_t iniblue  = 4278214071;
	uint32_t darkgrey = 4281019179;

	// Retrieve the lowest timestamp
	int64_t lowestTime = events.getLowestTime();

	for (const auto &event : events) {
		// Scale each axis accordingly
		pIter->x = static_cast<float>(event.x()) * invWidth;
		pIter->y = static_cast<float>(event.y()) * invHeight;
		pIter->z = static_cast<float>(event.timestamp() - lowestTime) * invTime;

		// Color depends on polarity
		if (event.polarity()) {
			pIter->rgba = iniblue;
		}
		else {
			pIter->rgba = darkgrey;
		}

		// Next point in the cloud
		pIter++;
	}

	return cloud;
}

int main(int ac, char **av) {
	using namespace std::chrono_literals;

	std::string cameraName;
	int64_t duration;

	CLI::App app{"Display event stream data in a 3D point cloud representation."};
	app.add_option(
		"-c,--camera-name", cameraName, "Name of the camera to open, if empty will open first found camera.");
	app.add_option("-d,--duration", duration, "Time in milliseconds of data to draw.")->default_val(250);

	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	// Gracefully handle signals to shut down properly
	std::signal(SIGINT, handleShutdown);
	std::signal(SIGTERM, handleShutdown);

	// Open a camera and retrieve the resolution
	dv::io::CameraCapture capture(cameraName);
	const cv::Size resolution = capture.getEventResolution().value();

	// Declare an spsc queue, the display will be handled in the main thread, while data reading is
	// going to be performed in a separate thread
	boost::lockfree::spsc_queue<dv::EventStore> buffer(3);

	// Initialize the preview window
	pcl::visualization::PCLVisualizer viewer("Event stream");
	viewer.setBackgroundColor(255., 255., 255.);
	viewer.initCameraParameters();
	viewer.setCameraPosition(0.0, 1.5, 0.0, 0.0, 0.0, 0.0);
	viewer.addCoordinateSystem();

	// An empty event store to initialize a placeholder for the preview
	dv::EventStore store;
	viewer.addPointCloud(eventsToPointCloud(store, resolution));
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	// Start a thread that reads data
	std::thread processing([&buffer, &capture, &duration] {
		dv::EventStore store;
		int64_t drawDuration        = duration * 1'000;
		int64_t renderInterval      = 50'000;
		int64_t lastRenderTimestamp = -1;
		while (!globalShutdown && capture.isConnected()) {
			if (auto batch = capture.getNextEventBatch(); batch.has_value()) {
				// Add data, slice back drawDuration and pass the data into rendering queue
				store.add(*batch);
				store = store.sliceTime(store.getHighestTime() - drawDuration, store.getHighestTime() + 1);
				if ((store.getHighestTime() - lastRenderTimestamp) > renderInterval) {
					buffer.push(store);
					lastRenderTimestamp = store.getHighestTime();
				}
			}
			else {
				// No events available yet, short sleep
				std::this_thread::sleep_for(10us);
				continue;
			}
		}

		globalShutdown = true;
	});

	// The rendering loop, it's on the main thread.
	while (!globalShutdown) {
		if (!buffer.empty()) {
			// Process the whole buffer
			buffer.consume_all([&viewer, &resolution](const auto &slice) {
				// Convert the event store into a colored point cloud
				viewer.updatePointCloud(eventsToPointCloud(slice, resolution, 3.f));
				// Draw the data
				viewer.spinOnce(1, true);
			});
			std::this_thread::sleep_for(100us);
		}
	}

	// Wait for the data acquisition thread to finish
	processing.join();
	return EXIT_SUCCESS;
}
