#include <dv-processing/core/frame.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>
#include <dv-processing/noise/fast_decay_noise_filter.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

#include <CLI/CLI.hpp>
#include <opencv2/highgui.hpp>

#include <thread>

int main(int ac, char **av) {
	using namespace std::chrono_literals;

	std::string aedat4Path;

	CLI::App app{"Command-line aedat4 preview player of recorded frames"};

	app.add_option("-i,--input", aedat4Path, "Path to an input aedat4 file to be played.");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	// Construct the reader
	dv::io::MonoCameraRecording reader(aedat4Path);

	// Test whether the frame stream available
	if (!reader.isEventStreamAvailable()) {
		throw dv::exceptions::InvalidArgument<std::string>(
			fmt::format("Noise filtering sample requires an aedat4 with an event stream, the supplied file [{}] does "
						"not contain event stream",
				aedat4Path));
	}

	// Create a display window for the frames
	cv::namedWindow("Noise Filtering", cv::WINDOW_AUTOSIZE);

	dv::EventStreamSlicer slicer;

	const cv::Size resolution = *reader.getEventResolution();

	dv::noise::FastDecayNoiseFilter<> fastFilter(resolution, 10ms, 4);
	dv::noise::BackgroundActivityNoiseFilter<> BGAFilter(resolution);
	dv::RefractoryPeriodFilter<> refractoryFilter(resolution, 350us);

	dv::visualization::EventVisualizer visualizer(resolution);

	slicer.doEveryTimeInterval(33'000, [&](const dv::EventStore &events) {
		fastFilter.accept(events);
		dv::EventStore fastFiltered = fastFilter.generateEvents();
		cv::Mat fastFilterPreview   = visualizer.generateImage(fastFiltered);

		BGAFilter.accept(events);
		dv::EventStore BGAFiltered = BGAFilter.generateEvents();
		cv::Mat BGAPreview         = visualizer.generateImage(BGAFiltered);

		refractoryFilter.accept(events);
		dv::EventStore refractoryFiltered = refractoryFilter.generateEvents();
		cv::Mat refractoryPreview         = visualizer.generateImage(BGAFiltered);

		cv::Mat noFilter = visualizer.generateImage(events);

		cv::Mat preview;
		std::vector<cv::Mat> images = {noFilter, fastFilterPreview, BGAPreview, refractoryPreview};
		cv::hconcat(images, preview);

		cv::imshow("Noise Filtering", preview);

		std::cout << "Fast reduction rate: " << fastFilter.getReductionFactor() << std::endl;
		std::cout << "BGA reduction rate: " << BGAFilter.getReductionFactor() << std::endl;
		std::cout << "Refractory period filter reduction rate: " << refractoryFilter.getReductionFactor() << std::endl;
		std::cout << "Refractory period filter reduction rate: "
				  << refractoryFilter.getNumIncomingEvents() - refractoryFilter.getNumOutgoingEvents() << std::endl;

		cv::waitKey(33);
	});

	while (const auto events = reader.getNextEventBatch()) {
		slicer.accept(*events);
	}

	return EXIT_SUCCESS;
}
