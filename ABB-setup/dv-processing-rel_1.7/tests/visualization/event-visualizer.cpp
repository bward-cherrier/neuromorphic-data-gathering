#include "../../include/dv-processing/visualization/event_visualizer.hpp"

#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <opencv2/highgui.hpp>

namespace dvv = dv::visualization;

int main(int ac, char **av) {
	using namespace boost::ut;

	bool showPreview = false;

	CLI::App app{"Image feature tracker test"};

	app.add_flag("-p,--preview", showPreview, "Display preview image");
	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	if (showPreview) {
		cv::namedWindow("Preview", cv::WINDOW_NORMAL);
	}

	"simple_image"_test = [showPreview] {
		const cv::Size resolution(10, 10);
		dvv::EventVisualizer visualizer(resolution);

		dv::EventStore events;
		cv::Mat image = visualizer.generateImage(events);

		if (showPreview) {
			cv::imshow("Preview", image);
			cv::waitKey();
		}

		const auto bgColor = visualizer.getBackgroundColor();
		for (int j = 0; j < resolution.height; j++) {
			for (int i = 0; i < resolution.width; i++) {
				const auto &color = image.at<cv::Vec3b>(j, i);
				expect(eq(color(0), bgColor(0)));
				expect(eq(color(1), bgColor(1)));
				expect(eq(color(2), bgColor(2)));
			}
		}

		events.emplace_back(100, 5, 5, true);
		events.emplace_back(100, 6, 6, false);
		image = visualizer.generateImage(events);

		if (showPreview) {
			cv::imshow("Preview", image);
			cv::waitKey();
		}

		const auto positiveColor = visualizer.getPositiveColor();
		const auto negativeColor = visualizer.getNegativeColor();
		for (int j = 0; j < resolution.height; j++) {
			for (int i = 0; i < resolution.width; i++) {
				const auto &color = image.at<cv::Vec3b>(j, i);

				if (i == 5 && j == 5) {
					expect(eq(color(0), positiveColor(0)));
					expect(eq(color(1), positiveColor(1)));
					expect(eq(color(2), positiveColor(2)));
				}
				else if (i == 6 && j == 6) {
					expect(eq(color(0), negativeColor(0)));
					expect(eq(color(1), negativeColor(1)));
					expect(eq(color(2), negativeColor(2)));
				}
				else {
					expect(eq(color(0), bgColor(0)));
					expect(eq(color(1), bgColor(1)));
					expect(eq(color(2), bgColor(2)));
				}
			}
		}

		visualizer.setBackgroundColor(dvv::colors::black);
		visualizer.setPositiveColor(dvv::colors::cyan);
		visualizer.setNegativeColor(dvv::colors::magenta);
		image = visualizer.generateImage(events);
		if (showPreview) {
			cv::imshow("Preview", image);
			cv::waitKey();
		}

		for (int j = 0; j < resolution.height; j++) {
			for (int i = 0; i < resolution.width; i++) {
				const auto &color = image.at<cv::Vec3b>(j, i);

				if (i == 5 && j == 5) {
					expect(eq(color(0), dvv::colors::cyan(0)));
					expect(eq(color(1), dvv::colors::cyan(1)));
					expect(eq(color(2), dvv::colors::cyan(2)));
				}
				else if (i == 6 && j == 6) {
					expect(eq(color(0), dvv::colors::magenta(0)));
					expect(eq(color(1), dvv::colors::magenta(1)));
					expect(eq(color(2), dvv::colors::magenta(2)));
				}
				else {
					expect(eq(color(0), dvv::colors::black(0)));
					expect(eq(color(1), dvv::colors::black(1)));
					expect(eq(color(2), dvv::colors::black(2)));
				}
			}
		}
	};

	return EXIT_SUCCESS;
}
