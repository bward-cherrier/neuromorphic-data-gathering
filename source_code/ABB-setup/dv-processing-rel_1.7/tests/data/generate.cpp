#include "../../include/dv-processing/data/generate.hpp"

#include "../../include/dv-processing/visualization/event_visualizer.hpp"

#include "CLI/CLI.hpp"
#include "boost/ut.hpp"

#include <opencv2/highgui.hpp>

int main(int ac, char **av) {
	using namespace boost::ut;

	bool showPreview = false;

	CLI::App app{"Data generation test"};

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

	"logo_generation"_test = [showPreview] {
		cv::Mat logo = dv::data::generate::dvLogo(cv::Size(100, 100));

		expect(!logo.empty());
		expect(eq(logo.rows, 100));
		expect(eq(logo.cols, 100));
		expect(eq(logo.type(), CV_8UC3));
		{
			const auto mean = cv::mean(logo);
			expect(neq(mean[0], 255.0));
			expect(neq(mean[1], 255.0));
			expect(neq(mean[2], 255.0));
		}

		if (showPreview) {
			cv::imshow("Preview", logo);
			cv::waitKey(0);
		}
		logo = dv::data::generate::dvLogo(cv::Size(200, 200));

		expect(!logo.empty());
		expect(eq(logo.rows, 200));
		expect(eq(logo.cols, 200));
		expect(eq(logo.type(), CV_8UC3));
		{
			const auto mean = cv::mean(logo);
			expect(neq(mean[0], 255.0));
			expect(neq(mean[1], 255.0));
			expect(neq(mean[2], 255.0));
		}

		if (showPreview) {
			cv::imshow("Preview", logo);
			cv::waitKey(0);
		}
		logo = dv::data::generate::dvLogo(cv::Size(640, 480));

		expect(!logo.empty());
		expect(eq(logo.cols, 640));
		expect(eq(logo.rows, 480));
		expect(eq(logo.type(), CV_8UC3));
		{
			const auto mean = cv::mean(logo);
			expect(neq(mean[0], 255.0));
			expect(neq(mean[1], 255.0));
			expect(neq(mean[2], 255.0));
		}

		if (showPreview) {
			cv::imshow("Preview", logo);
			cv::waitKey(0);
		}

		logo = dv::data::generate::dvLogo(cv::Size(640, 480), false);

		expect(!logo.empty());
		expect(eq(logo.cols, 640));
		expect(eq(logo.rows, 480));
		expect(eq(logo.type(), CV_8UC1));
		{
			const auto mean = cv::mean(logo);
			expect(neq(mean[0], 255.0));
		}
		if (showPreview) {
			cv::imshow("Preview", logo);
			cv::waitKey(0);
		}
	};

	"imageToEvents"_test = [] {
		const cv::Size resolution(640, 480);

		const cv::Scalar bgColor = dv::visualization::colors::white;
		const cv::Scalar pColor  = dv::visualization::colors::iniBlue;
		const cv::Scalar nColor  = dv::visualization::colors::darkGrey;

		const cv::Mat logoColored    = dv::data::generate::dvLogo(resolution, true, bgColor, pColor, nColor);
		const cv::Mat logoMonochrome = dv::data::generate::dvLogo(resolution, false, bgColor, pColor, nColor);

		const dv::EventStore events = dv::data::generate::dvLogoAsEvents(1000, resolution);

		const dv::visualization::EventVisualizer visualizer(resolution, bgColor, pColor, nColor);
		const cv::Mat logoPreview = visualizer.generateImage(events);

		const auto sum = cv::sum(logoColored - logoPreview);
		expect(eq(sum[0], 0));
		expect(eq(sum[1], 0));
		expect(eq(sum[2], 0));
	};

	return EXIT_SUCCESS;
}
