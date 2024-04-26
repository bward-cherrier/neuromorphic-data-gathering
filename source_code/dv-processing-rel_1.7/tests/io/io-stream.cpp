#include "../../include/dv-processing/io/stream.hpp"

#include "boost/ut.hpp"

using namespace boost::ut;

int main() {
	"basic_construction"_test = [] {
		const dv::io::Stream stream;

		expect(eq(stream.mId, 0));
		expect(eq(stream.mName.size(), 0ULL));
		expect(eq(stream.mTypeIdentifier.size(), 0ULL));

		expect(!stream.getSource().has_value());
		expect(!stream.getResolution().has_value());
		expect(!stream.getCompression().has_value());
		expect(!stream.getOutputName().has_value());
		expect(!stream.getTypeDescription().has_value());
	};

	"event_stream"_test = [] {
		dv::io::Stream stream = dv::io::Stream::EventStream(1, "events", "SOME_CAMERA", cv::Size(640, 480));

		expect(eq(stream.mId, 1));
		expect(eq(stream.mName, std::string("events")));
		expect(eq(stream.mTypeIdentifier, std::string(dv::EventPacket::TableType::identifier)));
		expect(eq(stream.mType.id, dv::types::IdentifierStringToId(dv::EventPacket::TableType::identifier)));

		const auto resolution = stream.getResolution().value();
		expect(eq(resolution.width, 640));
		expect(eq(resolution.height, 480));

		expect(eq(stream.getSource().value(), std::string("SOME_CAMERA")));

		expect(stream.getCompression().has_value());
		expect(stream.getOutputName().has_value());
		expect(!stream.getTypeDescription().has_value());

		stream.setCompression(dv::CompressionType::NONE);
		expect(stream.getCompression().value() == dv::CompressionType::NONE);
		stream.setCompression(dv::CompressionType::LZ4);
		expect(stream.getCompression().value() == dv::CompressionType::LZ4);
	};

	"frame_stream"_test = [] {
		dv::io::Stream stream = dv::io::Stream::FrameStream(1, "frames", "SOME_CAMERA", cv::Size(640, 480));

		expect(eq(stream.mId, 1));
		expect(eq(stream.mName, std::string("frames")));
		expect(eq(stream.mTypeIdentifier, std::string(dv::Frame::TableType::identifier)));
		expect(eq(stream.mType.id, dv::types::IdentifierStringToId(dv::Frame::TableType::identifier)));

		const auto resolution = stream.getResolution().value();
		expect(eq(resolution.width, 640));
		expect(eq(resolution.height, 480));

		expect(eq(stream.getSource().value(), std::string("SOME_CAMERA")));

		expect(stream.getCompression().has_value());
		expect(stream.getOutputName().has_value());
		expect(!stream.getTypeDescription().has_value());
	};

	"imu_stream"_test = [] {
		const dv::io::Stream stream = dv::io::Stream::IMUStream(1, "imu", "SOME_CAMERA");

		expect(eq(stream.mId, 1));
		expect(eq(stream.mName, std::string("imu")));
		expect(eq(stream.mTypeIdentifier, std::string(dv::IMUPacket::TableType::identifier)));
		expect(eq(stream.mType.id, dv::types::IdentifierStringToId(dv::IMUPacket::TableType::identifier)));

		expect(eq(stream.getSource().value(), std::string("SOME_CAMERA")));

		expect(!stream.getResolution().has_value());
		expect(stream.getCompression().has_value());
		expect(stream.getOutputName().has_value());
		expect(!stream.getTypeDescription().has_value());
	};

	"trigger_stream"_test = [] {
		const dv::io::Stream stream = dv::io::Stream::TriggerStream(1, "triggers", "SOME_CAMERA");

		expect(eq(stream.mId, 1));
		expect(eq(stream.mName, std::string("triggers")));
		expect(eq(stream.mTypeIdentifier, std::string(dv::TriggerPacket::TableType::identifier)));
		expect(eq(stream.mType.id, dv::types::IdentifierStringToId(dv::TriggerPacket::TableType::identifier)));

		expect(eq(stream.getSource().value(), std::string("SOME_CAMERA")));

		expect(!stream.getResolution().has_value());
		expect(stream.getCompression().has_value());
		expect(stream.getOutputName().has_value());
		expect(!stream.getTypeDescription().has_value());
	};

	"custom_stream"_test = [] {
		dv::io::Stream stream = dv::io::Stream::TypedStream<dv::TimedKeyPointPacket>(2, "keypoints", "SOME_CAMERA");

		expect(eq(stream.mId, 2));
		expect(eq(stream.mName, std::string("keypoints")));
		expect(eq(stream.mTypeIdentifier, std::string(dv::TimedKeyPointPacket::TableType::identifier)));
		expect(eq(stream.mType.id, dv::types::IdentifierStringToId(dv::TimedKeyPointPacket::TableType::identifier)));

		expect(eq(stream.getSource().value(), std::string("SOME_CAMERA")));

		expect(!stream.getResolution().has_value());
		expect(stream.getCompression().has_value());
		expect(stream.getOutputName().has_value());
		expect(!stream.getTypeDescription().has_value());

		stream.setResolution(cv::Size(640, 480));

		const auto resolution = stream.getResolution().value();
		expect(eq(resolution.width, 640));
		expect(eq(resolution.height, 480));
	};
}
