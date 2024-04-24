#pragma once

#include "mono_camera_writer.hpp"
#include "stereo_capture.hpp"

namespace dv::io {

class StereoCameraWriter {
private:
	MonoCameraWriter::Config leftUpdatedConfig;
	MonoCameraWriter::Config rightUpdatedConfig;

	struct StreamIdContainer {
		int32_t mEventStreamId   = -1;
		int32_t mImuStreamId     = -1;
		int32_t mTriggerStreamId = -1;
		int32_t mFrameStreamId   = -1;
	};

	StreamIdContainer leftIds;
	StreamIdContainer rightIds;

	MonoCameraWriter::StreamDescriptorMap mLeftOutputStreamDescriptors;
	MonoCameraWriter::StreamDescriptorMap mRightOutputStreamDescriptors;

	static void configureCameraOutput(int32_t &index, dv::io::support::XMLTreeNode &mRoot,
		MonoCameraWriter::Config &config, const std::string &compression, StreamIdContainer &ids,
		MonoCameraWriter::StreamDescriptorMap &streamDescriptors, const dv::io::support::TypeResolver &resolver,
		const std::string &outputPrefix) {
		// Create a new config instance so not to modify the one passed in parameters
		MonoCameraWriter::Config newConfig = config;

		if (newConfig.customDataStreams.empty()) {
			throw dv::exceptions::InvalidArgument<size_t>(
				"No output streams enabled! Output configuration must have at least one output stream enabled.",
				config.customDataStreams.size());
		}

		for (const auto &stream : newConfig.customDataStreams) {
			streamDescriptors.insert(std::make_pair(stream.first,
				MonoCameraWriter::StreamDescriptor(index, resolver(dv::types::IdentifierStringToId(stream.second)))));
			auto &customNode = mRoot.mChildren.emplace_back(std::to_string(index++));
			customNode.mAttributes.emplace_back("originalModuleName").mValue = "MonoCameraWriter";
			customNode.mAttributes.emplace_back("originalOutputName").mValue = stream.first;
			customNode.mAttributes.emplace_back("compression").mValue
				= dv::EnumNameCompressionType(newConfig.compression);
			customNode.mAttributes.emplace_back("typeDescription").mValue = "Custom data type output.";
			customNode.mAttributes.emplace_back("typeIdentifier").mValue  = stream.second;

			auto &info = customNode.mChildren.emplace_back("info");

			info.mAttributes.emplace_back("source").mValue = newConfig.cameraName;

			auto metadata = newConfig.customDataStreamsMetadata.find(stream.first);
			if (metadata != newConfig.customDataStreamsMetadata.end()) {
				for (const auto &meta : metadata->second) {
					info.mAttributes.emplace_back(meta.first).mValue = meta.second;
				}
			}
		}

		config = newConfig;
	}

	std::string createStereoHeader(const dv::io::support::TypeResolver &resolver) {
		MonoCameraWriter::validateConfig(leftUpdatedConfig);
		MonoCameraWriter::validateConfig(rightUpdatedConfig);

		dv::io::support::XMLTreeNode mRoot = dv::io::support::XMLTreeNode("outInfo");

		int32_t index = 0;

		// Note: always uses left-camera compression
		const auto compression = dv::EnumNameCompressionType(leftUpdatedConfig.compression);
		configureCameraOutput(
			index, mRoot, leftUpdatedConfig, compression, leftIds, mLeftOutputStreamDescriptors, resolver, "left_");
		configureCameraOutput(
			index, mRoot, rightUpdatedConfig, compression, rightIds, mRightOutputStreamDescriptors, resolver, "right_");

		dv::io::support::XMLConfigWriter xml(mRoot);
		return xml.getXMLContent();
	}

	void configureStreamIds() {
		left.mOutputStreamDescriptors  = mLeftOutputStreamDescriptors;
		right.mOutputStreamDescriptors = mRightOutputStreamDescriptors;
	}

	std::shared_ptr<WriteOnlyFile> file;

public:
	/// Left writing instance
	MonoCameraWriter left;

	/// Right writing instance
	MonoCameraWriter right;

	/**
	 * Open a file pass left / right camera configuration manually.
	 * @param aedat4Path	Path to output file.
	 * @param leftConfig	Left camera output stream configuration.
	 * @param rightConfig	Right camera output stream configuration.
	 * @param resolver 		Type resolver for the output file.
	 */
	StereoCameraWriter(const fs::path &aedat4Path, const MonoCameraWriter::Config &leftConfig,
		const MonoCameraWriter::Config &rightConfig,
		const dv::io::support::TypeResolver &resolver = dv::io::support::defaultTypeResolver) :
		leftUpdatedConfig(leftConfig),
		rightUpdatedConfig(rightConfig),
		file(std::make_shared<WriteOnlyFile>(aedat4Path, createStereoHeader(resolver), leftConfig.compression)),
		left(file, leftUpdatedConfig, resolver),
		right(file, rightUpdatedConfig, resolver) {
		configureStreamIds();
	}

	/**
	 * Open a file and use capture device to inspect the capabilities of the cameras. This will create
	 * all possible output streams the devices can supply.
	 * @param aedat4Path	Path to output file.
	 * @param capture 		Capture object to inspect capabilities of the cameras.
	 * @param compression 	Compression to be used for the output file.
	 * @param resolver 		Type resolver for the output file.
	 */
	StereoCameraWriter(const fs::path &aedat4Path, const StereoCapture &capture,
		const CompressionType compression             = CompressionType::LZ4,
		const dv::io::support::TypeResolver &resolver = dv::io::support::defaultTypeResolver) :
		leftUpdatedConfig(MonoCameraWriter::CaptureConfig(capture.left)),
		rightUpdatedConfig(MonoCameraWriter::CaptureConfig(capture.right)),
		file(std::make_shared<WriteOnlyFile>(aedat4Path, createStereoHeader(resolver), compression)),
		left(file, leftUpdatedConfig, resolver),
		right(file, rightUpdatedConfig, resolver) {
		configureStreamIds();
	}
};

} // namespace dv::io
