#pragma once

#include "support/utils.hpp"
#include "support/xml_config_io.hpp"

#include <opencv2/core.hpp>

#include <optional>

namespace dv::io {

/**
 * Structure defining a stream of data. This class holds metadata information of a stream - id, name, source,
 * resolution (if applicable), as well as data type, compression, and other technical information needed for
 * application to be able send or receive streams of data.
 */
struct Stream {
	/**
	 * Create an event stream.
	 * @param id Stream ID.
	 * @param name Name of the stream.
	 * @param sourceName Stream source, usually a camera name.
	 * @param resolution Event sensor resolution.
	 * @return Stream definition.
	 */
	static Stream EventStream(
		const int32_t id, const std::string &name, const std::string &sourceName, const cv::Size &resolution) {
		auto stream = Stream::TypedStream<dv::EventPacket>(id, name, sourceName);
		stream.setResolution(resolution);
		return stream;
	}

	/**
	 * Create a frame stream.
	 * @param id Stream ID.
	 * @param name Name of the stream.
	 * @param sourceName Stream source, usually a camera name.
	 * @param resolution Frame sensor resolution.
	 * @return Stream definition.
	 */
	static Stream FrameStream(
		const int32_t id, const std::string &name, const std::string &sourceName, const cv::Size &resolution) {
		auto stream = Stream::TypedStream<dv::Frame>(id, name, sourceName);
		stream.setResolution(resolution);
		return stream;
	}

	/**
	 * Create an IMU stream.
	 * @param id Stream ID.
	 * @param name Name of the stream.
	 * @param sourceName Stream source, usually a camera name.
	 * @return Stream definition.
	 */
	static Stream IMUStream(const int32_t id, const std::string &name, const std::string &sourceName) {
		return Stream::TypedStream<dv::IMUPacket>(id, name, sourceName);
	}

	/**
	 * Create an triger stream.
	 * @param id Stream ID.
	 * @param name Name of the stream.
	 * @param sourceName Stream source, usually a camera name.
	 * @return Stream definition.
	 */
	static Stream TriggerStream(const int32_t id, const std::string &name, const std::string &sourceName) {
		return Stream::TypedStream<dv::TriggerPacket>(id, name, sourceName);
	}

	/**
	 * Create a stream by providing providing a stream type packet type as a template parameter.
	 * @tparam Type Type of the stream.
	 * @param id Stream ID.
	 * @param name Name of the stream.
	 * @param sourceName Stream source, usually a camera name.
	 * @param resolver Type resolver, supports default streams, used only for custom generated type support.
	 * @return Stream definition.
	 */
	template<class Type>
	requires dv::concepts::FlatbufferPacket<Type>
	static Stream TypedStream(const int32_t id, const std::string &name, const std::string &sourceName,
		const dv::io::support::TypeResolver &resolver = dv::io::support::defaultTypeResolver) {
		return Stream(id, name, sourceName, Type::TableType::identifier, resolver);
	}

	/**
	 * Default constructor with no information about the stream.
	 */
	Stream() = default;

	/**
	 * Manual stream configuration.
	 * @param id Stream ID.
	 * @param name Name of the stream.
	 * @param sourceName Stream source, usually a camera name.
	 * @param typeIdentifier Flatbuffer compiler generated type identifier string, unique for the stream type.
	 * @param resolver Type resolver, supports default streams, used only for custom generated type support.
	 */
	Stream(const int32_t id, const std::string_view name, const std::string_view sourceName,
		const std::string_view typeIdentifier,
		const dv::io::support::TypeResolver &resolver = dv::io::support::defaultTypeResolver) :
		mId(id),
		mName(name),
		mTypeIdentifier(typeIdentifier),
		mType(*resolver(dv::types::IdentifierStringToId(mTypeIdentifier))),
		mXMLNode(std::to_string(id)) {
		mXMLNode.mAttributes.emplace_back("typeIdentifier").mValue = mTypeIdentifier;
		auto &info                                                 = mXMLNode.mChildren.emplace_back("info");
		info.mAttributes.emplace_back("source").mValue             = std::string(sourceName);
		setCompression(dv::CompressionType::NONE);
		setOutputName(mName);
		setModuleName("dv-processing");
	}

	/**
	 * Add metadata to the stream. If an entry already exists, it will be replaced with the new value.
	 * @param name Name of the metadata entry.
	 * @param value Metadata value.
	 */
	void addMetadata(const std::string &name, const dv::io::support::VariantValueOwning &value) {
		auto info = std::find(mXMLNode.mChildren.begin(), mXMLNode.mChildren.end(), "info");

		if (info == mXMLNode.mChildren.end()) {
			// Insert the missing info node to the metadata tree and insert the passed name-value pair.
			mXMLNode.mChildren.emplace_back("info").mAttributes.emplace_back(name).mValue = value;
			return;
		}

		// The info node exists, add the metadata
		auto attribute = std::find(info->mAttributes.begin(), info->mAttributes.end(), name);
		if (attribute == info->mAttributes.end()) {
			info->mAttributes.emplace_back(name).mValue = value;
		}
		else {
			// If the metadata attribute with given name exists, just replace the value.
			attribute->mValue = value;
		}
	}

	/**
	 * Get a metadata attribute value.
	 * @param name Name of a metadata attribute.
	 * @return Metadata value in a variant or std::nullopt if it's not found.
	 */
	[[nodiscard]] std::optional<dv::io::support::VariantValueOwning> getMetadataValue(
		const std::string_view name) const {
		auto info = std::find(mXMLNode.mChildren.begin(), mXMLNode.mChildren.end(), "info");

		if (info == mXMLNode.mChildren.end()) {
			// Stream does not have any metadata.
			return std::nullopt;
		}

		auto attribute = std::find(info->mAttributes.begin(), info->mAttributes.end(), name);
		if (attribute == info->mAttributes.end()) {
			// Metadata does not contain an attribute with such a value.
			return std::nullopt;
		}

		return attribute->mValue;
	}

	/**
	 * Set type description. This only sets type description metadata field.
	 * @param description Metadata string that describes the type in this stream.
	 */
	void setTypeDescription(const std::string &description) {
		setAttribute("typeDescription", description);
	}

	/**
	 * Set module name that originally produces the data. This only sets the original module name metadata field.
	 * @param moduleName Module name that originally produces the data.
	 */
	void setModuleName(const std::string &moduleName) {
		setAttribute("originalModuleName", moduleName);
	}

	/**
	 * Set original output name. This only sets the original output name metadata field.
	 * @param outputName Name of the output that produces the data, usually referring to DV module output.
	 */
	void setOutputName(const std::string &outputName) {
		setAttribute("originalOutputName", outputName);
	}

	/**
	 * Set compression metadata field for this stream. This only sets the metadata field of this stream.
	 * @param compression Type of compression.
	 */
	void setCompression(const dv::CompressionType compression) {
		setAttribute("compression", dv::EnumNameCompressionType(compression));
	}

	/**
	 * Get type description.
	 * @return Type description string if available, std::nullopt otherwise.
	 */
	[[nodiscard]] std::optional<std::string> getTypeDescription() const {
		return getAttributeValue<std::string>("typeDescription");
	}

	/**
	 * Get module name.
	 * @return Module name string if available, std::nullopt otherwise.
	 */
	[[nodiscard]] std::optional<std::string> getModuleName() const {
		return getAttributeValue<std::string>("originalModuleName");
	}

	/**
	 * Get output name.
	 * @return Output name string if available, std::nullopt otherwise.
	 */
	[[nodiscard]] std::optional<std::string> getOutputName() const {
		return getAttributeValue<std::string>("originalOutputName");
	}

	/**
	 * Get compression type string.
	 * @return compression type string if available, std::nullopt otherwise.
	 */
	[[nodiscard]] std::optional<dv::CompressionType> getCompression() const {
		const auto compression = getAttributeValue<std::string>("compression");

		if (!compression.has_value()) {
			return std::nullopt;
		}

		auto compressions      = dv::EnumNamesCompressionType();
		int32_t compressionNum = 0;
		while (compressions != nullptr) {
			if (*compression == std::string_view(*compressions)) {
				return static_cast<dv::CompressionType>(compressionNum);
			}
			compressions++;
			compressionNum++;
		}

		return std::nullopt;
	}

	/**
	 * Set an attribute of this stream, if the attribute field does not exist, it will be created.
	 * @param name Name of the attribute.
	 * @param value Attribute value.
	 */
	void setAttribute(const std::string_view name, const dv::io::support::VariantValueOwning &value) {
		auto attribute = std::find(mXMLNode.mAttributes.begin(), mXMLNode.mAttributes.end(), name);

		if (attribute == mXMLNode.mAttributes.end()) {
			mXMLNode.mAttributes.emplace_back(name).mValue = value;
			return;
		}

		attribute->mValue = value;
	}

	/**
	 * Get attribute value given it's name.
	 * @param name Name of the attribute.
	 * @return Return variant of the value if the an attribute with given name exists, std::nullopt otherwise.
	 */
	[[nodiscard]] std::optional<dv::io::support::VariantValueOwning> getAttribute(const std::string_view name) const {
		const auto attribute = std::find(mXMLNode.mAttributes.begin(), mXMLNode.mAttributes.end(), name);

		if (attribute == mXMLNode.mAttributes.end()) {
			return std::nullopt;
		}
		else {
			return attribute->mValue;
		}
	}

	/**
	 * Get attribute value given it's name.
	 * @tparam Type Type of the attribute.
	 * @param name Name of the attribute.
	 * @return Return the attribute value if the an attribute with given name exists, std::nullopt otherwise.
	 */
	template<typename Type>
	[[nodiscard]] std::optional<Type> getAttributeValue(const std::string_view name) const {
		const auto attribute = getAttribute(name);

		if (!attribute.has_value()) {
			return std::nullopt;
		}

		if (const auto *const value = std::get_if<Type>(&*attribute)) {
			return *value;
		}
		else {
			return std::nullopt;
		}
	}

	/**
	 * Get resolution of this stream by parsing metadata.
	 * @return Stream resolution or `std::nullopt` if resolution is not available.
	 */
	[[nodiscard]] std::optional<cv::Size> getResolution() const {
		const auto sizeX = getMetadataValue("sizeX");
		const auto sizeY = getMetadataValue("sizeY");

		if (!sizeX.has_value() || !sizeY.has_value()) {
			return std::nullopt;
		}

		return cv::Size(std::get<int32_t>(sizeX.value()), std::get<int32_t>(sizeY.value()));
	}

	/**
	 * Set the stream resolution in the metadata of this stream.
	 * @param resolution Stream resolution.
	 */
	void setResolution(const cv::Size &resolution) {
		addMetadata("sizeX", resolution.width);
		addMetadata("sizeY", resolution.height);
	}

	/**
	 * Get source name (usually the camera name) from metadata of the stream.
	 * @return Stream source or `std::nullopt` if a source name is not available.
	 */
	[[nodiscard]] std::optional<std::string> getSource() const {
		const auto source = getMetadataValue("source");

		if (!source.has_value()) {
			return std::nullopt;
		}

		return std::get<std::string>(*source);
	}

	/**
	 * Set a source name of this stream, usually camera name.
	 * @param source Source name, usually camera name string.
	 */
	void setSource(const std::string &source) {
		addMetadata("source", source);
	}

	/// Stream ID
	int32_t mId = 0;

	/// Name of the stream
	std::string mName;

	/// Stream type identifier
	std::string mTypeIdentifier;

	/// Internal type definition
	dv::types::Type mType;

	/// XML tree node that can be used to encode information about the stream
	dv::io::support::XMLTreeNode mXMLNode;
};

} // namespace dv::io
