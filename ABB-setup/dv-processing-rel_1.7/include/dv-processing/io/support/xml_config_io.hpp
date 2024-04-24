#pragma once

#include "../../core/utils.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <map>
#include <sstream>
#include <variant>

namespace dv::io::support {

using VariantValueOwning = std::variant<bool, int32_t, int64_t, float, double, std::string>;

struct XMLTreeNode;

struct XMLTreeCommon {
	std::string mName;

	XMLTreeCommon() = delete;

	explicit XMLTreeCommon(const std::string_view name) : mName(name) {
	}

	bool operator==(const XMLTreeCommon &rhs) const noexcept {
		return std::equal(mName.cbegin(), mName.cend(), rhs.mName.cbegin(), rhs.mName.cend());
	}

	auto operator<=>(const XMLTreeCommon &rhs) const noexcept {
		const auto result = mName.compare(rhs.mName);

		if (result == 0) {
			return std::strong_ordering::equal;
		}

		return (result < 0) ? (std::strong_ordering::less) : (std::strong_ordering::greater);
	}

	bool operator==(const std::string_view &rhs) const noexcept {
		return std::equal(mName.cbegin(), mName.cend(), rhs.cbegin(), rhs.cend());
	}

	auto operator<=>(const std::string_view &rhs) const noexcept {
		const auto result = mName.compare(rhs);

		if (result == 0) {
			return std::strong_ordering::equal;
		}

		return (result < 0) ? (std::strong_ordering::less) : (std::strong_ordering::greater);
	}
};

struct XMLTreeAttribute : public XMLTreeCommon {
	dv::io::support::VariantValueOwning mValue;

	XMLTreeAttribute() = delete;

	explicit XMLTreeAttribute(const std::string_view name) : XMLTreeCommon(name) {
	}
};

struct XMLTreeNode : public XMLTreeCommon {
	std::vector<XMLTreeNode> mChildren;
	std::vector<XMLTreeAttribute> mAttributes;

	explicit XMLTreeNode() : XMLTreeCommon("") {
	}

	explicit XMLTreeNode(const std::string_view name) : XMLTreeCommon(name) {
	}
};

class XMLConfigReader {
public:
	XMLConfigReader() = delete;

	XMLConfigReader(const std::string_view xmlContent) {
		parseXML(xmlContent, "");
	}

	XMLConfigReader(const std::string_view xmlContent, const std::string_view expectedRootName) {
		parseXML(xmlContent, expectedRootName);
	}

	const XMLTreeNode &getRoot() const {
		return mRoot;
	}

private:
	static std::vector<std::reference_wrapper<const boost::property_tree::ptree>> xmlFilterChildNodes(
		const boost::property_tree::ptree &content, const std::string &name) {
		std::vector<std::reference_wrapper<const boost::property_tree::ptree>> result;

		for (const auto &elem : content) {
			if (elem.first == name) {
				result.push_back(elem.second);
			}
		}

		return result;
	}

	static void consumeXML(const boost::property_tree::ptree &content, XMLTreeNode &node) {
		const auto attributes = xmlFilterChildNodes(content, "attr");

		for (const auto &attr : attributes) {
			// Check that the proper attributes exist.
			const auto key  = attr.get().get("<xmlattr>.key", "");
			const auto type = attr.get().get("<xmlattr>.type", "");

			if (key.empty() || type.empty()) {
				continue;
			}

			// Get the value and convert it from string.
			const auto value = attr.get().get_value("");

			const auto valueVariant = stringToValueConverter(type, value);

			auto &iter  = node.mAttributes.emplace_back(key);
			iter.mValue = valueVariant;
		}

		dv::vectorSortUnique(node.mAttributes);

		const auto children = xmlFilterChildNodes(content, "node");

		for (const auto &child : children) {
			// Check that the proper attributes exist.
			const auto childName = child.get().get("<xmlattr>.name", "");

			if (childName.empty()) {
				continue;
			}

			auto &iter = node.mChildren.emplace_back(childName);

			// And call recursively.
			consumeXML(child.get(), iter);
		}

		dv::vectorSortUnique(node.mChildren);
	}

	static dv::io::support::VariantValueOwning stringToValueConverter(
		const std::string &typeStr, const std::string &valueStr) {
		dv::io::support::VariantValueOwning value{};

		// Parse the values according to type.
		if (typeStr == "bool") {
			value = (valueStr == "true") ? (true) : (false);
		}
		else if (typeStr == "int") {
			value = static_cast<int32_t>(std::stoi(valueStr));
		}
		else if (typeStr == "long") {
			value = static_cast<int64_t>(std::stoll(valueStr));
		}
		else if (typeStr == "float") {
			value = std::stof(valueStr);
		}
		else if (typeStr == "double") {
			value = std::stod(valueStr);
		}
		else if (typeStr == "string") {
			value = valueStr;
		}

		return value;
	}

	XMLTreeNode mRoot;

	void parseXML(const std::string_view xmlContent, const std::string_view expectedRootName) {
		boost::property_tree::ptree xmlTree;

		{
			std::string xmlContentStr{xmlContent};
			std::istringstream inStream{std::move(xmlContentStr)};

			boost::property_tree::xml_parser::read_xml(inStream, xmlTree);
		}

		// Check name and version for compliance.
		const auto dvConfigVersion = xmlTree.get<std::string>("dv.<xmlattr>.version");
		if (dvConfigVersion != "2.0") {
			throw boost::property_tree::ptree_error("Unsupported configuration tree version (supported: '2.0').");
		}

		const auto rootNodes = xmlFilterChildNodes(xmlTree.get_child("dv"), "node");

		if (rootNodes.size() != 1) {
			throw boost::property_tree::ptree_error("Multiple or no root child nodes present, there can only be one.");
		}

		const auto &rootNode = rootNodes.front().get();

		// Strict mode: check if names match.
		const auto rootNodeName = rootNode.get<std::string>("<xmlattr>.name");

		if (rootNodeName != expectedRootName) {
			throw boost::property_tree::ptree_error(
				fmt::format("Invalid root node: name '{}' doesn't match expected root node name '{}'.", rootNodeName,
					expectedRootName));
		}

		consumeXML(rootNode, mRoot);
	}
};

class XMLConfigWriter {
public:
	XMLConfigWriter() = delete;

	XMLConfigWriter(const XMLTreeNode &root) {
		writeXML(root);
	}

	const std::string &getXMLContent() const {
		return mXMLOutputContent;
	}

private:
	static boost::property_tree::ptree generateXML(const XMLTreeNode &node, const std::string &prevPath) {
		boost::property_tree::ptree content;

		// First recurse down all the way to the leaf children, where attributes are kept.
		auto sortedChildren = node.mChildren;
		dv::vectorSortUnique(sortedChildren);

		for (const auto &child : sortedChildren) {
			const auto childContent = generateXML(child, prevPath + node.mName + "/");

			if (!childContent.empty()) {
				// Only add in nodes that have content (attributes or other nodes).
				content.add_child("node", childContent);
			}
		}

		// Then its attributes (key:value pairs).
		auto attrFirstIterator = content.begin();

		auto sortedAttributes = node.mAttributes;
		dv::vectorSortUnique(sortedAttributes);

		for (const auto &attr : sortedAttributes) {
			const auto value = valueToStringConverter(attr.mValue);

			boost::property_tree::ptree attrNode(value.second);
			attrNode.put("<xmlattr>.key", attr.mName);
			attrNode.put("<xmlattr>.type", value.first);

			// Attributes should be in order, but at the start of the node (before
			// other nodes), so we insert() them instead of just adding to the back.
			attrFirstIterator
				= content.insert(attrFirstIterator, boost::property_tree::ptree::value_type("attr", attrNode));
			attrFirstIterator++;
		}

		if (!content.empty()) {
			// Only add elements (name, path) if the node has any content
			// (attributes or other nodes), so that empty nodes are really empty.
			content.put("<xmlattr>.name", node.mName);
			content.put("<xmlattr>.path", prevPath + node.mName + "/");
		}

		return content;
	}

	static std::pair<std::string, std::string> valueToStringConverter(
		const dv::io::support::VariantValueOwning &value) {
		std::string typeStr;
		std::string valueStr;

		// Parse the values according to type.
		switch (value.index()) {
			case 0:
				typeStr  = "bool";
				valueStr = fmt::format("{}", std::get<bool>(value));
				break;

			case 1:
				typeStr  = "int";
				valueStr = fmt::format("{}", std::get<int32_t>(value));
				break;

			case 2:
				typeStr  = "long";
				valueStr = fmt::format("{}", std::get<int64_t>(value));
				break;

			case 3:
				typeStr  = "float";
				valueStr = fmt::format("{}", std::get<float>(value));
				break;

			case 4:
				typeStr  = "double";
				valueStr = fmt::format("{}", std::get<double>(value));
				break;

			case 5:
				typeStr  = "string";
				valueStr = std::get<std::string>(value);
				break;
		}

		return std::make_pair(typeStr, valueStr);
	}

	std::string mXMLOutputContent;

	void writeXML(const XMLTreeNode &root) {
		boost::property_tree::ptree xmlTree;

		// Add main configuration tree node and version.
		xmlTree.put("dv.<xmlattr>.version", "2.0");

		// Generate recursive XML for all nodes.
		xmlTree.put_child("dv.node", generateXML(root, (root.mName.empty()) ? ("") : ("/")));

		std::ostringstream xmlOutStream;

		boost::property_tree::xml_parser::xml_writer_settings<boost::property_tree::ptree::key_type> xmlIndent(' ', 4);

		// We manually call write_xml_element() here instead of write_xml() because
		// the latter always prepends the XML declaration, which we don't want.
		boost::property_tree::xml_parser::write_xml_element(
			xmlOutStream, boost::property_tree::ptree::key_type(), xmlTree, -1, xmlIndent);
		if (!xmlOutStream) {
			throw boost::property_tree::ptree_error("Failed to write XML to output stream.");
		}

		mXMLOutputContent = xmlOutStream.str();
	}
};

} // namespace dv::io::support

// fmt formatting compatibility.
namespace fmt {

template<>
class formatter<dv::io::support::VariantValueOwning> {
public:
	constexpr auto parse(const format_parse_context &ctx) {
		// Parse the presentation format and store it in the formatter:
		auto it       = ctx.begin();
		auto end      = ctx.end();
		size_t strPos = 0;

		mFmtForward[strPos] = '{';
		strPos++;

		// Any characters are part of the forwarded
		// format specifier (direct copy).
		while ((it != end) && (*it != '}')) {
			if (strPos == 1) {
				// If there actually are chars here to copy, we need to
				// also inject a : in the formatter to forward.
				mFmtForward[strPos] = ':';
				strPos++;
			}

			mFmtForward[strPos] = *it;
			strPos++;
			it++;

			if (strPos == (FORMATTER_MAX_LEN - 1)) {
				throw std::out_of_range("Formatter too long, cannot forward.");
			}
		}

		// Close and terminate formatter string.
		mFmtForward[strPos] = '}';
		strPos++;
		mFmtForward[strPos] = 0x00;

		// Return an iterator past the end of the parsed range:
		return it;
	}

	template<typename FormatContext>
	auto format(const dv::io::support::VariantValueOwning &obj, FormatContext &ctx) DV_EXT_FMT_CONST {
		const auto mFmtForwardView = DV_EXT_FMT_RUNTIME(mFmtForward.data());

		switch (obj.index()) {
			case 0:
				return fmt::format_to(ctx.out(), mFmtForwardView, std::get<bool>(obj));

			case 1:
				return fmt::format_to(ctx.out(), mFmtForwardView, std::get<int32_t>(obj));

			case 2:
				return fmt::format_to(ctx.out(), mFmtForwardView, std::get<int64_t>(obj));

			case 3:
				return fmt::format_to(ctx.out(), mFmtForwardView, std::get<float>(obj));

			case 4:
				return fmt::format_to(ctx.out(), mFmtForwardView, std::get<double>(obj));

			case 5:
				return fmt::format_to(ctx.out(), mFmtForwardView, std::get<std::string>(obj));

			default:
				throw std::out_of_range("Unknown type in dv::io::support::VariantValueOwning formatter.");
		}
	}

private:
	static constexpr size_t FORMATTER_MAX_LEN{32};

	std::array<char, FORMATTER_MAX_LEN> mFmtForward;
};

} // namespace fmt
