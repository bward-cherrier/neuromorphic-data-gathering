#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE 1
#define FLATBUFFERS_TRACK_VERIFIER_BUFFER_SIZE 1

#include <dv-processing/io/network_reader.hpp>

#include "CLI/CLI.hpp"

#include <boost/nowide/args.hpp>
#include <boost/nowide/iostream.hpp>

#include <filesystem>
#include <iostream>
#include <map>

static void printMessage(const std::string_view message, const int indentationLevel = 0) {
	boost::nowide::cout << fmt::format("{:\t>{}}{}", "", indentationLevel, message) << std::endl;
}

static void printXML(const dv::io::support::XMLTreeNode &node, const std::string &prevPath, const size_t indent) {
	printMessage(fmt::format("{}", prevPath + node.mName + "/"), indent);

	for (const auto &attr : node.mAttributes) {
		printMessage(fmt::format("{} = {}", attr.mName, attr.mValue), indent);
	}

	for (const auto &child : node.mChildren) {
		printXML(child, prevPath + node.mName + "/", indent + 2);
	}
}

int main(int argc, char **argv) {
	// UTF-8 support for CLI arguments on Windows.
	boost::nowide::args(argc, argv);

	CLI::App tcpstat{"Connect to a TCP streaming server and print information about it"};

	std::string ip;
	uint16_t port;

	bool verbose   = false;
	bool doReading = false;
	tcpstat.add_flag("--verbose,-v", verbose, "Print full packet table");
	tcpstat.add_flag("--try-reading,-r", doReading, "Try reading and printing packet information");
	tcpstat.add_option("--ip,-i", ip, "IP address of the server to connect to")->required()->check(CLI::ValidIPV4);
	tcpstat.add_option("--port,-p", port, "Server port number")->required()->check(CLI::Range(0, 65535));

	try {
		tcpstat.parse(argc, argv);
	}
	catch (const CLI::ParseError &e) {
		return tcpstat.exit(e);
	}

	printMessage(fmt::format("Attempting to connect to [{}:{}]...", ip, port));
	std::unique_ptr<dv::io::NetworkReader> client;
	try {
		client = std::make_unique<dv::io::NetworkReader>(ip, port);
	}
	catch (std::exception &exception) {
		boost::nowide::cerr << "Connection to [" << ip << ":" << port << "] failed" << std::endl;
		boost::nowide::cerr << "Reason:" << exception.what() << std::endl;
		return 1;
	}

	printMessage(fmt::format("Connected to [{}:{}]!", ip, port));

	const dv::io::Stream &stream = client->getStreamDefinition();

	printMessage("");
	printMessage(fmt::format("Stream info on stream ID {}:", stream.mId));
	printMessage(fmt::format("Stream name: \"{}\"", stream.mName), 1);
	printMessage(fmt::format("Stream type identifier: \"{}\"", stream.mTypeIdentifier), 1);

	if (verbose) {
		printMessage("Stream details:", 1);
		printXML(stream.mXMLNode, "/", 2);
	}

	if (!doReading) {
		// We are done
		return EXIT_SUCCESS;
	}

	// --------- THIS PROCEEDS ONLY IF USER PASSED THE FLAG ---------
	while (client->isRunning()) {
		switch (stream.mType.id) {
			case dv::types::IdentifierStringToId(dv::EventPacket::TableType::identifier):
				if (const auto packet = client->getNextEventBatch(); packet.has_value()) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			case dv::types::IdentifierStringToId(dv::Frame::TableType::identifier):
				if (const auto packet = client->getNextFrame(); packet.has_value()) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			case dv::types::IdentifierStringToId(dv::IMUPacket::TableType::identifier):
				if (const auto packet = client->getNextPacket<dv::IMUPacket>(); packet) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			case dv::types::IdentifierStringToId(dv::TriggerPacket::TableType::identifier):
				if (const auto packet = client->getNextPacket<dv::TriggerPacket>(); packet) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			case dv::types::IdentifierStringToId(dv::Pose::TableType::identifier):
				if (const auto packet = client->getNextPacket<dv::Pose>(); packet) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			case dv::types::IdentifierStringToId(dv::DepthEventPacket::TableType::identifier):
				if (const auto packet = client->getNextPacket<dv::DepthEventPacket>(); packet) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			case dv::types::IdentifierStringToId(dv::DepthFrame::TableType::identifier):
				if (const auto packet = client->getNextPacket<dv::DepthFrame>(); packet) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			case dv::types::IdentifierStringToId(dv::BoundingBoxPacket::TableType::identifier):
				if (const auto packet = client->getNextPacket<dv::BoundingBoxPacket>(); packet) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			case dv::types::IdentifierStringToId(dv::TimedKeyPointPacket::TableType::identifier):
				if (const auto packet = client->getNextPacket<dv::TimedKeyPointPacket>(); packet) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			case dv::types::IdentifierStringToId(dv::LandmarksPacket::TableType::identifier):
				if (const auto packet = client->getNextPacket<dv::LandmarksPacket>(); packet) {
					printMessage(fmt::format("[{}] Received: {}", dv::toTimePoint(dv::now()), *packet));
				}
				break;
			default:
				boost::nowide::cerr << "Unsupported data type for reading." << std::endl;
				exit(1);
		}
	}

	printMessage("Server has gone away");
	return EXIT_SUCCESS;
}
