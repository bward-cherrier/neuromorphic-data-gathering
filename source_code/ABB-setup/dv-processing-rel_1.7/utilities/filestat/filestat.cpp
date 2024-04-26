#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE 1
#define FLATBUFFERS_TRACK_VERIFIER_BUFFER_SIZE 1

#include <dv-processing/io/read_only_file.hpp>

#include "CLI/CLI.hpp"

#include <boost/nowide/args.hpp>
#include <boost/nowide/iostream.hpp>

#include <filesystem>
#include <iostream>
#include <map>

static void printXML(const dv::io::support::XMLTreeNode &node, const std::string &prevPath, const size_t indent) {
	boost::nowide::cout << fmt::format("{:{}}{}", "", indent, prevPath + node.mName + "/") << std::endl;

	for (const auto &attr : node.mAttributes) {
		boost::nowide::cout << fmt::format("{:{}}{} = {}", "", indent, attr.mName, attr.mValue) << std::endl;
	}

	for (const auto &child : node.mChildren) {
		printXML(child, prevPath + node.mName + "/", indent + 2);
	}
}

int main(int argc, char **argv) {
	// UTF-8 support for CLI arguments on Windows.
	boost::nowide::args utfArgs(argc, argv);

	CLI::App filestat{"Analyze AEDAT4 files and print information about their content"};

	bool verbose = false;
	filestat.add_flag("--verbose,-v", verbose, "Print full packet table");

	auto fileOpt = filestat.add_option("--file,-f,file", "File to analyze");
	fileOpt->required();
	fileOpt->check(CLI::ExistingFile);
	fileOpt->check(CLI::Validator(
		[](const std::string &arg) -> std::string {
			if (arg.ends_with(".aedat4")) {
				return std::string{}; // Empty for success.
			}
			else {
				return "AEDAT4 file must have '.aedat4' extension.";
			}
		},
		"EXT_AEDAT4"));

	try {
		filestat.parse(argc, argv);
	}
	catch (const CLI::ParseError &e) {
		return filestat.exit(e);
	}

	// Parse/check command-line options.
	const auto filePath = dv::pathResolveExisting(std::filesystem::u8path(filestat["--file"]->as<std::string>()));

	std::unique_ptr<dv::io::ReadOnlyFile> file;

	try {
		file = std::make_unique<dv::io::ReadOnlyFile>(filePath, dv::io::support::defaultTypeResolver);
	}
	catch (const std::exception &ex) {
		boost::nowide::cerr << fmt::format("File '{:s}': {:s}", filePath, ex.what()) << std::endl;

		return EXIT_FAILURE;
	}

	const auto &fileInfo = file->getFileInfo();

	boost::nowide::cout << "File path (canonical): " << filePath << std::endl;
	boost::nowide::cout << "File size (OS): " << std::filesystem::file_size(filePath) << std::endl;
	boost::nowide::cout << "File size (Parser): " << fileInfo.mFileSize << std::endl;
	boost::nowide::cout << "Compression: " << dv::EnumNameCompressionType(fileInfo.mCompression) << std::endl;
	boost::nowide::cout << "Timestamp lowest: " << fileInfo.mTimeLowest << std::endl;
	boost::nowide::cout << "Timestamp highest: " << fileInfo.mTimeHighest << std::endl;
	boost::nowide::cout << "Timestamp difference: " << fileInfo.mTimeDifference << std::endl;
	boost::nowide::cout << "Timestamp shift: " << fileInfo.mTimeShift << std::endl;

	if (fileInfo.mStreams.empty()) {
		boost::nowide::cerr << "Streams: NONE FOUND" << std::endl;
	}
	else {
		for (const auto &st : fileInfo.mStreams) {
			boost::nowide::cout << fmt::format("Stream {:d}: {:s} - {:s}", st.mId, st.mName, st.mTypeIdentifier)
								<< std::endl;

			// Verbose print: original XML string.
			if (verbose) {
				boost::nowide::cout << "XML content:" << std::endl;
				printXML(st.mXMLNode, "", 0);
				boost::nowide::cout << std::endl;
			}
		}
	}

	boost::nowide::cout << "DataTable file position: " << fileInfo.mDataTablePosition << std::endl;
	boost::nowide::cout << "DataTable file size: " << fileInfo.mDataTableSize << std::endl;
	boost::nowide::cout << "DataTable elements: " << fileInfo.mDataTable.Table.size() << std::endl;

	if (fileInfo.mDataTable.Table.empty()) {
		boost::nowide::cerr << "DataTable: NONE FOUND" << std::endl;
	}
	else {
		std::map<int32_t, int64_t> lastTimestamp;

		for (const auto &pkt : fileInfo.mDataTable.Table) {
			// Verbose print: full packet data from file table.
			if (verbose) {
				boost::nowide::cout << fmt::format("Packet at {:d}: StreamID {:d} - Size {:d} - NumElements {:d} - "
												   "TimestampStart {:d} - TimestampEnd {:d}",
					pkt.ByteOffset, pkt.PacketInfo.StreamID(), pkt.PacketInfo.Size(), pkt.NumElements,
					pkt.TimestampStart, pkt.TimestampEnd)
									<< std::endl;
			}

			if (pkt.TimestampEnd < pkt.TimestampStart) {
				boost::nowide::cout << fmt::format(
					"Packet at {:d}: ERROR: timestamps out of order inside packet, difference {:d}", pkt.ByteOffset,
					(pkt.TimestampEnd - pkt.TimestampStart))
									<< std::endl;
			}

			if (pkt.TimestampStart < lastTimestamp[pkt.PacketInfo.StreamID()]) {
				boost::nowide::cout << fmt::format(
					"Packet at {:d}: ERROR: timestamps out of order between packets, difference {:d}", pkt.ByteOffset,
					(pkt.TimestampStart - lastTimestamp[pkt.PacketInfo.StreamID()]))
									<< std::endl;
			}

			lastTimestamp[pkt.PacketInfo.StreamID()] = pkt.TimestampEnd;
		}
	}

	return EXIT_SUCCESS;
}
