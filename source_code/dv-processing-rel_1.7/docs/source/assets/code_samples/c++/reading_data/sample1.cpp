#include <dv-processing/io/discovery.hpp>

#include <iostream>

int main() {
	// Call the discovery method
	const std::vector<std::string> cameras = dv::io::discoverDevices();

	std::cout << "Device discovery: found " << cameras.size() << " devices." << std::endl;

	// Loop through detected camera names and print them
	for (const auto &cameraName : cameras) {
		std::cout << "Detected device [" << cameraName << "]" << std::endl;
	}

	return 0;
}
