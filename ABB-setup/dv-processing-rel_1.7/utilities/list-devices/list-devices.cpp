#include <dv-processing/core/utils.hpp>
#include <dv-processing/io/discovery.hpp>

#include "CLI/CLI.hpp"

#include <boost/nowide/args.hpp>
#include <boost/nowide/iostream.hpp>

#include <iostream>

int main(int argc, char **argv) {
	// UTF-8 support for CLI arguments on Windows.
	boost::nowide::args utfArgs(argc, argv);

	CLI::App app{"Utility to find iniVation camera devices available in the system."};

	bool verbose = false;
	app.add_flag("--verbose,-v", verbose, "Print more information about devices");

	try {
		app.parse(argc, argv);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	const auto devices = libcaer::devices::discover::all();

	if (devices.empty()) {
		boost::nowide::cout << "No devices found." << std::endl;
		return EXIT_SUCCESS;
	}

	boost::nowide::cout << "Device discovery: found " << devices.size() << " devices." << std::endl;

	for (const auto &device : devices) {
		fmt::print("Detected device [{}]\n", dv::io::internal::getDiscoveredCameraName(device));

		if (!verbose) {
			continue;
		}

		switch (device.deviceType) {
			case CAER_DEVICE_DVS128:
				fmt::print("- DVS128\n");
				fmt::print("\t- USB busNum:devAddr: {}:{}\n", device.deviceInfo.dvs128Info.deviceUSBBusNumber,
					device.deviceInfo.dvs128Info.deviceUSBDeviceAddress);
				fmt::print("\t- Device can be opened: {}\n", !device.deviceErrorOpen);
				if (!device.deviceErrorOpen) {
					fmt::print("\t- USB serial number: {}\n", device.deviceInfo.dvs128Info.deviceSerialNumber);
					fmt::print("\t- Device needs firmware update: {}\n", device.deviceErrorVersion);

					if (!device.deviceErrorVersion) {
						fmt::print("\t- Timestamp Master: {}\n", device.deviceInfo.dvs128Info.deviceIsMaster);
						fmt::print("\t- Firmware Version: {}\n", device.deviceInfo.dvs128Info.firmwareVersion);
						fmt::print("\t- DVS Size X: {}\n", device.deviceInfo.dvs128Info.dvsSizeX);
						fmt::print("\t- DVS Size Y: {}\n", device.deviceInfo.dvs128Info.dvsSizeY);
					}
				}
				break;

			case CAER_DEVICE_EDVS:
				// If a serial device exists, it means it could be opened. Firmware version is not checked.
				fmt::print("- EDVS-4337\n");
				fmt::print("\t- COM port: {}\n", device.deviceInfo.edvsInfo.serialPortName);
				fmt::print("\t- Baud rate: {}\n", device.deviceInfo.edvsInfo.serialBaudRate);
				fmt::print("\t- Timestamp Master: {}\n", device.deviceInfo.edvsInfo.deviceIsMaster);
				fmt::print("\t- DVS Size X: {}\n", device.deviceInfo.edvsInfo.dvsSizeX);
				fmt::print("\t- DVS Size Y: {}\n", device.deviceInfo.edvsInfo.dvsSizeY);
				break;

			case CAER_DEVICE_DAVIS:
			case CAER_DEVICE_DAVIS_FX2:
			case CAER_DEVICE_DAVIS_FX3:
			case CAER_DEVICE_DAVIS_RPI:
				fmt::print("- DAVIS (type {})\n", device.deviceType);
				fmt::print("\t- USB busNum:devAddr: {}:{}\n", device.deviceInfo.davisInfo.deviceUSBBusNumber,
					device.deviceInfo.davisInfo.deviceUSBDeviceAddress);
				fmt::print("\t- Device can be opened: {}\n", !device.deviceErrorOpen);
				if (!device.deviceErrorOpen) {
					fmt::print("\t- USB serial number: {}\n", device.deviceInfo.davisInfo.deviceSerialNumber);
					fmt::print("\t- Device needs firmware update: {}\n", device.deviceErrorVersion);

					if (!device.deviceErrorVersion) {
						fmt::print("\t- Timestamp Master: {}\n", device.deviceInfo.davisInfo.deviceIsMaster);
						fmt::print("\t- Firmware Version: {}\n", device.deviceInfo.davisInfo.firmwareVersion);
						fmt::print("\t- Logic Version: {}\n", device.deviceInfo.davisInfo.logicVersion);
						fmt::print("\t- Chip ID: {}\n", device.deviceInfo.davisInfo.chipID);
						fmt::print("\t- DVS Size X: {}\n", device.deviceInfo.davisInfo.dvsSizeX);
						fmt::print("\t- DVS Size Y: {}\n", device.deviceInfo.davisInfo.dvsSizeY);
						fmt::print("\t- DVS Statistics: {}\n", device.deviceInfo.davisInfo.dvsHasStatistics);
						fmt::print("\t- DVS ROI Filter: {}\n", device.deviceInfo.davisInfo.dvsHasROIFilter);
						fmt::print("\t- DVS Pixel Filter: {}\n", device.deviceInfo.davisInfo.dvsHasPixelFilter);
						fmt::print("\t- DVS Skip Filter: {}\n", device.deviceInfo.davisInfo.dvsHasSkipFilter);
						fmt::print("\t- DVS Polarity Filter: {}\n", device.deviceInfo.davisInfo.dvsHasPolarityFilter);
						fmt::print(" \t- DVS BA/Refr Filter: {}\n",
							device.deviceInfo.davisInfo.dvsHasBackgroundActivityFilter);
						fmt::print("\t- APS Size X: {}\n", device.deviceInfo.davisInfo.apsSizeX);
						fmt::print("\t- APS Size Y: {}\n", device.deviceInfo.davisInfo.apsSizeY);
						fmt::print("\t- APS Color Filter: {}\n",
							dv::EnumAsInteger(device.deviceInfo.davisInfo.apsColorFilter));
						fmt::print("\t- APS Global Shutter: {}\n", device.deviceInfo.davisInfo.apsHasGlobalShutter);
						fmt::print("\t- External IO Generator: {}\n", device.deviceInfo.davisInfo.extInputHasGenerator);
						fmt::print("\t- Multiplexer Statistics: {}\n", device.deviceInfo.davisInfo.muxHasStatistics);
						fmt::print("\t- IMU Model: {}\n",
							dv::io::internal::imuModelString(device.deviceInfo.davisInfo.imuType));
					}
				}
				break;

			case CAER_DEVICE_DVXPLORER:
				fmt::print("- DVXplorer (type {})\n", device.deviceType);
				fmt::print("\t- USB busNum:devAddr: {}:{}\n", device.deviceInfo.dvXplorerInfo.deviceUSBBusNumber,
					device.deviceInfo.dvXplorerInfo.deviceUSBDeviceAddress);
				fmt::print("\t- Device can be opened: {}\n", !device.deviceErrorOpen);
				if (!device.deviceErrorOpen) {
					fmt::print("\t- USB serial number: {}\n", device.deviceInfo.dvXplorerInfo.deviceSerialNumber);
					fmt::print("\t- Device needs firmware update: {}\n", device.deviceErrorVersion);

					if (!device.deviceErrorVersion) {
						fmt::print("\t- Timestamp Master: {}\n", device.deviceInfo.dvXplorerInfo.deviceIsMaster);
						fmt::print("\t- Firmware Version: {}\n", device.deviceInfo.dvXplorerInfo.firmwareVersion);
						fmt::print("\t- Logic Version: {}\n", device.deviceInfo.dvXplorerInfo.logicVersion);
						fmt::print("\t- Chip ID: {}\n", device.deviceInfo.dvXplorerInfo.chipID);
						fmt::print("\t- DVS Size X: {}\n", device.deviceInfo.dvXplorerInfo.dvsSizeX);
						fmt::print("\t- DVS Size Y: {}\n", device.deviceInfo.dvXplorerInfo.dvsSizeY);
						fmt::print("\t- DVS Statistics: {}\n", device.deviceInfo.dvXplorerInfo.dvsHasStatistics);
						fmt::print(
							"\t- External IO Generator: {}\n", device.deviceInfo.dvXplorerInfo.extInputHasGenerator);
						fmt::print(
							"\t- Multiplexer Statistics: {}\n", device.deviceInfo.dvXplorerInfo.muxHasStatistics);
						fmt::print("\t- IMU Model: {}\n",
							dv::io::internal::imuModelString(device.deviceInfo.dvXplorerInfo.imuType));
					}
				}
				break;

			case CAER_DEVICE_SAMSUNG_EVK:
				fmt::print("- Samsung EVK (type {})\n", device.deviceType);
				fmt::print("\t- USB busNum:devAddr: {}:{}\n", device.deviceInfo.samsungEVKInfo.deviceUSBBusNumber,
					device.deviceInfo.samsungEVKInfo.deviceUSBDeviceAddress);
				fmt::print("\t- Device can be opened: {}\n", !device.deviceErrorOpen);
				if (!device.deviceErrorOpen) {
					fmt::print("\t- USB serial number: {}\n", device.deviceInfo.samsungEVKInfo.deviceSerialNumber);
					fmt::print("\t- Device needs firmware update: {}\n", device.deviceErrorVersion);

					if (!device.deviceErrorVersion) {
						fmt::print("\t- Firmware Version: {}\n", device.deviceInfo.samsungEVKInfo.firmwareVersion);
						fmt::print("\t- Chip ID: {}\n", device.deviceInfo.samsungEVKInfo.chipID);
						fmt::print("\t- DVS Size X: {}\n", device.deviceInfo.samsungEVKInfo.dvsSizeX);
						fmt::print("\t- DVS Size Y: {}\n", device.deviceInfo.samsungEVKInfo.dvsSizeY);
					}
				}
				break;

			default:
				fmt::print("- Unknown device type!\n");
				break;
		}
	}

	boost::nowide::cout << std::endl;

	return EXIT_SUCCESS;
}
