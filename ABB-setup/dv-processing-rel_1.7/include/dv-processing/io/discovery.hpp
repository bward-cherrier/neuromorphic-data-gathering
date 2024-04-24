#pragma once

#include "../exception/exceptions/generic_exceptions.hpp"

#include <libcaercpp/devices/device_discover.hpp>

namespace dv::io::internal {

/**
 * Get a text representation of a chipID integer.
 * @param chipID 	Chip ID integer value.
 * @return 			Chip name string.
 */
[[nodiscard]] static inline std::string chipIDToName(const int16_t chipID) {
	switch (chipID) {
		case 0:
			return "DAVIS240A";
		case 1:
			return "DAVIS240B";
		case 2:
			return "DAVIS240C";
		case 3:
			return "DAVIS128";
		case 5: // DAVIS346B -> only FSI chip.
			return "DAVIS346";
		case 6:
			return "DAVIS640";
		case 7:
			return "DAVIS640H";
		case 8: // PixelParade.
			return "DAVIS208";
		case 9: // DAVIS346Cbsi -> only BSI chip.
			return "DAVIS346BSI";
		default:
			throw dv::exceptions::InvalidArgument<int16_t>("Unsupported chip id received", chipID);
	}
}

/**
 * Get a camera name from a discovered device structure.
 * @param discovery		Discovered device.
 * @return				A string with device name, that is used in the library to identify a unique device.
 */
[[nodiscard]] static inline std::string getDiscoveredCameraName(const caer_device_discovery_result &discovery) {
	switch (discovery.deviceType) {
		case CAER_DEVICE_DAVIS: {
			return chipIDToName(discovery.deviceInfo.davisInfo.chipID)
				.append("_")
				.append(discovery.deviceInfo.davisInfo.deviceSerialNumber);
		}
		case CAER_DEVICE_DVS128: {
			return std::string("DVS128_").append(discovery.deviceInfo.dvs128Info.deviceSerialNumber);
		}
		case CAER_DEVICE_EDVS: {
			return std::string("eDVS4337_").append(discovery.deviceInfo.edvsInfo.serialPortName);
		}
		case CAER_DEVICE_DVS132S: {
			return std::string("DVS132S_").append(discovery.deviceInfo.dvs132sInfo.deviceSerialNumber);
		}
		case CAER_DEVICE_DVXPLORER: {
			return std::string("DVXplorer_").append(discovery.deviceInfo.dvXplorerInfo.deviceSerialNumber);
		}
		case CAER_DEVICE_SAMSUNG_EVK: {
			return std::string("SamsungEVK_").append(discovery.deviceInfo.samsungEVKInfo.deviceSerialNumber);
		}
		default:
			throw dv::exceptions::InvalidArgument<uint16_t>("Unsupported device type", discovery.deviceType);
	}
}

/**
 * Get IMU model name from a IMU type identifier.
 * @param imuType	IMU type.
 * @return			IMU model string.
 */
[[nodiscard]] static inline std::string imuModelString(const caer_imu_types imuType) {
	switch (imuType) {
		case IMU_NONE:
			return "None";
		case IMU_INVENSENSE_6050_6150:
			return "InvenSense 6050 / 6150";
		case IMU_INVENSENSE_9250:
			return "InvenSense 9250";
		case IMU_BOSCH_BMI_160:
			return "Bosch BMI160";
		default:
			return "Unknown IMU type";
	}
}

} // namespace dv::io::internal

namespace dv::io {

/**
 * Retrieve a list of connected cameras. The list will contain camera names, which are supported
 * for `dv::io::CameraCapture` class.
 * @return	A list of currently connected camera names.
 */
[[nodiscard]] static inline std::vector<std::string> discoverDevices() {
	std::vector<std::string> names;
	const auto allCameras = libcaer::devices::discover::all();
	for (const caer_device_discovery_result &camera : allCameras) {
		try {
			names.push_back(internal::getDiscoveredCameraName(camera));
		}
		catch (std::exception &_) {
			// Ignore devices that cannot be parsed
			continue;
		}
	}
	return names;
}

} // namespace dv::io
