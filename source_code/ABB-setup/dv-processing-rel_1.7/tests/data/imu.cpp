#include "../../include/dv-processing/data/imu_base.hpp"

#include "boost/ut.hpp"

int main() {
	using namespace boost::ut;

	"unit_conversions"_test = [] {
		dv::IMU imu;
		expect(imu.getAccelerations().isApproxToConstant(0.f));
		expect(imu.getAngularVelocities().isApproxToConstant(0.f));

		imu.accelerometerX = 1.0f;
		imu.accelerometerY = 1.0f;
		imu.accelerometerZ = 1.0f;
		expect(imu.getAccelerations().isApproxToConstant(9.81f, 0.01f));

		imu.accelerometerX = 3.0f;
		imu.accelerometerY = 3.0f;
		imu.accelerometerZ = 3.0f;
		expect(imu.getAccelerations().isApproxToConstant(3 * 9.81f, 0.01f));

		imu.gyroscopeX = 180.f;
		imu.gyroscopeY = 180.f;
		imu.gyroscopeZ = 180.f;
		expect(imu.getAngularVelocities().isApproxToConstant(std::numbers::pi_v<float>, 0.01f));

		imu.gyroscopeX = 360.f;
		imu.gyroscopeY = 360.f;
		imu.gyroscopeZ = 360.f;
		expect(imu.getAngularVelocities().isApproxToConstant(2.f * std::numbers::pi_v<float>, 0.01f));

		imu.gyroscopeX = 720.f;
		imu.gyroscopeY = 720.f;
		imu.gyroscopeZ = 720.f;
		expect(imu.getAngularVelocities().isApproxToConstant(4.f * std::numbers::pi_v<float>, 0.01f));

		imu.gyroscopeX = -720.f;
		imu.gyroscopeY = -720.f;
		imu.gyroscopeZ = -720.f;
		expect(imu.getAngularVelocities().isApproxToConstant(-4.f * std::numbers::pi_v<float>, 0.01f));
	};

	return EXIT_SUCCESS;
}
