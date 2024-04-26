#include <dv-processing/processing.hpp>

#include "../external/pybind11_opencv_numpy/ndarray_converter.h"

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <utility>

PYBIND11_MAKE_OPAQUE(dv::cvector<dv::Event>)
PYBIND11_MAKE_OPAQUE(dv::cvector<dv::DepthEvent>)
PYBIND11_MAKE_OPAQUE(dv::cvector<dv::IMU>)
PYBIND11_MAKE_OPAQUE(dv::cvector<dv::Landmark>)
PYBIND11_MAKE_OPAQUE(dv::cvector<dv::TimedKeyPoint>)
PYBIND11_MAKE_OPAQUE(dv::cvector<dv::Trigger>)
PYBIND11_MAKE_OPAQUE(dv::cvector<dv::BoundingBox>)

namespace py = pybind11;

namespace pybind11::detail {

template<typename Type>
struct type_caster<dv::cvector<Type>> : list_caster<dv::cvector<Type>, Type> {};

template<>
struct type_caster<dv::Point2f> {
	PYBIND11_TYPE_CASTER(dv::Point2f, _("tuple_xy"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Point2f(x,y) should be a tuple!");
			return false;
		}

		auto pt = reinterpret_borrow<py::tuple>(obj);
		if (pt.size() != 2) {
			std::logic_error("Point2f(x,y) tuple should be size of 2");
			return false;
		}

		value = dv::Point2f(pt[0].cast<float>(), pt[1].cast<float>());
		return true;
	}

	static handle cast(const dv::Point2f &point, return_value_policy, handle) {
		return py::make_tuple(point.x(), point.y()).release();
	}
};

template<>
struct type_caster<dv::Vec2f> {
	PYBIND11_TYPE_CASTER(dv::Vec2f, _("tuple_xy"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Vec2f(x,y) should be a tuple!");
			return false;
		}

		auto pt = reinterpret_borrow<py::tuple>(obj);
		if (pt.size() != 2) {
			std::logic_error("Vec2f(x,y) tuple should be size of 2");
			return false;
		}

		value = dv::Vec2f(pt[0].cast<float>(), pt[1].cast<float>());
		return true;
	}

	static handle cast(const dv::Vec2f &point, return_value_policy, handle) {
		return py::make_tuple(point.x(), point.y()).release();
	}
};

template<>
struct type_caster<dv::Point3f> {
	PYBIND11_TYPE_CASTER(dv::Point3f, _("tuple_xyz"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Point3f(x,y,z) should be a tuple!");
			return false;
		}

		auto pt = reinterpret_borrow<py::tuple>(obj);
		if (pt.size() != 3) {
			std::logic_error("Point3f(x,y,z) tuple should be size of 3");
			return false;
		}

		value = dv::Point3f(pt[0].cast<float>(), pt[1].cast<float>(), pt[2].cast<float>());
		return true;
	}

	static handle cast(const dv::Point3f &point, return_value_policy, handle) {
		return py::make_tuple(point.x(), point.y(), point.z()).release();
	}
};

template<>
struct type_caster<dv::Vec3f> {
	PYBIND11_TYPE_CASTER(dv::Vec3f, _("tuple_xyz"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Vec3f(x,y,z) should be a tuple!");
			return false;
		}

		auto pt = reinterpret_borrow<py::tuple>(obj);
		if (pt.size() != 3) {
			std::logic_error("Vec3f(x,y,z) tuple should be size of 3");
			return false;
		}

		value = dv::Vec3f(pt[0].cast<float>(), pt[1].cast<float>(), pt[2].cast<float>());
		return true;
	}

	static handle cast(const dv::Vec3f &point, return_value_policy, handle) {
		return py::make_tuple(point.x(), point.y(), point.z()).release();
	}
};

template<>
struct type_caster<dv::cstring> {
	PYBIND11_TYPE_CASTER(dv::cstring, _("string"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::str>(obj)) {
			std::logic_error("Type is not a string!");
			return false;
		}

		auto str = reinterpret_borrow<py::str>(obj);

		value = dv::cstring(str.operator std::string());
		return true;
	}

	static handle cast(const dv::cstring &string, return_value_policy, handle) {
		return py::str(string.c_str()).release();
	}
};

template<>
struct type_caster<cv::Size> {
	PYBIND11_TYPE_CASTER(cv::Size, _("tuple_xy"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Size(width,height) should be a tuple!");
			return false;
		}

		auto pt = reinterpret_borrow<py::tuple>(obj);
		if (pt.size() != 2) {
			std::logic_error("Size(width,height) tuple should be size of 2");
			return false;
		}

		value = cv::Size(pt[0].cast<int>(), pt[1].cast<int>());
		return true;
	}

	static handle cast(const cv::Size &resolution, return_value_policy, handle) {
		return py::make_tuple(resolution.width, resolution.height).release();
	}
};

template<>
struct type_caster<cv::Point2f> {
	PYBIND11_TYPE_CASTER(cv::Point2f, _("tuple_xy"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Point2f(x,y) should be a tuple!");
			return false;
		}

		auto pt = reinterpret_borrow<py::tuple>(obj);
		if (pt.size() != 2) {
			std::logic_error("Point2f(x,y) tuple should be size of 2");
			return false;
		}

		value = cv::Point2f(pt[0].cast<float>(), pt[1].cast<float>());
		return true;
	}

	static handle cast(const cv::Point2f &point, return_value_policy, handle) {
		return py::make_tuple(point.x, point.y).release();
	}
};

template<>
struct type_caster<cv::Point3f> {
	PYBIND11_TYPE_CASTER(cv::Point3f, _("tuple_xyz"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Point3f(x,y,z) should be a tuple!");
			return false;
		}

		auto pt = reinterpret_borrow<py::tuple>(obj);
		if (pt.size() != 3) {
			std::logic_error("Point3f(x,y,z) tuple should be size of 3");
			return false;
		}

		value = cv::Point3f(pt[0].cast<float>(), pt[1].cast<float>(), pt[2].cast<float>());
		return true;
	}

	static handle cast(const cv::Point3f &point, return_value_policy, handle) {
		return py::make_tuple(point.x, point.y, point.z).release();
	}
};

template<>
struct type_caster<cv::Rect> {
	PYBIND11_TYPE_CASTER(cv::Rect, _("tuple_xywh"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Rect should be a tuple!");
			return false;
		}
		py::tuple rect = reinterpret_borrow<py::tuple>(obj);
		if (rect.size() != 4) {
			std::logic_error("Rect (x,y,w,h) tuple should be size of 4");
			return false;
		}

		value = cv::Rect(rect[0].cast<int>(), rect[1].cast<int>(), rect[2].cast<int>(), rect[3].cast<int>());
		return true;
	}

	static handle cast(const cv::Rect &rect, return_value_policy, handle) {
		return py::make_tuple(rect.x, rect.y, rect.width, rect.height).release();
	}
};

template<>
struct type_caster<dv::Quaternion> {
	PYBIND11_TYPE_CASTER(dv::Quaternion, _("tuple_wxyz"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Rect should be a tuple!");
			return false;
		}
		py::tuple quat = reinterpret_borrow<py::tuple>(obj);
		if (quat.size() != 4) {
			std::logic_error("Quaternion (w,x,y,z) tuple should be size of 4");
			return false;
		}

		value = dv::Quaternion(
			quat[0].cast<float>(), quat[1].cast<float>(), quat[2].cast<float>(), quat[3].cast<float>());
		return true;
	}

	static handle cast(const dv::Quaternion &quat, return_value_policy, handle) {
		return py::make_tuple(quat.w(), quat.x(), quat.y(), quat.z()).release();
	}
};

template<>
struct type_caster<cv::Point2i> {
	PYBIND11_TYPE_CASTER(cv::Point2i, _("tuple_xy"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Point2f(x,y) should be a tuple!");
			return false;
		}

		auto pt = reinterpret_borrow<py::tuple>(obj);
		if (pt.size() != 2) {
			std::logic_error("Point2f(x,y) tuple should be size of 2");
			return false;
		}

		value = cv::Point2i(pt[0].cast<int>(), pt[1].cast<int>());
		return true;
	}

	static handle cast(const cv::Point2i &point, return_value_policy, handle) {
		return py::make_tuple(point.x, point.y).release();
	}
};

template<>
struct type_caster<cv::Scalar> {
	PYBIND11_TYPE_CASTER(cv::Scalar, _("tuple_bgr"));

	bool load(handle obj, bool) {
		if (!py::isinstance<py::tuple>(obj)) {
			std::logic_error("Scalar(b,g,r) should be a tuple!");
			return false;
		}

		auto pt = reinterpret_borrow<py::tuple>(obj);
		if (pt.size() != 3) {
			std::logic_error("Scalar(b,g,r) tuple should be size of 3");
			return false;
		}

		value = cv::Scalar(pt[0].cast<double>(), pt[1].cast<double>(), pt[2].cast<double>());
		return true;
	}

	static handle cast(const cv::Scalar &scalar, return_value_policy, handle) {
		return py::make_tuple(scalar[0], scalar[1], scalar[2]).release();
	}
};

// `boost::optional` as an example -- can be any `std::optional`-like container
template<typename T>
struct type_caster<boost::optional<T>> : optional_caster<boost::optional<T>> {};

} // namespace pybind11::detail

PYBIND11_MODULE(dv_processing, m) {
	using pybind11::operator""_a;

	m.attr("__version__") = fmt::format(
		"{}.{}.{}", DV_PROCESSING_VERSION_MAJOR, DV_PROCESSING_VERSION_MINOR, DV_PROCESSING_VERSION_PATCH);

	NDArrayConverter::init_numpy();

	py::class_<std::filesystem::path>(m, "Path").def(py::init<std::string>(), py::arg("path"));
	py::implicitly_convertible<std::string, std::filesystem::path>();

	py::enum_<dv::FrameSource> frameSource(m, "FrameSource");
	for (auto enumValue = static_cast<int8_t>(dv::FrameSource::MIN);
		 static_cast<dv::FrameSource>(enumValue) <= dv::FrameSource::MAX; enumValue++) {
		auto currentEnum = static_cast<dv::FrameSource>(enumValue);
		frameSource.value(dv::EnumNameFrameSource(currentEnum), currentEnum);
	}
	frameSource.export_values();

	py::class_<dv::Frame>(m, "Frame", "A class containing a frame image with timestamp and additional metadata")
		.def(py::init<>(), "Constructs an empty image")
		.def(py::init<int64_t, const cv::Mat &>(), py::arg("timestamp"), py::arg("image"),
			"Minimal constructor defining only a timestamp and image contents")
		.def(py::init<int64_t, int64_t, int16_t, int16_t, const cv::Mat &, dv::FrameSource>(), py::arg("timestamp"),
			py::arg("exposure"), py::arg("positionX"), py::arg("positionY"), py::arg("image"), py::arg("source"),
			"Full constructor")
		.def_readwrite("image", &dv::Frame::image, "Image contents")
		.def_readwrite(
			"timestamp", &dv::Frame::timestamp, "Capture time of the image, usually refers to start of exposure")
		.def_readwrite("exposure", &dv::Frame::exposure, "Time duration of exposure for this image")
		.def_readwrite("source", &dv::Frame::source, "Definition of source of the image")
		.def_readwrite("positionX", &dv::Frame::positionX, "X coordinate if the image is a crop")
		.def_readwrite("positionY", &dv::Frame::positionY, "X coordinate if the image is a crop")
		.def(
			"time",
			[](const dv::Frame &self) {
				return dv::toTimePoint(self.timestamp);
			},
			"Timestamp of the captured data, compatible with python time library");

	py::class_<dv::Event>(m, "Event", "A structure defining a brightness change event captured by an event camera")
		.def(py::init<const int64_t, int16_t, int16_t, bool>(), py::arg("timestamp"), py::arg("x"), py::arg("y"),
			py::arg("polarity"))
		.def("timestamp", &dv::Event::timestamp, "Microsecond timestamp of the captured event")
		.def("x", &dv::Event::x, "X coordinate")
		.def("y", &dv::Event::y, "Y coordinate")
		.def("polarity", &dv::Event::polarity, "Polarity value, True for increase of brightness, False for decrease")
		.def(
			"time",
			[](const dv::Event &self) {
				return dv::toTimePoint(self.timestamp());
			},
			"Timestamp of the captured event, compatible with python time library");

	auto eventPacketClass = py::class_<dv::EventPacket>(m, "EventPacket");
	py::bind_vector<dv::cvector<dv::Event>>(eventPacketClass, "EventVector");
	eventPacketClass.def(py::init<>())
		.def(py::init<const dv::cvector<dv::Event> &>(), "events"_a, "Create an event packet from a list of events")
		.def_readwrite("elements", &dv::EventPacket::elements)
		.def("numpy",
			[](const dv::EventPacket &self) {
				py::list names;
				names.append("timestamp");
				names.append("x");
				names.append("y");
				names.append("polarity");
				py::list formats;
				formats.append("<i8");
				formats.append("<i2");
				formats.append("<i2");
				formats.append("<i1");
				py::list offsets;
				offsets.append(0);
				offsets.append(8);
				offsets.append(10);
				offsets.append(12);
				return py::array(
					py::dtype(names, formats, offsets, sizeof(dv::Event)), self.elements.size(), self.elements.data());
			})
		.def_static("GetFullyQualifiedName", &dv::EventPacket::GetFullyQualifiedName)
		.def_static("PacketIdentifier", &dv::EventPacketIdentifier);
	py::implicitly_convertible<dv::cvector<dv::Event>, dv::EventPacket>();

	py::class_<dv::IMU>(m, "IMU")
		.def(py::init<int64_t, float, float, float, float, float, float, float, float, float, float>(),
			py::arg("timestamp"), py::arg("temperature"), py::arg("accelerometerX"), py::arg("accelerometerY"),
			py::arg("accelerometerZ"), py::arg("gyroscopeX"), py::arg("gyroscopeY"), py::arg("gyroscopeZ"),
			py::arg("magnetometerX"), py::arg("magnetometerY"), py::arg("magnetometerZ"))
		.def_readwrite("timestamp", &dv::IMU::timestamp, "Unix microsecond timestamp of the captured IMU data")
		.def_readwrite("temperature", &dv::IMU::temperature, "Temperature, measured in °C.")
		.def_readwrite(
			"accelerometerX", &dv::IMU::accelerometerX, "Acceleration in the X axis, measured in g (9.81m/s²).")
		.def_readwrite(
			"accelerometerY", &dv::IMU::accelerometerY, "Acceleration in the Y axis, measured in g (9.81m/s²).")
		.def_readwrite(
			"accelerometerZ", &dv::IMU::accelerometerZ, "Acceleration in the Z axis, measured in g (9.81m/s²).")
		.def_readwrite("gyroscopeX", &dv::IMU::gyroscopeX, "Rotation in the X axis, measured in °/s.")
		.def_readwrite("gyroscopeY", &dv::IMU::gyroscopeY, "Rotation in the Y axis, measured in °/s.")
		.def_readwrite("gyroscopeZ", &dv::IMU::gyroscopeZ, "Rotation in the Z axis, measured in °/s.")
		.def_readwrite(
			"magnetometerX", &dv::IMU::magnetometerX, "Magnetometer X axis, measured in µT (magnetic flux density).")
		.def_readwrite(
			"magnetometerY", &dv::IMU::magnetometerY, "Magnetometer Y axis, measured in µT (magnetic flux density).")
		.def_readwrite(
			"magnetometerZ", &dv::IMU::magnetometerZ, "Magnetometer Z axis, measured in µT (magnetic flux density).")
		.def(
			"time",
			[](const dv::IMU &self) {
				return dv::toTimePoint(self.timestamp);
			},
			"Timestamp of the captured data, compatible with python time library")
		.def("getAccelerations", &dv::IMU::getAccelerations, "Get measured acceleration in m/s².")
		.def("getAngularVelocities", &dv::IMU::getAngularVelocities, "Get measured angular velocities in rad/s.");

	py::class_<dv::IMUPacket> imuPacketClass(m, "IMUPacket");
	py::bind_vector<dv::cvector<dv::IMU>>(imuPacketClass, "ImuVector");
	imuPacketClass.def(py::init<>())
		.def(py::init<const dv::cvector<dv::IMU> &>(), "imu_measurements"_a,
			"Create a packet from a list of IMU measurements")
		.def_readwrite("elements", &dv::IMUPacket::elements)
		.def_static("GetFullyQualifiedName", &dv::IMUPacket::GetFullyQualifiedName)
		.def_static("PacketIdentifier", &dv::IMUPacketIdentifier);
	py::implicitly_convertible<dv::cvector<dv::IMU>, dv::IMUPacket>();

	py::class_<dv::Point3f>(m, "Point3f", "Structure representing absolute position of a 3D point")
		.def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"))
		.def("x", &dv::Point3f::x)
		.def("y", &dv::Point3f::y)
		.def("z", &dv::Point3f::z);

	py::class_<dv::Point2f>(m, "Point2f", "Structure representing absolute position of a 2D point")
		.def(py::init<float, float>(), py::arg("x"), py::arg("y"))
		.def("x", &dv::Point2f::x)
		.def("y", &dv::Point2f::y);

	py::class_<dv::Vec3f>(m, "Vec3f", "Structure representing a 3D vector")
		.def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"))
		.def("x", &dv::Vec3f::x)
		.def("y", &dv::Vec3f::y)
		.def("z", &dv::Vec3f::z);

	py::class_<dv::Vec2f>(m, "Vec2f", "Structure representing a 2D vector")
		.def(py::init<float, float>(), py::arg("x"), py::arg("y"))
		.def("x", &dv::Vec2f::x)
		.def("y", &dv::Vec2f::y);

	py::class_<dv::Quaternion>(
		m, "Quaternion", "Quaternion representing a rotation, should follow the Hamilton convention")
		.def(py::init<float, float, float, float>(), py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
		.def("w", &dv::Quaternion::w)
		.def("x", &dv::Quaternion::x)
		.def("y", &dv::Quaternion::y)
		.def("z", &dv::Quaternion::z);

	py::class_<dv::Pose>(m, "Pose")
		.def(py::init<int64_t, const dv::Vec3f &, const dv::Quaternion &, const dv::cstring &, const dv::cstring &>(),
			py::arg("timestamp"), py::arg("translation"), py::arg("rotation"), py::arg("referenceFrame"),
			py::arg("targetFrame"))
		.def_readwrite("timestamp", &dv::Pose::timestamp, "Unix microsecond timestamp of the pose")
		.def_readwrite("translation", &dv::Pose::translation, "Translational vector of the transformation")
		.def_readwrite("rotation", &dv::Pose::rotation, "Rotation quaternion of the transformation")
		.def_readwrite("referenceFrame", &dv::Pose::referenceFrame,
			"Name of the source reference frame that transformation transforms from")
		.def_readwrite("targetFrame", &dv::Pose::targetFrame,
			"Name of the target reference frame that transformation transforms into")
		.def(
			"time",
			[](const dv::Pose &self) {
				return dv::toTimePoint(self.timestamp);
			},
			"Timestamp of the captured data, compatible with python time library");

	py::class_<dv::Observation>(m, "Observation")
		.def(py::init<>())
		.def(py::init<int32_t, int32_t, const dv::cstring &, int64_t>(), "trackId"_a, "cameraId"_a, "cameraName"_a,
			"timestamp"_a)
		.def_readwrite(
			"trackId", &dv::Observation::trackId, "The tracking sequence ID that the landmark is observed by a camera")
		.def_readwrite(
			"cameraId", &dv::Observation::cameraId, "Arbitrary ID of the camera, this can be application specific")
		.def_readwrite("cameraName", &dv::Observation::cameraName, "Name of the camera. Optional.")
		.def_readwrite("timestamp", &dv::Observation::timestamp, "Timestamp of the observation (µs).");

	py::class_<dv::Landmark>(m, "Landmark")
		.def(py::init<>())
		.def(py::init<const dv::Point3f &, int64_t, int64_t, const dv::cvector<int8_t> &, const dv::cstring &,
				 const dv::cvector<float> &, const dv::cvector<dv::Observation> &>(),
			"point"_a, "id"_a, "timestamp"_a, "descriptor"_a, "descriptorType"_a, "covariance"_a, "observations"_a)
		.def_readwrite("pt", &dv::Landmark::pt, "3D coordinate of the landmark.")
		.def_readwrite(
			"id", &dv::Landmark::id, "Landmark id (if the keypoints need to be clustered by an object they belong to).")
		.def_readwrite("timestamp", &dv::Landmark::timestamp, "Timestamp (µs).")
		.def_readwrite("descriptor", &dv::Landmark::descriptor, "Visual descriptor of the landmark.")
		.def_readwrite("descriptorType", &dv::Landmark::descriptorType, "Type of the visual descriptor.")
		.def_readwrite("covariance", &dv::Landmark::covariance,
			"Covariance matrix, must contain 9 numbers. It is represented as a 3x3 square matrix.")
		.def_readwrite("observations", &dv::Landmark::observations,
			"Observation info, can be from multiple cameras if they are matched using descriptor.");

	py::class_<dv::LandmarksPacket> landmarksPacketClass(m, "LandmarksPacket");
	py::bind_vector<dv::cvector<dv::Landmark>>(landmarksPacketClass, "LandmarksVector");
	landmarksPacketClass.def(py::init<>())
		.def(py::init<const dv::cvector<dv::Landmark> &, const dv::cstring &>(), "elements"_a, "referenceFrame"_a,
			"Create a landmarks packet from a list of landmarks")
		.def_readwrite("elements", &dv::LandmarksPacket::elements, "3D coordinate of the landmark.")
		.def_readwrite("referenceFrame", &dv::LandmarksPacket::referenceFrame,
			"Landmark id (if the keypoints need to be clustered by an object they belong to).")
		.def_static("GetFullyQualifiedName", &dv::LandmarksPacket::GetFullyQualifiedName)
		.def_static("PacketIdentifier", &dv::LandmarksPacketIdentifier);
	py::implicitly_convertible<dv::cvector<dv::Landmark>, dv::LandmarksPacket>();

	py::class_<dv::TimedKeyPoint>(m, "TimedKeyPoint")
		.def(py::init<const dv::Point2f &, float, float, float, int32_t, int32_t, int64_t>(), py::arg("pt"),
			py::arg("size"), py::arg("angle"), py::arg("response"), py::arg("octave"), py::arg("class_id"),
			py::arg("timestamp"))
		.def_readwrite("pt", &dv::TimedKeyPoint::pt, "Coordinates of the keypoint.")
		.def_readwrite("size", &dv::TimedKeyPoint::size, "Diameter of the meaningful keypoint neighborhood.")
		.def_readwrite("angle", &dv::TimedKeyPoint::angle,
			"computed orientation of the keypoint (-1 if not applicable); it's in [0,360) degrees and measured "
			"relative to image coordinate system, ie in clockwise.")
		.def_readwrite("response", &dv::TimedKeyPoint::response,
			"The response by which the most strong keypoints have been selected. Can be used for the further sorting "
			"or subsampling.")
		.def_readwrite(
			"octave", &dv::TimedKeyPoint::octave, "Octave (pyramid layer) from which the keypoint has been extracted.")
		.def_readwrite("class_id", &dv::TimedKeyPoint::class_id,
			"Object class (if the keypoints need to be clustered by an object they belong to).")
		.def_readwrite("timestamp", &dv::TimedKeyPoint::timestamp, "Timestamp (µs).")
		.def(
			"time",
			[](const dv::TimedKeyPoint &self) {
				return dv::toTimePoint(self.timestamp);
			},
			"Timestamp of the captured data, compatible with python time library");

	py::class_<dv::TimedKeyPointPacket> timedKeypointPacketClass(m, "TimedKeyPointPacket");
	py::bind_vector<dv::cvector<dv::TimedKeyPoint>>(timedKeypointPacketClass, "TimedKeyPointVector");
	timedKeypointPacketClass.def(py::init<>())
		.def(py::init<const dv::cvector<dv::TimedKeyPoint> &>(), "timedKeypoints"_a,
			"Create a packet from a list of timed keypoints")
		.def_readwrite("elements", &dv::TimedKeyPointPacket::elements)
		.def_static("GetFullyQualifiedName", &dv::TimedKeyPointPacket::GetFullyQualifiedName)
		.def_static("PacketIdentifier", &dv::TimedKeyPointPacketIdentifier);
	py::implicitly_convertible<dv::cvector<dv::TimedKeyPoint>, dv::TimedKeyPointPacket>();

	{
		py::enum_<dv::TriggerType> triggerType(m, "TriggerType");
		for (auto enumValue = static_cast<int8_t>(dv::TriggerType::MIN);
			 static_cast<dv::TriggerType>(enumValue) <= dv::TriggerType::MAX; enumValue++) {
			auto currentEnum = static_cast<dv::TriggerType>(enumValue);
			triggerType.value(dv::EnumNameTriggerType(currentEnum), currentEnum);
		}
		triggerType.export_values();
	}

	py::class_<dv::Trigger>(m, "Trigger", "Trigger event received by the camera.")
		.def(py::init<>())
		.def(py::init<int64_t, dv::TriggerType>(), py::arg("timestamp"), py::arg("type"))
		.def_readwrite("timestamp", &dv::Trigger::timestamp, "Timestamp (µs).")
		.def_readwrite("type", &dv::Trigger::type, "Type of trigger that occurred.")
		.def(
			"time",
			[](const dv::Trigger &self) {
				return dv::toTimePoint(self.timestamp);
			},
			"Timestamp of the captured data, compatible with python time library");

	py::class_<dv::TriggerPacket> triggerPacketClass(m, "TriggerPacket");
	py::bind_vector<dv::cvector<dv::Trigger>>(triggerPacketClass, "TriggersVector");
	triggerPacketClass.def(py::init<>())
		.def(py::init<const dv::cvector<dv::Trigger> &>(), "triggers"_a, "Create a packet from a list of triggers")
		.def_readwrite("elements", &dv::TriggerPacket::elements)
		.def_static("GetFullyQualifiedName", &dv::TriggerPacket::GetFullyQualifiedName)
		.def_static("PacketIdentifier", &dv::TriggerPacketIdentifier);
	py::implicitly_convertible<dv::cvector<dv::Trigger>, dv::TriggerPacket>();

	py::class_<dv::DepthEvent>(m, "DepthEvent")
		.def(py::init<const int64_t, int16_t, int16_t, bool, uint16_t>(), py::arg("timestamp"), py::arg("x"),
			py::arg("y"), py::arg("polarity"), py::arg("depth"))
		.def("timestamp", &dv::DepthEvent::timestamp, "Microsecond timestamp of the captured event")
		.def("x", &dv::DepthEvent::x, "X coordinate")
		.def("y", &dv::DepthEvent::y, "Y coordinate")
		.def("polarity", &dv::DepthEvent::polarity,
			"Polarity value, True for increase of brightness, False for decrease")
		.def("depth", &dv::DepthEvent::depth, "Depth value in millimeters")
		.def(
			"time",
			[](const dv::DepthEvent &self) {
				return dv::toTimePoint(self.timestamp());
			},
			"Timestamp of the captured data, compatible with python time library");

	auto depthEventPacketClass = py::class_<dv::DepthEventPacket>(m, "DepthEventPacket");
	py::bind_vector<dv::cvector<dv::DepthEvent>>(depthEventPacketClass, "DepthEventVector");
	depthEventPacketClass.def(py::init<>())
		.def(py::init<const dv::cvector<dv::DepthEvent> &>(), "depthEvents"_a,
			"Create a depth event packet from list of depth events")
		.def_readwrite("elements", &dv::DepthEventPacket::elements)
		.def("numpy",
			[](const dv::DepthEventPacket &self) {
				py::list names;
				names.append("timestamp");
				names.append("x");
				names.append("y");
				names.append("polarity");
				names.append("depth");
				py::list formats;
				formats.append("<i8");
				formats.append("<i2");
				formats.append("<i2");
				formats.append("<i1");
				formats.append("<u2");
				py::list offsets;
				offsets.append(0);
				offsets.append(8);
				offsets.append(10);
				offsets.append(12);
				offsets.append(14);
				return py::array(
					py::dtype(names, formats, offsets, sizeof(dv::Event)), self.elements.size(), self.elements.data());
			})
		.def_static("GetFullyQualifiedName", &dv::DepthEventPacket::GetFullyQualifiedName)
		.def_static("PacketIdentifier", &dv::DepthEventPacketIdentifier);
	py::implicitly_convertible<dv::cvector<dv::DepthEvent>, dv::DepthEventPacket>();

	py::class_<dv::DepthFrame>(m, "DepthFrame")
		.def(py::init())
		.def(py::init<int64_t, int16_t, int16_t, uint16_t, uint16_t, uint16_t, const dv::cvector<uint16_t> &>(),
			"timestamp"_a, "sizeX"_a, "sizeY"_a, "minDepth"_a, "maxDepth"_a, "step"_a, "depth"_a)
		.def_readwrite("timestamp", &dv::DepthFrame::timestamp, "Microsecond timestamp of the depth frame")
		.def_readwrite("sizeX", &dv::DepthFrame::sizeX, "Width of the frame")
		.def_readwrite("sizeY", &dv::DepthFrame::sizeY, "Height of the frame")
		.def_readwrite("minDepth", &dv::DepthFrame::minDepth, "Minimum measurable depth")
		.def_readwrite("maxDepth", &dv::DepthFrame::maxDepth, "Maximum measurable depth")
		.def_readwrite("step", &dv::DepthFrame::step, "Depth measurement step in millimeters, usually 1mm")
		.def_readwrite("depth", &dv::DepthFrame::depth, "Actual depth values")
		.def(
			"time",
			[](const dv::DepthFrame &self) {
				return dv::toTimePoint(self.timestamp);
			},
			"Timestamp of the captured data, compatible with python time library");

	py::class_<dv::BoundingBox>(m, "BoundingBox")
		.def(py::init<int64_t, float, float, float, float, float, const dv::cstring &>(), py::arg("timestamp"),
			py::arg("topLeftX"), py::arg("topLeftY"), py::arg("bottomRightX"), py::arg("bottomRightY"),
			py::arg("confidence"), py::arg("label"))
		.def_readwrite("timestamp", &dv::BoundingBox::timestamp, "Timestamp (µs).")
		.def_readwrite("topLeftX", &dv::BoundingBox::topLeftX, "Top left corner of bounding box x-coordinate.")
		.def_readwrite("topLeftY", &dv::BoundingBox::topLeftY, "Top left corner of bounding box y-coordinate.")
		.def_readwrite(
			"bottomRightX", &dv::BoundingBox::bottomRightX, "Bottom right corner of bounding box x-coordinate.")
		.def_readwrite(
			"bottomRightY", &dv::BoundingBox::bottomRightY, "Bottom right corner of bounding box y-coordinate.")
		.def_readwrite("confidence", &dv::BoundingBox::confidence, "Confidence of the given bounding box.")
		.def_readwrite("label", &dv::BoundingBox::label, "Label for the given bounding box.");

	py::class_<dv::BoundingBoxPacket> boundingBoxPacketClass(m, "BoundingBoxPacket");
	py::bind_vector<dv::cvector<dv::BoundingBox>>(boundingBoxPacketClass, "BoundingBoxVector");
	boundingBoxPacketClass.def(py::init<>())
		.def(py::init<const dv::cvector<dv::BoundingBox> &>(), "boundingBoxes"_a,
			"Create a packet from a list of bounding boxes")
		.def_readwrite("elements", &dv::BoundingBoxPacket::elements)
		.def_static("GetFullyQualifiedName", &dv::BoundingBoxPacket::GetFullyQualifiedName)
		.def_static("PacketIdentifier", &dv::BoundingBoxPacketIdentifier);
	py::implicitly_convertible<dv::cvector<dv::BoundingBox>, dv::BoundingBoxPacket>();

	py::class_<dv::EventStore>(m, "EventStore")
		.def(py::init<>())
		.def(
			"push_back",
			[](dv::EventStore &self, const py::tuple &t) {
				if (py::len(t) != 4) {
					throw std::runtime_error("dv::EventStore requires a four element tuple representing an event in a "
											 "format (timestamp, x, y, polarity)");
				}
				self.emplace_back(t[0].cast<int64_t>(), t[1].cast<int16_t>(), t[2].cast<int16_t>(), t[3].cast<bool>());
			},
			py::arg("event"))
		.def("size", &dv::EventStore::size, "Get the numbers of stored events.")
		.def("getHighestTime", &dv::EventStore::getHighestTime, "Highest timestamp stored.")
		.def("getLowestTime", &dv::EventStore::getLowestTime, "Lowest timestamp stored.")
		.def("at", &dv::EventStore::at, py::arg("index"), "Get an event at given index.")
		.def("isEmpty", &dv::EventStore::isEmpty, "True if the event store is empty, False otherwise.")
		.def("rate", &dv::EventStore::rate,
			"Get the event rate (events per second) for the events stored in this storage.")
		.def("retainDuration", &dv::EventStore::retainDuration, "duration"_a,
			"Retain a given duration of events, calculating the duration from the latest available event.")
		.def("sliceBack", &dv::EventStore::sliceBack, "length"_a,
			"Returns a slice which contains events from the back of the storage, it will contain no more events than "
			"given length variable.")
		.def("sliceRate", &dv::EventStore::sliceRate, "targetRate"_a,
			"Slices events from back of the EventStore, so that the EventStore would only contain given events of a "
			"given target rate.")
		.def(
			"slice",
			[](const dv::EventStore &self, const size_t start) {
				return self.slice(start);
			},
			py::arg("start"), "Forward slice of events from a given index.")
		.def(
			"slice",
			[](const dv::EventStore &self, const size_t start, const size_t length) {
				return self.slice(start, length);
			},
			py::arg("start"), py::arg("length"),
			"Returns a new EventStore which is a shallow representation of a slice of this EventStore. The slice is "
			"from `start` (number of events, minimum 0, maximum `size()`) and has a length of `length`.")
		.def(
			"add",
			[](dv::EventStore &self, const dv::EventStore &other) {
				return self.add(other);
			},
			py::arg("other"), "Adds all the events of the other event store to this event store.")
		.def(
			"push_back",
			[](dv::EventStore &self, const int64_t timestamp, const int16_t x, const int16_t y, const bool polarity) {
				return self.emplace_back(timestamp, x, y, polarity);
			},
			py::arg("timestamp"), py::arg("x"), py::arg("y"), py::arg("polarity"),
			"Adds the given event to the end of this EventStore.")
		.def(
			"push_back",
			[](dv::EventStore &self, const dv::Event &event) {
				return self.push_back(event);
			},
			"event"_a, "Adds the given event to the end of this EventStore.")
		.def("erase", &dv::EventStore::erase, py::arg("start"), py::arg("length"),
			"Erase given range of events from the event store. This does not necessarily delete the underlying data "
			"since event store maps the data using smart pointers, the data will be cleared only in the case "
			"that none of the stores is mapping the data. This erase function does not affect data shared with other "
			"event stores.")
		.def("eraseTime", &dv::EventStore::eraseTime, py::arg("startTime"), py::arg("endTime"),
			"Erase events in the range between given timestamps. This does not necessarily delete the underlying data "
			"since event store maps the data using smart pointers, the data will be cleared only in the case "
			"that none of the stores is mapping the data. This erase function does not affect data shared with other "
			"event stores.")
		.def("front", &dv::EventStore::front, "Returns the first element of the store")
		.def("back", &dv::EventStore::back, "Returns the last element of the store")
		.def("duration", &dv::EventStore::duration,
			"Return a timedelta representing the duration of events contained in this store.")
		.def("isWithinStoreTimeRange", &dv::EventStore::isWithinStoreTimeRange, py::arg("timestamp"),
			"Checks whether given timestamp is within the time range of the event store.")
		.def(
			"sliceTime",
			[](const dv::EventStore &self, const int64_t startTime, const int64_t endTime) -> dv::EventStore {
				return self.sliceTime(startTime, endTime);
			},
			py::arg("startTime"), py::arg("endTime"),
			"Returns a new EventStore which is a shallow representation of a slice of this EventStore. The slice is "
			"from a specific startTime (in event timestamps, microseconds) to a specific endTime (event timestamps, "
			"microseconds). The actual size (in events) of the resulting packet depends on the event rate in the "
			"requested time interval. The resulting packet may be empty, if there is no event that happend in the "
			"requested interval.")
		.def(
			"sliceTime",
			[](const dv::EventStore &self, const int64_t startTime) -> dv::EventStore {
				return self.sliceTime(startTime);
			},
			py::arg("startTime"),
			"Returns a new EventStore which is a shallow representation of a slice of this EventStore. The slice is "
			"from a specific startTime (in event timestamps, microseconds) to the end of the packet. The actual size "
			"(in events) of the resulting packet depends on the event rate in the requested time interval. The "
			"resulting packet may be empty, if there is no event that happened in the requested interval.")
		.def("timestamps", &dv::EventStore::timestamps,
			"Retrieve timestamps of events into a one-dimensional eigen matrix. This performs a copy of the values. "
			"The values are guaranteed to be monotonically increasing.")
		.def("coordinates", &dv::EventStore::coordinates,
			"Retrieve coordinates of events in a 2xN eigen matrix. Method performs a copy of the values. Coordinates "
			"maintain the same order as within the event store. First column is the x coordinate, second column is the "
			"y coordinate.")
		.def("polarities", &dv::EventStore::polarities,
			"Retrieve polarities of events in a one-dimensional eigen matrix. Method performs a copy of the values. "
			"Polarities maintain the same order as within the event store. Polarities are converted into unsigned "
			"8-bit integer values, where 0 stands for negative polarity event and 1 stands for positive polarity "
			"event.")
		.def(
			"numpy",
			[](const dv::EventStore &self) {
				// Convert into a continuous memory buffer
				auto *events = new dv::EventPacket(self.toPacket());

				// Deallocator
				py::capsule deallocate(events, [](void *ptr) {
					delete reinterpret_cast<dv::EventPacket *>(ptr);
				});

				py::list names;
				names.append("timestamp");
				names.append("x");
				names.append("y");
				names.append("polarity");
				py::list formats;
				formats.append("<i8");
				formats.append("<i2");
				formats.append("<i2");
				formats.append("<i1");
				py::list offsets;
				offsets.append(0);
				offsets.append(8);
				offsets.append(10);
				offsets.append(12);
				return py::array(py::dtype(names, formats, offsets, sizeof(dv::Event)), events->elements.size(),
					events->elements.data(), deallocate);
			},
			"Convert the event store into a array with named columns.")
		.def("toPacket", &dv::EventStore::toPacket,
			"Convert event store into a continuous memory packet. This performs a deep copy of underlying data.")
		.def("__len__", &dv::EventStore::size)
		.def("__getitem__", &dv::EventStore::at)
		.def("__repr__",
			[](const dv::EventStore &self) {
				std::stringstream ss;
				ss << self;
				return ss.str();
			})
		.def(
			"__iter__",
			[](const dv::EventStore &self) {
				return py::make_iterator(self.begin(), self.end());
			},
			py::keep_alive<0, 1>());

	py::class_<dv::DepthEventStore>(m, "DepthEventStore")
		.def(py::init<>())
		.def(
			"push_back",
			[](dv::DepthEventStore &self, const py::tuple &t) {
				if (py::len(t) != 5) {
					throw std::runtime_error(
						"dv::DepthEventStore requires a five element tuple representing an event in a "
						"format (timestamp, x, y, polarity, depth)");
				}
				self.emplace_back(t[0].cast<int64_t>(), t[1].cast<int16_t>(), t[2].cast<int16_t>(), t[3].cast<bool>(),
					t[4].cast<uint16_t>());
			},
			py::arg("event"))
		.def("size", &dv::DepthEventStore::size, "Get the numbers of stored events.")
		.def("getHighestTime", &dv::DepthEventStore::getHighestTime, "Highest timestamp stored.")
		.def("getLowestTime", &dv::DepthEventStore::getLowestTime, "Lowest timestamp stored.")
		.def("at", &dv::DepthEventStore::at, py::arg("index"), "Get an event at given index.")
		.def("isEmpty", &dv::DepthEventStore::isEmpty, "True if the event store is empty, False otherwise.")
		.def("rate", &dv::DepthEventStore::rate,
			"Get the event rate (events per second) for the events stored in this storage.")
		.def("retainDuration", &dv::DepthEventStore::retainDuration, "duration"_a,
			"Retain a given duration of events, calculating the duration from the latest available event.")
		.def("sliceBack", &dv::DepthEventStore::sliceBack)
		.def(
			"slice",
			[](const dv::DepthEventStore &self, const size_t start) {
				return self.slice(start);
			},
			py::arg("start"), "Forward slice of events from a given index.")
		.def(
			"slice",
			[](const dv::DepthEventStore &self, const size_t start, const size_t length) {
				return self.slice(start, length);
			},
			py::arg("start"), py::arg("length"),
			"Returns a new DepthEventStore which is a shallow representation of a slice of this DepthEventStore. The "
			"slice is "
			"from `start` (number of events, minimum 0, maximum `size()`) and has a length of `length`.")
		.def(
			"add",
			[](dv::DepthEventStore &self, const dv::DepthEventStore &other) {
				return self.add(other);
			},
			py::arg("other"), "Adds all the events of the other event store to this event store.")
		.def(
			"push_back",
			[](dv::DepthEventStore &self, const int64_t timestamp, const int16_t x, const int16_t y,
				const bool polarity, const uint16_t depth) {
				return self.emplace_back(timestamp, x, y, polarity, depth);
			},
			py::arg("timestamp"), py::arg("x"), py::arg("y"), py::arg("polarity"), py::arg("depth"),
			"Adds the given event to the end of this EventStore.")
		.def(
			"push_back",
			[](dv::DepthEventStore &self, const dv::DepthEvent &event) {
				return self.push_back(event);
			},
			"event"_a, "Adds the given event to the end of this EventStore.")
		.def("erase", &dv::DepthEventStore::erase, py::arg("start"), py::arg("length"),
			"Erase given range of events from the event store. This does not necessarily delete the underlying data "
			"since event store maps the data using smart pointers, the data will be cleared only in the case "
			"that none of the stores is mapping the data. This erase function does not affect data shared with other "
			"event stores.")
		.def("eraseTime", &dv::DepthEventStore::eraseTime, py::arg("startTime"), py::arg("endTime"),
			"Erase events in the range between given timestamps. This does not necessarily delete the underlying data "
			"since event store maps the data using smart pointers, the data will be cleared only in the case "
			"that none of the stores is mapping the data. This erase function does not affect data shared with other "
			"event stores.")
		.def("front", &dv::DepthEventStore::front, "Returns the first element of the store")
		.def("back", &dv::DepthEventStore::back, "Returns the last element of the store")
		.def("duration", &dv::DepthEventStore::duration,
			"Return a timedelta representing the duration of events contained in this store.")
		.def("isWithinStoreTimeRange", &dv::DepthEventStore::isWithinStoreTimeRange, py::arg("timestamp"),
			"Checks whether given timestamp is within the time range of the event store.")
		.def(
			"sliceTime",
			[](const dv::DepthEventStore &self, const int64_t startTime, const int64_t endTime) -> dv::DepthEventStore {
				return self.sliceTime(startTime, endTime);
			},
			py::arg("startTime"), py::arg("endTime"),
			"Returns a new DepthEventStore which is a shallow representation of a slice of this DepthEventStore. The "
			"slice is "
			"from a specific startTime (in event timestamps, microseconds) to a specific endTime (event timestamps, "
			"microseconds). The actual size (in events) of the resulting packet depends on the event rate in the "
			"requested time interval. The resulting packet may be empty, if there is no event that happend in the "
			"requested interval.")
		.def(
			"sliceTime",
			[](const dv::DepthEventStore &self, const int64_t startTime) -> dv::DepthEventStore {
				return self.sliceTime(startTime);
			},
			py::arg("startTime"),
			"Returns a new DepthEventStore which is a shallow representation of a slice of this DepthEventStore. The "
			"slice is "
			"from a specific startTime (in event timestamps, microseconds) to the end of the packet. The actual size "
			"(in events) of the resulting packet depends on the event rate in the requested time interval. The "
			"resulting packet may be empty, if there is no event that happened in the requested interval.")
		.def(
			"numpy",
			[](const dv::DepthEventStore &self) {
				// Convert into a continuous memory buffer
				auto *events = new dv::DepthEventPacket(self.toPacket());

				// Deallocator
				py::capsule deallocate(events, [](void *ptr) {
					delete reinterpret_cast<dv::DepthEventPacket *>(ptr);
				});

				py::list names;
				names.append("timestamp");
				names.append("x");
				names.append("y");
				names.append("polarity");
				names.append("depth");
				py::list formats;
				formats.append("<i8");
				formats.append("<i2");
				formats.append("<i2");
				formats.append("<i1");
				formats.append("<u2");
				py::list offsets;
				offsets.append(0);
				offsets.append(8);
				offsets.append(10);
				offsets.append(12);
				// Polarity 1byte, +1 byte of padding between polarity and depth
				offsets.append(14);
				return py::array(py::dtype(names, formats, offsets, sizeof(dv::DepthEvent)), events->elements.size(),
					events->elements.data(), deallocate);
			},
			"Convert the event store into a array with named columns.")
		.def("toPacket", &dv::DepthEventStore::toPacket,
			"Convert event store into a continuous memory packet. This performs a deep copy of underlying data.")
		.def("__len__", &dv::DepthEventStore::size)
		.def("__getitem__", &dv::DepthEventStore::at)
		.def("__repr__",
			[](const dv::DepthEventStore &self) {
				std::stringstream ss;
				ss << self;
				return ss.str();
			})
		.def(
			"__iter__",
			[](const dv::DepthEventStore &self) {
				return py::make_iterator(self.begin(), self.end());
			},
			py::keep_alive<0, 1>());

	auto m_data     = m.def_submodule("data");
	auto m_generate = m_data.def_submodule("generate");

	m_generate.def("sampleImage", &dv::data::generate::sampleImage, "resolution"_a,
		"Generate a sample image (single channel 8-bit unsigned integer) containing a few gray rectangles in a black "
		"background.");
	m_generate.def("eventLine", &dv::data::generate::eventLine, "timestamp"_a, "a"_a, "b"_a, "steps"_a = 0,
		"Generate events along a line between two given end-points.");
	m_generate.def("eventRectangle", &dv::data::generate::eventRectangle, "timestamp"_a, "tl"_a, "br"_a,
		"Generate events along a rectangle edges between two given top-left and bottom right points.");
	m_generate.def("eventTestSet", &dv::data::generate::eventTestSet, "timestamp"_a, "resolution"_a,
		"Generate an event test set that contains event for a few intersecting rectangle edges.");
	m_generate.def("uniformlyDistributedEvents", &dv::data::generate::uniformlyDistributedEvents, "timestamp"_a,
		"resolution"_a, "count"_a, "seed"_a = 0,
		"Generate a batch of uniformly distributed set of event within the given resolution.");
	m_generate.def("normallyDistributedEvents", &dv::data::generate::normallyDistributedEvents, "timestamp"_a,
		"center"_a, "stddev"_a, "count"_a, "seed"_a = 0,
		"Generate events normally distributed around a given center coordinates with given standard deviation.");
	m_generate.def("uniformEventsWithinTimeRange", &dv::data::generate::uniformEventsWithinTimeRange, "startTime"_a,
		"duration"_a, "resolution"_a, "count"_a, "seed"_a = 0,
		"Generate a batch of uniformly distributed (in pixel-space) randomly generated events. The timestamps are "
		"generated by monotonically increasing the timestamp within the time duration.");
	m_generate.def("dvLogo", &dv::data::generate::dvLogo, "size"_a, "colored"_a = true,
		"bgColor"_a = dv::visualization::colors::white, "pColor"_a = dv::visualization::colors::iniBlue,
		"nColor"_a = dv::visualization::colors::darkGrey,
		"Generate a DV logo using simple drawing methods. Generates in color or grayscale.");
	m_generate.def("imageToEvents", &dv::data::generate::imageToEvents, "timestamp"_a, "image"_a, "positive"_a,
		"negative"_a,
		"Convert an image into event by matching pixel intensities. The algorithm will match all pixel values "
		"available in the and match against positive and negative pixel intensity values, according events are going "
		"to be added into the output event store. Other pixel intensity values are ignored.");
	m_generate.def("dvLogoAsEvents", &dv::data::generate::dvLogoAsEvents, "timestamp"_a, "resolution"_a,
		"Generate a DV logo using simple drawing methods. Generates negative polarity events on the pixels where logo "
		"has dark pixels and positive polarity events where pixels have brighter events.");
	m_generate.def("levelImuMeasurement", &dv::data::generate::levelImuMeasurement, "timestamp"_a,
		"Generate an IMU measurement that measures a camera being on a stable and level surface. All measurement "
		"values are going to be zero, except for Y axis of accelerometer, it will measure -1.0G.");
	m_generate.def("addNoiseToImu", &dv::data::generate::addNoiseToImu, "measurement"_a, "accelerometerStddev"_a,
		"gyroscopeStddev"_a, "seed"_a = 0,
		"Apply noise to imu measurements (accelerometer and gyroscope). The noise is modelled as a normal distribution "
		"with 0 mean and given standard deviation. The modelled noise is added to the given measurement and return a "
		"new dv::IMU structure with added noise.");
	m_generate.def("levelImuWithNoise", &dv::data::generate::levelImuWithNoise, "timestamp"_a,
		"accelerometerStddev"_a = 0.1f, "gyroscopeStddev"_a = 0.1f, "seed"_a = 0,
		"Generate an IMU measurement that measures a camera being on a stable and level surface with additional "
		"measurement noise. The noise is modelled as a normal distribution with 0 mean and given standard deviation.");

	m.def("coordinateHash", &dv::coordinateHash, py::arg("x"), py::arg("y"),
		"Function that creates perfect hash for 2d coordinates.");
	m.def(
		"roiFilter",
		[](const dv::EventStore &events, const cv::Rect &roi) {
			dv::EventStore filtered;
			dv::roiFilter(events, filtered, roi);
			return filtered;
		},
		py::arg("events"), py::arg("roi"),
		"Extracts only the events that are within the defined region of interest. This function copies the events from "
		"the in EventStore into the given out EventStore, if they intersect with the given region of interest "
		"rectangle.");
	m.def(
		"maskFilter",
		[](const dv::EventStore &events, const cv::Mat &mask) {
			dv::EventStore filtered;
			dv::maskFilter(events, filtered, mask);
			return filtered;
		},
		py::arg("events"), py::arg("mask"),
		"Filter event with a coordinate mask. Discards any events that happen on coordinates where mask has a zero "
		"value and retains all events with coordinates where mask has a non-zero value.");
	m.def(
		"scale",
		[](const dv::EventStore &events, const double xDivision, const double yDivision) {
			dv::EventStore scaled;
			dv::scale(events, scaled, xDivision, yDivision);
			return scaled;
		},
		py::arg("events"), py::arg("xDivision"), py::arg("yDivision"),
		"Projects the event coordinates onto a smaller range. The x- and y-coordinates the divided by xFactor and "
		"yFactor respectively and floored to the next integer. This forms the new coordinates of the event. Due to the "
		"nature of this, it can happen that multiple events end up happening simultaneously at the same location. This "
		"is still a valid event stream, as time keeps monotonically increasing, but is something that is unlikely to "
		"be generated by an event camera.");
	m.def(
		"polarityFilter",
		[](const dv::EventStore &events, const bool polarity) {
			dv::EventStore filtered;
			dv::polarityFilter(events, filtered, polarity);
			return filtered;
		},
		py::arg("events"), py::arg("polarity"),
		"Filters events by polarity. Only events that exhibit the same polarity as given in polarity are kept.");
	m.def("boundingRect", &dv::boundingRect<dv::EventStore>, py::arg("events"),
		"Computes and returns a rectangle with dimensions such that all the events in the given `EventStore` fall into "
		"the bounding box.");
	m.def("now", &dv::now, "Returns current system clock time in unix microsecond format");
	m.def("isWithinDimensions", &dv::isWithinDimensions<Eigen::Vector2d>, "point"_a, "resolution"_a,
		"Check whether given point is non-negative and within dimensions of given resolution.");
	m.def("toTimePoint", &dv::toTimePoint, "timestamp"_a,
		"Convert a 64-bit integer microsecond timestamp into a time point.");

	py::class_<dv::EventStreamSlicer>(m, "EventStreamSlicer")
		.def(py::init<>())
		.def("doEveryNumberOfEvents", &dv::EventStreamSlicer::doEveryNumberOfEvents, py::arg("n"), py::arg("callback"),
			"Adds a number-of-events triggered job to the Slicer. A job is defined by its interval and callback "
			"function. The slicer calls the callback function every `n` events, with the corresponding data. The (cpu) "
			"time interval between individual calls to the function depends on the physical event rate as well as the "
			"bulk sizes of the incoming data.")
		.def(
			"doEveryTimeInterval",
			[](dv::EventStreamSlicer &self, const dv::Duration &time, std::function<void(const dv::EventStore &)> cb) {
				return self.doEveryTimeInterval(time, std::move(cb));
			},
			py::arg("time"), py::arg("callback"),
			"Adds an event-timestamp-interval triggered job to the Slicer. A job is defined by its interval and "
			"callback function. The slicer calls the callback whenever the timestamp difference of an incoming event "
			"to the last time the function was called is bigger than the interval. As the timing is based on event "
			"times rather than CPU time, the actual time periods are not guranteed, especially with a low event rate. "
			"The (cpu) time interval between individual calls to the function depends on the physical event rate as "
			"well as the bulk sizes of the incoming data.")
		.def("hasJob", &dv::EventStreamSlicer::hasJob, py::arg("job_id"),
			"Returns true if the slicer contains the slicejob with the provided id.")
		.def("removeJob", &dv::EventStreamSlicer::removeJob, py::arg("job_id"),
			"Removes the given job from the list of current jobs.")
		.def(
			"modifyTimeInterval",
			[](dv::EventStreamSlicer &self, const int jobId, const dv::Duration &time) {
				self.modifyTimeInterval(jobId, time);
			},
			py::arg("job_id"), py::arg("timeInterval"),
			"Modifies the time interval of the supplied job to the requested value.")
		.def(
			"modifyTimeInterval",
			[](dv::EventStreamSlicer &self, const int jobId, const int64_t time) {
				self.modifyTimeInterval(jobId, dv::Duration(time));
			},
			py::arg("job_id"), py::arg("timeInterval"),
			"Modifies the time interval of the supplied job to the requested value.")
		.def("modifyNumberInterval", &dv::EventStreamSlicer::modifyNumberInterval, py::arg("job_id"),
			py::arg("numberInterval"), "Modifies the number interval of the supplied job to the requested value.")
		.def(
			"accept",
			[](dv::EventStreamSlicer &self, const dv::EventStore &events) {
				self.accept(events);
			},
			py::arg("events"), "Adds full EventStore to the buffer and evaluates jobs.")
		.def(
			"accept",
			[](dv::EventStreamSlicer &self, const dv::Event &event) {
				self.accept(event);
			},
			py::arg("event"), "Adds a single event to the slicer buffer and evaluate jobs.");

	py::class_<dv::StereoEventStreamSlicer>(m, "StereoEventStreamSlicer")
		.def(py::init())
		.def("accept", &dv::StereoEventStreamSlicer::accept, "leftEvents"_a, "rightEvents"_a,
			"Adds EventStores from the left and right camera. Performs job evaluation immediately.")
		.def("doEveryTimeInterval", &dv::StereoEventStreamSlicer::doEveryTimeInterval, "interval"_a, "callback"_a,
			"Perform an action on the stereo stream data every given time interval. Event period is evaluated on the "
			"left camera stream and according time interval of data is sliced from the right camera event stream. "
			"Sliced data is passed into the callback function as soon as it arrived, first argument is left camera "
			"events and second is right camera events.")
		.def("doEveryNumberOfEvents", &dv::StereoEventStreamSlicer::doEveryNumberOfEvents, "n"_a, "callback"_a,
			"Perform an action on the stereo stream data every given amount of events. Event count is evaluated on the "
			"left camera stream and according time interval of data is sliced from the right camera event stream. "
			"Sliced data is passed into the callback function as soon as it arrived, first argument is left camera "
			"events and second is right camera events. Since right camera events are sliced by the time interval of "
			"left camera, the amount of events on right camera can be different.")
		.def("hasJob", &dv::StereoEventStreamSlicer::hasJob, "jobId"_a,
			"Returns true if the slicer contains the slicejob with the provided id")
		.def("removeJob", &dv::StereoEventStreamSlicer::removeJob, "jobId"_a,
			"Removes the given job from the list of current jobs.");

	py::class_<dv::MultiStreamSlicer<dv::EventStore>> multiStreamSlicer(m, "EventMultiStreamSlicer",
		"MultiStreamSlicer takes multiple stream of timestamped data, slices data with configured intervals and calls "
		"a given callback method on each interval. It is an extension of StreamSlicer class that can synchronously "
		"slice multiple streams. Each stream has to be named uniquely, the name is carried over to the callback method "
		"to identify each stream. The class is relies heavily on templating, so it supports different containers of "
		"data, as long as the container is an iterable and each element contains an accessible timestamp in "
		"microsecond format. The slicing is driven by the main stream, which needs to be specified during construction "
		"time, a type of the main stream is the first template argument and the name for the main stream is provided "
		"as the constructor first argument. By default, four types are supported without additional configuration: "
		"dv::EventStore, dv::TriggerPacket, dv::IMUPacket, dv::cvector . Additional types can be added by specifying "
		"them as additional template parameters.");

	py::enum_<dv::TimeSlicingApproach>(m, "TimeSlicingApproach")
		.value("Backward", dv::TimeSlicingApproach::BACKWARD)
		.value("Forward", dv::TimeSlicingApproach::FORWARD)
		.export_values();

	py::class_<dv::MultiStreamSlicer<dv::EventStore>::MapOfVariants>(multiStreamSlicer, "MapOfVariants")
		.def(
			"getEvents",
			[](dv::MultiStreamSlicer<dv::EventStore>::MapOfVariants &self, const std::string &streamName) {
				return self.template get<dv::EventStore>(streamName);
			},
			"streamName"_a, "Get events of a given stream name")
		.def(
			"getImu",
			[](dv::MultiStreamSlicer<dv::EventStore>::MapOfVariants &self, const std::string &streamName) {
				return self.template get<dv::IMUPacket>(streamName);
			},
			"streamName"_a, "Get imu of a given stream name")
		.def(
			"getTriggers",
			[](dv::MultiStreamSlicer<dv::EventStore>::MapOfVariants &self, const std::string &streamName) {
				return self.template get<dv::TriggerPacket>(streamName);
			},
			"streamName"_a, "Get triggers of a given stream name")
		.def(
			"getFrames",
			[](dv::MultiStreamSlicer<dv::EventStore>::MapOfVariants &self, const std::string &streamName) {
				return self.template get<dv::cvector<dv::Frame>>(streamName);
			},
			"streamName"_a, "Get frames of a given stream name");

	multiStreamSlicer
		.def(py::init<const std::string &>(), "streamName"_a,
			"Initialize a multi-stream slicer by providing a name for main stream")
		.def(
			"accept",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const std::string &streamName,
				const dv::EventStore &events) {
				self.accept(streamName, events);
			},
			"streamName"_a, "events"_a, "Pass data of a stream into the slicer")
		.def(
			"accept",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const std::string &streamName, const dv::IMUPacket &imu) {
				self.accept(streamName, imu);
			},
			"streamName"_a, "imu"_a, "Pass data of a stream into the slicer")
		.def(
			"accept",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const std::string &streamName,
				const dv::TriggerPacket &triggers) {
				self.accept(streamName, triggers);
			},
			"streamName"_a, "triggers"_a, "Pass data of a stream into the slicer")
		.def(
			"accept",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const std::string &streamName,
				const dv::cvector<dv::Frame> &frames) {
				self.accept(streamName, frames);
			},
			"streamName"_a, "frames"_a, "Pass data of a stream into the slicer")
		.def(
			"addEventStream",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const std::string &streamName) {
				self.addStream<dv::EventStore>(streamName);
			},
			"streamName"_a, "Add an event stream to the slicer.")
		.def(
			"addImuStream",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const std::string &streamName) {
				self.addStream<dv::IMUPacket>(streamName);
			},
			"streamName"_a, "Add an imu stream to the slicer.")
		.def(
			"addTriggerStream",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const std::string &streamName) {
				self.addStream<dv::TriggerPacket>(streamName);
			},
			"streamName"_a, "Add a trigger stream to the slicer.")
		.def(
			"addFrameStream",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const std::string &streamName) {
				self.addStream<dv::cvector<dv::Frame>>(streamName);
			},
			"streamName"_a, "Add a frame stream to the slicer.")
		.def(
			"doEveryTimeInterval",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const dv::Duration interval,
				std::function<void(const dv::MultiStreamSlicer<dv::EventStore>::MapOfVariants &)> callback) {
				return self.doEveryTimeInterval(interval, std::move(callback));
			},
			"interval"_a, "callback"_a,
			"Register a callback to be performed at a given interval. Data is passed as an argument to the method.")
		.def(
			"modifyTimeInterval",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const int jobId, const dv::Duration timeInterval) {
				self.modifyTimeInterval(jobId, timeInterval);
			},
			"jobId"_a, "timeInterval"_a, "Modify the execution interval of a job.")
		.def(
			"doEveryNumberOfElements",
			[](dv::MultiStreamSlicer<dv::EventStore> &self, const size_t n,
				std::function<void(const dv::MultiStreamSlicer<dv::EventStore>::MapOfVariants &)> callback,
				const dv::TimeSlicingApproach sliceTiming) {
				return self.doEveryNumberOfElements(n, std::move(callback), sliceTiming);
			},
			"n"_a, "callback"_a, "sliceTiming"_a = dv::TimeSlicingApproach::BACKWARD,
			"Register a callback to be performed every number of elements. Data is passed as an argument to the "
			"method.")
		.def("modifyNumberInterval", &dv::MultiStreamSlicer<dv::EventStore>::modifyNumberInterval, "jobId"_a, "n"_a,
			"Modify the number of elements to slice for a job.")
		.def("hasJob", &dv::MultiStreamSlicer<dv::EventStore>::hasJob, "jobId"_a,
			"Returns true if the slicer contains the slice-job with the provided id.")
		.def("removeJob", &dv::MultiStreamSlicer<dv::EventStore>::removeJob, "jobId"_a,
			"Removes the given job from the list of current jobs.");

	py::class_<dv::TimeSurface>(m, "TimeSurface")
		.def(py::init<>(), "Dummy constructor Constructs a new, empty TimeSurface without any data allocated to it.")
		.def(py::init<uint32_t, uint32_t>(), py::arg("rows"), py::arg("cols"),
			"Creates a new TimeSurface with the given size. The data is zero initialized")
		.def(py::init<const cv::Size &>(), py::arg("resolution"),
			"Creates a new TimeSurface of the given size. The data is zero initialized.")
		.def(
			"empty",
			[](const dv::TimeSurface &self) {
				PyErr_WarnEx(PyExc_DeprecationWarning, "empty() is deprecated, use isEmpty() instead.", 1);
				return self.isEmpty();
			},
			"Returns true if the TimeSurface has zero size. In this case, it was not allocated with a size.")
		.def("isEmpty", &dv::TimeSurface::isEmpty,
			"Returns true if the TimeSurface has zero size. In this case, it was not allocated with a size.")
		.def("cols", &dv::TimeSurface::cols, "Returns the number of columns of the TimeSurface")
		.def("rows", &dv::TimeSurface::rows, "Returns the number of rows of the TimeSurface")
		.def("size", &dv::TimeSurface::size, "The size of the TimeSurface.")
		.def("reset", &dv::TimeSurface::reset, "Sets all values in the time surface to zero.")
		.def("generateFrame", &dv::TimeSurface::generateFrame,
			"Generates a frame from the data contained in the event store")
		.def(
			"at",
			[](const dv::TimeSurface &self, const int16_t y, const int16_t x) -> int64_t {
				return self(y, x);
			},
			py::arg("y"), py::arg("x"), "Returns a value of the element at the given coordinates.")
		.def(
			"accept",
			[](dv::TimeSurface &self, const dv::EventStore &events) {
				return self.accept(events);
			},
			py::arg("events"), "Inserts the event store into the time surface.")
		.def(
			"accept",
			[](dv::TimeSurface &self, const dv::Event &event) {
				return self.accept(event);
			},
			py::arg("event"), "Inserts an event into the time surface.")
		.def(
			"subtract",
			[](dv::TimeSurface &self, const int64_t value) {
				self -= value;
			},
			py::arg("value"),
			"Subtracts a constant from the TimeSurface. Values are bounds checked to 0. If the new time would become "
			"negative, it is set to 0.")
		.def(
			"add",
			[](dv::TimeSurface &self, const int64_t value) {
				self += value;
			},
			py::arg("value"),
			"Adds a constant to the time surface. Values are bounds checked to 0. If the new time would become "
			"negative, it is set to 0.")
		.def(
			"setConstant",
			[](dv::TimeSurface &self, const int64_t value) {
				self = value;
			},
			py::arg("value"),
			"Assigns constant to the TimeSurface. Values are bounds checked to 0. If the new time would become "
			"negative, it is set to 0.")
		.def(
			"__getitem__",
			[](const dv::TimeSurface &self, const int16_t y, const int16_t x) -> int64_t {
				return self(y, x);
			},
			py::arg("y"), py::arg("x"), "Returns a value of the element at the given coordinates.");

	py::class_<dv::TimeSurfaceBase<dv::EventStore, uint8_t>> m_TimeSurface8U(m, "TimeSurface8U");

	py::class_<dv::SpeedInvariantTimeSurface>(m, "SpeedInvariantTimeSurface")
		.def(py::init<const cv::Size &>(), py::arg("resolution"))
		.def("isEmpty", &dv::SpeedInvariantTimeSurface::isEmpty,
			"Returns true if the TimeSurface has zero size. In this case, it was not allocated with a size.")
		.def("cols", &dv::SpeedInvariantTimeSurface::cols, "Returns the number of columns of the TimeSurface")
		.def("rows", &dv::SpeedInvariantTimeSurface::rows, "Returns the number of rows of the TimeSurface")
		.def("size", &dv::SpeedInvariantTimeSurface::size, "The size of the TimeSurface.")
		.def("reset", &dv::SpeedInvariantTimeSurface::reset, "Sets all values in the time surface to zero.")
		.def("generateFrame", &dv::SpeedInvariantTimeSurface::generateFrame,
			"Generates a frame from the data contained in the event store")
		.def("at",
			[](const dv::SpeedInvariantTimeSurface &self, const int16_t y, const int16_t x) -> int64_t {
				return self(y, x);
			})
		.def(
			"at",
			[](const dv::SpeedInvariantTimeSurface &self, const int16_t y, const int16_t x) -> int64_t {
				return self(y, x);
			},
			py::arg("y"), py::arg("x"), "Returns a value of the element at the given coordinates.")
		.def(
			"accept",
			[](dv::SpeedInvariantTimeSurface &self, const dv::EventStore &events) {
				return self.accept(events);
			},
			py::arg("events"), "Inserts the event store into the time surface.")
		.def(
			"accept",
			[](dv::SpeedInvariantTimeSurface &self, const dv::Event &event) {
				return self.accept(event);
			},
			py::arg("event"), "Inserts an event into the time surface.")
		.def(
			"subtract",
			[](dv::SpeedInvariantTimeSurface &self, const int64_t value) {
				self -= value;
			},
			py::arg("value"),
			"Subtracts a constant from the TimeSurface. Values are bounds checked to 0. If the new time would become "
			"negative, it is set to 0.")
		.def(
			"add",
			[](dv::SpeedInvariantTimeSurface &self, const int64_t value) {
				self += value;
			},
			py::arg("value"),
			"Adds a constant to the time surface. Values are bounds checked to 0. If the new time would become "
			"negative, it is set to 0.")
		.def(
			"__getitem__",
			[](const dv::SpeedInvariantTimeSurface &self, const int16_t y, const int16_t x) -> int64_t {
				return self(y, x);
			},
			py::arg("y"), py::arg("x"), "Returns a value of the element at the given coordinates.");

	py::class_<dv::EdgeMapAccumulator>(m, "EdgeMapAccumulator",
		"EdgeMapAccumulator accumulates events in a histogram representation with configurable contribution, but it is "
		"more efficient compared to generic accumulator since it uses 8-bit unsigned integers as internal memory type. "
		"The EdgeMapAccumulator behaves the same as a generic `dv::Accumulator` with STEP decay function, neutral and "
		"minimum value of 0.0, maximum value of 1.0 and configurable event contribution. The difference is that it "
		"doesn't use floating point numbers for the potential surface representation. The output data type of this "
		"accumulator is single channel 8-bit unsigned integer (CV_8UC1). Accumulation is performed using integer "
		"operations as well.")
		.def(py::init<const cv::Size &, const float, const bool, const float, const float>(), "resolution"_a,
			"contribution"_a = 0.25f, "ignorePolarity"_a = true, "neutralValue"_a = 0.f, "decay"_a = 1.f)
		.def("setIgnorePolarity", &dv::EdgeMapAccumulator::setIgnorePolarity, "ignorePolarity"_a,
			"Set ignore polarity option. All events are considered positive if enabled.")
		.def("isIgnorePolarity", &dv::EdgeMapAccumulator::isIgnorePolarity,
			"Check whether ignore polarity option is set to true.")
		.def("getNeutralValue", &dv::EdgeMapAccumulator::getNeutralValue,
			"Get the neutral potential value for the accumulator. The range for potential value is [0.0; 1.0], where "
			"1.0 stands for maximum possible potential - 255 in 8-bit pixel representation.")
		.def("setNeutralValue", &dv::EdgeMapAccumulator::setNeutralValue, "neutralValue"_a,
			"Set the neutral potential value. The value should be in range 0.0 to 1.0, other values will be clamped to "
			"this range.")
		.def("getNeutralPotential", &dv::EdgeMapAccumulator::getNeutralPotential,
			"Get the neutral potential value for the accumulator. The range for potential value is [0.0; 1.0], where "
			"1.0 stands for maximum possible potential - 255 in 8-bit pixel representation.")
		.def("setNeutralPotential", &dv::EdgeMapAccumulator::setNeutralPotential, "neutralValue"_a,
			"Set the neutral potential value. The value should be in range 0.0 to 1.0, other values will be clamped to "
			"this range.")
		.def("getDecay", &dv::EdgeMapAccumulator::getDecay, "Get current decay value.")
		.def("setDecay", &dv::EdgeMapAccumulator::setDecay, "decay"_a,
			"Set the decay value. Decay value is clamped to range of [0.0; 1.0].")
		.def("reset", &dv::EdgeMapAccumulator::reset, "Clear the buffered events.")
		.def("setContribution", &dv::EdgeMapAccumulator::setContribution, py::arg("contribution"),
			"Set new contribution coefficient.")
		.def("getContribution", &dv::EdgeMapAccumulator::getContribution,
			"Get the contribution coefficient for a single event. The contribution value is multiplied by the maximum "
			"possible pixel value (255) to get the increment value. E.g. contribution value of 0.1 will increment a "
			"pixel value at a single event coordinates by 26.")
		.def(
			"accept",
			[](dv::EdgeMapAccumulator &self, const dv::EventStore &events) {
				self.accept(events);
			},
			py::arg("events"), "Add events to the accumulator")
		.def(
			"generateFrame",
			[](dv::EdgeMapAccumulator &self) -> dv::Frame {
				return self.generateFrame();
			},
			"Generates the accumulation frame (potential surface) at the time of the last consumed event. The function "
			"writes the output image into the given `outFrame` argument. The output frame will contain data with type "
			"CV_8UC1. The function resets any events accumulated up to this function call.");

	// New class as a temporary alias for deprecated name "PixelAccumulator"
	class PixelAccumulator : public dv::EdgeMapAccumulator {
	public:
		PixelAccumulator(const cv::Size &resolution, const float contribution, const bool ignorePolarity,
			const float neutralValue, const float decay) :
			dv::EdgeMapAccumulator(resolution, contribution, ignorePolarity, neutralValue, decay) {
		}
	};

	PixelAccumulator a(cv::Size(100, 100), 0, 0, 0, 0);

	py::class_<PixelAccumulator>(m, "PixelAccumulator",
		"EdgeMapAccumulator accumulates events in a histogram representation with configurable contribution, but it is "
		"more efficient compared to generic accumulator since it uses 8-bit unsigned integers as internal memory type. "
		"The EdgeMapAccumulator behaves the same as a generic `dv::Accumulator` with STEP decay function, neutral and "
		"minimum value of 0.0, maximum value of 1.0 and configurable event contribution. The difference is that it "
		"doesn't use floating point numbers for the potential surface representation. The output data type of this "
		"accumulator is single channel 8-bit unsigned integer (CV_8UC1). Accumulation is performed using integer "
		"operations as well.")
		.def(py::init([](const cv::Size &resolution, const float contribution, const bool ignorePolarity,
						  const float neutralValue, const float decay) {
			PyErr_WarnEx(
				PyExc_DeprecationWarning, "dv.PixelAccumulator is deprecated, use dv.EdgeMapAccumulator instead", 1);
			return PixelAccumulator(resolution, contribution, ignorePolarity, neutralValue, decay);
		}),
			"resolution"_a, "contribution"_a = 0.25f, "ignorePolarity"_a = true, "neutralValue"_a = 0.f,
			"decay"_a = 1.f)
		.def("setIgnorePolarity", &PixelAccumulator::setIgnorePolarity, "ignorePolarity"_a,
			"Set ignore polarity option. All events are considered positive if enabled.")
		.def("isIgnorePolarity", &PixelAccumulator::isIgnorePolarity,
			"Check whether ignore polarity option is set to true.")
		.def("getNeutralValue", &PixelAccumulator::getNeutralValue,
			"Get the neutral potential value for the accumulator. The range for potential value is [0.0; 1.0], where "
			"1.0 stands for maximum possible potential - 255 in 8-bit pixel representation.")
		.def("setNeutralValue", &PixelAccumulator::setNeutralValue, "neutralValue"_a,
			"Set the neutral potential value. The value should be in range 0.0 to 1.0, other values will be clamped to "
			"this range.")
		.def("getDecay", &PixelAccumulator::getDecay, "Get current decay value.")
		.def("setDecay", &PixelAccumulator::setDecay, "decay"_a,
			"Set the decay value. Decay value is clamped to range of [0.0; 1.0].")
		.def("reset", &PixelAccumulator::reset, "Clear the buffered events.")
		.def("setContribution", &PixelAccumulator::setContribution, py::arg("contribution"),
			"Set new contribution coefficient.")
		.def("getContribution", &PixelAccumulator::getContribution,
			"Get the contribution coefficient for a single event. The contribution value is multiplied by the maximum "
			"possible pixel value (255) to get the increment value. E.g. contribution value of 0.1 will increment a "
			"pixel value at a single event coordinates by 26.")
		.def(
			"accept",
			[](PixelAccumulator &self, const dv::EventStore &events) {
				self.accept(events);
			},
			py::arg("events"), "Add events to the accumulator")
		.def(
			"generateFrame",
			[](PixelAccumulator &self) -> dv::Frame {
				return self.generateFrame();
			},
			"Generates the accumulation frame (potential surface) at the time of the last consumed event. The function "
			"writes the output image into the given `outFrame` argument. The output frame will contain data with type "
			"CV_8UC1. The function resets any events accumulated up to this function call.");

	py::class_<dv::Accumulator> accumulatorClass(m, "Accumulator");

	py::enum_<dv::Accumulator::Decay>(accumulatorClass, "Decay")
		.value("NONE", dv::Accumulator::Decay::NONE)
		.value("LINEAR", dv::Accumulator::Decay::LINEAR)
		.value("EXPONENTIAL", dv::Accumulator::Decay::EXPONENTIAL)
		.value("STEP", dv::Accumulator::Decay::STEP)
		.export_values();

	accumulatorClass
		.def(py::init<const cv::Size &>(), py::arg("resolution"),
			"Default parameter constructors, applies reasonable default accumulation parameters (the same defaults as "
			"within dv-runtime module): Decay function: Exponential Decay param: 1.0e+6 Event contribution: 0.04 "
			"Minimum potential: 0.0 Neutral potential: 0.0 Maximum potential: 0.3 Synchronous decay: off Polarity "
			"rectification (all event regarded as positive): off")
		.def(py::init<const cv::Size &, dv::Accumulator::Decay, double, bool, float, float, float, float, bool>(),
			py::arg("resolution"), py::arg("decayFunction"), py::arg("decayParam"), py::arg("synchronousDecay"),
			py::arg("eventContribution"), py::arg("maxPotential"), py::arg("neutralPotential"), py::arg("minPotential"),
			py::arg("rectifyPolarity"),
			"Accumulator constructor Creates a new Accumulator with the given params. By selecting the params the "
			"right way, the Accumulator can be used for a multitude of applications. The class also provides static "
			"factory functions that adjust the parameters for common use cases.")
		.def(
			"accept",
			[](dv::Accumulator &self, const dv::EventStore &events) {
				self.accept(events);
			},
			py::arg("events"), "Add events to the accumulator")
		.def(
			"generateFrame",
			[](dv::Accumulator &self) -> dv::Frame {
				return self.generateFrame();
			},
			"Generates the accumulation frame (potential surface) at the time of the last consumed event. The function "
			"writes the output image into the given `image` argument. The output frame will contain data with type "
			"CV_8U.")
		.def("clear", &dv::Accumulator::clear,
			"Clears the potential surface by setting it to the neutral value. This function does not reset the time "
			"surface.")
		.def("setDecayFunction", &dv::Accumulator::setDecayFunction, py::arg("decayFunction"),
			"The decay function the module should use to perform the decay")
		.def("setDecayParam", &dv::Accumulator::setDecayParam, py::arg("decayParam"),
			"The decay param. This is slope for linear decay, tau for exponential decay")
		.def("setEventContribution", &dv::Accumulator::setEventContribution, py::arg("contribution"),
			"Contribution to the potential surface an event shall incur. This contribution is either counted "
			"positively (for positive events or when `rectifyPolarity` is set).")
		.def("setMinPotential", &dv::Accumulator::setMinPotential, py::arg("minPotential"),
			"Set the minimum potential at which the surface should be capped at.")
		.def("setMaxPotential", &dv::Accumulator::setMaxPotential, py::arg("maxPotential"),
			"Set the max potential at which the surface should be capped at.")
		.def("setNeutralPotential", &dv::Accumulator::setNeutralPotential, py::arg("neutralPotential"),
			"Set the neutral potential to which the decay function should go. Exponential decay always goes to 0. The "
			"parameter is ignored there.")
		.def(
			"setRectifyPolarity",
			[](dv::Accumulator &self, bool rectifyPolarity) {
				PyErr_WarnEx(
					PyExc_DeprecationWarning, "setRectifyPolarity() is deprecated, use setIgnorePolarity() instead", 1);
				self.setIgnorePolarity(rectifyPolarity);
			},
			py::arg("rectifyPolarity"),
			"If set to true, all events will incur a positive contribution to the potential surface.")
		.def("setIgnorePolarity", &dv::Accumulator::setIgnorePolarity, py::arg("ignorePolarity"),
			"If set to true, all events will incur a positive contribution to the potential surface.")
		.def("setSynchronousDecay", &dv::Accumulator::setSynchronousDecay, py::arg("synchronousDecay"),
			"If set to true, all valued get decayed to the frame generation time at frame generation. If set to false, "
			"the values only get decayed on activity.")
		.def("getDecayFunction", &dv::Accumulator::getDecayFunction)
		.def("getDecayParam", &dv::Accumulator::getDecayParam)
		.def("getEventContribution", &dv::Accumulator::getEventContribution)
		.def("getMinPotential", &dv::Accumulator::getMinPotential)
		.def("getMaxPotential", &dv::Accumulator::getMaxPotential)
		.def("getNeutralPotential", &dv::Accumulator::getNeutralPotential)
		.def("isIgnorePolarity", &dv::Accumulator::isIgnorePolarity, "Check whether polarity of events is ignored.")
		.def("getPotentialSurface", &dv::Accumulator::getPotentialSurface,
			"Retrieve a copy of the currently accumulated potential surface");

	auto m_types = m.def_submodule("types");

	py::enum_<dv::CompressionType> compressionType(m, "CompressionType");
	for (auto enumValue = static_cast<int8_t>(dv::CompressionType::MIN);
		 static_cast<dv::CompressionType>(enumValue) <= dv::CompressionType::MAX; enumValue++) {
		auto currentEnum = static_cast<dv::CompressionType>(enumValue);
		compressionType.value(dv::EnumNameCompressionType(currentEnum), currentEnum);
	}
	compressionType.export_values();

	py::class_<dv::types::Type>(m_types, "Type", "Structure describing a DV type")
		.def(py::init<>())
		.def_readonly("id", &dv::types::Type::id);

	auto m_io = m.def_submodule("io");

	m_io.def("discoverDevices", &dv::io::discoverDevices,
		"Retrieve a list of connected cameras. The list will contain camera names, which are supported for "
		"`dv.io.CameraCapture` class.");

	py::class_<dv::io::DataReadHandler> readHandler(m_io, "DataReadHandler");

	py::class_<dv::io::Stream>(m_io, "Stream")
		.def(py::init<>())
		.def(py::init([](const int32_t id, const std::string &name, const std::string &sourceName,
						  const std::string &typeIdentifier) {
			return dv::io::Stream(id, name, sourceName, typeIdentifier);
		}),
			"id"_a, "name"_a, "sourceName"_a, "typeIdentifier"_a)
		.def("addMetadata", &dv::io::Stream::addMetadata, "name"_a, "value"_a,
			"Add metadata to the stream. If an entry already exists, it will be replaced with the new value.")
		.def("getMetadataValue", &dv::io::Stream::getMetadataValue, "name"_a, "Get a metadata value of the stream.")
		.def("setTypeDescription", &dv::io::Stream::setTypeDescription, "description"_a,
			"Set type description. This only sets type description metadata field.")
		.def("setModuleName", &dv::io::Stream::setModuleName, "moduleName"_a,
			"Set module name that originally produces the data. This only sets the original module name metadata "
			"field.")
		.def("setOutputName", &dv::io::Stream::setOutputName, "outputName"_a,
			"Set original output name. This only sets the original output name metadata field.")
		.def("setCompression", &dv::io::Stream::setCompression, "compression"_a,
			"Set compression metadata field for this stream. This only sets the metadata field of this stream.")
		.def("getTypeDescription", &dv::io::Stream::getTypeDescription, "Get type description.")
		.def("getModuleName", &dv::io::Stream::getModuleName, "Get module name.")
		.def("getOutputName", &dv::io::Stream::getOutputName, "Get output name.")
		.def("getCompression", &dv::io::Stream::getCompression, "Get compression type string.")
		.def("getResolution", &dv::io::Stream::getResolution, "Get resolution of this stream by parsing metadata.")
		.def("getSource", &dv::io::Stream::getSource,
			"Get source name (usually the camera name) from metadata of the stream.")
		.def("setResolution", &dv::io::Stream::setResolution, "resolution"_a, "Set the stream resolution.")
		.def("setSource", &dv::io::Stream::setSource, "source"_a, "Set the stream source, usually source camera name.")
		.def("setAttribute", &dv::io::Stream::setAttribute, "name"_a, "value"_a,
			"Set an attribute of this stream, if the attribute field does not exist, it will be created.")
		.def("getAttribute", &dv::io::Stream::getAttribute, "name"_a, "Get attribute value given it's name.")
		.def_readwrite("mId", &dv::io::Stream::mId)
		.def_readwrite("mName", &dv::io::Stream::mName)
		.def_readwrite("mTypeIdentifier", &dv::io::Stream::mTypeIdentifier)
		.def_readwrite("mType", &dv::io::Stream::mType)
		.def_static("EventStream", &dv::io::Stream::EventStream, "id"_a, "name"_a, "sourceName"_a, "resolution"_a)
		.def_static("FrameStream", &dv::io::Stream::FrameStream, "id"_a, "name"_a, "sourceName"_a, "resolution"_a)
		.def_static("IMUStream", &dv::io::Stream::IMUStream, "id"_a, "name"_a, "sourceName"_a)
		.def_static("TriggerStream", &dv::io::Stream::TriggerStream, "id"_a, "name"_a, "sourceName"_a);

	py::enum_<dv::io::DataReadHandler::OutputFlag>(readHandler, "OutputFlag")
		.value("Continue", dv::io::DataReadHandler::OutputFlag::Continue)
		.value("EndOfFile", dv::io::DataReadHandler::OutputFlag::EndOfFile)
		.export_values();

	py::class_<dv::io::MonoCameraRecording>(m_io, "MonoCameraRecording",
		"A convenience class for reading recordings containing data captured from a single camera. Looks for an event, "
		"frame, imu, and trigger streams within the supplied aedat4 file.")
		.def(py::init<const fs::path &>(), py::arg("filePath"),
			"Create a reader that reads single camera data recording from an aedat4 file.")
		.def(py::init<const fs::path &, const std::string &>(), py::arg("filePath"), py::arg("cameraSerial"),
			"Create a reader that reads single camera data recording from an aedat4 file and tries to detect streams "
			"of a camera with given serial number.")
		.def(
			"getNextEventBatch",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.getNextEventBatch(streamName);
			},
			"streamName"_a = "events",
			"Sequentially read a batch of recorded events. This function increments an internal seek counter which "
			"will return the next batch at each call.")
		.def(
			"getNextFrame",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.getNextFrame(streamName);
			},
			"streamName"_a = "frames",
			"Sequential read of a frame. This function increments an internal seek counter which will return the next "
			"frame at each call.")
		.def(
			"getNextImuBatch",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.getNextImuBatch(streamName);
			},
			"streamName"_a = "imu",
			"Sequentially read a batch of recorded imu data. This function increments an internal seek counter which "
			"will return the next batch at each call.")
		.def(
			"getNextTriggerBatch",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.getNextTriggerBatch(streamName);
			},
			"streamName"_a = "triggers",
			"Sequentially read a batch of recorded triggers. This function increments an internal seek counter which "
			"will return the next batch at each call.")
		.def("getNextPose", &dv::io::MonoCameraRecording::getNextStreamPacket<dv::Pose>, "streamName"_a = "poses",
			"Sequential read of poses. This function increments an internal seek counter which will return the next "
			"packet at each call.")
		.def("getNextTimedKeyPoints", &dv::io::MonoCameraRecording::getNextStreamPacket<dv::TimedKeyPointPacket>,
			"streamName"_a = "timedKeyPoints",
			"Sequential read of dv::TimedKeyPointPacket. This function increments an internal seek counter which will "
			"return the next packet at each call.")
		.def("getNextBoundingBox", &dv::io::MonoCameraRecording::getNextStreamPacket<dv::BoundingBoxPacket>,
			"streamName"_a = "boundingBoxes",
			"Sequential read of dv::BoundingBoxPacket. This function increments an internal seek counter which will "
			"return the next packet at each call.")
		.def(
			"isEventStreamAvailable",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.isEventStreamAvailable(streamName);
			},
			"streamName"_a = "events", "Checks whether event data stream is present in the file.")
		.def(
			"isFrameStreamAvailable",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.isFrameStreamAvailable(streamName);
			},
			"streamName"_a = "frames", "Checks whether frame data stream is present in the file.")
		.def(
			"isImuStreamAvailable",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.isImuStreamAvailable(streamName);
			},
			"streamName"_a = "imu", "Checks whether imu data stream is present in the file.")
		.def(
			"isTriggerStreamAvailable",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.isTriggerStreamAvailable(streamName);
			},
			"streamName"_a = "triggers", "Checks whether trigger data stream is present in the file.")
		.def("getTimeRange", &dv::io::MonoCameraRecording::getTimeRange,
			"Return a pair containing start (first) and end (second) time of the recording file.")
		.def("getEventsTimeRange", &dv::io::MonoCameraRecording::getEventsTimeRange, "startTime"_a, "endTime"_a,
			"streamName"_a = "events", "Get events within given time range.")
		.def("getFramesTimeRange", &dv::io::MonoCameraRecording::getFramesTimeRange, "startTime"_a, "endTime"_a,
			"streamName"_a = "frames", "Get frames within given time range.")
		.def("getImuTimeRange", &dv::io::MonoCameraRecording::getImuTimeRange, "startTime"_a, "endTime"_a,
			"streamName"_a = "imu", "Get IMU data within given time range.")
		.def("getTriggersTimeRange", &dv::io::MonoCameraRecording::getTriggersTimeRange, "startTime"_a, "endTime"_a,
			"streamName"_a = "triggers", "Get trigger data within given time range.")
		.def("getPosesTimeRange", &dv::io::MonoCameraRecording::getStreamTimeRange<dv::Pose>, "startTime"_a,
			"endTime"_a, "streamName"_a = "poses", "Get poses within given time range.")
		.def("getTimedKeyPointsTimeRange", &dv::io::MonoCameraRecording::getStreamTimeRange<dv::TimedKeyPointPacket>,
			"startTime"_a, "endTime"_a, "streamName"_a = "timedKeyPoints",
			"Get timed key points within given time range.")
		.def("getBoundingBoxesTimeRange", &dv::io::MonoCameraRecording::getStreamTimeRange<dv::BoundingBoxPacket>,
			"startTime"_a, "endTime"_a, "streamName"_a = "boundingBoxes", "Get bounding boxes within given time range.")
		.def("resetSequentialRead", &dv::io::MonoCameraRecording::resetSequentialRead,
			"Reset the sequential read function to start from the beginning of the file.")
		.def("getCameraName", &dv::io::MonoCameraRecording::getCameraName,
			"Return the camera name that is detected in the recording.")
		.def(
			"getEventResolution",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.getEventResolution(streamName);
			},
			"streamName"_a = "events", "Get the resolution of the event data stream if it is available.")
		.def(
			"getFrameResolution",
			[](dv::io::MonoCameraRecording &self, const std::string &streamName) {
				return self.getFrameResolution(streamName);
			},
			"streamName"_a = "frames", "Get the resolution of the frame data stream if it is available.")
		.def("getCameraName", &dv::io::MonoCameraRecording::getCameraName,
			"Get camera name, which is a combination of the camera model and the serial number.")
		.def("isStreamOfEventType", &dv::io::MonoCameraRecording::isStreamOfDataType<dv::EventPacket>,
			"streamName"_a = "events", "Check whether a stream is of events type.")
		.def("isStreamOfFrameType", &dv::io::MonoCameraRecording::isStreamOfDataType<dv::Frame>,
			"streamName"_a = "frames", "Check whether a stream is of frame type.")
		.def("isStreamOfImuType", &dv::io::MonoCameraRecording::isStreamOfDataType<dv::IMUPacket>,
			"streamName"_a = "imu", "Check whether a stream is of imu type.")
		.def("isStreamOfTriggerType", &dv::io::MonoCameraRecording::isStreamOfDataType<dv::TriggerPacket>,
			"streamName"_a = "triggers", "Check whether a stream is of trigger type.")
		.def("isStreamOfPoseType", &dv::io::MonoCameraRecording::isStreamOfDataType<dv::Pose>, "streamName"_a = "poses",
			"Check whether a stream is of pose type.")
		.def("isStreamOfTimedKeyPointType", &dv::io::MonoCameraRecording::isStreamOfDataType<dv::TimedKeyPointPacket>,
			"streamName"_a = "timedKeyPoints", "Check whether a stream is of timed key point type.")
		.def("isStreamOfBoundingBoxType", &dv::io::MonoCameraRecording::isStreamOfDataType<dv::BoundingBoxPacket>,
			"streamName"_a = "boundingBoxes", "Check whether a stream is of bounding box type.")
		.def("getStreamNames", &dv::io::MonoCameraRecording::getStreamNames,
			"Return a vector containing all available stream names.")
		.def("getStreamMetadata", &dv::io::MonoCameraRecording::getStreamMetadata, "streamName"_a,
			"Get all metadata of a stream.")
		.def("getStreamMetadataValue", &dv::io::MonoCameraRecording::getStreamMetadataValue, "streamName"_a, "key"_a,
			"Get a value of a given metadata key. Throws an exception if given stream doesn't exist and returns "
			"None if a metadata entry with given key is not found for the stream.")
		.def("isRunning", &dv::io::MonoCameraRecording::isRunning,
			"Check whether sequential read functions have not yet reached end-of-stream.");

	py::class_<dv::io::CameraCapture> cameraCapture(
		m_io, "CameraCapture", "A convenience class for reading directly from an event camera.");

	py::enum_<dv::io::CameraCapture::BiasSensitivity>(cameraCapture, "BiasSensitivity")
		.value("VeryLow", dv::io::CameraCapture::BiasSensitivity::VeryLow)
		.value("Low", dv::io::CameraCapture::BiasSensitivity::Low)
		.value("Default", dv::io::CameraCapture::BiasSensitivity::Default)
		.value("High", dv::io::CameraCapture::BiasSensitivity::High)
		.value("VeryHigh", dv::io::CameraCapture::BiasSensitivity::VeryHigh)
		.export_values();

	py::enum_<dv::io::CameraCapture::DVXeFPS>(cameraCapture, "DVXeFPS")
		.value("EFPS_CONSTANT_100", dv::io::CameraCapture::DVXeFPS::EFPS_CONSTANT_100)
		.value("EFPS_CONSTANT_200", dv::io::CameraCapture::DVXeFPS::EFPS_CONSTANT_200)
		.value("EFPS_CONSTANT_500", dv::io::CameraCapture::DVXeFPS::EFPS_CONSTANT_500)
		.value("EFPS_CONSTANT_1000", dv::io::CameraCapture::DVXeFPS::EFPS_CONSTANT_1000)
		.value("EFPS_CONSTANT_LOSSY_2000", dv::io::CameraCapture::DVXeFPS::EFPS_CONSTANT_LOSSY_2000)
		.value("EFPS_CONSTANT_LOSSY_5000", dv::io::CameraCapture::DVXeFPS::EFPS_CONSTANT_LOSSY_5000)
		.value("EFPS_CONSTANT_LOSSY_10000", dv::io::CameraCapture::DVXeFPS::EFPS_CONSTANT_LOSSY_10000)
		.value("EFPS_VARIABLE_2000", dv::io::CameraCapture::DVXeFPS::EFPS_VARIABLE_2000)
		.value("EFPS_VARIABLE_5000", dv::io::CameraCapture::DVXeFPS::EFPS_VARIABLE_5000)
		.value("EFPS_VARIABLE_10000", dv::io::CameraCapture::DVXeFPS::EFPS_VARIABLE_10000)
		.value("EFPS_VARIABLE_15000", dv::io::CameraCapture::DVXeFPS::EFPS_VARIABLE_15000)
		.export_values();

	py::enum_<dv::io::CameraCapture::DavisReadoutMode>(cameraCapture, "DavisReadoutMode")
		.value("EventsAndFrames", dv::io::CameraCapture::DavisReadoutMode::EventsAndFrames)
		.value("EventsOnly", dv::io::CameraCapture::DavisReadoutMode::EventsOnly)
		.value("FramesOnly", dv::io::CameraCapture::DavisReadoutMode::FramesOnly)
		.export_values();

	py::enum_<dv::io::CameraCapture::DavisColorMode>(cameraCapture, "DavisColorMode")
		.value("Grayscale", dv::io::CameraCapture::DavisColorMode::Grayscale)
		.value("Color", dv::io::CameraCapture::DavisColorMode::Color)
		.export_values();

	py::enum_<dv::io::CameraCapture::CameraType>(cameraCapture, "CameraType")
		.value("Any", dv::io::CameraCapture::CameraType::Any)
		.value("DAVIS", dv::io::CameraCapture::CameraType::DAVIS)
		.value("DVS", dv::io::CameraCapture::CameraType::DVS)
		.export_values();

	cameraCapture.def(py::init<>(), "Create a camera capture class which opens first discovered camera of any type.")
		.def(py::init<const std::string &, const dv::io::CameraCapture::CameraType>(), py::arg("cameraName"),
			py::arg("type") = dv::io::CameraCapture::CameraType::Any,
			"Create a camera capture class which opens a camera according to given parameters.")
		.def("getNextEventBatch", &dv::io::CameraCapture::getNextEventBatch,
			"Sequentially read a batch of incoming events.")
		.def("getNextFrame", &dv::io::CameraCapture::getNextFrame, "Sequentially read a frame.")
		.def("getNextTriggerBatch", &dv::io::CameraCapture::getNextTriggerBatch,
			"Sequentially read a batch of triggers.")
		.def("getNextImuBatch", &dv::io::CameraCapture::getNextImuBatch, "Sequentially read a batch of imu data.")
		.def("isEventStreamAvailable", &dv::io::CameraCapture::isEventStreamAvailable,
			"Check whether event stream is available.")
		.def("isFrameStreamAvailable", &dv::io::CameraCapture::isFrameStreamAvailable,
			"Checks whether frame data stream is present in the file.")
		.def("isImuStreamAvailable", &dv::io::CameraCapture::isImuStreamAvailable,
			"Check whether device outputs IMU data.")
		.def("isTriggerStreamAvailable", &dv::io::CameraCapture::isTriggerStreamAvailable,
			"Check whether device outputs trigger data.")
		.def("getEventResolution", &dv::io::CameraCapture::getEventResolution,
			"Get the resolution of the event data stream if it is available.")
		.def("getFrameResolution", &dv::io::CameraCapture::getFrameResolution,
			"Get the resolution of the frame data stream if it is available.")
		.def("readNext", &dv::io::CameraCapture::readNext,
			"Read a packet from the camera and return a variant of any packet. You can use std::visit with "
			"`dv::io::DataReadHandler` to handle each type of packet using callback methods. This method might not "
			"maintain timestamp monotonicity between different stream types.")
		.def("setDavisColorMode", &dv::io::CameraCapture::setDavisColorMode, py::arg("colorMode"),
			"Set davis color mode. The configuration will be performed if the connected camera is a DAVIS camera.")
		.def("setDavisReadoutMode", &dv::io::CameraCapture::setDavisReadoutMode, py::arg("mode"),
			"Set davis data readout mode. The configuration will be performed if the connected camera is a DAVIS")
		.def("setDVSGlobalHold", &dv::io::CameraCapture::setDVSGlobalHold, py::arg("state"),
			"Enable or disable DVS global hold setting.")
		.def("setDVSBiasSensitivity", &dv::io::CameraCapture::setDVSBiasSensitivity, py::arg("sensitivity"),
			"Set DVS chip bias sensitivity preset.")
		.def("setDVXplorerGlobalReset", &dv::io::CameraCapture::setDVXplorerGlobalReset, py::arg("state"),
			"Enable or disable DVXplorer global reset setting.")
		.def("setDVXplorerEFPS", &dv::io::CameraCapture::setDVXplorerEFPS, py::arg("eFPS"),
			"Set DVXplorer event FPS value.")
		.def("setDavisExposureDuration", &dv::io::CameraCapture::setDavisExposureDuration, py::arg("exposure"),
			"Disable auto-exposure and set a new fixed exposure value.")
		.def("getDavisExposureDuration", &dv::io::CameraCapture::getDavisExposureDuration,
			"Get the current exposure duration.")
		.def("setDavisFrameInterval", &dv::io::CameraCapture::setDavisFrameInterval, py::arg("interval"),
			"Set a new frame interval value. This interval defines the framerate output of the camera. The frames will "
			"be produced at the given interval, the interval can be reduced in case exposure time is longer than the "
			"frame interval.")
		.def("getDavisFrameInterval", &dv::io::CameraCapture::getDavisFrameInterval,
			"Get the configured frame interval.")
		.def("enableDavisAutoExposure", &dv::io::CameraCapture::enableDavisAutoExposure,
			"Enable auto-exposure. To disable the auto-exposure, use the manual set exposure function.")
		.def("getCameraName", &dv::io::CameraCapture::getCameraName,
			"Get camera name, which is a combination of the camera model and the serial number.")
		.def("isConnected", &dv::io::CameraCapture::isConnected, "Check whether camera is still connected.")
		.def("isRunning", &dv::io::CameraCapture::isRunning, "Check whether camera is connected and active.")
		.def("getImuRate", &dv::io::CameraCapture::getImuRate,
			"Get the configured IMU measurement rate. DVXplorer cameras support individual rates for accelerometer and "
			"gyroscope, in the case camera configured to have different rates, this function return the lowest value.")
		.def("getPixelPitch", &dv::io::CameraCapture::getPixelPitch,
			"Return pixel pitch distance for the connected camera model.")
		.def("deviceConfigSet", &dv::io::CameraCapture::deviceConfigSet, "moduleAddress"_a, "parameterAddress"_a,
			"value"_a, "Set a configuration setting to a given value.")
		.def("deviceConfigGet", &dv::io::CameraCapture::deviceConfigGet, "moduleAddress"_a, "parameterAddress"_a,
			"Get a configuration setting value from the connected device.");

	py::class_<dv::io::StereoCapture>(
		m_io, "StereoCapture", "A convenience class for reading time-synchronized pair of cameras.")
		.def(py::init<const std::string &, const std::string &, const dv::Duration &>(), "leftName"_a, "rightName"_a,
			"synchronizationTimeout"_a = dv::Duration(1'000'000),
			"Open a stereo camera setup consisting of two cameras. Finds the devices connected to the system and "
			"performs timestamp synchronization on them.")
		.def_readonly("left", &dv::io::StereoCapture::left)
		.def_readonly("right", &dv::io::StereoCapture::right);

	py::class_<dv::io::NetworkReader, std::shared_ptr<dv::io::NetworkReader>>(m_io, "NetworkReader",
		"Network capture class. Connect to a network available TCP or UNIX socket server providing a data stream. The "
		"class provides a single data stream per network capture.")
		.def(py::init<const fs::path &>(), "socketPath"_a,
			"Initialize a network capture class, it will connect to a given UNIX socket with a given file system path.")
		.def(py::init<const std::string_view, const uint16_t>(), "ipAddress"_a, "port"_a,
			"Initialize a network capture class, it will connect to a given TCP port with given IP address.")
		.def(py::init([](const std::string_view ipAddress, const uint16_t port,
						  const std::filesystem::path &certificateChain, const std::filesystem::path &privateKey) {
			return std::make_shared<dv::io::NetworkReader>(
				ipAddress, port, dv::io::encrypt::defaultEncryptionClient(certificateChain, privateKey));
		}),
			"ipAddress"_a, "port"_a, "certificateChain"_a, "privateKey"_a,
			"Initialize an encrypted network capture class, it will connect to a given TCP port with given IP address. "
			"Provide a client certificate chain and private key for the TLS encrypted connection.")
		.def("getNextEventBatch", &dv::io::NetworkReader::getNextEventBatch,
			"Read next event batch. This is a non-blocking method, if there is no data to read, it will return a "
			"`None`.")
		.def("getNextFrame", &dv::io::NetworkReader::getNextFrame,
			"Read next frame. This is a non-blocking method, if there is no data to read, it will return a `None`.")
		.def("getNextImuBatch", &dv::io::NetworkReader::getNextImuBatch,
			"Read next imu measurement batch. This is a non-blocking method, if there is no data to read, it will "
			"return a `None`.")
		.def("getNextTriggerBatch", &dv::io::NetworkReader::getNextTriggerBatch,
			"Read next trigger batch. This is a non-blocking method, if there is no data to read, it will return a "
			"`None`.")
		.def("getEventResolution", &dv::io::NetworkReader::getEventResolution,
			"Retrieve the event sensor resolution. The method returns `None` if event stream is not available "
			"or the metadata does not contain resolution.")
		.def("getFrameResolution", &dv::io::NetworkReader::getFrameResolution,
			"Retrieve the frame sensor resolution. The method returns `None` if frame stream is not available or the "
			"metadata does not contain resolution.")
		.def("isEventStreamAvailable", &dv::io::NetworkReader::isEventStreamAvailable,
			"Check whether an event stream is available in this capture class.")
		.def("isFrameStreamAvailable", &dv::io::NetworkReader::isFrameStreamAvailable,
			"Check whether a frame stream is available in this capture class.")
		.def("isImuStreamAvailable", &dv::io::NetworkReader::isImuStreamAvailable,
			"Check whether an IMU data stream is available in this capture class.")
		.def("isTriggerStreamAvailable", &dv::io::NetworkReader::isTriggerStreamAvailable,
			"Check whether a trigger stream is available in this capture class.")
		.def("getCameraName", &dv::io::NetworkReader::getCameraName,
			"Get camera name, which is a combination of the camera model and the serial number.")
		.def("isRunning", &dv::io::NetworkReader::isRunning, "Check whether the network stream is still connected.")
		.def("getStreamDefinition", &dv::io::NetworkReader::getStreamDefinition,
			"Get the stream definition object, which describe the available data stream by this reader.");

	py::class_<dv::io::NetworkWriter, std::shared_ptr<dv::io::NetworkWriter>>(
		m_io, "NetworkWriter", "Network server class for streaming AEDAT4 serialized data types.")
		.def(py::init(
				 [](const std::string_view ip, const uint16_t port, const dv::io::Stream &stream,
					 const size_t maxClientConnections, std::function<void(const int32_t, std::string_view)> callback) {
					 return std::make_shared<dv::io::NetworkWriter>(ip, port, stream, maxClientConnections,
						 [callback = std::move(callback)](
							 const boost::system::error_code &error, const std::string_view message) {
							 callback(error.value(), message);
						 });
				 }),
			"ipAddress"_a, "port"_a, "stream"_a, "maxClientConnections"_a = 10,
			"errorMessageCallback"_a = std::function([](const int32_t, std::string_view) {
			}),
			"Create a non-encrypted server that listens for connections on a given IP address. Supports multiple "
			"clients.")
		.def(py::init([](const std::filesystem::path &socketPath, const dv::io::Stream &stream,
						  const size_t maxClientConnections,
						  std::function<void(const int32_t, const std::string_view)> callback) {
			return std::make_shared<dv::io::NetworkWriter>(socketPath, stream, maxClientConnections,
				[callback = std::move(callback)](
					const boost::system::error_code &error, const std::string_view message) {
					callback(error.value(), std::string(message));
				});
		}),
			"socketPath"_a, "stream"_a, "maxClientConnections"_a = 10,
			"errorMessageCallback"_a = std::function([](const int32_t, std::string_view) {
			}),
			"Create a local socket server. Provide a path to the socket, if a file already exists on a given path, the "
			"connection will fail by throwing an exception. It is required that the given socket path does not point "
			"to an existing socket file. If the file *can* exist, it is up to the user of this class to decide whether "
			"it is safe to remove any existing socket files or the class should not bind to the path.")
		.def(py::init([](const std::string_view ipAddress, const uint16_t port, const dv::io::Stream &stream,
						  const std::filesystem::path &certificateChain, const std::filesystem::path &privateKey,
						  const std::filesystem::path &CAFile, const size_t maxClientConnections,
						  std::function<void(const int32_t, const std::string_view)> callback) {
			return std::make_shared<dv::io::NetworkWriter>(ipAddress, port, stream,
				dv::io::encrypt::defaultEncryptionServer(certificateChain, privateKey, CAFile), maxClientConnections,
				[callback = std::move(callback)](
					const boost::system::error_code &error, const std::string_view message) {
					callback(error.value(), std::string(message));
				});
		}),
			"ipAddress"_a, "port"_a, "stream"_a, "certificateChain"_a, "privateKey"_a, "CAFile"_a,
			"maxClientConnections"_a = 10,
			"errorMessageCallback"_a = std::function([](const int32_t, std::string_view) {
			}),
			"Create an encrypted server that listens for connections on a given IP address. Supports multiple clients. "
			"Provide server certificate chain, private key, and CAFile for encrypted connection and client(s) "
			"certificate verification.")
		.def("writeEvents", &dv::io::NetworkWriter::writeEvents, "events"_a,
			"Write an event store to the network stream.")
		.def("writeFrame", &dv::io::NetworkWriter::writeFrame, "frame"_a, "Write a frame image to the network stream.")
		.def("writeIMU", &dv::io::NetworkWriter::writeIMU, "imu"_a, "Write IMU data to the socket.")
		.def("writeTriggers", &dv::io::NetworkWriter::writeTriggers, "triggers"_a,
			"Write trigger data to the network stream.")
		.def(
			"writePose",
			[](dv::io::NetworkWriter &self, const dv::Pose &pose) {
				self.writePacket(pose);
			},
			"pose"_a, "Write pose data")
		.def(
			"writeBoundingBox",
			[](dv::io::NetworkWriter &self, const dv::BoundingBoxPacket &boundingBox) {
				self.writePacket(boundingBox);
			},
			"boundingBox"_a, "Write bounding boxes")
		.def(
			"writeTimedKeyPoints",
			[](dv::io::NetworkWriter &self, const dv::TimedKeyPointPacket &timedKeyPoints) {
				self.writePacket(timedKeyPoints);
			},
			"timedKeyPoints"_a, "Write timed key points")
		.def("getCameraName", &dv::io::NetworkWriter::getCameraName,
			"Get camera name. It is looked up from the stream definition during construction.")
		.def("getQueuedPacketCount", &dv::io::NetworkWriter::getQueuedPacketCount,
			"Get number of packets in the write queue.")
		.def("getClientCount", &dv::io::NetworkWriter::getClientCount, "Get number of active connected clients.");

	py::class_<dv::io::MonoCameraWriter> monoCameraWriter(
		m_io, "MonoCameraWriter", "Create an aedat4 file writer class.");

	py::class_<dv::io::MonoCameraWriter::Config>(
		monoCameraWriter, "Config", "A configuration structure for the MonoCameraWriter.")
		.def(py::init<const std::string &, dv::CompressionType>(), "cameraName"_a,
			"compression"_a = dv::CompressionType::LZ4)
		.def_readwrite("compression", &dv::io::MonoCameraWriter::Config::compression)
		.def_readwrite("cameraName", &dv::io::MonoCameraWriter::Config::cameraName)
		.def("addEventStream", &dv::io::MonoCameraWriter::Config::addEventStream, "resolution"_a,
			"streamName"_a = "events", "source"_a = std::nullopt, "Add an event stream with a given resolution.")
		.def("addFrameStream", &dv::io::MonoCameraWriter::Config::addFrameStream, "resolution"_a,
			"streamName"_a = "frames", "source"_a = std::nullopt, "Add a frame stream with a given resolution.")
		.def("addImuStream", &dv::io::MonoCameraWriter::Config::addImuStream, "streamName"_a = "imu",
			"source"_a = std::nullopt, "Add an IMU data stream.")
		.def("addTriggerStream", &dv::io::MonoCameraWriter::Config::addTriggerStream, "streamName"_a = "triggers",
			"source"_a = std::nullopt, "Add a trigger stream.")
		.def("addPoseStream", &dv::io::MonoCameraWriter::Config::addStream<dv::Pose>, "streamName"_a = "poses",
			"source"_a = std::nullopt, "Add a pose stream.")
		.def("addTimedKeyPointStream", &dv::io::MonoCameraWriter::Config::addStream<dv::TimedKeyPointPacket>,
			"streamName"_a = "timedKeyPoints", "source"_a = std::nullopt, "Add a timed key point stream.")
		.def("addBoundingBoxStream", &dv::io::MonoCameraWriter::Config::addStream<dv::BoundingBoxPacket>,
			"streamName"_a = "boundingBoxes", "source"_a = std::nullopt, "Add a bounding box stream.")
		.def("addStreamMetadata", &dv::io::MonoCameraWriter::Config::addStreamMetadata)
		.def("findStreamResolution", &dv::io::MonoCameraWriter::Config::findStreamResolution, "streamName"_a,
			"Parse resolution of the stream from metadata of the stream. Resolution should be set as two metadata "
			"parameters: \"sizeX\" and \"sizeY\" parameters.");

	monoCameraWriter
		.def(py::init<const fs::path &, const dv::io::MonoCameraWriter::Config &>(), "aedat4Path"_a, "config"_a,
			"Create an aedat4 file writer with simplified API.")
		.def(py::init<const fs::path &, const dv::io::CameraCapture &, dv::CompressionType>(), "aedat4Path"_a,
			"capture"_a, "compression"_a = dv::CompressionType::LZ4,
			"Create an aedat4 file writer with output streams available in camera capture class.")
		.def_static("EventOnlyConfig", &dv::io::MonoCameraWriter::EventOnlyConfig, py::arg("cameraName"),
			py::arg("resolution"), py::arg("compression") = dv::CompressionType::LZ4,
			"Generate a config for a writer that will expect a stream of events only.")
		.def_static("FrameOnlyConfig", &dv::io::MonoCameraWriter::FrameOnlyConfig, py::arg("cameraName"),
			py::arg("resolution"), py::arg("compression") = dv::CompressionType::LZ4,
			"Generate a config for a writer that will expect a stream of frames only.")
		.def_static("DVSConfig", &dv::io::MonoCameraWriter::DVSConfig, py::arg("cameraName"), py::arg("resolution"),
			py::arg("compression") = dv::CompressionType::LZ4,
			"Generate a config for a writer that will expect data from a DVS camera - events, IMU, triggers.")
		.def_static("DAVISConfig", &dv::io::MonoCameraWriter::DAVISConfig, py::arg("cameraName"), py::arg("resolution"),
			py::arg("compression") = dv::CompressionType::LZ4,
			"Generate a config for a writer that will expect data from a DAVIS camera - frames, events, IMU, triggers.")
		.def_static("CaptureConfig", &dv::io::MonoCameraWriter::CaptureConfig, py::arg("capture"),
			py::arg("compression") = dv::CompressionType::LZ4,
			"Generate a config for a given capture device, performs capability inspection.")
		.def("writeEvents", &dv::io::MonoCameraWriter::writeEvents, "eventStore"_a, "streamName"_a = "events",
			"Write an event store into the output file. The store is written by maintaining internal data partial "
			"ordering and fragmentation. The data is passed directly into the serialization procedure without "
			"performing copies. Data is serialized and the actual file IO is performed on a separate thread.")
		.def("writeFrame", &dv::io::MonoCameraWriter::writeFrame, "frame"_a, "streamName"_a = "frames",
			"Write a frame image into the file. The data is passed directly into the serialization procedure without "
			"performing copies. Data is serialized and the actual file IO is performed on a separate thread.")
		.def("writeImu", &dv::io::MonoCameraWriter::writeImu, "imu"_a, "streamName"_a = "imu",
			"Write an IMU measurement. This function is not immediate, it batches the measurements until a configured "
			"amount is reached, only then the data is passed to the serialization step. Only then the data will be "
			"passed to the file write IO thread. If the file is closed (the object gets destroyed), destructor will "
			"dump the rest of the buffered measurements to the serialization step.")
		.def("writeTrigger", &dv::io::MonoCameraWriter::writeTrigger, "trigger"_a, "streamName"_a = "triggers",
			"Write a trigger measurement. This function is not immediate, it batches the measurements until a "
			"configured amount is reached, only then the data is passed to the serialization step. Only then the data "
			"will be passed to the file write IO thread. If the file is closed (the object gets destroyed), destructor "
			"will dump the rest of the buffered measurements to the serialization step.")
		.def("writePose", &dv::io::MonoCameraWriter::writePacket<dv::Pose>, "pose"_a, "streamName"_a = "poses",
			"Write a pose into the file. The data is passed directly into the serialization procedure without "
			"performing copies. Data is serialized and the actual file IO is performed on a separate thread.")
		.def("writeTimedKeyPoint",
			&dv::io::MonoCameraWriter::writePacketElement<dv::TimedKeyPointPacket, dv::TimedKeyPoint>,
			"timedKeyPoint"_a, "streamName"_a = "timedKeyPoints",
			"Write a timed key point measurement. This function is not immediate, it batches the measurements until a "
			"configured amount is reached, only then the data is passed to the serialization step. Only then the data "
			"will be passed to the file write IO thread. If the file is closed (the object gets destroyed), destructor "
			"will dump the rest of the buffered measurements to the serialization step.")
		.def("writeBoundingBox", &dv::io::MonoCameraWriter::writePacketElement<dv::BoundingBoxPacket, dv::BoundingBox>,
			"boundingBox"_a, "streamName"_a = "boundingBoxes",
			"Write a bounding box. This function is not immediate, it batches the measurements until a "
			"configured amount is reached, only then the data is passed to the serialization step. Only then the data "
			"will be passed to the file write IO thread. If the file is closed (the object gets destroyed), destructor "
			"will dump the rest of the buffered measurements to the serialization step.")
		.def("writeEventPacket", &dv::io::MonoCameraWriter::writeEventPacket, "eventPacket"_a,
			"streamName"_a = "events",
			"Write an event packet into the output file. The data is passed directly into the serialization procedure "
			"without performing copies. Data is serialized and the actual file IO is performed on a separate thread.")
		.def("writeImuPacket", &dv::io::MonoCameraWriter::writeImuPacket, "imuPacket"_a, "streamName"_a = "imu",
			"Write a packet of IMU measurement. The data is passed directly into the serialization procedure "
			"without performing copies. Data is serialized and the actual file IO is performed on a separate thread.")
		.def("writeTriggerPacket", &dv::io::MonoCameraWriter::writeTriggerPacket, "triggerPacket"_a,
			"streamName"_a = "triggers",
			"Write a packet of trigger measurement. The data is passed directly into the serialization procedure "
			"without performing copies. Data is serialized and the actual file IO is performed on a separate thread.")
		.def("writeTimedKeyPointPacket", &dv::io::MonoCameraWriter::writePacket<dv::TimedKeyPointPacket>,
			"timedKeyPoints"_a, "streamName"_a = "timedKeyPoints",
			"Write timed keypoint packet into the file. The data is passed directly into the serialization procedure "
			"without performing copies. Data is serialized and the actual file IO is performed on a separate thread.")
		.def("writeBoundingBoxesPacket", &dv::io::MonoCameraWriter::writePacket<dv::BoundingBoxPacket>,
			"boundingBoxes"_a, "streamName"_a = "boundingBoxes",
			"Write bounding box packet into the file. The data is passed directly into the serialization procedure "
			"without performing copies. Data is serialized and the actual file IO is performed on a separate thread.")
		.def("setPackagingCount", &dv::io::MonoCameraWriter::setPackagingCount, "packagingCount"_a,
			"Set the size batch size for trigger and imu buffering. The single measurements passed into `writeTrigger` "
			"and `writeImu` functions will packed into batches of the given size before writing to the file. A "
			"packaging value of 0 or 1 will cause each measurement to be serialized immediately.")
		.def("isEventStreamConfigured", &dv::io::MonoCameraWriter::isEventStreamConfigured, "streamName"_a = "events",
			"Check if a event stream is configured for this writer.")
		.def("isFrameStreamConfigured", &dv::io::MonoCameraWriter::isFrameStreamConfigured, "streamName"_a = "frames",
			"Check if a frame stream is configured for this writer.")
		.def("isImuStreamConfigured", &dv::io::MonoCameraWriter::isImuStreamConfigured, "streamName"_a = "imu",
			"Check if a IMU stream is configured for this writer.")
		.def("isTriggerStreamConfigured", &dv::io::MonoCameraWriter::isTriggerStreamConfigured,
			"streamName"_a = "triggers", "Check if a trigger stream is configured for this writer.")
		.def("isPoseStreamConfigured", &dv::io::MonoCameraWriter::isStreamConfigured<dv::Pose>,
			"streamName"_a = "poses", "Check if a pose stream is configured for this writer.")
		.def("isTimedKeyPointStreamConfigured", &dv::io::MonoCameraWriter::isStreamConfigured<dv::TimedKeyPointPacket>,
			"streamName"_a = "timedKeyPoint", "Check if a timed key point stream is configured for this writer.")
		.def("isBoundingBoxesStreamConfigured", &dv::io::MonoCameraWriter::isStreamConfigured<dv::BoundingBoxPacket>,
			"streamName"_a = "boundingBoxes", "Check if a bounding box stream is configured for this writer.");

	py::class_<dv::io::StereoCameraWriter>(
		m_io, "StereoCameraWriter", "Create an aedat4 file writer class for a stereo rig.")
		.def(py::init<const std::string &, const dv::io::MonoCameraWriter::Config &,
				 const dv::io::MonoCameraWriter::Config &>(),
			py::arg("aedat4Path"), py::arg("leftConfig"), py::arg("rightConfig"),
			"Create an aedat4 file using manual configuration.")
		.def(py::init<const std::string &, const dv::io::StereoCapture &>(), py::arg("aedat4Path"),
			py::arg("stereoCapture"),
			"Create an aedat4 file using stereo capture instance to inspect the capabilities of cameras.")
		.def_readonly("left", &dv::io::StereoCameraWriter::left, "Left camera writer instance")
		.def_readonly("right", &dv::io::StereoCameraWriter::right, "Right camera writer instance");

	py::class_<dv::io::StereoCameraRecording>(m_io, "StereoCameraRecording",
		"Create a reader for stereo camera recording. Expects at least one stream from two cameras available. Prior "
		"knowledge of stereo setup is required, otherwise it is not possible to differentiate between left and right "
		"cameras. This is just a convenience class that gives access to distinguished data streams in the recording.")
		.def(py::init<const fs::path &, const std::string &, const std::string &>(), "aedat4Path"_a, "leftCameraName"_a,
			"rightCameraName"_a, "Open an aedat4 file containing recordings from a stereo camera setup.")
		.def("getLeftReader", &dv::io::StereoCameraRecording::getLeftReader, "Access the left camera reader.")
		.def("getRightReader", &dv::io::StereoCameraRecording::getRightReader, "Access the right camera reader.");

	auto m_measurements = m.def_submodule("measurements");
	py::class_<dv::measurements::Depth>(
		m_measurements, "Depth", "A depth measurement structure that contains a timestamped measurement of depth.")
		.def(py::init<int64_t, float>(), py::arg("timestamp"), py::arg("depth"))
		.def_readwrite("mTimestamp", &dv::measurements::Depth::mTimestamp, "UNIX Microsecond timestamp.")
		.def_readwrite(
			"mDepth", &dv::measurements::Depth::mDepth, "Depth measurement value, expected to be in meters.");

	auto m_camera = m.def_submodule("camera");

	py::enum_<dv::camera::DistortionModel>(m_camera, "DistortionModel")
		.value("NONE", dv::camera::DistortionModel::None)
		.value("RadTan", dv::camera::DistortionModel::RadTan)
		.value("Equidistant", dv::camera::DistortionModel::Equidistant)
		.export_values();

	m_camera.def("stringToDistortionModel", &dv::camera::stringToDistortionModel, "modelName"_a,
		"Convert a string into the Enum of the DistortionModel");
	m_camera.def("distortionModelToString", &dv::camera::distortionModelToString, "model"_a,
		"Convert a DistortionModel Enum into a string");

	py::class_<dv::camera::CameraGeometry, std::shared_ptr<dv::camera::CameraGeometry>>(m_camera, "CameraGeometry")
		.def(py::init<const float, const float, const float, const float, const cv::Size &>(), py::arg("fx"),
			py::arg("fy"), py::arg("cx"), py::arg("cy"), py::arg("resolution"),
			"Create a camera geometry model without distortion model. Currently only radial tangential model is "
			"supported. Any calls to function dependent on distortion will cause exceptions or segfaults.")
		.def(py::init<const std::vector<float> &, const float, const float, const float, const float, const cv::Size &,
				 const dv::camera::DistortionModel>(),
			py::arg("distortion"), py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"), py::arg("resolution"),
			py::arg("distortionModel"),
			"Create a camera geometry model with distortion model. Currently only radial tangential model is "
			"supported.")
		.def("undistort", &dv::camera::CameraGeometry::undistort<Eigen::Vector2f, Eigen::Vector2f>, py::arg("point"),
			"Returns pixel coordinates of given point with applied back projection, undistortion, and projection. This "
			"function uses look-up table and is designed for minimal execution speed. WARNING: will cause a segfault "
			"if coordinates are out-of-bounds or if distortion model is not available.")
		.def("undistortEvents", &dv::camera::CameraGeometry::undistortEvents, py::arg("events"),
			"Undistort event coordinates, discards events which fall beyond camera resolution.")
		.def("undistortSequence",
			&dv::camera::CameraGeometry::undistortSequence<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector3f>>,
			py::arg("points"), "Undistort point coordinates.")
		.def("distort", &dv::camera::CameraGeometry::distort<Eigen::Vector3f, Eigen::Vector3f>, py::arg("point"),
			"Apply distortion to a 3D point.")
		.def("distortSequence",
			&dv::camera::CameraGeometry::distortSequence<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>>,
			py::arg("points"), "Apply direct distortion on the 3D points.")

		.def(
			"backProject",
			[](const dv::camera::CameraGeometry &self, const Eigen::Vector2f &point,
				const bool subPixel = false) -> Eigen::Vector3f {
				if (subPixel) {
					return self.backProject<Eigen::Vector3f, Eigen::Vector2f,
						dv::camera::CameraGeometry::FunctionImplementation::SubPixel>(point);
				}
				else {
					return self.backProject<Eigen::Vector3f, Eigen::Vector2f,
						dv::camera::CameraGeometry::FunctionImplementation::LUT>(point);
				}
			},
			"point"_a, "subPixel"_a = false,
			"Back-project pixel coordinates into a plane at depth = 1.0 meters from focal point. If subPixel argument "
			"is False - the function just performs lookup-table operation within pixel bound, rounding the coordinates "
			"to first integer. If subPixel is set to True, it will perform the precise math operations")
		.def(
			"backProjectSequence",
			[](const dv::camera::CameraGeometry &self, const std::vector<Eigen::Vector2f> &points,
				const bool subPixel = false) -> std::vector<Eigen::Vector3f> {
				if (subPixel) {
					return self.backProjectSequence<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector2f>,
						dv::camera::CameraGeometry::FunctionImplementation::SubPixel>(points);
				}
				else {
					return self.backProjectSequence<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector2f>,
						dv::camera::CameraGeometry::FunctionImplementation::LUT>(points);
				}
			},
			"points"_a, "subPixel"_a = false,
			"Back-project a sequence of pixel coordinates into rays at a plane at depth = 1.0 meters from focal point. "
			"If subPixel argument "
			"is False - the function just performs lookup-table operation within pixel bound, rounding the coordinates "
			"to first integer. If subPixel is set to True, it will perform the precise math operations")
		.def(
			"backProjectSequence",
			[](const dv::camera::CameraGeometry &self, const dv::EventStore &points,
				const bool subPixel = false) -> std::vector<Eigen::Vector3f> {
				if (subPixel) {
					return self.backProjectSequence<std::vector<Eigen::Vector3f>, dv::EventStore,
						dv::camera::CameraGeometry::FunctionImplementation::SubPixel>(points);
				}
				else {
					return self.backProjectSequence<std::vector<Eigen::Vector3f>, dv::EventStore,
						dv::camera::CameraGeometry::FunctionImplementation::LUT>(points);
				}
			},
			"events"_a, "subPixel"_a = false,
			"Back-project a sequence of pixel coordinates into rays at a plane at depth = 1.0 meters from focal point. "
			"If subPixel argument "
			"is False - the function just performs lookup-table operation within pixel bound, rounding the coordinates "
			"to first integer. If subPixel is set to True, it will perform the precise math operations")
		.def("backProjectUndistort",
			&dv::camera::CameraGeometry::backProjectUndistort<Eigen::Vector3f, Eigen::Vector2f>, py::arg("point"),
			"Returns a unit ray of given coordinates with applied back projection and undistortion. This function uses "
			"look-up table and is designed for minimal execution speed. WARNING: will cause a segfault if coordinates "
			"are out-of-bounds or if distortion model is not available.")
		.def("backProjectUndistortSequence",
			&dv::camera::CameraGeometry::backProjectUndistortSequence<std::vector<Eigen::Vector3f>,
				std::vector<Eigen::Vector2f>>,
			py::arg("points"),
			"Undistort and back project a batch of points. Output is normalized point coordinates as unit rays.")
		.def("project", &dv::camera::CameraGeometry::project<Eigen::Vector2f, Eigen::Vector3f>, py::arg("point"),
			"Project a 3D point into pixel plane. WARNING: Does not perform range checking!")
		.def("projectSequence",
			&dv::camera::CameraGeometry::projectSequence<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector3f>>,
			py::arg("points"), py::arg("dimensionCheck") = true, "Project a batch of 3D points into pixel plane.")
		.def("isWithinDimensions", &dv::camera::CameraGeometry::isWithinDimensions<Eigen::Vector2f>, py::arg("point"),
			"Check whether given coordinates are within valid range.")
		.def("isUndistortionAvailable", &dv::camera::CameraGeometry::isUndistortionAvailable,
			"Checks whether this camera geometry calibration contains coefficient for an undistortion model.")
		.def(
			"getCameraMatrix",
			[](const dv::camera::CameraGeometry &self) -> Eigen::Matrix3f {
				Eigen::Matrix3f mat;
				cv::cv2eigen(self.getCameraMatrix(), mat);
				return mat;
			},
			"Get camera matrix in the format: | mFx 0 mCx | | 0 mFy mCy | | 0 0 1 |")
		.def("getFocalLength", &dv::camera::CameraGeometry::getFocalLength<Eigen::Vector2f>, "Focal length in pixels.")
		.def("getCentralPoint", &dv::camera::CameraGeometry::getCentralPoint<Eigen::Vector2f>,
			"Central point coordinates.")
		.def("getDistortion", &dv::camera::CameraGeometry::getDistortion, "Get distortion coefficients.")
		.def("getDistortionModel", &dv::camera::CameraGeometry::getDistortionModel, "Get distortion model.")
		.def("getResolution", &dv::camera::CameraGeometry::getResolution, "Get the camera resolution.");

	auto m_calibrations = m_camera.def_submodule("calibrations");

	py::class_<dv::camera::calibrations::CameraCalibration> cameraCalibration(m_calibrations, "CameraCalibration");

	py::class_<dv::camera::calibrations::CameraCalibration::Metadata>(cameraCalibration, "Metadata")
		.def(py::init<const cv::Size &, const cv::Size &, const std::string &, float, float,
				 const std::optional<float> &, const std::string &, const std::string &, const std::string &,
				 const std::optional<float> &>(),
			py::arg("patternShape"), py::arg("internalPatternShape"), py::arg("patternType"), py::arg("patternSize"),
			py::arg("patternSpacing"), py::arg("calibrationError"), py::arg("calibrationTime"), py::arg("quality"),
			py::arg("comment"), py::arg("pixelPitch"))
		.def(py::init<>())
		.def_readonly("patternShape", &dv::camera::calibrations::CameraCalibration::Metadata::patternShape,
			"Shape of the calibration pattern")
		.def_readonly("internalPatternShape",
			&dv::camera::calibrations::CameraCalibration::Metadata::internalPatternShape,
			"Shape of the internal calibration pattern")
		.def_readonly("patternType", &dv::camera::calibrations::CameraCalibration::Metadata::patternType,
			"Type of the calibration pattern used (e.g. apriltag)")
		.def_readonly("patternSize", &dv::camera::calibrations::CameraCalibration::Metadata::patternSize,
			"Size of the calibration pattern in [m]")
		.def_readonly("patternSpacing", &dv::camera::calibrations::CameraCalibration::Metadata::patternSpacing,
			"Ratio between tags to patternSize (apriltag only)")
		.def_readonly("calibrationError", &dv::camera::calibrations::CameraCalibration::Metadata::calibrationError,
			"Calibration reprojection error")
		.def_readonly("calibrationTime", &dv::camera::calibrations::CameraCalibration::Metadata::calibrationTime,
			"Timestamp when the calibration was conducted")
		.def_readonly("quality", &dv::camera::calibrations::CameraCalibration::Metadata::quality,
			"Description of the calibration quality (excellent/good/bad etc)")
		.def_readonly(
			"comment", &dv::camera::calibrations::CameraCalibration::Metadata::comment, "Any additional information")
		.def_readonly(
			"pixelPitch", &dv::camera::calibrations::CameraCalibration::Metadata::pixelPitch, "Pixel pitch in meters");

	cameraCalibration.def(py::init<>())
		.def(py::init<const std::string &, const std::string &, const bool, const cv::Size &, const cv::Point2f &,
				 const cv::Point2f &, const std::vector<float> &, const dv::camera::DistortionModel &,
				 const std::vector<float> &,
				 const std::optional<dv::camera::calibrations::CameraCalibration::Metadata> &>(),
			py::arg("name"), py::arg("position"), py::arg("master"), py::arg("resolution"), py::arg("principalPoint"),
			py::arg("focalLength"), py::arg("distortion"), py::arg("distortionModel"), py::arg("transformationToC0"),
			py::arg("metadata"))
		.def_readonly(
			"name", &dv::camera::calibrations::CameraCalibration::name, "Camera name (e.g. 'DVXplorer_DXA02137')")
		.def_readonly(
			"resolution", &dv::camera::calibrations::CameraCalibration::resolution, "Camera sensor resolution")
		.def_readonly("master", &dv::camera::calibrations::CameraCalibration::master,
			"Indicate whether it is the master camera in a multi-camera rig")
		.def_readonly("focalLength", &dv::camera::calibrations::CameraCalibration::focalLength, "Focal length")
		.def_readonly("principalPoint", &dv::camera::calibrations::CameraCalibration::principalPoint,
			"Intersection of optical axis and image plane")
		.def_readonly("position", &dv::camera::calibrations::CameraCalibration::position,
			"Description of the location of the camera in the camera rig (e.g. 'left')")
		.def_readonly("distortion", &dv::camera::calibrations::CameraCalibration::distortion, "Distortion coefficients")
		.def_readonly("distortionModel", &dv::camera::calibrations::CameraCalibration::distortionModel,
			"Distortion model used (e.g. radialTangential)")
		.def_readonly("metadata", &dv::camera::calibrations::CameraCalibration::metadata, "Metadata")
		.def("getTransformMatrix", &dv::camera::calibrations::CameraCalibration::getTransformMatrix,
			"Transformation from camera zero to this camera")
		.def("getCameraGeometry", &dv::camera::calibrations::CameraCalibration::getCameraGeometry,
			"Retrieve camera geometry instance from this calibration instance. Distortion model is going to be ignored "
			"if the `CameraGeometry` class doesn't support the distortion model.")
		.def(
			"getCameraMatrix",
			[](const dv::camera::calibrations::CameraCalibration &self) -> Eigen::Matrix3f {
				Eigen::Matrix3f mat;
				cv::cv2eigen(self.getCameraMatrix(), mat);
				return mat;
			},
			"Get camera matrix in the format: | mFx 0 mCx | | 0 mFy mCy | | 0 0 1 |");

	py::class_<dv::camera::calibrations::IMUCalibration> imuCalibration(m_calibrations, "IMUCalibration");

	py::class_<dv::camera::calibrations::IMUCalibration::Metadata>(imuCalibration, "Metadata")
		.def(py::init<const std::string &, const std::string &>(), py::arg("calibrationTime") = "",
			py::arg("comment") = "")
		.def_readonly("calibrationTime", &dv::camera::calibrations::IMUCalibration::Metadata::calibrationTime,
			"Timestamp when the calibration was conducted")
		.def_readonly(
			"comment", &dv::camera::calibrations::IMUCalibration::Metadata::comment, "Any additional information");

	imuCalibration.def(py::init<>())
		.def(py::init<const std::string &, float, float, const cv::Point3f &, const cv::Point3f &, float, float, float,
				 float, float, float, float, const std::vector<float> &,
				 const std::optional<dv::camera::calibrations::IMUCalibration::Metadata> &>(),
			py::arg("name"), py::arg("omegaMax"), py::arg("accMax"), py::arg("omegaOffsetAvg"), py::arg("accOffsetAvg"),
			py::arg("omegaOffsetVar"), py::arg("accOffsetVar"), py::arg("omegaNoiseDensity"),
			py::arg("accNoiseDensity"), py::arg("omegaNoiseRandomWalk"), py::arg("accNoiseRandomWalk"),
			py::arg("timeOffset"), py::arg("transformationToC0"), py::arg("metadata"))
		.def_readonly(
			"name", &dv::camera::calibrations::IMUCalibration::name, "Sensor name (e.g. 'IMU_DVXplorer_DXA02137')")
		.def_readonly("omegaMax", &dv::camera::calibrations::IMUCalibration::omegaMax,
			"Maximum (saturation) angular velocity of the gyroscope [rad/s]")
		.def_readonly("accMax", &dv::camera::calibrations::IMUCalibration::accMax,
			"Maximum (saturation) acceleration of the accelerometer [m/s^2]")
		.def_readonly("omegaOffsetAvg", &dv::camera::calibrations::IMUCalibration::omegaOffsetAvg,
			"Average offset (bias) of the angular velocity [rad/s]")
		.def_readonly("accOffsetAvg", &dv::camera::calibrations::IMUCalibration::accOffsetAvg,
			"Average offset (bias) of the acceleration [m/s^2]")
		.def_readonly("omegaOffsetVar", &dv::camera::calibrations::IMUCalibration::omegaOffsetVar,
			"Variance of the offset of the angular velocity [rad/s]")
		.def_readonly("accOffsetVar", &dv::camera::calibrations::IMUCalibration::accOffsetVar,
			"Variance of the offset of the acceleration [m/s^2]")
		.def_readonly("omegaNoiseDensity", &dv::camera::calibrations::IMUCalibration::omegaNoiseDensity,
			"Noise density of the gyroscope [rad/s^s/sqrt(Hz)]")
		.def_readonly("accNoiseDensity", &dv::camera::calibrations::IMUCalibration::accNoiseDensity,
			"Noise density of the accelerometer [m/s^2/sqrt(Hz)]")
		.def_readonly("omegaNoiseRandomWalk", &dv::camera::calibrations::IMUCalibration::omegaNoiseRandomWalk,
			"Noise random walk of the gyroscope [rad/s^s/sqrt(Hz)]")
		.def_readonly("accNoiseRandomWalk", &dv::camera::calibrations::IMUCalibration::accNoiseRandomWalk,
			"Noise random walk of the accelerometer [m/s^2/sqrt(Hz)]")
		.def_readonly("timeOffsetMicros", &dv::camera::calibrations::IMUCalibration::timeOffsetMicros,
			"Offset between the camera and IMU timestamps in microseconds (t_correct = t_imu - offset)")
		.def_readonly("transformationToC0", &dv::camera::calibrations::IMUCalibration::transformationToC0,
			"Transformation converting points in IMU frame to C0 frame p_C0= T * p_IMU")
		.def_readonly("metadata", &dv::camera::calibrations::IMUCalibration::metadata, "Metadata");

	py::class_<dv::camera::calibrations::StereoCalibration> stereoCalibration(m_calibrations, "StereoCalibration");

	py::class_<dv::camera::calibrations::StereoCalibration::Metadata>(
		stereoCalibration, "Metadata", "Metadata for the stereo calibration.")
		.def(py::init<const std::optional<float> &, const std::string &>(), py::arg("epipolarError") = std::nullopt,
			py::arg("comment") = "")
		.def_readonly("epipolarError", &dv::camera::calibrations::StereoCalibration::Metadata::epipolarError,
			"Average epipolar error (can be None if unknown)")
		.def_readonly(
			"comment", &dv::camera::calibrations::StereoCalibration::Metadata::comment, "Any additional information");

	stereoCalibration.def(py::init<>())
		.def(py::init<const std::string &, const std::string &, const std::vector<float> &, const std::vector<float> &,
			const std::optional<dv::camera::calibrations::StereoCalibration::Metadata> &>())
		.def_readonly(
			"leftCameraName", &dv::camera::calibrations::StereoCalibration::leftCameraName, "Name of the left camera")
		.def_readonly("rightCameraName", &dv::camera::calibrations::StereoCalibration::rightCameraName,
			"Name of the right camera")
		.def_readonly("fundamentalMatrix", &dv::camera::calibrations::StereoCalibration::fundamentalMatrix,
			"Stereo calibration Fundamental Matrix")
		.def_readonly("essentialMatrix", &dv::camera::calibrations::StereoCalibration::essentialMatrix,
			"Stereo calibration Essential Matrix")
		.def_readonly("metadata", &dv::camera::calibrations::StereoCalibration::metadata, "Metadata")
		.def("getFundamentalMatrix", &dv::camera::calibrations::StereoCalibration::getFundamentalMatrix)
		.def("getEssentialMatrix", &dv::camera::calibrations::StereoCalibration::getEssentialMatrix);

	py::class_<dv::camera::CalibrationSet>(m_camera, "CalibrationSet")
		.def(py::init<>())
		.def_static("LoadFromFile", &dv::camera::CalibrationSet::LoadFromFile, py::arg("path"),
			"Create a calibration file representation from a persistent file. Supports legacy '.xml' calibration "
			"files produced by DV as well as JSON files containing calibration of a new format. The file format is "
			"distinguished using the file path extension.")
		.def("getCameraList", &dv::camera::CalibrationSet::getCameraList,
			"Get a list of cameras available by their designation.")
		.def("getImuList", &dv::camera::CalibrationSet::getImuList,
			"Get a list camera designations which have imu calibrations available in this calibration set.")
		.def("getStereoList", &dv::camera::CalibrationSet::getStereoList,
			"Get a list of designations of stereo calibrations available here.")
		.def("getCameraCalibrations", &dv::camera::CalibrationSet::getCameraCalibrations,
			"Retrieve the full list of camera intrinsic calibrations.")
		.def("getImuCalibrations", &dv::camera::CalibrationSet::getImuCalibrations,
			"Retrieve the full list of IMU extrinsic calibrations.")
		.def("getStereoCalibrations", &dv::camera::CalibrationSet::getStereoCalibrations,
			"Retrieve the full list of stereo extrinsic calibrations.")
		.def("getCameraCalibration", &dv::camera::CalibrationSet::getCameraCalibration, py::arg("designation"),
			"Retrieve a camera calibration by designation (e.g. 'C0').")
		.def("getImuCalibration", &dv::camera::CalibrationSet::getImuCalibration, py::arg("designation"),
			"Get IMU calibration by IMU sensor designation (e.g. 'S0').")
		.def("getStereoCalibration", &dv::camera::CalibrationSet::getStereoCalibration, py::arg("designation"),
			"Get stereo calibration by stereo rig designation (e.g. 'C0C1').")
		.def("getCameraCalibrationByName", &dv::camera::CalibrationSet::getCameraCalibrationByName,
			py::arg("cameraName"),
			"Retrieve a camera calibration by camera name, which consist of model and serial number concatenation with "
			"an underscore separator (e.g. 'DVXplorer_DXA00000'). Camera name is usually available in recording "
			"files and when connected directly to a camera.")
		.def("getImuCalibrationByName", &dv::camera::CalibrationSet::getImuCalibrationByName, py::arg("cameraName"),
			"Retrieve an IMU calibration by camera name, which consist of model and serial number concatenation with "
			"an underscore separator (e.g. 'DVXplorer_DXA00000'). Camera name is usually available in recording "
			"files and when connected directly to a camera.")
		.def("getStereoCalibrationByLeftCameraName", &dv::camera::CalibrationSet::getStereoCalibrationByLeftCameraName,
			py::arg("cameraName"),
			"Retrieve a stereo calibration by matching camera name to left camera name in the stereo calibrations. "
			"Camera name consist of model and serial number concatenation with an underscore separator (e.g. "
			"'DVXplorer_DXA00000'). Camera name is usually available in recording files and when connected directly "
			"to a camera.")
		.def("getStereoCalibrationByRightCameraName",
			&dv::camera::CalibrationSet::getStereoCalibrationByRightCameraName, py::arg("cameraName"),
			"Retrieve a stereo calibration by matching camera name to right camera name in the stereo calibrations. "
			"Camera name consist of model and serial number concatenation with an underscore separator (e.g. "
			"'DVXplorer_DXA00000'). Camera name is usually available in recording files and when connected directly "
			"to a camera.")
		.def("addCameraCalibration", &dv::camera::CalibrationSet::addCameraCalibration, py::arg("calibration"),
			"Add an intrinsic calibration to the camera calibration set. Camera designation is going to be generated "
			"automatically.")
		.def("addImuCalibration", &dv::camera::CalibrationSet::addImuCalibration, py::arg("calibration"),
			"Add an IMU extrinsics calibration to the calibration set.")
		.def("addStereoCalibration", &dv::camera::CalibrationSet::addStereoCalibration, py::arg("calibration"),
			"Add a stereo calibration to the calibration set. Intrinsic calibrations of the sensors should already be "
			"added using `addCameraCalibration` prior to adding the stereo extrinsic calibration.")
		.def("updateImuCalibration", &dv::camera::CalibrationSet::updateImuCalibration, py::arg("calibration"),
			"Update an IMU calibration in the calibration set.")
		.def("updateCameraCalibration", &dv::camera::CalibrationSet::updateCameraCalibration, py::arg("calibration"),
			"Update a camera intrinsic calibration in the calibration set.")
		.def("updateStereoCameraCalibration", &dv::camera::CalibrationSet::updateStereoCameraCalibration,
			py::arg("calibration"), "Update a stereo calibration in the calibration set.")
		.def("writeToFile", &dv::camera::CalibrationSet::writeToFile, py::arg("outputFile"),
			"Write the contents of this calibration set into a file at given path. This function requires that "
			"supplied path contains '.json' extension.");

	py::class_<dv::camera::StereoGeometry, std::shared_ptr<dv::camera::StereoGeometry>> stereoGeometryClass(
		m_camera, "StereoGeometry");

	py::enum_<dv::camera::StereoGeometry::CameraPosition>(stereoGeometryClass, "CameraPosition")
		.value("Left", dv::camera::StereoGeometry::CameraPosition::Left)
		.value("Right", dv::camera::StereoGeometry::CameraPosition::Right)
		.export_values();

	stereoGeometryClass
		.def(py::init<const dv::camera::CameraGeometry &, const dv::camera::CameraGeometry &,
				 const std::vector<float> &, std::optional<cv::Size>>(),
			"leftCamera"_a, "rightCamera"_a, "transformToLeft"_a, "rectifiedResolution"_a = std::nullopt)
		.def(py::init<const dv::camera::calibrations::CameraCalibration &,
				 const dv::camera::calibrations::CameraCalibration &>(),
			"leftCalibration"_a, "rightCalibration"_a)
		.def("remapImage", &dv::camera::StereoGeometry::remapImage, "cameraPosition"_a, "image"_a,
			"Apply remapping to an input image to rectify it.")
		.def("remapEvents", &dv::camera::StereoGeometry::remapEvents, "cameraPosition"_a, "events"_a,
			"Apply remapping of input events coordinates into undistorted and rectified pixel space.")
		.def("remapPoint", &dv::camera::StereoGeometry::remapPoint<Eigen::Vector2i, Eigen::Vector2i>,
			"cameraPosition"_a, "point"_a,
			"Apply remapping on a single coordinate into undistorted and rectified pixel space.")
		.def(
			"unmapPoint",
			[](const dv::camera::StereoGeometry &self, const dv::camera::StereoGeometry::CameraPosition position,
				const Eigen::Vector2f &point, const bool subPixel) {
				if (subPixel) {
					return self
						.unmapPoint<Eigen::Vector2f, dv::camera::StereoGeometry::FunctionImplementation::SubPixel>(
							position, point);
				}
				else {
					return self.unmapPoint<Eigen::Vector2f>(position, point);
				}
			},
			"cameraPosition"_a, "point"_a, "subPixel"_a = false,
			"Unmap a point coordinate from undistorted and rectified pixel space into original distorted pixel.")
		.def("getLeftCameraGeometry", &dv::camera::StereoGeometry::getLeftCameraGeometry,
			"Retrieve left camera geometry class that can project coordinates into stereo rectified space.")
		.def("getRightCameraGeometry", &dv::camera::StereoGeometry::getRightCameraGeometry,
			"Retrieve right camera geometry class that can project coordinates into stereo rectified space.")
		.def("estimateDepth", &dv::camera::StereoGeometry::estimateDepth, "disparity"_a, "events"_a,
			"disparityScale"_a = 16.f,
			"Estimate depth given the disparity map and a list of events. The coordinates will be rectified and a "
			"disparity value will be looked up in the disparity map. The depth of each event is calculated using an "
			"equation: depth = (focalLength * baseline) / disparity. Focal length and disparity are expressed in "
			"pixels. For practical applications, depth estimation should be evaluated prior to any use. The directly "
			"estimated depth values can contain measurable errors which should be accounted for - the errors can "
			"usually be within 10-20% fixed absolute error distance. Usually this comes from various inaccuracies and "
			"can be mitigated by introducing a correction factor for the depth estimate.")
		.def("toDepthFrame", &dv::camera::StereoGeometry::toDepthFrame, "disparity"_a, "disparityScale"_a = 16.f,
			"Convert a disparity map into a depth frame. Each disparity value is converted into depth using the "
			"equation depth = (focalLength * baseline) / disparity. Output frame contains distance "
			"values expressed in integer values of millimeter distance.");

	auto m_kinematics = m.def_submodule("kinematics");
	py::class_<dv::kinematics::Transformationf>(m_kinematics, "Transformationf",
		"Basic transformation wrapper containing homogenous 3D transformation and timestamp.")
		.def(py::init<>(), "Construct an identity transformation from with timestamp.")
		.def(py::init<int64_t, const Eigen::Matrix4f &>(), py::arg("timestamp"), py::arg("transformation"),
			"Construct the transformation from a timestamp and a homogenous 4x4 transformation matrix.")
		.def(py::init([](int64_t timestamp, const Eigen::Vector3f &translation, const dv::Quaternion &rotation) {
			return dv::kinematics::Transformationf(
				timestamp, translation, Eigen::Quaternionf(rotation.w(), rotation.x(), rotation.y(), rotation.z()));
		}),
			py::arg("timestamp"), py::arg("translation"), py::arg("rotation"),
			"Construct the transformation from timestamp, 3D translation vector and quaternion describing the "
			"rotation.")
		.def(py::init<int64_t, const Eigen::Vector3f &, const Eigen::Matrix3f &>(), py::arg("timestamp"),
			py::arg("translation"), py::arg("rotation"),
			"Construct the transformation from timestamp, 3D translation vector and a rotation matrix describing the "
			"rotation.")
		.def("getTimestamp", &dv::kinematics::Transformationf::getTimestamp,
			"Unix timestamp of the transformation in microseconds.")
		.def("getTransform", &dv::kinematics::Transformationf::getTransform, "Get the transformation matrix.")
		.def("getRotationMatrix", &dv::kinematics::Transformationf::getRotationMatrix,
			"Retrieve a copy of 3x3 rotation matrix.")
		.def(
			"getQuaternion",
			[](const dv::kinematics::Transformationf &self) {
				auto quat = self.getQuaternion();
				return dv::Quaternion(quat.w(), quat.x(), quat.y(), quat.z());
			},
			"Retrieve rotation expressed as a quaternion.")
		.def("getTranslation", &dv::kinematics::Transformationf::getTranslation<Eigen::Vector3f>,
			"Retrieve translation as 3D vector.")
		.def("transformPoint", &dv::kinematics::Transformationf::transformPoint<Eigen::Vector3f, Eigen::Vector3f>,
			py::arg("point"), "Transform a point using this transformation.")
		.def("rotatePoint", &dv::kinematics::Transformationf::rotatePoint<Eigen::Vector3f, Eigen::Vector3f>,
			py::arg("point"), "Apply rotation only transformation on the given point.")
		.def("inverse", &dv::kinematics::Transformationf::inverse,
			"Calculate the inverse homogenous transformation of this transform.")
		.def("delta", &dv::kinematics::Transformationf::delta, py::arg("target"),
			"Get a transformation from this transformation into the given other transformation.");

	py::class_<dv::kinematics::LinearTransformerf>(m_kinematics, "LinearTransformerf",
		"A buffer containing time increasing 3D transformations and capable of timewise linear interpolation between "
		"available transforms. Can be used with different underlying floating point types supported by Eigen.")
		.def(py::init<size_t>(), py::arg("capacity"))
		.def("pushTransformation", &dv::kinematics::LinearTransformerf::pushTransformation, py::arg("transformation"),
			"Push a transformation into the transformation buffer.")
		.def("clear", &dv::kinematics::LinearTransformerf::clear, "Delete all transformations from the buffer.")
		.def("empty", &dv::kinematics::LinearTransformerf::empty, "Check whether the buffer is empty.")
		.def("getTransformAt", &dv::kinematics::LinearTransformerf::getTransformAt, py::arg("timestamp"),
			"Get a transform at the given timestamp. If no transform with the exact timestamp was pushed, estimates a "
			"transform assuming linear motion.")
		.def("isWithinTimeRange", &dv::kinematics::LinearTransformerf::isWithinTimeRange, py::arg("timestamp"),
			"Checks whether the timestamp is within the range of transformations available in the buffer.")
		.def("size", &dv::kinematics::LinearTransformerf::size, "Return the size of the buffer.")
		.def("latestTransformation", &dv::kinematics::LinearTransformerf::latestTransformation,
			"Return transformation with highest timestamp.")
		.def("earliestTransformation", &dv::kinematics::LinearTransformerf::earliestTransformation,
			"Return transformation with lowest timestamp.")
		.def("setCapacity", &dv::kinematics::LinearTransformerf::setCapacity,
			"Set new capacity, if the size of the buffer is larger than the newCapacity, oldest transformations from "
			"the start will be removed.")
		.def("getTransformsBetween", &dv::kinematics::LinearTransformerf::getTransformsBetween, py::arg("start"),
			py::arg("end"),
			"Extract transformation between two given timestamps. If timestamps are not at exact available "
			"transformations, additional transformations will be added so the resulting transformer would complete "
			"overlap over the period (if that is possible).")
		.def("resampleTransforms", &dv::kinematics::LinearTransformerf::resampleTransforms, py::arg("samplingInterval"),
			"Resample containing transforms into a new transformer, containing interpolated transforms at given "
			"interval. Will contain the last transformation as well, although the interval might not be maintained for "
			"the last transform.");

	py::class_<dv::kinematics::PixelMotionPredictor>(m_kinematics, "PixelMotionPredictor",
		"A buffer containing time increasing 3D transformations and capable of timewise linear interpolation between "
		"available transforms. Can be used with different underlying floating point types supported by Eigen.")
		.def(py::init<std::shared_ptr<dv::camera::CameraGeometry>>(), py::arg("cameraGeometry"),
			"Construct pixel motion predictor class.")
		.def("predictEvents", &dv::kinematics::PixelMotionPredictor::predictEvents, py::arg("events"), py::arg("dT"),
			py::arg("depth"),
			"Apply delta transformation to event input and generate new transformed event store with new events that "
			"are within the new camera perspective (after applying delta transform).")
		.def("predict", &dv::kinematics::PixelMotionPredictor::predict<Eigen::Vector2f, Eigen::Vector2f>,
			py::arg("point"), py::arg("dT"), py::arg("depth"),
			"Reproject given pixel coordinates using the delta transformation and depth.")
		.def("predictSequence",
			&dv::kinematics::PixelMotionPredictor::predictSequence<std::vector<Eigen::Vector2f>,
				std::vector<Eigen::Vector2f>>,
			py::arg("points"), py::arg("dT"), py::arg("depth"),
			"Apply delta transformation to coordinate input and generate new transformed coordinate array with new "
			"coordinates that are within the new camera perspective (after applying delta transform).")
		.def("isUseDistortion", &dv::kinematics::PixelMotionPredictor::isUseDistortion,
			"Is the distortion model enabled for the reprojection of coordinates.")
		.def("setUseDistortion", &dv::kinematics::PixelMotionPredictor::setUseDistortion, py::arg("useDistortion"),
			"Enable of disable the usage of a distortion model.");

	py::class_<dv::kinematics::MotionCompensator<>::Info>(m_kinematics, "MotionCompensatorInfo")
		.def_readwrite("imageCompensated", &dv::kinematics::MotionCompensator<>::Info::imageCompensated)
		.def_readwrite("depthAvailable", &dv::kinematics::MotionCompensator<>::Info::depthAvailable)
		.def_readwrite("transformsAvailable", &dv::kinematics::MotionCompensator<>::Info::transformsAvailable)
		.def_readwrite("depthTime", &dv::kinematics::MotionCompensator<>::Info::depthTime)
		.def_readwrite("generationTime", &dv::kinematics::MotionCompensator<>::Info::generationTime)
		.def_readwrite("inputEventCount", &dv::kinematics::MotionCompensator<>::Info::inputEventCount)
		.def_readwrite("accumulatedEventCount", &dv::kinematics::MotionCompensator<>::Info::accumulatedEventCount);

	py::class_<dv::kinematics::MotionCompensator<>>(m_kinematics, "MotionCompensator")
		.def(py::init<std::shared_ptr<dv::camera::CameraGeometry>>(), py::arg("cameraGeometry"),
			"Construct a motion compensator instance with default accumulator. Default accumulator is a "
			"`dv::EdgeMapAccumulator` with default parameters.")
		.def(py::init([](const std::shared_ptr<dv::camera::CameraGeometry> &cameraGeometry,
						  const dv::EdgeMapAccumulator &acc) {
			return dv::kinematics::MotionCompensator<>(cameraGeometry, std::make_unique<dv::EdgeMapAccumulator>(acc));
		}))
		.def("getInfo", &dv::kinematics::MotionCompensator<>::getInfo,
			"Return an info class instance containing motion compensator state for the algorithm iteration. The info "
			"object contains debug information about the execution of the motion compensator.")
		.def("generateFrame", &dv::kinematics::MotionCompensator<>::generateFrame, py::arg("generationTime") = -1,
			"Generate the motion compensated frame output and reset the events contained in the buffer.")
		.def("generateEvents", &dv::kinematics::MotionCompensator<>::generateEvents, py::arg("generationTime") = -1,
			"Generate the motion compensated events contained in the buffer.")
		.def("reset", &dv::kinematics::MotionCompensator<>::reset, "Clear the event buffer.")
		.def(
			"accept",
			[](dv::kinematics::MotionCompensator<> &self, const dv::kinematics::Transformationf &transform) {
				self.accept(transform);
			},
			py::arg("transform"), "Push camera pose measurement.")
		.def(
			"accept",
			[](dv::kinematics::MotionCompensator<> &self, const dv::measurements::Depth &timeDepth) {
				self.accept(timeDepth);
			},
			py::arg("timeDepth"), "Scene depth measurement in meters.")
		.def(
			"accept",
			[](dv::kinematics::MotionCompensator<> &self, const dv::EventStore &events) {
				self.accept(events);
			},
			py::arg("events"), "Push event camera input.")
		.def(
			"accept",
			[](dv::kinematics::MotionCompensator<> &self, const dv::Event &event) {
				self.accept(event);
			},
			py::arg("event"), "Push event camera input.")
		.def("getConstantDepth", &dv::kinematics::MotionCompensator<>::getConstantDepth,
			"Get currently assumed constant depth value. It is used if no depth measurements are provided.")
		.def("setConstantDepth", &dv::kinematics::MotionCompensator<>::setConstantDepth, "depth"_a,
			"Set constant depth value that is assumed if no depth measurement is passed using "
			"`accept(dv::measurements::Depth)`. By default the constant depth is assumed to be 3.0 meters, which is "
			"just a reasonable guess.");

	auto m_visualization = m.def_submodule("visualization");
	py::class_<dv::visualization::PoseVisualizer>(
		m_visualization, "PoseVisualizer", "Visualize the current and past poses as an image.")
		.def(py::init<size_t, const cv::Size &>(), py::arg("trajectoryLength") = 10000,
			py::arg("resolution") = cv::Size(640, 480))
		.def("updateCameraPosition", &dv::visualization::PoseVisualizer::updateCameraPosition, py::arg("newPosition"),
			"Update the position in which camera is located.")
		.def(
			"setViewMode",
			[](dv::visualization::PoseVisualizer &self, const std::string &mode) {
				return self.setViewMode(mode);
			},
			py::arg("mode"), "Set the mode in which the pose viewer will be working.")
		.def(
			"setGridPlane",
			[](dv::visualization::PoseVisualizer &self, const std::string &plane) {
				return self.setGridPlane(plane);
			},
			py::arg("plane"), "Set the plane on which the grid will be displayed.")
		.def("updateCameraOrientation", &dv::visualization::PoseVisualizer::updateCameraOrientation, py::arg("yawDeg"),
			py::arg("pitchDeg"), py::arg("rollDeg"),
			"Update the orientation of the camera expressed as XYZ Euler angles.")
		.def("setFrameSize", &dv::visualization::PoseVisualizer::setFrameSize, py::arg("newSize"),
			"Update the size of output image.")
		.def("setCoordinateDimensions", &dv::visualization::PoseVisualizer::setCoordinateDimensions, py::arg("newSize"),
			"Update the displayed coordinate frame size.")
		.def("setLineThickness", &dv::visualization::PoseVisualizer::setLineThickness, py::arg("newThickness"),
			"Update the line thickness of the drawing.")
		.def(
			"accept",
			[](dv::visualization::PoseVisualizer &self, const dv::kinematics::Transformationf &pose) {
				self.accept(pose);
			},
			"pose"_a, "Add a new pose to the visualization.")
		.def(
			"accept",
			[](dv::visualization::PoseVisualizer &self, const dv::LandmarksPacket &landmarks) {
				self.accept(landmarks);
			},
			"landmarks"_a,
			"Add landmarks to the visualizer, they will be drawn as points with optional lines to currently visible "
			"landmarks.")
		.def("getTimestamp", &dv::visualization::PoseVisualizer::getTimestamp,
			"Return the timestamp of the most recent pose.")
		.def("generateFrame", &dv::visualization::PoseVisualizer::generateFrame, "Return a visualization image.")
		.def("reset", &dv::visualization::PoseVisualizer::reset,
			"Reset the pose history and set an offset to the last pose.")
		.def("getBackgroundColor", &dv::visualization::PoseVisualizer::getBackgroundColor, "Get the background color.")
		.def("setBackgroundColor", &dv::visualization::PoseVisualizer::setBackgroundColor, py::arg("color"),
			"Set new background color.")
		.def("getGridColor", &dv::visualization::PoseVisualizer::getGridColor, "Get the grid line color.")
		.def("setGridColor", &dv::visualization::PoseVisualizer::setGridColor, py::arg("color"),
			"Set new grid line color")
		.def("getDrawLinesToLandmarks", &dv::visualization::PoseVisualizer::getDrawLinesToLandmarks,
			"Check whether drawing of lines to landmark markers is enabled.")
		.def("setDrawLinesToLandmarks", &dv::visualization::PoseVisualizer::setDrawLinesToLandmarks,
			"drawLinesToLandmarks"_a,
			"Enable or disable drawing of lines from camera to active landmarks. Active landmarks are those which were "
			"accepted by the visualizer with last `accept(dv::LandmarksPacket)` call.")
		.def("getLandmarkLimit", &dv::visualization::PoseVisualizer::getLandmarkLimit,
			"Get the maximum number of landmarks to be drawn.")
		.def("setLandmarkLimit", &dv::visualization::PoseVisualizer::setLandmarkLimit, "landmarkLimit"_a,
			"Set a limit for number of landmarks that are stored and drawn.")
		.def("getLandmarkSize", &dv::visualization::PoseVisualizer::getLandmarkSize,
			"Get the number of landmarks currently stored in the visualizer.")
		.def("clearLandmarks", &dv::visualization::PoseVisualizer::clearLandmarks,
			"Remove all landmarks stored in the landmarks buffer.");

	auto m_colors = m_visualization.def_submodule("colors");
	m_colors.def("black", []() {
		return dv::visualization::colors::black;
	});
	m_colors.def("white", []() {
		return dv::visualization::colors::white;
	});
	m_colors.def("red", []() {
		return dv::visualization::colors::red;
	});
	m_colors.def("lime", []() {
		return dv::visualization::colors::lime;
	});
	m_colors.def("blue", []() {
		return dv::visualization::colors::blue;
	});
	m_colors.def("yellow", []() {
		return dv::visualization::colors::yellow;
	});
	m_colors.def("silver", []() {
		return dv::visualization::colors::silver;
	});
	m_colors.def("gray", []() {
		return dv::visualization::colors::gray;
	});
	m_colors.def("navy", []() {
		return dv::visualization::colors::navy;
	});
	m_colors.def("green", []() {
		return dv::visualization::colors::green;
	});
	m_colors.def("darkGrey", []() {
		return dv::visualization::colors::darkGrey;
	});
	m_colors.def("darkgrey", []() {
		PyErr_WarnEx(PyExc_DeprecationWarning, "darkgrey() is deprecated, use darkGrey() instead.", 1);
		return dv::visualization::colors::darkGrey;
	});
	m_colors.def("iniBlue", []() {
		return dv::visualization::colors::iniBlue;
	});
	m_colors.def("iniblue", []() {
		PyErr_WarnEx(PyExc_DeprecationWarning, "iniblue() is deprecated, use iniBlue() instead.", 1);
		return dv::visualization::colors::iniBlue;
	});
	m_colors.def("someNeonColor", &dv::visualization::colors::someNeonColor, "someNumber"_a);

	py::class_<dv::visualization::EventVisualizer>(m_visualization, "EventVisualizer", "Visualize events.")
		.def(py::init<const cv::Size &, const cv::Scalar &, const cv::Scalar &, const cv::Scalar &>(), "resolution"_a,
			"backgroundColor"_a = dv::visualization::colors::white,
			"positiveColor"_a   = dv::visualization::colors::iniBlue,
			"negativeColor"_a   = dv::visualization::colors::darkGrey)
		.def(
			"generateImage",
			[](dv::visualization::EventVisualizer &self, const dv::EventStore &events) {
				return self.generateImage(events);
			},
			"events"_a, "Generate a preview image from an event store.")
		.def(
			"generateImage",
			[](dv::visualization::EventVisualizer &self, const dv::EventStore &events, cv::Mat &background) {
				self.generateImage(events, background);
				return background;
			},
			"events"_a, "background"_a,
			"Generate a preview image of a given event store on the given background. Return the generated image "
			"preview.")
		.def("getBackgroundColor", &dv::visualization::EventVisualizer::getBackgroundColor,
			"Get currently configured background color.")
		.def("setBackgroundColor", &dv::visualization::EventVisualizer::setBackgroundColor, "backgroundColor"_a,
			"Set new background color.")
		.def("getPositiveColor", &dv::visualization::EventVisualizer::getPositiveColor,
			"Get currently configured positive polarity color.")
		.def("setPositiveColor", &dv::visualization::EventVisualizer::setPositiveColor, "positiveColor"_a,
			"Set new positive polarity color.")
		.def("getNegativeColor", &dv::visualization::EventVisualizer::getNegativeColor, "Get negative polarity color.")
		.def("setNegativeColor", &dv::visualization::EventVisualizer::setNegativeColor, "negativeColor"_a,
			"Set new negative polarity color.");

	auto m_imgproc = m.def_submodule("imgproc");
	m_imgproc.def(
		"L1distance",
		[](const cv::Mat &i1, const cv::Mat &i2) {
			return dv::imgproc::L1Distance(i1, i2);
		},
		py::arg("image1"), py::arg("image1"), "Computes the L1 distance between two matrices");
	m_imgproc.def(
		"pearsonCorrelation",
		[](const cv::Mat &i1, const cv::Mat &i2) {
			return dv::imgproc::pearsonCorrelation(i1, i2);
		},
		py::arg("image1"), py::arg("image1"), "Computes the Pearson Correlation between two matrices");
	m_imgproc.def(
		"cosineDistance",
		[](const cv::Mat &i1, const cv::Mat &i2) {
			return dv::imgproc::cosineDistance(i1, i2);
		},
		py::arg("image1"), py::arg("image1"), "Computes the Cosine Distance between two matrices");

	auto m_features = m.def_submodule("features");

	py::class_<dv::features::LucasKanadeConfig>(
		m_features, "LucasKanadeConfig", "Lucas-Kanade tracker configuration parameters.")
		.def(py::init<>())
		.def("__repr__",
			[](const dv::features::LucasKanadeConfig &config) -> std::string {
				return "LucasKanadeConfig()";
			})
		.def_readwrite("maskedFeatureDetect", &dv::features::LucasKanadeConfig::maskedFeatureDetect)
		.def_readwrite("terminationEpsilon", &dv::features::LucasKanadeConfig::terminationEpsilon)
		.def_readwrite("numPyrLayers", &dv::features::LucasKanadeConfig::numPyrLayers)
		.def_readwrite("searchWindowSize", &dv::features::LucasKanadeConfig::searchWindowSize);

	py::class_<dv::features::TrackerBase::Result, std::shared_ptr<dv::features::TrackerBase::Result>>(
		m_features, "TrackingResult")
		.def_readonly("timestamp", &dv::features::TrackerBase::Result::timestamp)
		.def_readonly("keypoints", &dv::features::TrackerBase::Result::keypoints)
		.def_readonly("asKeyFrame", &dv::features::TrackerBase::Result::asKeyFrame);

	py::class_<dv::features::TrackerBase>(m_features, "TrackerBase",
		"A base class for implementing feature trackers, that track sets of features against streams of various "
		"inputs. This class specifically does not define an input type, so it could be defined by the specific "
		"implementation.")
		.def("setMaxTracks", &dv::features::TrackerBase::setMaxTracks, py::arg("maxTracks"),
			"Set the maximum number of tracks.")
		.def("getMaxTracks", &dv::features::TrackerBase::getMaxTracks, "Get the maximum number of tracks.")
		.def("getLastFrameResults", &dv::features::TrackerBase::getLastFrameResults,
			"Retrieve cached last frame detection results.")
		.def("runTracking", &dv::features::TrackerBase::runTracking, "Perform the tracking and cache the results.")
		.def("removeTracks", &dv::features::TrackerBase::removeTracks, py::arg("trackIds"),
			"Remove tracks from cached results, so the wouldn't be tracked anymore. TrackIds are the `class_id` value "
			"of the keypoint structure.");

	py::enum_<dv::features::ImageFeatureDetector::FeaturePostProcessing>(m_features, "FeaturePostProcessing")
		.value("NONE", dv::features::ImageFeatureDetector::FeaturePostProcessing::None)
		.value("TopN", dv::features::ImageFeatureDetector::FeaturePostProcessing::TopN)
		.value("AdaptiveNMS", dv::features::ImageFeatureDetector::FeaturePostProcessing::AdaptiveNMS)
		.export_values();

	py::class_<dv::features::ImagePyramid>(
		m_features, "ImagePyramid", "Class that holds image pyramid layers with an according timestamp.")
		.def(py::init<const dv::Frame &, const cv::Size &, int>(), py::arg("frame"), py::arg("winSize"),
			py::arg("maxPyrLevel"), "Construct the image pyramid.")
		.def(py::init<int64_t, const cv::Mat &, const cv::Size &, int>(), py::arg("timestamp"), py::arg("image"),
			py::arg("winSize"), py::arg("maxPyrLevel"), "Construct the image pyramid.")
		.def(py::init<int64_t, const cv::Mat &>(), py::arg("timestamp"), py::arg("image"),
			"Create a single layer image representation (no pyramid is going to be built).")
		.def_readwrite("timestamp", &dv::features::ImagePyramid::timestamp, "Timestamp of the image pyramid.")
		.def_readwrite("pyramid", &dv::features::ImagePyramid::pyramid, "Pyramid layers of the image.");

	py::class_<dv::features::KeyPointResampler>(m_features, "KeyPointResampler",
		"Create a feature resampler, which resamples given keypoints with homogenous distribution in pixel space.")
		.def(
			py::init<const cv::Size &>(), py::arg("resolution"), "Initialize resampler with an input image resolution.")
		.def("resample", &dv::features::KeyPointResampler::resample<std::vector<dv::TimedKeyPoint>>,
			py::arg("keyPoints"), py::arg("numRetPoints"), "Perform resampling on given keypoints.")
		.def("getTolerance", &dv::features::KeyPointResampler::getTolerance,
			"Get currently set tolerance for output keypoint count.")
		.def("setTolerance", &dv::features::KeyPointResampler::setTolerance, py::arg("tolerance"),
			"Set a new output size tolerance value. The algorithm search for an optimal distance between keypoints so "
			"the resulting vector would contain the expected amount of keypoints. This search is performed with a "
			"given tolerance, by default - 0.1 (so by default the final resampled amount of events will be within "
			"+/-10% of requested amount).");

	py::class_<dv::features::NoRedetection>(m_features, "NoRedetection", "No redetection strategy.")
		.def(py::init<>())
		.def("decideRedetection", &dv::features::NoRedetection::decideRedetection);

	py::class_<dv::features::FeatureCountRedetection>(
		m_features, "FeatureCountRedetection", "Redetection strategy based on number of features.")
		.def(py::init<float>(), py::arg("numberOfFeatures"),
			"Redetection strategy based on number of features. Pass feature count coefficient, redetection is "
			"performed when feature count goes lower than the given proportion of maximum tracks, redetection will be "
			"executed.")
		.def("decideRedetection", &dv::features::FeatureCountRedetection::decideRedetection, py::arg("tracker"),
			"Check whether to perform redetection.");

	py::class_<dv::features::UpdateIntervalRedetection>(
		m_features, "UpdateIntervalRedetection", "Redetection strategy based on interval from last detection.")
		.def(py::init([](const dv::Duration updateInterval) {
			return dv::features::UpdateIntervalRedetection(updateInterval);
		}))
		.def("decideRedetection", &dv::features::UpdateIntervalRedetection::decideRedetection, py::arg("tracker"),
			"Check whether to perform redetection.");

	py::class_<dv::features::UpdateIntervalOrFeatureCountRedetection>(m_features,
		"UpdateIntervalOrFeatureCountRedetection",
		"Redetection strategy based on interval from last detection or minimum number of tracks. This class combines"
		" redetection logic from UpdateIntervalRedetection and FeatureCountRedetection.")
		.def(py::init([](const dv::Duration updateInterval, const float numberOfFeatures) {
			return dv::features::UpdateIntervalOrFeatureCountRedetection(updateInterval, numberOfFeatures);
		}))
		.def("decideRedetection", &dv::features::UpdateIntervalOrFeatureCountRedetection::decideRedetection,
			py::arg("tracker"), "Check whether to perform redetection.");

	py::class_<dv::features::FeatureTracks>(m_features, "FeatureTracks",
		"A class to store a time limited amount of feature tracks. Sorts and stores the data in separate queues for "
		"each track id. Provides `visualize` function to generate visualization images of the tracks.")
		.def(py::init<>())
		.def(
			"accept",
			[](dv::features::FeatureTracks &self, const dv::TimedKeyPoint &keypoint) {
				self.accept(keypoint);
			},
			py::arg("keypoint"), "Add a keypoint measurement into the feature track.")
		.def(
			"accept",
			[](dv::features::FeatureTracks &self, const dv::TimedKeyPointPacket &keypoints) {
				self.accept(keypoints);
			},
			py::arg("keypoints"), "Add a batch of keypoints measurement into the feature track.")
		.def(
			"accept",
			[](dv::features::FeatureTracks &self, const dv::features::TrackerBase::Result::ConstPtr &trackingResult) {
				self.accept(trackingResult);
			},
			py::arg("trackingResult"), "Add keypoint tracking result from a tracker.")
		.def("getHistoryDuration", &dv::features::FeatureTracks::getHistoryDuration, "Retrieve the history duration.")
		.def("setHistoryDuration", &dv::features::FeatureTracks::setHistoryDuration, py::arg("historyDuration"),
			"Set new history duration limit to buffer. If the new limit is shorter than the previously set, the tracks "
			"will be reduced to the new limit right away.")
		.def(
			"getTrack",
			[](const dv::features::FeatureTracks &self,
				const int trackId) -> std::optional<std::vector<dv::TimedKeyPoint>> {
				if (const auto track = self.getTrack(trackId); track.has_value()) {
					return std::vector<dv::TimedKeyPoint>(track.value()->begin(), track.value()->end());
				}
				else {
					return std::nullopt;
				}
			},
			py::arg("trackId"), "Retrieve a track of given track id.")
		.def("getTrackIds", &dv::features::FeatureTracks::getTrackIds,
			"Return all track ids that are available in the buffer.")
		.def("getLatestTrackKeypoints", &dv::features::FeatureTracks::getLatestTrackKeypoints,
			"Return last keypoint from all tracks in the history.")
		.def("visualize", &dv::features::FeatureTracks::visualize, py::arg("background"),
			"Draws tracks on the input image, by default uses neon color palette from the `dv::visualization::colors` "
			"namespace for each of the tracks.")
		.def("isEmpty", &dv::features::FeatureTracks::isEmpty,
			"Checks whether the feature track history buffer is empty.")
		.def("clear", &dv::features::FeatureTracks::clear,
			"Deletes any data stored in feature track buffer and resets visualization image.")
		.def("getHighestTime", &dv::features::FeatureTracks::getHighestTime,
			"Return latest time from all existing tracks.");

	py::class_<dv::features::ImageFeatureDetector>(m_features, "ImageFeatureDetector",
		"A base class to implement feature detectors on different input types, specifically either images, time "
		"surfaces, or event stores. The implementing class should override the `detect` function and output a vector "
		"of unordered features with a quality score. The API will handle margin calculations and post processing of "
		"the features.")
		.def(py::init([](const cv::Size &resolution) {
			return dv::features::ImageFeatureDetector(resolution, cv::GFTTDetector::create());
		}),
			"resolution"_a,
			"Create a detector that uses good-features-to-track (GFTT) algorithm for feature detection. This "
			"constructor defaults post-processing step to none as GFTT does NMS already and a margin coefficient value "
			"of 0.02.")
		.def("runDetection", &dv::features::ImageFeatureDetector::runDetection, py::arg("input"), py::arg("numPoints"),
			py::arg("mask") = cv::Mat(),
			"Public detection call. Calls the overloaded `detect` function, applies margin and post processing.")
		.def("runRedetection", &dv::features::ImageFeatureDetector::runRedetection, py::arg("prior"), py::arg("input"),
			py::arg("numPoints"), py::arg("mask") = cv::Mat(),
			"Redetect new features and add them to already detected features. This function performs detection within "
			"masked region (if mask is non-empty), runs postprocessing and appends the additional features to the "
			"prior keypoint list.")
		.def("getPostProcessing", &dv::features::ImageFeatureDetector::getPostProcessing,
			"Get the type of post-processing.")
		.def("setPostProcessing", &dv::features::ImageFeatureDetector::setPostProcessing, py::arg("postProcessing"),
			"Set the type of post-processing.")
		.def("getMargin", &dv::features::ImageFeatureDetector::getMargin,
			"Get currently applied margin coefficient. Margin coefficient is multiplied by the width and height of the "
			"image to calculate an adaptive border alongside the edges of image, where features should not be "
			"detected.")
		.def("setMargin", &dv::features::ImageFeatureDetector::setMargin, py::arg("margin"),
			"Set the margin coefficient. Margin coefficient is multiplied by the width and height of the image to "
			"calculate an adaptive border alongside the edges of image, where features should not be detected.")
		.def("isWithinROI", &dv::features::ImageFeatureDetector::isWithinROI, py::arg("point"),
			"Check whether a point belongs to the ROI without the margins.")
		.def("getImageDimensions", &dv::features::ImageFeatureDetector::getImageDimensions,
			"Get configured image dimensions.");

	py::class_<dv::features::ImagePyrFeatureDetector>(m_features, "ImagePyrFeatureDetector",
		"A base class to implement feature detectors on different input types, specifically either images, time "
		"surfaces, or event stores. The implementing class should override the `detect` function and output a vector "
		"of unordered features with a quality score. The API will handle margin calculations and post processing of "
		"the features.")
		.def(py::init([](const cv::Size &resolution) {
			return dv::features::ImagePyrFeatureDetector(resolution, cv::GFTTDetector::create());
		}),
			"resolution"_a,
			"Create a detector that uses good-features-to-track (GFTT) algorithm for feature detection. This "
			"constructor defaults post-processing step to none as GFTT does NMS already and a margin coefficient value "
			"of 0.02.")
		.def("runDetection", &dv::features::ImagePyrFeatureDetector::runDetection, py::arg("input"),
			py::arg("numPoints"), py::arg("mask") = cv::Mat(),
			"Public detection call. Calls the overloaded `detect` function, applies margin and post processing.")
		.def("runRedetection", &dv::features::ImagePyrFeatureDetector::runRedetection, py::arg("prior"),
			py::arg("input"), py::arg("numPoints"), py::arg("mask") = cv::Mat(),
			"Redetect new features and add them to already detected features. This function performs detection within "
			"masked region (if mask is non-empty), runs postprocessing and appends the additional features to the "
			"prior keypoint list.")
		.def(
			"getPostProcessing",
			[](const dv::features::ImagePyrFeatureDetector &self)
				-> dv::features::ImageFeatureDetector::FeaturePostProcessing {
				return static_cast<dv::features::ImageFeatureDetector::FeaturePostProcessing>(self.getPostProcessing());
			},
			"Get the type of post-processing.")
		.def(
			"setPostProcessing",
			[](dv::features::ImagePyrFeatureDetector &self,
				const dv::features::ImageFeatureDetector::FeaturePostProcessing postProcessing) {
				self.setPostProcessing(
					static_cast<dv::features::ImagePyrFeatureDetector::FeaturePostProcessing>(postProcessing));
			},
			py::arg("postProcessing"), "Set the type of post-processing.")
		.def("getMargin", &dv::features::ImagePyrFeatureDetector::getMargin,
			"Get currently applied margin coefficient. Margin coefficient is multiplied by the width and height of the "
			"image to calculate an adaptive border alongside the edges of image, where features should not be "
			"detected.")
		.def("setMargin", &dv::features::ImagePyrFeatureDetector::setMargin, py::arg("margin"),
			"Set the margin coefficient. Margin coefficient is multiplied by the width and height of the image to "
			"calculate an adaptive border alongside the edges of image, where features should not be detected.")
		.def("isWithinROI", &dv::features::ImagePyrFeatureDetector::isWithinROI, py::arg("point"),
			"Check whether a point belongs to the ROI without the margins.")
		.def("getImageDimensions", &dv::features::ImagePyrFeatureDetector::getImageDimensions,
			"Get configured image dimensions.");

	py::class_<dv::features::ImageFeatureLKTracker>(m_features, "ImageFeatureLKTracker",
		"A feature based sparse Lucas-Kanade feature tracker based on image pyramids.")
		.def_static(
			"RegularTracker",
			[](const cv::Size &resolution, const dv::features::LucasKanadeConfig &config) {
				return dv::features::ImageFeatureLKTracker::RegularTracker(resolution, config).release();
			},
			py::arg("resolution"), py::arg("config") = dv::features::LucasKanadeConfig(),
			"Create a tracker instance that performs tracking of features on images.")
		.def_static(
			"MotionAwareTracker",
			[](const dv::camera::CameraGeometry::SharedPtr &camera, const dv::features::LucasKanadeConfig &config) {
				return dv::features::ImageFeatureLKTracker::MotionAwareTracker(camera, config).release();
			},
			py::arg("camera"), py::arg("config") = dv::features::LucasKanadeConfig(),
			"Create a tracker instance that performs tracking of features on images. Features are  Additionally, "
			"camera motion and scene depth are used to predict probable locations of the feature tracks, increasing "
			"the quality of tracking.")
		.def(
			"accept",
			[](dv::features::ImageFeatureLKTracker &self, const dv::Frame &frame) {
				self.accept(frame);
			},
			py::arg("frame"), "Add an input image for the tracker. Image pyramid will be built from the given image.")
		.def(
			"accept",
			[](dv::features::ImageFeatureLKTracker &self, const dv::measurements::Depth &depth) {
				self.accept(depth);
			},
			py::arg("depth"), "Add scene depth, a median depth value of tracked landmarks usually works well enough.")
		.def(
			"accept",
			[](dv::features::ImageFeatureLKTracker &self, const dv::kinematics::Transformationf &transformation) {
				self.accept(transformation);
			},
			py::arg("transformation"),
			"Add camera transformation, usually in the world coordinate frame (`T_WC`). Although the class only "
			"extract the motion difference, so any other reference frame should also work as long as reference frames "
			"are not mixed up.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::ImageFeatureLKTracker &self, const dv::features::NoRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::NoRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::ImageFeatureLKTracker &self, const dv::features::FeatureCountRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::FeatureCountRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::ImageFeatureLKTracker &self, const dv::features::UpdateIntervalRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::UpdateIntervalRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::ImageFeatureLKTracker &self,
				const dv::features::UpdateIntervalOrFeatureCountRedetection &strategy) {
				self.setRedetectionStrategy(
					std::make_unique<dv::features::UpdateIntervalOrFeatureCountRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setDetector",
			[](dv::features::ImageFeatureLKTracker &self, const dv::features::ImagePyrFeatureDetector &detector) {
				self.setDetector(std::make_unique<dv::features::ImagePyrFeatureDetector>(detector));
			},
			py::arg("detector"), "Set a new feature (corner) detector.")
		.def(
			"setMotionPredictor",
			[](dv::features::ImageFeatureLKTracker &self, const dv::kinematics::PixelMotionPredictor &predictor) {
				self.setMotionPredictor(std::make_unique<dv::kinematics::PixelMotionPredictor>(predictor));
			},
			py::arg("predictor"),
			"Set new pixel motion predictor instance. Warning: motion prediction requires camera calibration to "
			"be set, otherwise the function will not instantiate the motion predictor.")
		.def("getMaxTracks", &dv::features::ImageFeatureLKTracker::getMaxTracks, "Get the maximum number of tracks.")
		.def("setMaxTracks", &dv::features::ImageFeatureLKTracker::setMaxTracks, py::arg("maxTracks"),
			"Set the maximum number of tracks.")
		.def("runTracking", &dv::features::ImageFeatureLKTracker::runTracking,
			"Perform the tracking and cache the results.")
		.def("removeTracks", &dv::features::ImageFeatureLKTracker::removeTracks, py::arg("trackIds"),
			"Remove tracks from cached results, so the wouldn't be tracked anymore. TrackIds are the `class_id` value "
			"of the keypoint structure.")
		.def("isLookbackRejectionEnabled", &dv::features::ImageFeatureLKTracker::isLookbackRejectionEnabled,
			"Check whether lookback is enabled.")
		.def("setLookbackRejection", &dv::features::ImageFeatureLKTracker::setLookbackRejection,
			py::arg("lookbackRejection"),
			"Enable or disable lookback rejection based on Forward-Backward error. Lookback rejection applies "
			"Lucas-Kanade tracking backwards after running the usual tracking and rejects any tracks that fails to "
			"successfully track back to same approximate location by measuring Euclidean distance. Euclidean distance "
			"threshold for rejection can be set using `setRejectionDistanceThreshold` method. This is a real-time "
			"implementation of the method proposed by Zdenek et al. 2010, that only performs forward-backward error "
			"measurement within a single pair of latest and previous frame: "
			"http://kahlan.eps.surrey.ac.uk/featurespace/tld/Publications/2010_icpr.pdf")
		.def("getRejectionDistanceThreshold", &dv::features::ImageFeatureLKTracker::getRejectionDistanceThreshold,
			"Get the current rejection distance threshold for the lookback rejection feature.")
		.def("setRejectionDistanceThreshold", &dv::features::ImageFeatureLKTracker::setRejectionDistanceThreshold,
			py::arg("rejectionDistanceThreshold"),
			"Set the threshold for lookback rejection feature. This value is a maximum Euclidean distance value that "
			"is considered successful when performing backwards tracking check after forward tracking. If the backward "
			"tracked feature location is further away from initial position than this given value, the tracker will "
			"reject the track as a failed track. See method `setLookbackRejection` documentation for further "
			"explanation of the approach.")
		.def("getConstantDepth", &dv::features::ImageFeatureLKTracker::getConstantDepth,
			"Get currently assumed constant depth value. It is used if no depth measurements are provided.")
		.def("setConstantDepth", &dv::features::ImageFeatureLKTracker::setConstantDepth, "depth"_a,
			"Set constant depth value that is assumed if no depth measurement is passed using "
			"`accept(dv::measurements::Depth)`. By default the constant depth is assumed to be 3.0 meters, which is "
			"just a reasonable guess.");

	py::class_<dv::features::ArcCornerDetector<>>(m_features, "ArcCornerDetector",
		"This class implement the Arc* corner detector presented in the following paper: "
		"https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/277131/RAL2018-camera-ready.pdf")
		.def(py::init<const cv::Size, const int64_t, const bool>(), py::arg("resolution"), py::arg("range"),
			py::arg("resetTsAtEachIteration"))
		.def("detect", &dv::features::ArcCornerDetector<>::detect, py::arg("events"), py::arg("roi"), py::arg("mask"),
			"Runs the detection algorithm.")
		.def("getTimeSurface", &dv::features::ArcCornerDetector<>::getTimeSurface, py::arg("polarity"),
			"Returns the TimeSurface for a given polarity");

	using ArcEventFeatureDetector = dv::features::FeatureDetector<dv::EventStore, dv::features::ArcCornerDetector<>>;
	py::class_<ArcEventFeatureDetector>(m_features, "ArcEventFeatureDetector")
		.def(py::init([](const cv::Size &resolution, const int64_t cornerRange, const bool resetTS) {
			return ArcEventFeatureDetector(
				resolution, std::make_shared<dv::features::ArcCornerDetector<>>(resolution, cornerRange, resetTS));
		}))
		.def("runDetection", &ArcEventFeatureDetector::runDetection, py::arg("input"), py::arg("numPoints"),
			py::arg("mask") = cv::Mat(),
			"Public detection call. Calls the overloaded `detect` function, applies margin and post processing.")
		.def("runRedetection", &ArcEventFeatureDetector::runRedetection, py::arg("prior"), py::arg("input"),
			py::arg("numPoints"), py::arg("mask") = cv::Mat(),
			"Redetect new features and add them to already detected features. This function performs detection within "
			"masked region (if mask is non-empty), runs postprocessing and appends the additional features to the "
			"prior keypoint list.")
		.def(
			"getPostProcessing",
			[](const ArcEventFeatureDetector &self) -> dv::features::ImageFeatureDetector::FeaturePostProcessing {
				return static_cast<dv::features::ImageFeatureDetector::FeaturePostProcessing>(self.getPostProcessing());
			},
			"Get the type of post-processing.")
		.def(
			"setPostProcessing",
			[](ArcEventFeatureDetector &self,
				const dv::features::ImageFeatureDetector::FeaturePostProcessing postProcessing) {
				self.setPostProcessing(static_cast<ArcEventFeatureDetector::FeaturePostProcessing>(postProcessing));
			},
			"postProcessing"_a, "Set the type of post-processing.")
		.def("getMargin", &ArcEventFeatureDetector::getMargin,
			"Get currently applied margin coefficient. Margin coefficient is multiplied by the width and height of the "
			"image to calculate an adaptive border alongside the edges of image, where features should not be "
			"detected.")
		.def("setMargin", &ArcEventFeatureDetector::setMargin, py::arg("margin"),
			"Set the margin coefficient. Margin coefficient is multiplied by the width and height of the image to "
			"calculate an adaptive border alongside the edges of image, where features should not be detected.")
		.def("isWithinROI", &ArcEventFeatureDetector::isWithinROI, py::arg("point"),
			"Check whether a point belongs to the ROI without the margins.")
		.def("getImageDimensions", &ArcEventFeatureDetector::getImageDimensions, "Get configured image dimensions.");

	py::class_<dv::features::EventBlobDetector>(
		m_features, "EventBlobDetector", "Event-based blob detector performing detection on accumulated event images.")
		.def(py::init(
				 [](const cv::Size &resolution, const int pyramidLevel, std::function<cv::Mat(cv::Mat &)> preprocess) {
					 return dv::features::EventBlobDetector(resolution, pyramidLevel, [preprocess](cv::Mat &image) {
						 image = preprocess(image);
					 });
				 }),
			"resolution"_a, "pyramidLevel"_a = 0,
			"preprocess"_a = std::function<cv::Mat(cv::Mat &)>([](cv::Mat &image) {
				return image;
			}),
			"Please note that function signature for preprocessing (in python) expect a matrix to be returned.")
		.def("detect", &dv::features::EventBlobDetector::detect, py::arg("events"), py::arg("roi") = cv::Rect(),
			py::arg("mask") = cv::Mat(), "Detection step.");

	using EventFeatureBlobDetector = dv::features::FeatureDetector<dv::EventStore, dv::features::EventBlobDetector>;
	py::class_<EventFeatureBlobDetector>(m_features, "EventFeatureBlobDetector")
		.def(py::init([](const cv::Size &resolution, const int pyramidLevel,
						  std::function<cv::Mat(cv::Mat &)> preprocess) {
			return EventFeatureBlobDetector(resolution, std::make_shared<dv::features::EventBlobDetector>(
															resolution, pyramidLevel, [preprocess](cv::Mat &image) {
																image = preprocess(image);
															}));
		}),
			"resolution"_a, "pyramidLevel"_a = 0,
			"preprocess"_a = std::function<cv::Mat(cv::Mat &)>([](cv::Mat &image) {
				return image;
			}))

		.def("runDetection", &EventFeatureBlobDetector::runDetection, py::arg("input"), py::arg("numPoints"),
			py::arg("mask") = cv::Mat(),
			"Public detection call. Calls the overloaded `detect` function, applies margin and post processing.")
		.def("runRedetection", &EventFeatureBlobDetector::runRedetection, py::arg("prior"), py::arg("input"),
			py::arg("numPoints"), py::arg("mask") = cv::Mat(),
			"Redetect new features and add them to already detected features. This function performs detection within "
			"masked region (if mask is non-empty), runs postprocessing and appends the additional features to the "
			"prior keypoint list.")
		.def("getPostProcessing", &EventFeatureBlobDetector::getPostProcessing, "Get the type of post-processing.")
		.def("setPostProcessing", &EventFeatureBlobDetector::setPostProcessing, py::arg("postProcessing"),
			"Set the type of post-processing.")
		.def("getMargin", &EventFeatureBlobDetector::getMargin,
			"Get currently applied margin coefficient. Margin coefficient is multiplied by the width and height of the "
			"image to calculate an adaptive border alongside the edges of image, where features should not be "
			"detected.")
		.def("setMargin", &EventFeatureBlobDetector::setMargin, py::arg("margin"),
			"Set the margin coefficient. Margin coefficient is multiplied by the width and height of the image to "
			"calculate an adaptive border alongside the edges of image, where features should not be detected.")
		.def("isWithinROI", &EventFeatureBlobDetector::isWithinROI, py::arg("point"),
			"Check whether a point belongs to the ROI without the margins.")
		.def("getImageDimensions", &EventFeatureBlobDetector::getImageDimensions, "Get configured image dimensions.");

	py::class_<dv::features::EventFeatureLKTracker<>>(m_features, "EventFeatureLKTracker",
		"Event-based Lucas-Kanade tracker, the tracking is achieved by accumulating frames and running the "
		"classic LK "
		"frame based tracker on them.")
		.def_static(
			"RegularTracker",
			[](const cv::Size &resolution, const dv::features::LucasKanadeConfig &config) {
				return dv::features::EventFeatureLKTracker<>::RegularTracker(resolution, config).release();
			},
			py::arg("resolution"), py::arg("config") = dv::features::LucasKanadeConfig(),
			"Create a tracker instance that performs tracking of features on event accumulated frames. Features are "
			"detected and tracked on event accumulated frames.")
		.def_static(
			"MotionAwareTracker",
			[](const dv::camera::CameraGeometry::SharedPtr &camera, const dv::features::LucasKanadeConfig &config) {
				return dv::features::EventFeatureLKTracker<>::MotionAwareTracker(camera, config).release();
			},
			py::arg("camera"), py::arg("config") = dv::features::LucasKanadeConfig(),
			"Create a tracker instance that performs tracking of features on event accumulated frames. Features are "
			"detected and tracked on event accumulated frames. Additionally, camera motion and scene depth are used to "
			"generate motion compensated frames, which are way sharper than usual accumulated frames. This requires "
			"camera sensor to be calibrated.")
		.def(
			"accept",
			[](dv::features::EventFeatureLKTracker<> &self, const dv::EventStore &events) {
				self.accept(events);
			},
			py::arg("events"),
			"Add the input events. Since the batch of events might contain information for more than a single tracking "
			"iteration configurable by the framerate parameter, the tracking function should be executed on a loop "
			"until it returns a null-pointer, signifying end of available data processing")
		.def(
			"accept",
			[](dv::features::EventFeatureLKTracker<> &self, const dv::measurements::Depth &depth) {
				self.accept(depth);
			},
			py::arg("depth"), "Add scene depth, a median depth value of tracked landmarks usually works well enough.")
		.def(
			"accept",
			[](dv::features::EventFeatureLKTracker<> &self, const dv::kinematics::Transformationf &transformation) {
				self.accept(transformation);
			},
			py::arg("transformation"),
			"Add camera transformation, usually in the world coordinate frame (`T_WC`). Although the class only "
			"extract the motion difference, so any other reference frame should also work as long as reference frames "
			"are not mixed up.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::EventFeatureLKTracker<> &self, const dv::features::NoRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::NoRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::EventFeatureLKTracker<> &self, const dv::features::FeatureCountRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::FeatureCountRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::EventFeatureLKTracker<> &self, const dv::features::UpdateIntervalRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::UpdateIntervalRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::EventFeatureLKTracker<> &self,
				const dv::features::UpdateIntervalOrFeatureCountRedetection &strategy) {
				self.setRedetectionStrategy(
					std::make_unique<dv::features::UpdateIntervalOrFeatureCountRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setDetector",
			[](dv::features::EventFeatureLKTracker<> &self, const dv::features::ImagePyrFeatureDetector &detector) {
				self.setDetector(std::make_unique<dv::features::ImagePyrFeatureDetector>(detector));
			},
			py::arg("detector"), "Set a new feature (corner) detector.")
		.def(
			"setMotionPredictor",
			[](dv::features::EventFeatureLKTracker<> &self, const dv::kinematics::PixelMotionPredictor &predictor) {
				self.setMotionPredictor(std::make_unique<dv::kinematics::PixelMotionPredictor>(predictor));
			},
			py::arg("predictor"),
			"Set new pixel motion predictor instance. Warning: motion prediction requires camera calibration to "
			"be set, otherwise the function will not instantiate the motion predictor.")
		.def(
			"setAccumulator",
			[](dv::features::EventFeatureLKTracker<> &self, const dv::EdgeMapAccumulator &accumulator) {
				self.setAccumulator(std::make_unique<dv::EdgeMapAccumulator>(accumulator));
			},
			py::arg("accumulator"), "Set an accumulator instance to be used for frame generation.")
		.def("getAccumulatedFrame", &dv::features::EventFeatureLKTracker<>::getAccumulatedFrame,
			"Get the latest accumulated frame.")
		.def("getMaxTracks", &dv::features::EventFeatureLKTracker<>::getMaxTracks, "Get the maximum number of tracks.")
		.def("setMaxTracks", &dv::features::EventFeatureLKTracker<>::setMaxTracks, py::arg("maxTracks"))
		.def("getFramerate", &dv::features::EventFeatureLKTracker<>::getFramerate, "Get configured framerate.")
		.def("setFramerate", &dv::features::EventFeatureLKTracker<>::setFramerate, py::arg("framerate"),
			"Set the accumulation and tracking framerate.")
		.def("getStoreTimeLimit", &dv::features::EventFeatureLKTracker<>::getStoreTimeLimit,
			"Get the event storage time limit.")
		.def("setStoreTimeLimit", &dv::features::EventFeatureLKTracker<>::setStoreTimeLimit, py::arg("storeTimeLimit"),
			"Set the event buffer storage duration limit.")
		.def("getNumberOfEvents", &dv::features::EventFeatureLKTracker<>::getNumberOfEvents,
			"Get the number of latest events that are going to be accumulated for each frame. The default number of "
			"event is a third of of total pixels in the sensor.")
		.def("setNumberOfEvents", &dv::features::EventFeatureLKTracker<>::setNumberOfEvents, py::arg("numberOfEvents"),
			"Set the number of latest events that are going to be accumulated for each frame. The default number of "
			"event is a third of of total pixels in the sensor.")
		.def("runTracking", &dv::features::EventFeatureLKTracker<>::runTracking,
			"Perform the tracking and cache the results.")
		.def("removeTracks", &dv::features::EventFeatureLKTracker<>::removeTracks, py::arg("trackIds"),
			"Remove tracks from cached results, so the wouldn't be tracked anymore. TrackIds are the `class_id` value "
			"of the keypoint structure.")
		.def("isLookbackRejectionEnabled", &dv::features::EventFeatureLKTracker<>::isLookbackRejectionEnabled,
			"Check whether lookback is enabled.")
		.def("setLookbackRejection", &dv::features::EventFeatureLKTracker<>::setLookbackRejection,
			py::arg("lookbackRejection"),
			"Enable or disable lookback rejection based on Forward-Backward error. Lookback rejection applies "
			"Lucas-Kanade tracking backwards after running the usual tracking and rejects any tracks that fails to "
			"successfully track back to same approximate location by measuring Euclidean distance. Euclidean distance "
			"threshold for rejection can be set using `setRejectionDistanceThreshold` method. This is a real-time "
			"implementation of the method proposed by Zdenek et al. 2010, that only performs forward-backward error "
			"measurement within a single pair of latest and previous frame: "
			"http://kahlan.eps.surrey.ac.uk/featurespace/tld/Publications/2010_icpr.pdf")
		.def("getRejectionDistanceThreshold", &dv::features::EventFeatureLKTracker<>::getRejectionDistanceThreshold,
			"Get the current rejection distance threshold for the lookback rejection feature.")
		.def("setRejectionDistanceThreshold", &dv::features::EventFeatureLKTracker<>::setRejectionDistanceThreshold,
			py::arg("rejectionDistanceThreshold"),
			"Set the threshold for lookback rejection feature. This value is a maximum Euclidean distance value that "
			"is considered successful when performing backwards tracking check after forward tracking. If the backward "
			"tracked feature location is further away from initial position than this given value, the tracker will "
			"reject the track as a failed track. See method `setLookbackRejection` documentation for further "
			"explanation of the approach.")
		.def("getConstantDepth", &dv::features::EventFeatureLKTracker<>::getConstantDepth,
			"Get currently assumed constant depth value. It is used if no depth measurements are provided.")
		.def("setConstantDepth", &dv::features::EventFeatureLKTracker<>::setConstantDepth, "depth"_a,
			"Set constant depth value that is assumed if no depth measurement is passed using "
			"`accept(dv::measurements::Depth)`. By default the constant depth is assumed to be 3.0 meters, which is "
			"just a reasonable guess.");

	py::class_<dv::features::EventCombinedLKTracker<>>(m_features, "EventCombinedLKTracker",
		"Implements an event combined Lucas-Kanade tracker. The algorithms detects and tracks features on a regular "
		"frame image, but to improve tracking quality, it accumulates intermediate frames from events, performs "
		"tracking on those frames and uses the output to predict the track locations on the regular frame.")
		.def_static(
			"RegularTracker",
			[](const cv::Size &resolution, const dv::features::LucasKanadeConfig &config) {
				return dv::features::EventCombinedLKTracker<>::RegularTracker(resolution, config).release();
			},
			py::arg("resolution"), py::arg("config") = dv::features::LucasKanadeConfig(),
			"Create a tracker instance that performs tracking of features on both - event accumulated and regular "
			"images. Tracking is performed by detecting and tracking features on a regular image. It also uses events "
			"to generate intermediate accumulated frames between the regular frames, track the features on them and "
			"use the intermediate tracking results as feature position priors for the image frame.")
		.def_static(
			"MotionAwareTracker",
			[](const dv::camera::CameraGeometry::SharedPtr &camera, const dv::features::LucasKanadeConfig &config) {
				return dv::features::EventCombinedLKTracker<>::MotionAwareTracker(camera, config).release();
			},
			py::arg("camera"), py::arg("config") = dv::features::LucasKanadeConfig(),
			"Create a tracker instance that performs tracking of features on both - event accumulated and regular "
			"images. Tracking is performed by detecting and tracking features on a regular image. It also uses events "
			"to generate intermediate accumulated frames between the regular frames, track the features on them and "
			"use the intermediate tracking results as feature position priors for the image frame. The implementation "
			"also uses camera motion and scene depth to motion compensate events, so the intermediate accumulated "
			"frames are sharp and the Lucas-Kanade tracker works more accurately. This requires camera sensor to be "
			"calibrated.")
		.def(
			"accept",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::EventStore &events) {
				self.accept(events);
			},
			py::arg("events"),
			"Add an event batch. Added events should contain at least some events that were registered further in the "
			"future of the next image.")
		.def(
			"accept",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::measurements::Depth &depth) {
				self.accept(depth);
			},
			py::arg("depth"), "Add scene depth, a median depth value of tracked landmarks usually works well enough.")
		.def(
			"accept",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::kinematics::Transformationf &transformation) {
				self.accept(transformation);
			},
			py::arg("transformation"),
			"Add camera transformation, usually in the world coordinate frame (`T_WC`). Although the class only "
			"extract the motion difference, so any other reference frame should also work as long as reference frames "
			"are not mixed up.")
		.def(
			"accept",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::Frame &frame) {
				self.accept(frame);
			},
			py::arg("frame"), "Add an input image for the tracker. Image pyramid will be built from the given image.")

		.def(
			"setRedetectionStrategy",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::features::NoRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::NoRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::features::FeatureCountRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::FeatureCountRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::features::UpdateIntervalRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::UpdateIntervalRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::EventCombinedLKTracker<> &self,
				const dv::features::UpdateIntervalOrFeatureCountRedetection &strategy) {
				self.setRedetectionStrategy(
					std::make_unique<dv::features::UpdateIntervalOrFeatureCountRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setDetector",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::features::ImagePyrFeatureDetector &detector) {
				self.setDetector(std::make_unique<dv::features::ImagePyrFeatureDetector>(detector));
			},
			py::arg("detector"), "Set a new feature (corner) detector.")
		.def(
			"setMotionPredictor",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::kinematics::PixelMotionPredictor &predictor) {
				self.setMotionPredictor(std::make_unique<dv::kinematics::PixelMotionPredictor>(predictor));
			},
			py::arg("predictor"),
			"Set new pixel motion predictor instance. Warning: motion prediction requires camera calibration to "
			"be set, otherwise the function will not instantiate the motion predictor.")
		.def(
			"setAccumulator",
			[](dv::features::EventCombinedLKTracker<> &self, const dv::EdgeMapAccumulator &accumulator) {
				self.setAccumulator(std::make_unique<dv::EdgeMapAccumulator>(accumulator));
			},
			py::arg("accumulator"), "Set an accumulator instance to be used for frame generation.")
		.def("getAccumulatedFrames", &dv::features::EventCombinedLKTracker<>::getAccumulatedFrames,
			"Retrieve an array of accumulated frames that were used to predict track positions for the last processed "
			"frame.")
		.def("getMaxTracks", &dv::features::EventCombinedLKTracker<>::getMaxTracks, "Get the maximum number of tracks.")
		.def("setMaxTracks", &dv::features::EventCombinedLKTracker<>::setMaxTracks, py::arg("maxTracks"),
			"Set the accumulation and tracking framerate.")
		.def("getStoreTimeLimit", &dv::features::EventCombinedLKTracker<>::getStoreTimeLimit,
			"Get the event storage time limit.")
		.def("setStoreTimeLimit", &dv::features::EventCombinedLKTracker<>::setStoreTimeLimit, py::arg("storeTimeLimit"),
			"Set the event buffer storage duration limit.")
		.def("getNumberOfEvents", &dv::features::EventCombinedLKTracker<>::getNumberOfEvents,
			"Get the number of latest events that are going to be accumulated for each frame. The default number of "
			"event is a third of of total pixels in the sensor.")
		.def("setNumberOfEvents", &dv::features::EventCombinedLKTracker<>::setNumberOfEvents, py::arg("numberOfEvents"),
			"Set the number of latest events that are going to be accumulated for each frame. The default number of "
			"event is a third of of total pixels in the sensor.")
		.def("getNumIntermediateFrames", &dv::features::EventCombinedLKTracker<>::getNumIntermediateFrames,
			"Get the number of intermediate frames that are going to be generated.")
		.def("setNumIntermediateFrames", &dv::features::EventCombinedLKTracker<>::setNumIntermediateFrames,
			py::arg("numIntermediateFrames"), "Set the number of intermediate frames that are going to be generated.")
		.def("runTracking", &dv::features::EventCombinedLKTracker<>::runTracking,
			"Perform the tracking and cache the results.")
		.def("removeTracks", &dv::features::EventCombinedLKTracker<>::removeTracks, py::arg("trackIds"),
			"Remove tracks from cached results, so the wouldn't be tracked anymore. TrackIds are the `class_id` value "
			"of the keypoint structure.")
		.def("getMinRateForIntermediateTracking",
			&dv::features::EventCombinedLKTracker<>::getMinRateForIntermediateTracking,
			"Get the minimum event rate that is required to perform intermediate tracking.")
		.def("setMinRateForIntermediateTracking",
			&dv::features::EventCombinedLKTracker<>::setMinRateForIntermediateTracking,
			"minRateForIntermediateTracking"_a,
			"Set a minimum event rate per second value that is used to perform intermediate. If the event rate between "
			"last and current frame is lower than this, tracker assumes very little motion and does not perform "
			"intermediate tracking.")
		.def("isLookbackRejectionEnabled", &dv::features::EventCombinedLKTracker<>::isLookbackRejectionEnabled,
			"Check whether lookback is enabled.")
		.def("setLookbackRejection", &dv::features::EventCombinedLKTracker<>::setLookbackRejection,
			py::arg("lookbackRejection"),
			"Enable or disable lookback rejection based on Forward-Backward error. Lookback rejection applies "
			"Lucas-Kanade tracking backwards after running the usual tracking and rejects any tracks that fails to "
			"successfully track back to same approximate location by measuring Euclidean distance. Euclidean distance "
			"threshold for rejection can be set using `setRejectionDistanceThreshold` method. This is a real-time "
			"implementation of the method proposed by Zdenek et al. 2010, that only performs forward-backward error "
			"measurement within a single pair of latest and previous frame: "
			"http://kahlan.eps.surrey.ac.uk/featurespace/tld/Publications/2010_icpr.pdf")
		.def("getRejectionDistanceThreshold", &dv::features::EventCombinedLKTracker<>::getRejectionDistanceThreshold,
			"Get the current rejection distance threshold for the lookback rejection feature.")
		.def("setRejectionDistanceThreshold", &dv::features::EventCombinedLKTracker<>::setRejectionDistanceThreshold,
			py::arg("rejectionDistanceThreshold"),
			"Set the threshold for lookback rejection feature. This value is a maximum Euclidean distance value that "
			"is considered successful when performing backwards tracking check after forward tracking. If the backward "
			"tracked feature location is further away from initial position than this given value, the tracker will "
			"reject the track as a failed track. See method `setLookbackRejection` documentation for further "
			"explanation of the approach.")
		.def("getConstantDepth", &dv::features::EventCombinedLKTracker<>::getConstantDepth,
			"Get currently assumed constant depth value. It is used if no depth measurements are provided.")
		.def("setConstantDepth", &dv::features::EventCombinedLKTracker<>::setConstantDepth, "depth"_a,
			"Set constant depth value that is assumed if no depth measurement is passed using "
			"`accept(dv::measurements::Depth)`. By default the constant depth is assumed to be 3.0 meters, which is "
			"just a reasonable guess.");

	py::class_<dv::features::MeanShiftTracker>(
		m_features, "MeanShiftTracker", "Track event blobs using mean shift algorithm on time surface event data.")
		.def(py::init([](const cv::Size &resolution, const int bandwidth, const dv::Duration timeWindow,
						  const float stepSize, const float weightMultiplier, float convergenceNorm, int maxIters) {
			return dv::features::MeanShiftTracker(resolution, bandwidth, timeWindow, nullptr, nullptr, stepSize,
				weightMultiplier, convergenceNorm, maxIters);
		}),
			"resolution"_a, "bandwidth"_a, "timeWindow"_a, "stepSize"_a = 0.5, "weightMultiplier"_a = 1,
			"convergenceNorm"_a = 0.01, "maxIters"_a = 2000,
			"Constructor for mean shift tracker using Epanechnikov kernel as weights for the time surface of events "
			"used to update track location. The kernel weights have highest value on the previous track location. This "
			"assumption is based on the idea that the new track location is \"close\" to last track location. The "
			"consecutive track updates are performed until the maximum number of iteration is reached or the shift "
			"between consecutive updates is below a threshold.")
		.def("accept", &dv::features::MeanShiftTracker::accept, py::arg("store"),
			"Add events to time surface and update latest timestamp.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::MeanShiftTracker &self, const dv::features::NoRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::NoRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::MeanShiftTracker &self, const dv::features::FeatureCountRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::FeatureCountRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::MeanShiftTracker &self, const dv::features::UpdateIntervalRedetection &strategy) {
				self.setRedetectionStrategy(std::make_unique<dv::features::UpdateIntervalRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setRedetectionStrategy",
			[](dv::features::MeanShiftTracker &self,
				const dv::features::UpdateIntervalOrFeatureCountRedetection &strategy) {
				self.setRedetectionStrategy(
					std::make_unique<dv::features::UpdateIntervalOrFeatureCountRedetection>(strategy));
			},
			py::arg("strategy"), "Set a new redetection strategy.")
		.def(
			"setDetector",
			[](dv::features::MeanShiftTracker &self, const dv::features::EventFeatureBlobDetector &detector) {
				self.setDetector(std::make_unique<dv::features::EventFeatureBlobDetector>(detector));
			},
			py::arg("detector"), "Set a new feature (blob) detector from accumulated images.")
		.def("getMaxTracks", &dv::features::MeanShiftTracker::getMaxTracks, "Get the maximum number of tracks.")
		.def("setMaxTracks", &dv::features::MeanShiftTracker::setMaxTracks, py::arg("maxTracks"),
			"Set the maximum number of tracks.")
		.def("runTracking", &dv::features::MeanShiftTracker::runTracking, "Perform the tracking and cache the results.")
		.def("removeTracks", &dv::features::MeanShiftTracker::removeTracks, py::arg("trackIds"),
			"Remove tracks from cached results, so the wouldn't be tracked anymore. TrackIds are the `class_id` value "
			"of the keypoint structure.")
		.def("getBandwidth", &dv::features::MeanShiftTracker::getBandwidth,
			"Getter for bandwidth value that defines the search area for a new track.")
		.def("setBandwidth", &dv::features::MeanShiftTracker::setBandwidth, py::arg("bandwidth"),
			"Setter for bandwidth value.")
		.def("getTimeWindow", &dv::features::MeanShiftTracker::getTimeWindow,
			"Get time duration used to compute time surface.")
		.def("setTimeWindow", &dv::features::MeanShiftTracker::setTimeWindow, py::arg("timeWindow"),
			"Setter for window size for normalized time surface.")
		.def("getStepSize", &dv::features::MeanShiftTracker::getStepSize,
			"Get learning rate value used for track location update.")
		.def("setStepSize", &dv::features::MeanShiftTracker::setStepSize, py::arg("stepSize"),
			"Setter for learning rate for motion towards new center during one mean shift iteration.")
		.def("getWeightMultiplier", &dv::features::MeanShiftTracker::getWeightMultiplier,
			"Get scaling factor for Epanechnikov weights used in the computation of the mean shift cost update.")
		.def("setWeightMultiplier", &dv::features::MeanShiftTracker::setWeightMultiplier, py::arg("multiplier"),
			"Setter for scaling factor used in the computation of the mean shift cost update.");

	auto m_cluster    = m.def_submodule("cluster");
	auto m_mean_shift = m_cluster.def_submodule("mean_shift");

	py::class_<dv::cluster::mean_shift::MeanShiftEventStoreAdaptor>(m_mean_shift, "MeanShiftEventStoreAdaptor",
		"This class implements the Mean Shift clustering algorithm with an Epanechnikov Kernel for event store data.")
		.def(py::init<const dv::EventStore &, const int16_t, float, const uint32_t,
				 const std::vector<dv::TimedKeyPoint, Eigen::aligned_allocator<dv::TimedKeyPoint>> &, const uint32_t>(),
			"events"_a, "bw"_a, "conv"_a, "maxIter"_a, "startingPoints"_a, "numLeaves"_a = 32768)
		.def(py::init<const dv::EventStore &, const int16_t, float, const uint32_t, const uint32_t, const uint32_t>(),
			"events"_a, "bw"_a, "conv"_a, "maxIter"_a, "numStartingPoints"_a, "numLeaves"_a = 32768)
		.def_static("generateStartingPointsFromData",
			&dv::cluster::mean_shift::MeanShiftEventStoreAdaptor::generateStartingPointsFromData,
			py::arg("numStartingPoints"), py::arg("events"),
			"Generates a vector of vectors containing the starting points by randomly selecting from provided data")
		.def_static("generateStartingPointsFromRange",
			&dv::cluster::mean_shift::MeanShiftEventStoreAdaptor::generateStartingPointsFromRange,
			py::arg("numStartingPoints"), py::arg("ranges"),
			"Generates a vector of vectors containing the starting points by generating random points within a given "
			"range for each dimension")
		.def("fit",
			&dv::cluster::mean_shift::MeanShiftEventStoreAdaptor::fit<dv::cluster::mean_shift::kernel::Epanechnikov>,
			"Executes the algorithm with an Epanechnikov algorithm.")
		.def("findClusterCentresEpanechnikov",
			&dv::cluster::mean_shift::MeanShiftEventStoreAdaptor::findClusterCentres<
				dv::cluster::mean_shift::kernel::Epanechnikov>,
			"Performs the search using the Epanechnikov kernel for the cluster centres for each given starting point. "
			"A detected centre is added to the set of centres if it isn't closer than the bandwidth to any previously "
			"detected centre..")
		.def("findClusterCentresGaussian",
			&dv::cluster::mean_shift::MeanShiftEventStoreAdaptor::findClusterCentres<
				dv::cluster::mean_shift::kernel::Gaussian>,
			"Performs the search using the Gaussian kernel for the cluster centres for each given starting point. A "
			"detected centre is added to the set of centres if it isn't closer than the bandwidth to any previously "
			"detected centre..")
		.def("fitGaussian",
			&dv::cluster::mean_shift::MeanShiftEventStoreAdaptor::fit<dv::cluster::mean_shift::kernel::Gaussian>,
			"Executes the algorithm with a Gaussian kernel.");

	auto m_containers = m.def_submodule("containers");
	auto m_kd_tree    = m.def_submodule("kd_tree");

	py::class_<dv::containers::kd_tree::KDTreeEventStoreAdaptor>(m_kd_tree, "KDTreeEventStoreAdaptor",
		"Wrapper class around nanoflann::KDTree for dv::EventStore data, which provides efficient approximate nearest "
		"neighbour search as well as radius search.")
		.def(py::init<const dv::EventStore &, const uint32_t>(), py::arg("events"), py::arg("numLeaves") = 32768)
		.def(
			"knnSearch",
			[](const dv::containers::kd_tree::KDTreeEventStoreAdaptor &self, const cv::Point2f &centrePoint,
				const size_t numClosest) {
				return self.knnSearch(centrePoint, numClosest);
			},
			py::arg("centrePoint"), py::arg("numClosest"),
			"Searches for the k nearest neighbours surrounding centrePoint.")
		.def(
			"radiusSearch",
			[](const dv::containers::kd_tree::KDTreeEventStoreAdaptor &self, const cv::Point2i &centrePoint,
				const int16_t &radius, float eps, bool sorted) {
				return self.radiusSearch(centrePoint.x, centrePoint.y, radius, eps, sorted);
			},
			py::arg("centrePoint"), py::arg("radius"), py::arg("eps") = 0.0f, py::arg("sorted") = false,
			"Searches for all neighbours surrounding centrePoint that are within a certain radius.");

	py::class_<dv::EventFilterBase<>, std::shared_ptr<dv::EventFilterBase<>>> m_eventFilterBase(m, "EventFilterBase");

	py::class_<dv::EventRegionFilter<>, std::shared_ptr<dv::EventRegionFilter<>>>(
		m, "EventRegionFilter", "Event filter that filters events based on a given ROI.")
		.def(py::init<const cv::Rect &>(), py::arg("roi"))
		.def("accept", &dv::EventRegionFilter<>::accept, py::arg("events"), "Accepts incoming events.")
		.def("generateEvents", &dv::EventRegionFilter<>::generateEvents,
			"Apply the filter algorithm and return only the filtered events from the ones that were accepted as input.")
		.def("retain", &dv::EventRegionFilter<>::retain, "Test whether event belongs to an ROI.")
		.def("getNumIncomingEvents", &dv::EventRegionFilter<>::getNumIncomingEvents,
			"Get number of total events that were accepted by the noise filter.")
		.def("getNumOutgoingEvents", &dv::EventRegionFilter<>::getNumOutgoingEvents,
			"Total number of outgoing events from this filter instance.")
		.def("getReductionFactor", &dv::EventRegionFilter<>::getReductionFactor,
			"Get the reduction factor of this filter. It's a fraction representation of events that were discard by "
			"this filter compared to the amount of incoming events.");

	py::class_<dv::RefractoryPeriodFilter<>, std::shared_ptr<dv::RefractoryPeriodFilter<>>>(m, "RefractoryPeriodFilter",
		"Refractory period filter discards any events that are registered at a pixel location that already had an "
		"event within the refractory period. Refractory period should be relatively small value (in the range of one "
		"or a few hundred microseconds).")
		.def(py::init<const cv::Size &, const dv::Duration>(), py::arg("resolution"),
			py::arg("refractoryPeriod") = dv::Duration(250))
		.def("accept", &dv::RefractoryPeriodFilter<>::accept, py::arg("events"), "Accepts incoming events.")
		.def("generateEvents", &dv::RefractoryPeriodFilter<>::generateEvents,
			"Apply the filter algorithm and return only the filtered events from the ones that were accepted as input.")
		.def("retain", &dv::RefractoryPeriodFilter<>::retain, "Test whether event belongs to an ROI.")
		.def("getNumIncomingEvents", &dv::RefractoryPeriodFilter<>::getNumIncomingEvents,
			"Get number of total events that were accepted by the noise filter.")
		.def("getNumOutgoingEvents", &dv::RefractoryPeriodFilter<>::getNumOutgoingEvents,
			"Total number of outgoing events from this filter instance.")
		.def("getReductionFactor", &dv::RefractoryPeriodFilter<>::getReductionFactor,
			"Get the reduction factor of this filter. It's a fraction representation of events that were discard by "
			"this filter compared to the amount of incoming events.")
		.def("getRefractoryPeriod", &dv::RefractoryPeriodFilter<>::getRefractoryPeriod, "Get the refractory period.")
		.def("setRefractoryPeriod", &dv::RefractoryPeriodFilter<>::setRefractoryPeriod, py::arg("refractoryPeriod"),
			"Set a new refractory period value.");

	py::class_<dv::EventPolarityFilter<>, std::shared_ptr<dv::EventPolarityFilter<>>>(
		m, "EventPolarityFilter", "Event filter based on polarity.")
		.def(py::init<bool>(), py::arg("polarity"))
		.def("accept", &dv::EventPolarityFilter<>::accept, py::arg("events"), "Accepts incoming events.")
		.def("generateEvents", &dv::EventPolarityFilter<>::generateEvents,
			"Apply the filter algorithm and return only the filtered events from the ones that were accepted as input.")
		.def("retain", &dv::EventPolarityFilter<>::retain, "Test whether event is of configured polarity.")
		.def("getNumIncomingEvents", &dv::EventPolarityFilter<>::getNumIncomingEvents,
			"Get number of total events that were accepted by the noise filter.")
		.def("getNumOutgoingEvents", &dv::EventPolarityFilter<>::getNumOutgoingEvents,
			"Total number of outgoing events from this filter instance.")
		.def("getReductionFactor", &dv::EventPolarityFilter<>::getReductionFactor,
			"Get the reduction factor of this filter. It's a fraction representation of events that were discard by "
			"this filter compared to the amount of incoming events.");

	py::class_<dv::EventMaskFilter<>, std::shared_ptr<dv::EventMaskFilter<>>>(
		m, "EventMaskFilter", "Event filter that applies a mask.")
		.def(py::init<cv::Mat>(), py::arg("mask"),
			"Create an event masking filter. Discards any events that happen on coordinates where mask has a zero "
			"value and retains all events with coordinates where mask has a non-zero value. The mask to requires to be "
			"unsigned 8bit integer single channel array.")
		.def("accept", &dv::EventMaskFilter<>::accept, py::arg("events"), "Accepts incoming events.")
		.def("generateEvents", &dv::EventMaskFilter<>::generateEvents,
			"Apply the filter algorithm and return only the filtered events from the ones that were accepted as input.")
		.def("retain", &dv::EventMaskFilter<>::retain, "Test whether event is of configured polarity.")
		.def("getNumIncomingEvents", &dv::EventMaskFilter<>::getNumIncomingEvents,
			"Get number of total events that were accepted by the noise filter.")
		.def("getNumOutgoingEvents", &dv::EventMaskFilter<>::getNumOutgoingEvents,
			"Total number of outgoing events from this filter instance.")
		.def("getReductionFactor", &dv::EventMaskFilter<>::getReductionFactor,
			"Get the reduction factor of this filter. It's a fraction representation of events that were discard by "
			"this filter compared to the amount of incoming events.")
		.def("getMask", &dv::EventMaskFilter<>::getMask, "Get the mask that is currently applied.")
		.def("setMask", &dv::EventMaskFilter<>::setMask, py::arg("mask"), "Set a new mask to this filter.");

	auto m_noise = m.def_submodule("noise");
	py::class_<dv::noise::BackgroundActivityNoiseFilter<>, std::shared_ptr<dv::noise::BackgroundActivityNoiseFilter<>>>(
		m_noise, "BackgroundActivityNoiseFilter",
		"A background activity noise filter, which test the neighbourhoods of incoming events for other supporting "
		"events that happened within the background activity period.")
		.def(py::init<const cv::Size &, const dv::Duration>(), py::arg("resolution"),
			py::arg("backgroundActivityDuration") = dv::Duration(2000))
		.def("accept", &dv::noise::BackgroundActivityNoiseFilter<>::accept, py::arg("events"),
			"Accepts incoming events.")
		.def("generateEvents", &dv::noise::BackgroundActivityNoiseFilter<>::generateEvents,
			"Apply the filter algorithm and return only the filtered events from the ones that were accepted as input.")
		.def("retain", &dv::noise::BackgroundActivityNoiseFilter<>::retain,
			"Test whether event is of configured polarity.")
		.def("getNumIncomingEvents", &dv::noise::BackgroundActivityNoiseFilter<>::getNumIncomingEvents,
			"Get number of total events that were accepted by the noise filter.")
		.def("getNumOutgoingEvents", &dv::noise::BackgroundActivityNoiseFilter<>::getNumOutgoingEvents,
			"Total number of outgoing events from this filter instance.")
		.def("getReductionFactor", &dv::noise::BackgroundActivityNoiseFilter<>::getReductionFactor,
			"Get the reduction factor of this filter. It's a fraction representation of events that were discard by "
			"this filter compared to the amount of incoming events.")
		.def("getBackgroundActivityDuration",
			&dv::noise::BackgroundActivityNoiseFilter<>::getBackgroundActivityDuration,
			"Get currently configured background activity duration value.")
		.def("setBackgroundActivityDuration",
			&dv::noise::BackgroundActivityNoiseFilter<>::setBackgroundActivityDuration,
			py::arg("backgroundActivityDuration"), "Set new background activity duration value.");

	py::class_<dv::noise::FastDecayNoiseFilter<>, std::shared_ptr<dv::noise::FastDecayNoiseFilter<>>>(m_noise,
		"FastDecayNoiseFilter",
		"Fast decay noise filter uses a concept that performs a fast decay on a low resolution representation of the "
		"image and checks whether corresponding neighbourhood of the event has recent activity.")
		.def(py::init<const cv::Size &, const dv::Duration, const int, const float>(), py::arg("resolution"),
			py::arg("halfLife") = dv::Duration(10'000), py::arg("subdivisionFactor") = 4, py::arg("noiseThreshold") = 6)
		.def("accept", &dv::noise::FastDecayNoiseFilter<>::accept, py::arg("events"), "Accepts incoming events.")
		.def("generateEvents", &dv::noise::FastDecayNoiseFilter<>::generateEvents,
			"Apply the filter algorithm and return only the filtered events from the ones that were accepted as input.")
		.def("retain", &dv::noise::FastDecayNoiseFilter<>::retain, "Test whether event is of configured polarity.")
		.def("getNumIncomingEvents", &dv::noise::FastDecayNoiseFilter<>::getNumIncomingEvents,
			"Get number of total events that were accepted by the noise filter.")
		.def("getNumOutgoingEvents", &dv::noise::FastDecayNoiseFilter<>::getNumOutgoingEvents,
			"Total number of outgoing events from this filter instance.")
		.def("getReductionFactor", &dv::noise::FastDecayNoiseFilter<>::getReductionFactor,
			"Get the reduction factor of this filter. It's a fraction representation of events that were discard by "
			"this filter compared to the amount of incoming events.")
		.def("getNoiseThreshold", &dv::noise::FastDecayNoiseFilter<>::getNoiseThreshold,
			"Get the currently configured noise threshold.")
		.def("setNoiseThreshold", &dv::noise::FastDecayNoiseFilter<>::setNoiseThreshold, py::arg("noiseThreshold"),
			"Set a new noise threshold value.")
		.def("getHalfLife", &dv::noise::FastDecayNoiseFilter<>::getHalfLife,
			"Get the current configured half-life value. Half-life is the amount of time it takes for the internal "
			"event counter to halve. Decreasing this will increase the strength of the noise filter (cause it to "
			"reject more events).")
		.def("setHalfLife", &dv::noise::FastDecayNoiseFilter<>::setHalfLife, py::arg("halfLife"),
			"Set a new counter half-life value. Half-life is the amount of time it takes for the internal event "
			"counter to halve. Decreasing this will increase the strength of the noise filter (cause it to reject more "
			"events).");

	py::class_<dv::EventFilterChain<>, std::shared_ptr<dv::EventFilterChain<>>>(m, "EventFilterChain",
		"Event filter based on multiple event filter applied sequentially. Internally stores any added filters and")
		.def(py::init<>())
		.def("accept", &dv::EventFilterChain<>::accept, py::arg("events"), "Accepts incoming events.")
		.def("generateEvents", &dv::EventFilterChain<>::generateEvents,
			"Apply the filter algorithm and return only the filtered events from the ones that were accepted as input.")
		.def("retain", &dv::EventFilterChain<>::retain, "Test whether event belongs to an ROI.")
		.def("getNumIncomingEvents", &dv::EventFilterChain<>::getNumIncomingEvents,
			"Get number of total events that were accepted by the noise filter.")
		.def("getNumOutgoingEvents", &dv::EventFilterChain<>::getNumOutgoingEvents,
			"Total number of outgoing events from this filter instance.")
		.def("getReductionFactor", &dv::EventFilterChain<>::getReductionFactor,
			"Get the reduction factor of this filter. It's a fraction representation of events that were discard by "
			"this filter compared to the amount of incoming events.")
		.def("addFilter", &dv::EventFilterChain<>::addFilter, "Add a filter to the chain of filtering.")
		// Ugh, I need to list all available filter types.
		.def(
			"addFilter",
			[](dv::EventFilterChain<> &self, const std::shared_ptr<dv::RefractoryPeriodFilter<>> &filter) {
				self.addFilter(filter);
			},
			"Add a filter to the chain of filtering.")
		.def(
			"addFilter",
			[](dv::EventFilterChain<> &self, const std::shared_ptr<dv::EventRegionFilter<>> &filter) {
				self.addFilter(filter);
			},
			"Add a filter to the chain of filtering.")
		.def(
			"addFilter",
			[](dv::EventFilterChain<> &self, const std::shared_ptr<dv::EventPolarityFilter<>> &filter) {
				self.addFilter(filter);
			},
			"Add a filter to the chain of filtering.")
		.def(
			"addFilter",
			[](dv::EventFilterChain<> &self,
				const std::shared_ptr<dv::noise::BackgroundActivityNoiseFilter<>> &filter) {
				self.addFilter(filter);
			},
			"Add a filter to the chain of filtering.")
		.def(
			"addFilter",
			[](dv::EventFilterChain<> &self, const std::shared_ptr<dv::noise::FastDecayNoiseFilter<>> &filter) {
				self.addFilter(filter);
			},
			"Add a filter to the chain of filtering.");

	py::class_<dv::SemiDenseStereoMatcher<>, std::shared_ptr<dv::SemiDenseStereoMatcher<>>>(m, "SemiDenseStereoMatcher",
		"Semi-dense stereo matcher - a class that performs disparity calculation using an OpenCV dense disparity "
		"calculation algorithm. The implementation performs accumulation of a stereo pair of images of input events "
		"and applies the semi-global block matching (SGBM) algorithm. ")
		.def(
			py::init([](const dv::EdgeMapAccumulator &leftAccumulator, const dv::EdgeMapAccumulator &rightAccumulator) {
				return dv::SemiDenseStereoMatcher<>(std::make_unique<dv::EdgeMapAccumulator>(leftAccumulator),
					std::make_unique<dv::EdgeMapAccumulator>(rightAccumulator), dv::depth::defaultStereoMatcher());
			}),
			"leftAccumulator"_a, "rightAccumulator"_a,
			"Construct a semi dense stereo matcher object by providing custom accumulators for left and right cameras."
			"The SGBM algorithm is initialized with minimum dispartiy = 0, number of disparities = 48, block size = "
			"11.")
		.def(py::init([](const cv::Size &leftResolution, const cv::Size &rightResolution) {
			return dv::SemiDenseStereoMatcher<>(leftResolution, rightResolution, dv::depth::defaultStereoMatcher());
		}),
			"leftResolution"_a, "rightResolution"_a,
			"Construct a semi dense stereo matcher object from resolution with default accumulators. "
			"The SGBM algorithm is initialized with minimum dispartiy = 0, number of disparities = 48, block size = "
			"11.")
		.def(py::init([](const dv::camera::StereoGeometry &stereoGeometry) {
			return dv::SemiDenseStereoMatcher<>(
				std::make_unique<dv::camera::StereoGeometry>(stereoGeometry), dv::depth::defaultStereoMatcher());
		}),
			"stereoGeometry"_a,
			"Construct a semi dense stereo matcher object using stereo geometry instance, enabling depth estimation. "
			"The SGBM algorithm is initialized with minimum dispartiy = 0, number of disparities = 48, block size = "
			"11.")
		.def(
			py::init([](const dv::camera::StereoGeometry &stereoGeometry, const dv::EdgeMapAccumulator &leftAccumulator,
						 const dv::EdgeMapAccumulator &rightAccumulator) {
				return dv::SemiDenseStereoMatcher<>(std::make_unique<dv::camera::StereoGeometry>(stereoGeometry),
					std::make_unique<dv::EdgeMapAccumulator>(leftAccumulator),
					std::make_unique<dv::EdgeMapAccumulator>(rightAccumulator), dv::depth::defaultStereoMatcher());
			}),
			"stereoGeometry"_a, "leftAccumulator"_a, "rightAccumulator"_a,
			"Construct a semi dense stereo matcher object using stereo geometry instance, enabling depth estimation "
			"and with custom left / right accumulators. The SGBM algorithm is initialized with minimum dispartiy = 0, "
			"number of disparities = 48, block size = 11.")
		.def("computeDisparity", &dv::SemiDenseStereoMatcher<>::computeDisparity, "leftEvents"_a, "rightEvents"_a,
			"Compute disparity of the two given event stores. The events will be accumulated using the accumulators "
			"for left and right camera accordingly and disparity is computed using the configured block matching "
			"algorithm. The function is not going to slice the input events, so event streams have to be synchronized "
			"and sliced accordingly. The `dv::StereoEventStreamSlicer` class is a good option for slicing stereo event "
			"streams. NOTE: Accumulated frames will be rectified only if a stereo geometry class was provided during "
			"construction.")
		.def("compute", &dv::SemiDenseStereoMatcher<>::compute, "leftImage"_a, "rightImage"_a,
			"Compute stereo disparity given a time synchronized pair of images. Images will be rectified before "
			"computing disparity if a StereoGeometry class instance was provided.")
		.def("getLeftFrame", &dv::SemiDenseStereoMatcher<>::getLeftFrame,
			"Retrieve the accumulated frame from the left camera event stream.")
		.def("getRightFrame", &dv::SemiDenseStereoMatcher<>::getRightFrame,
			"Retrieve the accumulated frame from the right camera event stream.")
		.def("estimateDepth", &dv::SemiDenseStereoMatcher<>::estimateDepth, "disparity"_a, "events"_a,
			"disparityScale"_a = 16.f,
			"Estimate depth given the disparity map and a list of events. The coordinates will be rectified and a "
			"disparity value will be looked up in the disparity map. The depth of each event is calculated using an "
			"equation: depth = (focalLength * baseline) / disparity. Focal length and disparity are expressed in meter "
			"distance. For practical applications, depth estimation should be evaluated prior to any use. The directly "
			"estimated depth values can contain measurable errors which should be accounted for - the errors can "
			"usually be within 10-20% fixed absolute error distance. Usually this comes from various inaccuracies and "
			"can be mitigated by introducing a correction factor for the depth estimate.")
		.def("estimateDepthFrame", &dv::SemiDenseStereoMatcher<>::estimateDepthFrame, "disparity"_a,
			"disparityScale"_a = 16.f,
			"Convert a disparity map into a depth frame. Each disparity value is converted into depth using the "
			"equation: depth = (focalLength * baseline) / disparity. Output frame contains distance "
			"values expressed in integer values of millimeter distance.");

	py::class_<dv::SparseEventBlockMatcher, std::shared_ptr<dv::SparseEventBlockMatcher>> m_sparseMatcher(
		m, "SparseEventBlockMatcher");

	py::class_<dv::SparseEventBlockMatcher::PixelDisparity>(m_sparseMatcher, "PixelDisparity")
		.def(py::init<const cv::Point2i &, const bool, const std::optional<float>, const std::optional<float>,
				 const std::optional<int32_t>>(),
			"coordinates"_a, "valid"_a, "correlation"_a = std::nullopt, "score"_a = std::nullopt,
			"disparity"_a = std::nullopt)
		.def_readwrite("coordinates", &dv::SparseEventBlockMatcher::PixelDisparity::coordinates)
		.def_readwrite("valid", &dv::SparseEventBlockMatcher::PixelDisparity::valid)
		.def_readwrite("correlation", &dv::SparseEventBlockMatcher::PixelDisparity::correlation)
		.def_readwrite("score", &dv::SparseEventBlockMatcher::PixelDisparity::score)
		.def_readwrite("disparity", &dv::SparseEventBlockMatcher::PixelDisparity::disparity)
		.def_readwrite("templatePosition", &dv::SparseEventBlockMatcher::PixelDisparity::templatePosition)
		.def_readwrite("matchedPosition", &dv::SparseEventBlockMatcher::PixelDisparity::matchedPosition);

	m_sparseMatcher
		.def(py::init([](const dv::camera::StereoGeometry &stereoGeometry, const cv::Size &windowSize,
						  const int32_t maxDisparity, const int32_t minDisparity, const float minScore) {
			return dv::SparseEventBlockMatcher(std::make_unique<dv::camera::StereoGeometry>(stereoGeometry), windowSize,
				maxDisparity, minDisparity, minScore);
		}),
			"geometry"_a, "windowSize"_a = cv::Size(24, 24), "maxDisparity"_a = 40, "minDisparity"_a = 0,
			"minScore"_a = 1.0f,
			"Initialize a sparse stereo block matcher with a calibrated stereo geometry. This allows event "
			"rectification "
			"while calculating the disparity.")
		.def(py::init<const cv::Size &, const cv::Size &, const int32_t, const int32_t, const float>(), "resolution"_a,
			"windowSize"_a = cv::Size(24, 24), "maxDisparity"_a = 40, "minDisparity"_a = 0, "minScore"_a = 1.0f,
			"Initialize sparse event block matcher. This constructor initializes the matcher in non-rectified space, "
			"so for accurate results the event coordinates should be already rectified.")
		.def("computeDisparitySparse",
			&dv::SparseEventBlockMatcher::computeDisparitySparse<std::vector<Eigen::Vector2i>>, "left"_a, "right"_a,
			"interestPoints"_a,
			"Compute sparse disparity on given interest points. The events are accumulated sparsely only on the "
			"selected interest point regions. Returns a list of coordinates with their according disparity values, "
			"correlations and scores for each disparity match. If rectification is enabled, the returned disparity "
			"result will have `valid` flag set to false if the interest point coordinate lies outside of valid "
			"rectified pixel space. Input event has to be passed in synchronized batches, no time validation is "
			"performed during accumulation.")
		.def("getLeftMask", &dv::SparseEventBlockMatcher::getLeftMask,
			"Get the left camera image mask. The algorithm only accumulates the frames where actual matching is going "
			"to happen. The mask will contain non-zero pixel values where accumulation needs to happen.")
		.def("getRightMask", &dv::SparseEventBlockMatcher::getRightMask,
			"Get the right camera image mask. The algorithm only accumulates the frames where actual matching is going "
			"to happen. The mask will contain non-zero pixel values where accumulation needs to happen.")
		.def("getLeftFrame", &dv::SparseEventBlockMatcher::getLeftFrame, "Get the latest accumulated left frame.")
		.def("getRightFrame", &dv::SparseEventBlockMatcher::getRightFrame, "Get the latest accumulated right frame.")
		.def("getWindowSize", &dv::SparseEventBlockMatcher::getWindowSize, "Get matching window size.")
		.def("setWindowSize", &dv::SparseEventBlockMatcher::setWindowSize, "windowSize"_a,
			"Set matching window size. This is the size of cropped template image that is matched along the epipolar "
			"line of the stereo geometry.")
		.def("getMaxDisparity", &dv::SparseEventBlockMatcher::getMaxDisparity, "Get maximum disparity value.")
		.def("setMaxDisparity", &dv::SparseEventBlockMatcher::setMaxDisparity, "maxDisparity"_a,
			"Set maximum measured disparity. This parameter limits the matching space in pixels on the right camera "
			"image.")
		.def("getMinDisparity", &dv::SparseEventBlockMatcher::getMinDisparity, "Get minimum disparity value.")
		.def("setMinDisparity", &dv::SparseEventBlockMatcher::setMinDisparity, "minDisparity"_a,
			"Set minimum measured disparity. This parameter limits the matching space in pixels on the right camera "
			"image.")
		.def("getMinScore", &dv::SparseEventBlockMatcher::getMinScore, "Get minimum matching score value.")
		.def("setMinScore", &dv::SparseEventBlockMatcher::setMinScore, "minimumScore"_a,
			"Set minimum matching score value to consider the matching valid. If matching score is below this "
			"threshold, the value for a point will be set to an invalid value and `valid` boolean to false. Score is "
			"calculated by applying softmax function on the discrete distribution of correlation values from matching "
			"the template left patch on the epipolar line of the right camera image. This retrieves the probability "
			"mass function of the correlations. The best match is found by finding the max probability value and score "
			"is calculated for the best match by computing z-score over the probabilities.");
}
