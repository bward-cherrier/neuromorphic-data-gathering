#pragma once

#include "../camera/camera_geometry.hpp"

#include <utility>

namespace dv::kinematics {

class PixelMotionPredictor {
private:
	const dv::camera::CameraGeometry::SharedPtr camera;

	bool useDistortion = false;

public:
	using SharedPtr = std::shared_ptr<PixelMotionPredictor>;
	using UniquePtr = std::unique_ptr<PixelMotionPredictor>;

	/**
	 * Construct pixel motion predictor class.
	 * @param cameraGeometry 	Camera geometry class instance containing intrinsic calibration of the camera sensor.
	 */
	explicit PixelMotionPredictor(const camera::CameraGeometry::SharedPtr &cameraGeometry) : camera(cameraGeometry) {
	}

	virtual ~PixelMotionPredictor() = default;

	/**
	 * Apply delta transformation to event input and generate new transformed event store with new events that are
	 * within the new camera perspective (after applying delta transform).
	 * @param events 	Input events.
	 * @param dT 		Delta transformation to be applied.
	 * @param depth 	Scene depth.
	 * @return 			Transformed events.
	 */
	[[nodiscard]] dv::EventStore predictEvents(
		const dv::EventStore &events, const Transformationf &dT, const float depth) const {
		dv::EventStore output;
		for (const auto &event : events) {
			auto out = predict<cv::Point2f>(event, dT, depth);
			if (camera->isWithinDimensions(out)) {
				output.emplace_back(
					event.timestamp(), static_cast<int16_t>(out.x), static_cast<int16_t>(out.y), event.polarity());
			}
		}
		return output;
	}

	/**
	 * Apply delta transformation to coordinate input and generate new transformed coordinate array with new
	 * coordinates that are within the new camera perspective (after applying delta transform).
	 * @param points 	Input coordinate array.
	 * @param dT 		Delta transformation to be applied.
	 * @param depth 	Scene depth.
	 * @return 			Transformed point coordinates.
	 */
	template<concepts::Coordinate2DMutableIterable Output, concepts::Coordinate2DIterable Input>
	requires concepts::Coordinate2DCostructible<concepts::iterable_element_type<Output>>
	[[nodiscard]] Output predictSequence(const Input &points, const Transformationf &dT, const float depth) const {
		Output output;
		output.reserve(points.size());
		for (const auto &point : points) {
			auto out = predict<concepts::iterable_element_type<Output>>(point, dT, depth);
			if (camera->isWithinDimensions(out)) {
				output.push_back(out);
			}
		}
		output.shrink_to_fit();
		return output;
	}

	/**
	 * Reproject given pixel coordinates using the delta transformation and depth.
	 * @param pixel 	Input pixel coordinates.
	 * @param dT 		Delta transformation.
	 * @param depth 	Scene depth.
	 * @return 			Transformed pixel coordinate using the delta transform, camera geometry and scene depth.
	 */
	template<concepts::Coordinate2DCostructible Output, concepts::Coordinate2D Input>
	[[nodiscard]] inline Output predict(const Input &pixel, const Transformationf &dT, const float depth) const {
		if (useDistortion) {
			auto point = camera->backProjectUndistort<Eigen::Vector3f>(pixel);
			return camera->project<Output>(camera->distort<Eigen::Vector3f>(dT.transformPoint(point * depth)));
		}
		else {
			auto point = camera->backProject<Eigen::Vector3f>(pixel);
			return camera->project<Output>(dT.transformPoint(point * depth));
		}
	}

	/**
	 * Is the distortion model enabled for the reprojection of coordinates.
	 * @return 	True if the distortion model is enabled, false otherwise.
	 */
	[[nodiscard]] bool isUseDistortion() const {
		return useDistortion;
	}

	/**
	 * Enable of disable the usage of a distortion model.
	 * @param useDistortion_ 	Pass true to enable usage of the distortion model, false otherwise.
	 */
	void setUseDistortion(bool useDistortion_) {
		useDistortion = useDistortion_;
	}
};

} // namespace dv::kinematics
