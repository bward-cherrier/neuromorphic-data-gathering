#pragma once

#include "../core/concepts.hpp"
#include "../core/utils.hpp"
#include "../data/frame_base.hpp"
#include "../data/landmark_base.hpp"
#include "../exception/exceptions/generic_exceptions.hpp"
#include "../kinematics/linear_transformer.hpp"
#include "../visualization/colors.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <numbers>

namespace dv::visualization {

/**
 * Visualize the current and past poses as an image.
 */
class PoseVisualizer {
public:
	enum class Mode {
		CUSTOM = 0,
		VIEW_XY,
		VIEW_YZ,
		VIEW_ZX,
		VIEW_XZ,
		VIEW_YX,
		VIEW_ZY
	};

	enum class GridPlane {
		PLANE_NONE = 0,
		PLANE_XY,
		PLANE_YZ,
		PLANE_ZX
	};

private:
	struct Marker {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		int64_t timestamp;
		bool active;
		Eigen::Vector3f point;

		Marker(int64_t timestamp, bool active, const Eigen::Vector3f &point) :
			timestamp(timestamp),
			active(active),
			point(point) {
		}
	};

	cv::Scalar mBackgroundColor = cv::Scalar(30, 30, 30);
	cv::Scalar mGridColor       = cv::Scalar(128, 128, 128);

	// Image on which the visualization is drawn
	cv::Size2i mResolution;
	int mLineThickness   = 1; // [px]
	GridPlane mGridPlane = GridPlane::PLANE_ZX;

	// Min and max coordinates that can be displayed on the image in meters
	Eigen::Vector4f mMinPoint_W;
	Eigen::Vector4f mMaxPoint_W;

	// Size of a robot coordinate frame visualization
	float mFrameSize = 1.0; // [m]

	// Motion path of the robot
	boost::circular_buffer<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> mPath;
	dv::kinematics::LinearTransformerf mTrajectory;
	std::map<int64_t, Marker> mMarkers;
	size_t mMarkerLimit = 10'000;

	bool mDrawLinesToMarker = true;

private:
	std::vector<int64_t> mTimestamps;

	// For convenience keep the last pose separately
	dv::kinematics::Transformationf mLastPose;
	int64_t mLastTimestamp = 0;

	// Camera details
	Eigen::Vector3f mCameraPosition;
	Eigen::Quaternionf mCameraOrientation;
	const float mFocalLength = 100;
	Eigen::Matrix<float, 3, 3> mCamMat;
	Eigen::Matrix<float, 4, 4> mT_CW;
	Mode mViewMode = Mode::VIEW_ZX;

	// Offset of the positions
	dv::kinematics::Transformationf mT_OW; // Transformation from "World" to "Offset" frame

	/**
	 * Convert a pose from 3D coordinates to image frame.
	 *
	 * @param pose_W pose to project in the World frame
	 * @param mask Mask will be applied as a component-wise multiplication on the pose
	 * @return Projected pose coordinates
	 */
	[[nodiscard]] cv::Point2f projectPose(
		const Eigen::Vector4f &pose_W, const Eigen::Vector4f &mask = Eigen::Vector4f(1.f, 1.f, 1.f, 1.f)) const {
		const auto maskedPose_W = pose_W.cwiseProduct(mask);
		const auto pose_C       = mT_CW * maskedPose_W;
		if (pose_C.z() < 0) {
			// Point is behind the camera
			return {};
		}

		const auto pose_I = mCamMat * pose_C.block<3, 1>(0, 0) / pose_C.z();
		return {pose_I.x(), pose_I.y()};
	}

	/**
	 * Update the camera matrix based on the current image size.
	 */
	void refreshCameraMatrix() {
		mCamMat << mFocalLength, 0, static_cast<float>(mResolution.width) / 2.f, 0, mFocalLength,
			static_cast<float>(mResolution.height) / 2.f, 0, 0, 1;
	}

	/**
	 * Initialize minimum and maximum point coordinates.
	 */
	void initMinMax() {
		mMinPoint_W = Eigen::Vector4f(-0.5, -0.5, -0.5, 1.0);
		mMaxPoint_W = Eigen::Vector4f(0.5, 0.5, 0.5, 1.0);
	}

	/**
	 * Calculate the optimal grid span based on the maximum position span and the user defined density.
	 *
	 * @param maxSpan 	Maximum arbitrary span
	 * @return			Optimal span value
	 */
	static int getGridSpan(const float maxSpan) {
		const int maxSpanCm = static_cast<int>(maxSpan) * 100; // [cm]

		// Available grid spans in centimeters
		const std::vector<int> gridSpans{10, 25, 50, 100, 200, 500, 1000};

		// Search for grid span which gives less than nGrids grid lines
		static constexpr int nGrids = 20;
		for (const auto &gridSpan : gridSpans) {
			if (maxSpanCm / gridSpan < nGrids) {
				return gridSpan;
			}
		}
		return gridSpans.back();
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * Constructor.
	 *
	 * @param resolution size of the generated images in pixels
	 */
	explicit PoseVisualizer(
		const size_t trajectoryLength = 10000, const cv::Size2i &resolution = cv::Size2i(640, 480)) :
		mResolution(resolution),
		mPath(trajectoryLength),
		mTrajectory(trajectoryLength),
		mMarkerLimit(trajectoryLength),
		mLastPose() {
		initMinMax();
		refreshCameraMatrix();

		setViewMode(Mode::VIEW_XY);
		setGridPlane(GridPlane::PLANE_XY);
	}

	/**
	 * Update the position in which camera is located.
	 *
	 * @param newPosition New translational position of the camera in world coordinate frame.
	 */
	void updateCameraPosition(const Eigen::Vector3f &newPosition) {
		mCameraPosition      = newPosition;
		const auto transform = dv::kinematics::Transformationf(0, mCameraPosition, mCameraOrientation);
		mT_CW                = transform.inverse().getTransform();
	}

	/**
	 * Set the mode in which the pose viewer will be working.
	 *
	 * @param mode New viewing mode
	 */
	void setViewMode(const Mode mode) {
		mViewMode = mode;
	}

	void setViewMode(const std::string &str) {
		static std::unordered_map<std::string, Mode> const stringToMode = {
			{"custom", Mode::CUSTOM },
			{"XY",     Mode::VIEW_XY},
			{"YZ",     Mode::VIEW_YZ},
			{"ZX",     Mode::VIEW_ZX},
			{"XZ",     Mode::VIEW_XZ},
			{"YX",     Mode::VIEW_YX},
			{"ZY",     Mode::VIEW_ZY},
		};
		const auto it = stringToMode.find(str);
		if (it == stringToMode.end()) {
			throw std::invalid_argument("Incorrect view mode: " + str);
		}
		mViewMode = it->second;
	}

	/**
	 * Set the plane on which the grid will be displayed.
	 *
	 * @param plane Grid plane
	 */
	void setGridPlane(const GridPlane plane) {
		mGridPlane = plane;
	}

	void setGridPlane(const std::string &str) {
		static std::unordered_map<std::string, GridPlane> const stringToGridPlane = {
			{"none", GridPlane::PLANE_NONE},
            {"XY",   GridPlane::PLANE_XY  },
            {"YZ",   GridPlane::PLANE_YZ  },
			{"ZX",   GridPlane::PLANE_ZX  }
        };

		const auto it = stringToGridPlane.find(str);
		if (it == stringToGridPlane.end()) {
			throw std::invalid_argument("Incorrect grid plane: " + str);
		}
		mGridPlane = it->second;
	}

	/**
	 * Update the orientation of the camera expressed as XYZ Euler angles.
	 *
	 * @param yawDeg		Camera yaw in degrees
	 * @param pitchDeg		Camera pitch in degrees
	 * @param rollDeg		Camera roll in degrees
	 */
	void updateCameraOrientation(const float yawDeg, const float pitchDeg, const float rollDeg) {
		static constexpr float toRad = 1.f / 180.f * std::numbers::pi_v<float>;

		// The axes in euler angles should be reversed (notice the Z-Y-X ordering)
		mCameraOrientation = Eigen::AngleAxisf(rollDeg * toRad, Eigen::Vector3f::UnitZ())
						   * Eigen::AngleAxisf(pitchDeg * toRad, Eigen::Vector3f::UnitY())
						   * Eigen::AngleAxisf(yawDeg * toRad, Eigen::Vector3f::UnitX());
		auto transform = dv::kinematics::Transformationf(0, mCameraPosition, mCameraOrientation);
		mT_CW          = transform.inverse().getTransform();
	}

	/**
	 * Update the size of output image.
	 *
	 * @param newSize	New output image dimensions.
	 */
	void setFrameSize(const cv::Size2i &newSize) {
		mResolution = newSize;

		refreshCameraMatrix();
	}

	/**
	 * Update the displayed coordinate frame size.
	 *
	 * @param newSize [m]
	 */
	void setCoordinateDimensions(const float newSize) {
		assert(newSize > 0.f);
		mFrameSize = newSize;
	}

	/**
	 * Update the line thickness of the drawing.
	 *
	 * @param newThickness Drawing line thickness in pixels.
	 */
	void setLineThickness(const int newThickness) {
		assert(newThickness >= 1);
		mLineThickness = newThickness;
	}

	/**
	 * Add markers for drawing.
	 * @param landmarks 	A packet of landmarks to be drawn on the preview.
	 */
	void accept(const dv::LandmarksPacket &landmarks) {
		for (auto &[_, marker] : mMarkers) {
			marker.active = false;
		}
		Eigen::Vector4f point = Eigen::Vector4f::Ones();
		for (const auto &landmark : landmarks.elements) {
			auto iter = mMarkers.find(landmark.id);
			if (iter == mMarkers.end()) {
				mMarkers.insert(std::make_pair(
					landmark.id, Marker(landmark.timestamp, true,
									 Eigen::Vector3f(landmark.pt.x(), landmark.pt.y(), landmark.pt.z()))));
			}
			else {
				iter->second.active    = true;
				iter->second.timestamp = landmark.timestamp;
				iter->second.point     = Eigen::Vector3f(landmark.pt.x(), landmark.pt.y(), landmark.pt.z());
			}
		}
		while (mMarkers.size() > mMarkerLimit) {
			// Remove oldest batches of markers
			auto minimum = std::min_element(mMarkers.begin(), mMarkers.end(), [](const auto &a, const auto &b) {
				return a.second.timestamp < b.second.timestamp;
			});
			mMarkers.erase(minimum);
		}
	}

	/**
	 * Add a new pose to the visualization.
	 *
	 * @param pose	New pose for visualization.
	 */
	void accept(const dv::kinematics::Transformationf &pose) {
		const auto T_OB = mT_OW.getTransform() * pose.getTransform();
		const dv::kinematics::Transformationf poseInFrame(0, T_OB);

		const Eigen::Vector3f position = poseInFrame.getTranslation();

		mLastTimestamp = pose.getTimestamp();
		Eigen::Vector4f position4(position.x(), position.y(), position.z(), 1.0);
		// If moves at least one centimeter
		if (mPath.empty() || (position4 - mPath.back()).norm() > 0.01) {
			// Add a new point
			mPath.push_back(position4);
			mTimestamps.emplace_back(mLastTimestamp);
		}
		else {
			// Just update the last position timestamp instead of pushing this into a the trajectory
			mTimestamps.back() = mLastTimestamp;
		}

		// Make sure that there is always mFrameSize meters frame around the drawn trajectory
		if (position.x() < mMinPoint_W.x()) {
			mMinPoint_W.x() = position.x();
		}
		if (position.y() < mMinPoint_W.y()) {
			mMinPoint_W.y() = position.y();
		}
		if (position.z() < mMinPoint_W.z()) {
			mMinPoint_W.z() = position.z();
		}
		if (position.x() > mMaxPoint_W.x()) {
			mMaxPoint_W.x() = position.x();
		}
		if (position.y() > mMaxPoint_W.y()) {
			mMaxPoint_W.y() = position.y();
		}
		if (position.z() > mMaxPoint_W.z()) {
			mMaxPoint_W.z() = position.z();
		}

		// Update the bookkeping
		mLastPose = poseInFrame;

		mTrajectory.pushTransformation(pose);

		assert(mPath.size() == mTimestamps.size());
	}

	/**
	 * Return the timestamp of the most recent pose.
	 *
	 * @return Timestamp in Unix microsecond format.
	 */
	[[nodiscard]] int64_t getTimestamp() const {
		return mLastTimestamp;
	}

	/**
	 * Return a visualization image.
	 *
	 * @return The generated image.
	 */
	[[nodiscard]] dv::Frame generateFrame() {
		// Initialize variables used to draw on the image
		auto image            = cv::Mat(mResolution, CV_8UC3, mBackgroundColor);
		const auto pointsSpan = mMaxPoint_W - mMinPoint_W;
		auto smallerDimension = static_cast<const float>(std::min(mResolution.width, mResolution.height));
		const auto midPoint   = mMinPoint_W + pointsSpan / 2;
		float maxSpan         = 1.f;
		auto poseMask         = Eigen::Vector4f(1, 1, 1, 1);

		// Set the position and orientation of the camera
		switch (mViewMode) {
			case Mode::CUSTOM:
				break;
			case Mode::VIEW_XY: {
				maxSpan                      = std::max(pointsSpan.x(), pointsSpan.y());
				poseMask                     = Eigen::Vector4f(1, 1, 0, 1);
				const float requiredDistance = maxSpan * mFocalLength / smallerDimension;
				auto newCamPose              = Eigen::Vector3f(midPoint.x(), midPoint.y(), requiredDistance + 0.1f);
				updateCameraPosition(newCamPose);
				updateCameraOrientation(180.f, 0.f, 0.f);
				break;
			}
			case Mode::VIEW_YZ: {
				maxSpan                      = std::max(pointsSpan.y(), pointsSpan.z());
				poseMask                     = Eigen::Vector4f(0, 1, 1, 1);
				const float requiredDistance = maxSpan * mFocalLength / smallerDimension;
				auto newCamPose              = Eigen::Vector3f(requiredDistance + 0.1f, midPoint.y(), midPoint.z());
				updateCameraPosition(newCamPose);
				updateCameraOrientation(-90.f, 0.f, 90.f);
				break;
			}
			case Mode::VIEW_ZX: {
				maxSpan                      = std::max(pointsSpan.x(), pointsSpan.z());
				poseMask                     = Eigen::Vector4f(1, 0, 1, 1);
				const float requiredDistance = maxSpan * mFocalLength / smallerDimension;
				auto newCamPose              = Eigen::Vector3f(midPoint.x(), requiredDistance + 0.1f, midPoint.z());
				updateCameraPosition(newCamPose);
				updateCameraOrientation(-90.f, -90.f, 180.f);
				break;
			}
			case Mode::VIEW_XZ: {
				maxSpan                      = std::max(pointsSpan.x(), pointsSpan.z());
				poseMask                     = Eigen::Vector4f(1, 0, 1, 1);
				const float requiredDistance = maxSpan * mFocalLength / smallerDimension;
				auto newCamPose              = Eigen::Vector3f(midPoint.x(), -(requiredDistance + 0.1f), midPoint.z());
				updateCameraPosition(newCamPose);
				updateCameraOrientation(-90.f, 0.f, 0.f);
				break;
			}
			case Mode::VIEW_YX: {
				maxSpan                      = std::max(pointsSpan.y(), pointsSpan.x());
				poseMask                     = Eigen::Vector4f(1, 1, 0, 1);
				const float requiredDistance = maxSpan * mFocalLength / smallerDimension;
				auto newCamPose              = Eigen::Vector3f(midPoint.x(), midPoint.y(), -(requiredDistance + 0.1f));
				updateCameraPosition(newCamPose);
				updateCameraOrientation(0.f, 0.f, 90.f);
				break;
			}
			case Mode::VIEW_ZY: {
				maxSpan                      = std::max(pointsSpan.y(), pointsSpan.z());
				poseMask                     = Eigen::Vector4f(0, 1, 1, 1);
				const float requiredDistance = maxSpan * mFocalLength / smallerDimension;
				auto newCamPose              = Eigen::Vector3f(-(requiredDistance + 0.1f), midPoint.y(), midPoint.z());
				updateCameraPosition(newCamPose);
				updateCameraOrientation(0.f, -90.f, 180.f);
				break;
			}
			default:
				throw std::runtime_error("Incorrect mode");
		}

		// Draw the grid on the image
		const int gridSpan              = getGridSpan(maxSpan); // [cm]
		const auto gridOffset           = Eigen::Vector4f(10.f, 10.f, 10.f, 1.f);
		const Eigen::Vector4i gridStart = (mMinPoint_W - gridOffset).cast<int>() * 100; // [cm]
		const Eigen::Vector4i gridEnd   = (mMaxPoint_W + gridOffset).cast<int>() * 100; // [cm]

		// Draw grid by iterating over one index and keeping another index constant
		auto drawGrid = [&image, &gridStart, &gridEnd, &gridSpan, this](long iteratingIdx, long constantIdx) {
			assert(0 <= iteratingIdx && iteratingIdx <= 2);
			assert(0 <= constantIdx && constantIdx <= 2);

			for (int majorCoord = gridStart[iteratingIdx]; majorCoord <= gridEnd[iteratingIdx];
				 majorCoord     += gridSpan) {
				auto from_W          = Eigen::Vector4f(0.f, 0.f, 0.f, 1.f);
				auto to_W            = Eigen::Vector4f(0.f, 0.f, 0.f, 1.f);
				from_W[iteratingIdx] = static_cast<float>(majorCoord) / 100.f;             // [m]
				to_W[iteratingIdx]   = static_cast<float>(majorCoord) / 100.f;             // [m]
				from_W[constantIdx]  = static_cast<float>(gridStart[constantIdx]) / 100.f; // [m]
				to_W[constantIdx]    = static_cast<float>(gridEnd[constantIdx]) / 100.f;   // [m]

				const auto from_I = projectPose(from_W);
				const auto to_I   = projectPose(to_W);

				if (from_I == cv::Point2f() || to_I == cv::Point2f()) {
					continue;
				}

				cv::line(image, from_I, to_I, mGridColor, 1, cv::LINE_AA);
			}
		};

		switch (mGridPlane) {
			case GridPlane::PLANE_NONE: {
				break;
			}
			case GridPlane::PLANE_XY: {
				drawGrid(0, 1);
				drawGrid(1, 0);
				break;
			}
			case GridPlane::PLANE_YZ: {
				drawGrid(1, 2);
				drawGrid(2, 1);
				break;
			}
			case GridPlane::PLANE_ZX: {
				drawGrid(0, 2);
				drawGrid(2, 0);
				break;
			}
			default:
				throw std::runtime_error("Incorrect grid plane mode");
		}

		// Draw the World frame
		// W - world
		// C - camera
		// I - image (2D)
		// CW - transformation from world to camera
		auto origin_W   = Eigen::Vector4f(0.f, 0.f, 0.f, 1.f);
		auto xAxisEnd_W = Eigen::Vector4f(mFrameSize, 0.f, 0.f, 1.f);
		auto yAxisEnd_W = Eigen::Vector4f(0.f, mFrameSize, 0.f, 1.f);
		auto zAxisEnd_W = Eigen::Vector4f(0.f, 0.f, mFrameSize, 1.f);

		auto origin_I   = projectPose(origin_W, poseMask);
		auto xAxisEnd_I = projectPose(xAxisEnd_W, poseMask);
		auto yAxisEnd_I = projectPose(yAxisEnd_W, poseMask);
		auto zAxisEnd_I = projectPose(zAxisEnd_W, poseMask);
		if (origin_I != cv::Point2f()) {
			const auto axisLabelOffset = cv::Point2f(10.f, 10.f);

			if (xAxisEnd_I != cv::Point2f()) {
				cv::arrowedLine(
					image, origin_I, xAxisEnd_I, cv::Scalar(0, 0, 255), mLineThickness, cv::LINE_AA, 0, 0.05);
				cv::putText(
					image, "x", xAxisEnd_I + axisLabelOffset, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
			}

			if (yAxisEnd_I != cv::Point2f()) {
				cv::arrowedLine(
					image, origin_I, yAxisEnd_I, cv::Scalar(0, 255, 0), mLineThickness, cv::LINE_AA, 0, 0.05);
				cv::putText(
					image, "y", yAxisEnd_I + axisLabelOffset, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
			}

			if (zAxisEnd_I != cv::Point2f()) {
				cv::arrowedLine(
					image, origin_I, zAxisEnd_I, cv::Scalar(255, 0, 0), mLineThickness, cv::LINE_AA, 0, 0.05);
				cv::putText(
					image, "z", zAxisEnd_I + axisLabelOffset, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
			}
		}

		// Draw the path history
		const cv::Point2f zeroPoint;
		for (size_t i = 0; i + 1 < mPath.size(); ++i) {
			auto from = projectPose(mPath.at(i), poseMask);
			auto to   = projectPose(mPath.at(i + 1), poseMask);

			cv::Point2f diff = to - from;
			if (diff.dot(diff) < std::numeric_limits<float>::epsilon() || from == zeroPoint || to == zeroPoint) {
				// short segment || invalid from || invalid to
				continue;
			}

			auto relativeHeight = [&i, this](long coordinate) {
				assert(0 <= coordinate && coordinate <= 2);

				return (mPath.at(i)[coordinate] - mMinPoint_W[coordinate] + mPath.at(i + 1)[coordinate]
						   - mMinPoint_W[coordinate])
					 * 0.5f / (mMaxPoint_W[coordinate] - mMinPoint_W[coordinate]);
			};

			float relHeight;
			switch (mViewMode) {
				case Mode::CUSTOM:
				case Mode::VIEW_XY:
				case Mode::VIEW_XZ: {
					relHeight = relativeHeight(2);
					break;
				}
				case Mode::VIEW_YX:
				case Mode::VIEW_YZ: {
					relHeight = relativeHeight(0);
					break;
				}
				case Mode::VIEW_ZX:
				case Mode::VIEW_ZY: {
					relHeight = relativeHeight(1);
					break;
				}
				default:
					throw std::runtime_error("Incorrect mode");
			}

			const auto color1 = cv::Scalar(255, 255, 0);
			const auto color2 = cv::Scalar(0, 255, 255);
			cv::line(image, from, to, relHeight * color1 + (1.0f - relHeight) * color2, mLineThickness, cv::LINE_AA);
		}

		auto vec3toVec4 = [](const Eigen::Vector3f &vec) {
			return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.0);
		};

		std::optional<cv::Point2f> camera = std::nullopt;

		// Draw robot coordinate frame
		if (!mLastPose.getTranslation().isZero()) {
			const Eigen::Matrix3f rotMat = mLastPose.getRotationMatrix();
			const auto eX                = rotMat.col(0) * mFrameSize / 3;
			const auto eY                = rotMat.col(1) * mFrameSize / 3;
			const auto eZ                = rotMat.col(2) * mFrameSize / 3;

			camera   = projectPose(vec3toVec4(mLastPose.getTranslation()), poseMask);
			auto xTo = projectPose(
				vec3toVec4(mLastPose.getTranslation()) + Eigen::Vector4f(eX.x(), eX.y(), eX.z(), 0.0), poseMask);
			auto yTo = projectPose(
				vec3toVec4(mLastPose.getTranslation()) + Eigen::Vector4f(eY.x(), eY.y(), eY.z(), 0.0), poseMask);
			auto zTo = projectPose(
				vec3toVec4(mLastPose.getTranslation()) + Eigen::Vector4f(eZ.x(), eZ.y(), eZ.z(), 0.0), poseMask);

			cv::arrowedLine(image, *camera, xTo, cv::Scalar(0, 0, 255), mLineThickness, cv::LINE_AA, 0, 0.05);
			cv::arrowedLine(image, *camera, yTo, cv::Scalar(0, 255, 0), mLineThickness, cv::LINE_AA, 0, 0.05);
			cv::arrowedLine(image, *camera, zTo, cv::Scalar(255, 0, 0), mLineThickness, cv::LINE_AA, 0, 0.05);

			// Format the timestamp to a nice string
			auto tsToString = [](const int64_t ts) {
				const int64_t s         = ts / 1000000;
				const int64_t us        = ts % 1000000;
				const int64_t roundedMs = us / 1000;

				// Determine whether the timestamp is in UTC or counting from 0 assuming nobody records longer than 1w
				if (s > 3600 * 24 * 7) {
					return fmt::format("{}.{:06}", dv::toTimePoint(ts), us);
				}
				else {
					// Simple timestamp in seconds . milliseconds
					std::stringstream ss;
					ss << "timestamp = " << s << "." << roundedMs << "s";
					return ss.str();
				}
			};

			// Round to two decimal places
			auto roundTwo = [](float val) {
				const int value = static_cast<int>(lround(val * 100.f));
				return static_cast<float>(value) / 100;
			};

			// Text on the top of the frame:
			std::stringstream posText;
			posText << "position = [" << roundTwo(mLastPose.getTranslation().x()) << ", "
					<< roundTwo(mLastPose.getTranslation().y()) << ", " << roundTwo(mLastPose.getTranslation().z())
					<< "]";
			cv::putText(
				image, posText.str(), cv::Point(15, 15), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

			std::stringstream timeText;
			timeText << tsToString(mLastTimestamp);
			cv::putText(
				image, timeText.str(), cv::Point(15, 35), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

			if (mGridPlane != GridPlane::PLANE_NONE) {
				std::string gridText;

				if (gridSpan < 100) {
					// Centimeters (below one meter)
					gridText = fmt::format("Grid span: {} cm", gridSpan);
				}
				else if (gridSpan < 100'000) {
					// Meters (between one meter and one kilometer)
					gridText = fmt::format("Grid span: {:.1} m", static_cast<float>(gridSpan) / 100.f);
				}
				else {
					// Kilometers (above one kilometer, I guess we don't need to handle light-year distance drift?
					gridText = fmt::format("Grid span: {:.1} km", static_cast<float>(gridSpan) / 100'000.f);
				}

				cv::putText(
					image, gridText, cv::Point(15, 55), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
			}
		}

		if (!mMarkers.empty()) {
			Eigen::Vector4f pointSpace = Eigen::Vector4f::Ones();
			for (const auto &[id, marker] : mMarkers) {
				pointSpace.block<3, 1>(0, 0) = mT_OW.transformPoint<Eigen::Vector3f>(marker.point);

				auto point        = projectPose(pointSpace, poseMask);
				const auto &color = dv::visualization::colors::someNeonColor(static_cast<int32_t>(id));
				cv::circle(image, point, 3.0f, color, cv::FILLED);
				if (camera.has_value() && marker.active) {
					cv::line(image, *camera, point, color, 1);
				}
			}
		}

		dv::Frame frame(mLastTimestamp, image);
		frame.source = dv::FrameSource::VISUALIZATION;
		return frame;
	}

	/**
	 * Reset the pose history and set an offset to the last pose.
	 */
	void reset() {
		if (mPath.empty()) {
			return;
		}
		assert(!mTimestamps.empty());
		mLastTimestamp = mTimestamps.back();
		mFrameSize     = 1.0;

		mPath.clear();
		mTimestamps.clear();
		const auto T_WO1  = mT_OW.inverse().getTransform();
		const auto T_O1O2 = mLastPose.getTransform();
		const auto T_WO2  = T_WO1 * T_O1O2;
		mT_OW             = dv::kinematics::Transformationf(mLastPose.getTimestamp(), T_WO2).inverse();
		mTrajectory.clear();

		initMinMax();
	}

	/**
	 * Get the background color.
	 * @return		Background color.
	 */
	[[nodiscard]] const cv::Scalar &getBackgroundColor() const {
		return mBackgroundColor;
	}

	/**
	 * Set new background color.
	 * @param backgroundColor 	OpenCV scalar for the background color.
	 */
	void setBackgroundColor(const cv::Scalar &backgroundColor) {
		PoseVisualizer::mBackgroundColor = backgroundColor;
	}

	/**
	 * Get the grid line color.
	 * @return 		Grid line color
	 */
	[[nodiscard]] const cv::Scalar &getGridColor() const {
		return mGridColor;
	}

	/**
	 * Set new grid line color
	 * @param mGridColor 	OpenCV scalar for the grid line color.
	 */
	void setGridColor(const cv::Scalar &gridColor) {
		PoseVisualizer::mGridColor = gridColor;
	}

	/**
	 * Check whether drawing of lines to landmark markers is enabled.
	 * @return 		True if drawing of lines is enabled, false otherwise.
	 */
	[[nodiscard]] bool getDrawLinesToLandmarks() const {
		return mDrawLinesToMarker;
	}

	/**
	 * Enable or disable drawing of lines from camera to active landmarks. Active
	 * landmarks are those which were accepted by the visualizer with last `accept(dv::LandmarksPacket)` call.
	 * @param drawLinesToLandmarks
	 */
	void setDrawLinesToLandmarks(bool drawLinesToLandmarks) {
		mDrawLinesToMarker = drawLinesToLandmarks;
	}

	/**
	 * Get the maximum number of landmarks to be drawn.
	 * @return 		Maximum number of landmarks
	 */
	[[nodiscard]] size_t getLandmarkLimit() const {
		return mMarkerLimit;
	}

	/**
	 * Set a limit for number of landmarks that are stored and drawn.
	 * @param numLandmarks 		Number of landmarks
	 */
	void setLandmarkLimit(size_t numLandmarks) {
		mMarkerLimit = numLandmarks;
	}

	/**
	 * Get the number of landmarks currently stored in the visualizer.
	 * @return 		Number of landmarks stored in the visualizer.
	 */
	[[nodiscard]] size_t getLandmarkSize() const {
		return mMarkers.size();
	}

	/**
	 * Remove all landmarks stored in the landmarks buffer.
	 */
	void clearLandmarks() {
		mMarkers.clear();
	}
};

static_assert(concepts::FrameOutputGenerator<dv::visualization::PoseVisualizer>);
static_assert(concepts::Accepts<dv::visualization::PoseVisualizer, dv::kinematics::Transformationf>);
static_assert(concepts::Accepts<dv::visualization::PoseVisualizer, dv::LandmarksPacket>);

} // namespace dv::visualization
