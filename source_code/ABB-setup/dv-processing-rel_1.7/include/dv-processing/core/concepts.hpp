#pragma once

#include "../data/bounding_box_base.hpp"
#include "../data/depth_event_base.hpp"
#include "../data/depth_frame_base.hpp"
#include "../data/event_base.hpp"
#include "../data/frame_base.hpp"
#include "../data/imu_base.hpp"
#include "../data/landmark_base.hpp"
#include "../data/pose_base.hpp"
#include "../data/timed_keypoint_base.hpp"
#include "../data/trigger_base.hpp"

#include <Eigen/Core>
#include <boost/callable_traits.hpp>
#include <opencv2/core.hpp>

#include <concepts>
#include <iterator>

namespace dv {

namespace concepts {

namespace internal {

template<typename T>
struct is_eigen_impl : std::false_type {};

template<typename T, int... Is>
struct is_eigen_impl<Eigen::Matrix<T, Is...>> : std::true_type {};

} // namespace internal

template<typename T>
constexpr bool is_eigen_type = internal::is_eigen_impl<T>::value;

template<class T>
using iterable_element_type = typename std::remove_reference_t<decltype(*(std::declval<T>().begin()))>;

template<typename T>
concept Iterable = requires(T t) {
					   { t.begin() };
					   { t.end() };
					   { t.front() };
					   { t.back() };
					   { t.size() } -> std::same_as<size_t>;
				   };

static_assert(Iterable<dv::cvector<int>>);

template<typename Needle, typename... Haystack>
inline constexpr bool is_type_one_of = std::disjunction_v<std::is_same<Needle, Haystack>...>;

template<typename T>
concept MutableIterable = Iterable<T> and requires(T t, size_t sizeType) {
											  { t.emplace_back() };
											  { t.push_back(*t.begin()) };
											  { t.shrink_to_fit() };
											  { t.resize(sizeType) };
											  { t.reserve(sizeType) };
											  // Single element erase
											  { t.erase(t.begin()) };
											  // Range erase
											  { t.erase(t.begin(), t.end()) };
											  { t.operator[](sizeType) };
										  };

static_assert(MutableIterable<dv::cvector<int>>);
static_assert(MutableIterable<std::vector<int>>);

template<typename T>
concept number = std::integral<T> || std::floating_point<T>;

template<class T>
concept Enum = std::is_enum_v<T>;

template<class T>
concept AddressableEvent = requires(T t) {
							   { t.x() } -> std::signed_integral;
							   { t.y() } -> std::signed_integral;
							   { t.timestamp() } -> std::same_as<int64_t>;
							   { t.polarity() } -> std::same_as<bool>;
						   };

static_assert(AddressableEvent<dv::DepthEvent>);
static_assert(AddressableEvent<dv::Event>);

template<class T>
concept TimestampedByAccessor = requires(const T &t) {
									{ t.timestamp() } -> std::same_as<int64>;
								};

template<class T>
concept FlatbufferPacket
	= std::derived_from<std::remove_cvref_t<T>, flatbuffers::NativeTable>
  and requires(T t) {
		  typename std::remove_cvref_t<T>::TableType;
		  requires std::convertible_to<decltype(std::remove_cvref_t<T>::TableType::identifier), std::string>;
	  };

static_assert(FlatbufferPacket<dv::EventPacket>);
static_assert(FlatbufferPacket<const dv::EventPacket &>);
static_assert(FlatbufferPacket<dv::EventPacket &>);
static_assert(FlatbufferPacket<dv::Pose>);
static_assert(FlatbufferPacket<const dv::Pose &>);
static_assert(FlatbufferPacket<dv::Pose &>);
static_assert(FlatbufferPacket<dv::TimedKeyPointPacket>);
static_assert(FlatbufferPacket<const dv::TimedKeyPointPacket &>);
static_assert(FlatbufferPacket<dv::TimedKeyPointPacket &>);

template<class T>
concept TimestampedByMember = requires {
								  // See comments in https://stackoverflow.com/a/63347691
								  requires std::same_as<decltype(T::timestamp), int64_t>;
							  };

template<class T>
concept Timestamped = TimestampedByAccessor<T> or TimestampedByMember<T>;

template<class T>
concept TimestampedIterable = Iterable<T> and Timestamped<iterable_element_type<T>>;

static_assert(TimestampedIterable<dv::cvector<dv::Frame>>);
static_assert(not TimestampedIterable<dv::TriggerPacket>);

static_assert(Timestamped<dv::BoundingBox>);
static_assert(Timestamped<dv::DepthEvent>);
static_assert(Timestamped<dv::DepthFrame>);
static_assert(Timestamped<dv::Event>);
static_assert(Timestamped<dv::Frame>);
static_assert(Timestamped<dv::IMU>);
static_assert(Timestamped<dv::Landmark>);
static_assert(Timestamped<dv::Pose>);
static_assert(Timestamped<dv::TimedKeyPoint>);
static_assert(Timestamped<dv::Trigger>);

template<class T>
concept HasElementsVector = requires(const T &t) {
								t.elements;
								requires Iterable<decltype(t.elements)>;
							};

template<class T>
concept HasTimestampedElementsVectorByAccessor
	= HasElementsVector<T>
  and requires(const T &t) { requires TimestampedByAccessor<std::remove_reference_t<decltype(t.elements.front())>>; };

template<class T>
concept HasTimestampedElementsVectorByMember
	= HasElementsVector<T>
  and requires(const T &t) { requires TimestampedByMember<std::remove_reference_t<decltype(t.elements.front())>>; };

template<class T>
concept HasTimestampedElementsVector
	= HasTimestampedElementsVectorByAccessor<T> or HasTimestampedElementsVectorByMember<T>;

static_assert(HasTimestampedElementsVector<dv::BoundingBoxPacket>);
static_assert(HasTimestampedElementsVector<dv::DepthEventPacket>);
static_assert(!HasTimestampedElementsVector<dv::DepthFrame>);
static_assert(HasTimestampedElementsVector<dv::EventPacket>);
static_assert(!HasTimestampedElementsVector<dv::Frame>);
static_assert(HasTimestampedElementsVector<dv::IMUPacket>);
static_assert(HasTimestampedElementsVector<dv::LandmarksPacket>);
static_assert(!HasTimestampedElementsVector<dv::Pose>);
static_assert(HasTimestampedElementsVector<dv::TimedKeyPointPacket>);
static_assert(HasTimestampedElementsVector<dv::TriggerPacket>);

template<class T>
concept Coordinate2DMembers = requires {
								  // See comments in https://stackoverflow.com/a/63347691
								  requires number<decltype(T::x)>;
								  requires number<decltype(T::y)>;
							  };

template<class T>
concept Coordinate2DAccessors = requires(T t) {
									requires number<std::remove_reference_t<decltype(t.x())>>;
									requires number<std::remove_reference_t<decltype(t.y())>>;
								};

template<class T>
concept Coordinate3DMembers = requires {
								  requires number<decltype(T::x)>;
								  requires number<decltype(T::y)>;
								  requires number<decltype(T::z)>;
							  };

template<class T>
concept Coordinate3DAccessors = requires(T t) {
									requires number<std::remove_reference_t<decltype(t.x())>>;
									requires number<std::remove_reference_t<decltype(t.y())>>;
									requires number<std::remove_reference_t<decltype(t.z())>>;
								};

template<class T>
concept Coordinate2D
	= (Coordinate2DAccessors<T> or Coordinate2DMembers<T>)
  and (!is_eigen_type<T> or (static_cast<bool>(T::IsVectorAtCompileTime) and T::SizeAtCompileTime >= 2));

template<class T>
concept Coordinate3D
	= (Coordinate3DAccessors<T> or Coordinate3DMembers<T>)
  and (!is_eigen_type<T> or (static_cast<bool>(T::IsVectorAtCompileTime) and T::SizeAtCompileTime >= 3));

template<class T>
concept Coordinate2DCostructible = Coordinate2D<T> and requires {
														   { T(0, 0) } -> std::same_as<T>;
													   };

template<class T>
concept Coordinate3DCostructible = Coordinate3D<T> and requires {
														   { T(0, 0, 0) } -> std::same_as<T>;
													   };

template<class T>
concept Coordinate2DIterable = Iterable<T> and Coordinate2D<iterable_element_type<T>>;

template<class T>
concept Coordinate2DMutableIterable = MutableIterable<T> and Coordinate2D<iterable_element_type<T>>
								  and Coordinate2DCostructible<iterable_element_type<T>>;

template<class T>
concept Coordinate3DIterable = Iterable<T> and Coordinate3D<iterable_element_type<T>>;

template<class T>
concept Coordinate3DMutableIterable = MutableIterable<T> and Coordinate3D<iterable_element_type<T>>
								  and Coordinate3DCostructible<iterable_element_type<T>>;

static_assert(Coordinate2D<cv::Point2i>);
static_assert(Coordinate2D<Eigen::Vector2i>);
static_assert(Coordinate2D<cv::Point2f>);
static_assert(Coordinate2D<Eigen::Vector2f>);
static_assert(Coordinate2D<cv::Point2d>);
static_assert(Coordinate2D<Eigen::Vector2d>);

static_assert(Coordinate2D<cv::Point3i>);
static_assert(Coordinate2D<Eigen::Vector3i>);
static_assert(Coordinate2D<cv::Point3f>);
static_assert(Coordinate2D<Eigen::Vector3f>);
static_assert(Coordinate2D<cv::Point3d>);
static_assert(Coordinate2D<Eigen::Vector3d>);

static_assert(not Coordinate3D<cv::Point2i>);
static_assert(not Coordinate3D<Eigen::Vector2i>);
static_assert(not Coordinate3D<cv::Point2f>);
static_assert(not Coordinate3D<Eigen::Vector2f>);
static_assert(not Coordinate3D<cv::Point2d>);
static_assert(not Coordinate3D<Eigen::Vector2d>);

static_assert(Coordinate3D<cv::Point3i>);
static_assert(Coordinate3D<Eigen::Vector3i>);
static_assert(Coordinate3D<cv::Point3f>);
static_assert(Coordinate3D<Eigen::Vector3f>);
static_assert(Coordinate3D<cv::Point3d>);
static_assert(Coordinate3D<Eigen::Vector3d>);

template<class T>
concept EigenType = is_eigen_type<T>;

template<class T1, class T2>
concept InputStreamableFrom = requires(T1 t1, T2 t2) {
								  { t2 >> t1 } -> std::same_as<T1 &>;
							  };

template<class T1, class T2>
concept InputStreamableTo = requires(T1 t1, T2 t2) {
								{ t1 >> t2 } -> std::same_as<T2 &>;
							};

template<class T1, class T2>
concept Accepts = requires(T1 t1, T2 t2) {
					  { t1.accept(t2) };
				  };

template<class T1, class T2>
concept OutputStreamableFrom = requires(T1 t1, T2 t2) {
								   { t1 << t2 } -> std::same_as<T1 &>;
							   };

template<class T1, class T2>
concept OutputStreamableTo = requires(T1 t1, T2 t2) {
								 { t2 << t1 } -> std::same_as<T2 &>;
							 };

template<class T>
concept FrameOutputGenerator = requires(T t) {
								   { t.generateFrame() } -> std::same_as<dv::Frame>;
							   };

template<class T, class EventStoreType>
concept EventOutputGenerator = requires(T t) {
								   { t.generateEvents() } -> std::same_as<EventStoreType>;
							   };

template<class T1, class T2>
concept IOStreamableFrom = InputStreamableFrom<T1, T2> && OutputStreamableFrom<T1, T2>;

template<class T1, class T2>
concept IOStreamableTo = InputStreamableTo<T1, T2> && OutputStreamableTo<T1, T2>;

template<class T, class EventStoreType>
concept EventToFrameConverter = OutputStreamableFrom<T, EventStoreType> && Accepts<T, EventStoreType>
							 && InputStreamableTo<T, dv::Frame> && FrameOutputGenerator<T>;

template<class T>
concept FrameToFrameConverter = OutputStreamableFrom<T, dv::Frame> && Accepts<T, dv::Frame>
							 && InputStreamableTo<T, dv::Frame> && FrameOutputGenerator<T>;

template<class T, class EventStoreType>
concept EventToEventConverter
	= OutputStreamableFrom<T, EventStoreType> && Accepts<T, EventStoreType> && InputStreamableTo<T, EventStoreType>
   && InputStreamableTo<T, EventStoreType> && EventOutputGenerator<T, EventStoreType>;

template<class T, class EventStoreType>
concept FrameToEventConverter = OutputStreamableFrom<T, dv::Frame> && Accepts<T, dv::Frame>
							 && InputStreamableTo<T, EventStoreType> && EventOutputGenerator<T, EventStoreType>;

template<class T>
concept BlockAccessible = requires(T t, int16_t i) {
							  { t.block(i, i, i, i) };
						  };

template<class T>
concept TimestampMatrixContainer = requires(T t, int16_t i) {
									   { t(i, i) } -> std::same_as<int64_t &>;
									   { t.at(i, i) } -> std::same_as<int64_t &>;
								   };

template<class T>
concept TimedImageContainer = requires {
								  // See comments in https://stackoverflow.com/a/63347691
								  requires std::same_as<decltype(T::image), cv::Mat>;
								  requires std::same_as<decltype(T::timestamp), int64_t>;
							  };

template<class T>
concept SupportsConstantDepth = requires(T t, float d) {
									{ t.setConstantDepth(d) };
								};

template<class T, class EventStoreType>
concept TimeSurface = BlockAccessible<T> && EventToFrameConverter<T, EventStoreType>;

template<class T, class Input>
concept DVFeatureDetectorAlgorithm = requires(T t, Input i, cv::Rect roi, cv::Mat mask) {
										 { t.detect(i, roi, mask) } -> std::same_as<dv::cvector<dv::TimedKeyPoint>>;
									 };

template<class T>
concept OpenCVFeatureDetectorAlgorithm = requires(T *t, cv::Mat m, std::vector<cv::KeyPoint> keypoints, cv::Mat mask) {
											 { t->detect(m, keypoints, mask) } -> std::same_as<void>;
										 };

template<class T, class Input>
concept FeatureDetectorAlgorithm = DVFeatureDetectorAlgorithm<T, Input> || OpenCVFeatureDetectorAlgorithm<T>;

template<class T, class EventStoreType>
concept EventFilter
	= EventToEventConverter<T, EventStoreType> and requires(T t, typename EventStoreType::value_type event) {
													   { t.retain(event) } -> std::same_as<bool>;
												   };

/**
 * Checks if function is invocable with the given argument types exactly and its
 * return value is the same as the given return type.
 *
 * @tparam FUNC function-like object to check.
 * @tparam RETURN_TYPE required return type.
 * @tparam ARGUMENTS_TYPES required argument types.
 */
template<typename FUNC, typename RETURN_TYPE, typename... ARGUMENTS_TYPES>
concept InvocableReturnArgumentsStrong
	= std::regular_invocable<FUNC, ARGUMENTS_TYPES...>
   && std::same_as<boost::callable_traits::return_type_t<FUNC>, RETURN_TYPE>
   && std::same_as<boost::callable_traits::args_t<FUNC>, std::tuple<ARGUMENTS_TYPES...>>;

/**
 * Checks if function is invocable with the given argument types and its
 * return value is convertible to the given return type.
 *
 * @tparam FUNC function-like object to check.
 * @tparam RETURN_TYPE required return type.
 * @tparam ARGUMENTS_TYPES required argument types.
 */
template<typename FUNC, typename RETURN_TYPE, typename... ARGUMENTS_TYPES>
concept InvocableReturnArgumentsWeak = std::regular_invocable<FUNC, ARGUMENTS_TYPES...>
									&& std::convertible_to<boost::callable_traits::return_type_t<FUNC>, RETURN_TYPE>;

template<class T>
concept KeyPointVector = MutableIterable<T> and requires(iterable_element_type<T> t) {
													{ Coordinate2D<decltype(t.pt)> };
												};

template<class T>
concept EventStorage = Iterable<T> and requires(T t, int64_t timestamp, size_t size) {
										   { t.getLowestTime() } -> std::same_as<int64_t>;
										   { t.getHighestTime() } -> std::same_as<int64_t>;
										   { t.slice(size, size) } -> std::same_as<T>;
										   { t.sliceBack(size) } -> std::same_as<T>;
										   { t.sliceTime(timestamp, timestamp) } -> std::same_as<T>;
										   { t.isEmpty() } -> std::same_as<bool>;
									   };

template<class Packet>
concept DataPacket = requires(Packet t) {
						 { TimestampedIterable<decltype(t.elements)> };
					 };

} // namespace concepts
} // namespace dv
