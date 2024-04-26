#pragma once

#include "../../core/utils.hpp"
#include "../../data/bounding_box_base.hpp"
#include "../../data/depth_event_base.hpp"
#include "../../data/depth_frame_base.hpp"
#include "../../data/event_base.hpp"
#include "../../data/frame_base.hpp"
#include "../../data/imu_base.hpp"
#include "../../data/landmark_base.hpp"
#include "../../data/pose_base.hpp"
#include "../../data/timed_keypoint_base.hpp"
#include "../../data/trigger_base.hpp"
#include "../../data/types.hpp"
#include "../data/FileDataTable.hpp"
#include "../data/IOHeader.hpp"
#include "io_data_buffer.hpp"
#include "io_statistics.hpp"

#include <string_view>

namespace dv::io::support {

static constexpr std::string_view AEDAT4_FILE_EXTENSION{".aedat4"};
static constexpr std::string_view AEDAT4_HEADER_VERSION{"#!AER-DAT4.0\r\n"};

struct Sizes {
	uint64_t mPacketElements{0};
	uint64_t mPacketSize{0};
	uint64_t mDataSize{0};
};

using TypeResolver = dv::std_function_exact<const dv::types::Type *(const uint32_t)>;

[[nodiscard]] static inline const dv::types::Type *defaultTypeResolver(const uint32_t typeId) {
	static constexpr auto eventType       = dv::types::makeTypeDefinition<dv::EventPacket, dv::Event>();
	static constexpr auto frameType       = dv::types::makeTypeDefinition<dv::Frame, dv::Frame>();
	static constexpr auto imuType         = dv::types::makeTypeDefinition<dv::IMUPacket, dv::IMU>();
	static constexpr auto triggerType     = dv::types::makeTypeDefinition<dv::TriggerPacket, dv::Trigger>();
	static constexpr auto boundingBoxType = dv::types::makeTypeDefinition<dv::BoundingBoxPacket, dv::BoundingBox>();
	static constexpr auto depthEventType  = dv::types::makeTypeDefinition<dv::DepthEventPacket, dv::DepthEvent>();
	static constexpr auto depthFrameType  = dv::types::makeTypeDefinition<dv::DepthFrame, dv::DepthFrame>();
	static constexpr auto poseType        = dv::types::makeTypeDefinition<dv::Pose, dv::Pose>();
	static constexpr auto timedKeyPointType
		= dv::types::makeTypeDefinition<dv::TimedKeyPointPacket, dv::TimedKeyPoint>();
	static constexpr auto landmarksType = dv::types::makeTypeDefinition<dv::LandmarksPacket, dv::Landmark>();

	switch (typeId) {
		case eventType.id:
			return &eventType;

		case frameType.id:
			return &frameType;

		case imuType.id:
			return &imuType;

		case triggerType.id:
			return &triggerType;

		case boundingBoxType.id:
			return &boundingBoxType;

		case depthEventType.id:
			return &depthEventType;

		case depthFrameType.id:
			return &depthFrameType;

		case poseType.id:
			return &poseType;

		case timedKeyPointType.id:
			return &timedKeyPointType;

		case landmarksType.id:
			return &landmarksType;

		default:
			return nullptr;
	}
}

template<class PacketType>
requires dv::concepts::FlatbufferPacket<PacketType>
[[nodiscard]] inline std::shared_ptr<dv::types::TypedObject> packetToObject(
	PacketType &&packet, const TypeResolver &resolver = defaultTypeResolver) {
	using UnderlyingPacketType = std::remove_cvref_t<PacketType>;

	const dv::types::Type *type
		= resolver(dv::types::IdentifierStringToId(UnderlyingPacketType::TableType::identifier));

	auto objPtr = std::make_shared<dv::types::TypedObject>(*type);

	// Will call proper copy or move assignment depending on argument.
	*reinterpret_cast<UnderlyingPacketType *>(objPtr->obj) = std::forward<PacketType>(packet);

	return objPtr;
}

} // namespace dv::io::support
