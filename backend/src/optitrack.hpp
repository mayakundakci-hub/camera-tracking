// =============================================================
// optitrack: Motive (NatNet) -> pose_opti
// =============================================================
#pragma once

#include <middleware/EcalProtoTopic.hpp>
#include "camera_tracking.pb.h"

#include <NatNetTypes.h>
#include <NatNetClient.h>

#include <string>

namespace optitrack {

// Connects to Motive and registers the frame callback. Fatal on failure --
// there is no point running fanuc/transform_sync without camera data.
bool start(const std::string& motiveIp, int handRigidBodyId, NatNetClient& client,
           middleware::EcalProtoPublisher<camera_tracking::PosePacket>& pub);

} // namespace optitrack
