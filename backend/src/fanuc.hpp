// =============================================================
// fanuc: simulated stub -> pose_fanuc
// =============================================================
#pragma once

#include <middleware/EcalProtoTopic.hpp>
#include "camera_tracking.pb.h"

namespace fanuc {

// Runs on its own thread — unlike optitrack/transform_sync this is a
// synthetic generator, not something driven by an external callback.
// Loops until eCAL::Ok() returns false, so the caller should join() it
// after the shutdown signal, not stop it directly.
void runStubLoop(int publishRateHz, middleware::EcalProtoPublisher<camera_tracking::PosePacket>& pub);

} // namespace fanuc
