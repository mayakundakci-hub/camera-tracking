#pragma once

#include <middleware/EcalProtoTopic.hpp>
#include "camera_tracking.pb.h"

namespace fanuc {
void runStubLoop(int publishRateHz,
                 middleware::EcalProtoPublisher<camera_tracking::PosePacket>& pub);

}   // namespace fanuc
