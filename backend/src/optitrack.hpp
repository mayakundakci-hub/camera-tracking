// optitrack: Motive (NatNet) -> scene placement
#pragma once

#include "../../nodes/common/object_placement.hpp"

#include <NatNetTypes.h>
#include <NatNetClient.h>

#include <functional>
#include <string>
#include <vector>

namespace optitrack {

using FrameCallback =
    std::function<void(const std::vector<placement::BodyObservation>&, double stamp)>;

bool start(const std::string& motiveIp, const std::string& localAddress, NatNetClient& client,
           FrameCallback onFrame);

} // namespace optitrack
