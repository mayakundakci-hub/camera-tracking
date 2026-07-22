#include "fanuc.hpp"

#include "../../nodes/common/time_utils.hpp"

#include <ecal/ecal.h>

#include <chrono>
#include <cmath>
#include <thread>

using camera_tracking::PosePacket;

namespace fanuc {
namespace {

// nowSeconds() comes from nodes/common/time_utils.hpp -- the same clock the
// optitrack node stamps with, which transform_sync's match gate depends on.

// Simulated circular motion in the fanuc_base frame (meters)
void generateStubPose(double t, PosePacket& msg)
{
    msg.set_timestamp(t);
    msg.set_pos_x(0.400 + 0.050 * std::cos(t * 0.5));
    msg.set_pos_y(0.100 + 0.050 * std::sin(t * 0.5));
    msg.set_pos_z(0.200);
    msg.set_quat_w(1.0);
    msg.set_quat_x(0.0);
    msg.set_quat_y(0.0);
    msg.set_quat_z(0.0);
    msg.set_valid(true);
    msg.set_frame_id("fanuc_base");
}

} // namespace

void runStubLoop(int publishRateHz, middleware::EcalProtoPublisher<PosePacket>& pub)
{
    PosePacket msg;
    while (eCAL::Ok())
    {
        generateStubPose(nowSeconds(), msg);
        pub.send(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / publishRateHz));
    }
}

} // namespace fanuc
