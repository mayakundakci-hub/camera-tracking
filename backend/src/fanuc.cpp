#include "fanuc.hpp"

#include <ecal/ecal.h>

#include <chrono>
#include <cmath>
#include <thread>

using camera_tracking::PosePacket;

namespace fanuc {
namespace {

double nowSeconds()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

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

void runStubLoop(int publishRateHz, mu::middleware::EcalProtoPublisher<PosePacket>& pub)
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
