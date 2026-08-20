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
// optitrack node stamps with. Sharing it matters: comparison time-matches two
// continuous placements against each other, and samples stamped from different
// clocks can never pair.

// Simulated circular motion (metres).
constexpr double kPi            = 3.14159265358979323846;  // MSVC's <cmath> has no M_PI
constexpr double kStubRadiusM   = 0.5;   // circle radius, metres
constexpr double kStubHeightM   = 1.0;   // height above the cell base, metres
constexpr double kStubPeriodSec = 8.0;   // one full lap

void generateStubPose(double t, PosePacket& msg)
{
    const double angle = 2.0 * kPi * t / kStubPeriodSec;

    msg.set_timestamp(t);
    msg.set_pos_x(kStubRadiusM * std::cos(angle));
    msg.set_pos_y(kStubRadiusM * std::sin(angle));
    msg.set_pos_z(kStubHeightM);

    // Yaw about Z so the hand turns to face along the circle as it travels.
    msg.set_quat_w(std::cos(angle * 0.5));
    msg.set_quat_x(0.0);
    msg.set_quat_y(0.0);
    msg.set_quat_z(std::sin(angle * 0.5));

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
