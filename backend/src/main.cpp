#include <ecal/ecal.h>
#include <middleware/EcalProtoTopic.hpp>
#include "camera_tracking.pb.h"
#include "../../nodes/common/config.hpp"
#include "../../nodes/common/transform_math.hpp"

#include "optitrack.hpp"
#include "fanuc.hpp"
#include "transform_sync.hpp"

#include <NatNetClient.h>

#include <chrono>
#include <cstdio>
#include <thread>

using camera_tracking::PosePacket;
using camera_tracking::ValidationPacket;

int main(int argc, char** argv)
{
    const auto& cfg = Config::load();
    const std::string fanucMode = cfg["fanuc"]["connection_mode"];
    if (fanucMode != "stub" && fanucMode != "live")
    {
        std::fprintf(stderr, "[backend] unknown fanuc.connection_mode '%s' (expected 'stub' or 'live')\n",
                      fanucMode.c_str());
        return 1;
    }
    if (fanucMode == "live")
    {
        std::fprintf(stderr,
            "[backend] fanuc.connection_mode='live' is not implemented yet. "
            "Set it back to 'stub' in config.json to run with simulated poses.\n");
        return 1;
    }
    const bool fanucIsStub = true;  // only "stub" reaches this point

    RigidTransform calib;
    const std::string calibPath = cfg["transform_sync"]["calibration_file"];
    try {
        calib = loadCalibration(calibPath);
        std::printf("[backend] Loaded calibration from %s (registration residual: %.3f mm — measurement floor)\n",
                    calibPath.c_str(), calib.residualMm);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[backend] FATAL: %s\n", e.what());
        return 1;  // refuse to run on identity/no transform -- errors would be meaningless
    }

    const double maxMatchGapSec = cfg["transform_sync"]["max_match_gap_sec"];
    const size_t bufferLen      = cfg["transform_sync"]["buffer_len"];
    transform_sync::Matcher matcher(maxMatchGapSec, bufferLen);

    const std::string topicOpti  = cfg["ecal"]["topic_pose_opti"];
    const std::string topicFanuc = cfg["ecal"]["topic_pose_fanuc"];
    const std::string topicOut   = cfg["ecal"]["topic_validation"];

    // ---- optitrack ----
    const bool optitrackRequired = cfg["optitrack"].value("required", true);
    NatNetClient natnetClient;
    middleware::EcalProtoPublisher<PosePacket> optiPub("backend", topicOpti);
    bool optitrackConnected = false;
    if (optitrackRequired)
    {
        const std::string motiveIp = cfg["optitrack"]["motive_server_ip"];
        const int handRigidBodyId  = cfg["optitrack"]["hand_rigid_body_id"];
        if (!optitrack::start(motiveIp, handRigidBodyId, natnetClient, optiPub))
        {
            return 1;  // no point running fanuc/transform_sync without camera data
        }
        optitrackConnected = true;
    }
    else
    {
        std::printf("[backend/optitrack] optitrack.required=false — skipping Motive/NatNet "
                     "(no pose_opti data; transform_sync will report valid=false)\n");
    }

    // ---- fanuc (stub) ----
    middleware::EcalProtoPublisher<PosePacket> fanucPub("backend", topicFanuc);
    const int fanucRateHz = cfg["fanuc"]["publish_rate_hz"];
    std::thread fanucThread(fanuc::runStubLoop, fanucRateHz, std::ref(fanucPub));
    std::printf("[backend/fanuc] mode='stub' — publishing simulated poses @ %d Hz\n", fanucRateHz);

    // ---- transform_sync ----
    middleware::EcalProtoSubscriber<PosePacket> subOpti("backend", topicOpti,
        [&matcher](const PosePacket& msg) { matcher.onOptiPose(msg); });

    middleware::EcalProtoPublisher<ValidationPacket> validationPub("backend", topicOut);

    middleware::EcalProtoSubscriber<PosePacket> subFanuc("backend", topicFanuc,
        [&matcher, &validationPub, &calib, fanucIsStub](const PosePacket& fanucMsg)
        {
            validationPub.send(matcher.computeValidation(fanucMsg, fanucIsStub, calib));
        });

    std::printf("[backend] Running (optitrack%s + fanuc[stub] + transform_sync).\n",
                optitrackConnected ? "" : "[disabled]");
    while (eCAL::Ok())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    fanucThread.join();
    if (optitrackConnected) natnetClient.Disconnect();
    return 0;
}
