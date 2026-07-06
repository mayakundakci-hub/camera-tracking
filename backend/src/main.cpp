// =============================================================
// backend  (C++)
//
// Merges what used to be three separate node executables into one
// process, since they're always run together in practice:
//   optitrack     (Motive/NatNet -> "pose_opti")
//   fanuc [stub]  (synthetic poses -> "pose_fanuc")
//   transform_sync (pose_opti + pose_fanuc -> "hand/validation")
//
// optitrack and transform_sync are purely event-driven (NatNet's own
// callback thread / eCAL's own subscriber callback thread), so they
// only need setup code here, not a dedicated loop. fanuc's stub is a
// synthetic generator that has to actively tick, so it gets its own
// thread.
// =============================================================

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include "camera_tracking.pb.h"
#include "../../nodes/common/config.hpp"
#include "../../nodes/common/transform_math.hpp"

#include <NatNetTypes.h>
#include <NatNetClient.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <mutex>
#include <thread>

using camera_tracking::PosePacket;
using camera_tracking::ValidationPacket;

// =============================================================
// optitrack: Motive -> pose_opti
// =============================================================
namespace optitrack {

static int g_handRigidBodyId = 1;
static eCAL::protobuf::CPublisher<PosePacket>* g_pub = nullptr;

// NatNet frame callback — HOT PATH: keep allocation-free, only repackage + publish
void NATNET_CALLCONV OnFrameReceived(sFrameOfMocapData* data, void* /*userData*/)
{
    for (int i = 0; i < data->nRigidBodies; ++i)
    {
        const sRigidBodyData& rb = data->RigidBodies[i];
        if (rb.ID != g_handRigidBodyId) continue;

        PosePacket msg;
        msg.set_timestamp(data->fTimestamp);  // Motive's capture timestamp, NOT arrival time
        msg.set_pos_x(rb.x);
        msg.set_pos_y(rb.y);
        msg.set_pos_z(rb.z);
        msg.set_quat_w(rb.qw);
        msg.set_quat_x(rb.qx);
        msg.set_quat_y(rb.qy);
        msg.set_quat_z(rb.qz);
        msg.set_valid((rb.params & 0x01) != 0);  // bit 0x01 == tracking valid this frame
        msg.set_frame_id("optitrack_world");
        g_pub->Send(msg);
    }
}

// Connects to Motive and registers the frame callback. Fatal on failure --
// there is no point running fanuc/transform_sync without camera data.
bool start(const std::string& motiveIp, int handRigidBodyId, NatNetClient& client,
           eCAL::protobuf::CPublisher<PosePacket>& pub)
{
    g_handRigidBodyId = handRigidBodyId;
    g_pub = &pub;

    sNatNetClientConnectParams params;
    params.connectionType = ConnectionType_Multicast;
    params.serverAddress  = motiveIp.c_str();
    client.SetFrameReceivedCallback(OnFrameReceived, nullptr);

    if (client.Connect(params) != ErrorCode_OK)
    {
        std::fprintf(stderr, "[backend/optitrack] FAILED to connect to Motive at %s\n", motiveIp.c_str());
        return false;
    }
    std::printf("[backend/optitrack] Connected. Streaming rigid body %d\n", g_handRigidBodyId);
    return true;
}

} // namespace optitrack

// =============================================================
// fanuc: simulated stub -> pose_fanuc
// =============================================================
namespace fanuc {

static double nowSeconds()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// Simulated circular motion in the fanuc_base frame (meters)
static void generateStubPose(double t, PosePacket& msg)
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

// Runs on its own thread — unlike optitrack/transform_sync this is a
// synthetic generator, not something driven by an external callback.
void runStubLoop(int publishRateHz, eCAL::protobuf::CPublisher<PosePacket>& pub)
{
    PosePacket msg;
    while (eCAL::Ok())
    {
        generateStubPose(nowSeconds(), msg);
        pub.Send(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / publishRateHz));
    }
}

} // namespace fanuc

// =============================================================
// transform_sync: pose_opti + pose_fanuc -> hand/validation
// =============================================================
namespace transform_sync {

static double g_maxMatchGapSec = 0.020;
static size_t g_bufferLen      = 256;
static std::mutex             g_mtx;
static std::deque<PosePacket> g_optiBuf;

static void onOptiPose(const PosePacket& msg)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_optiBuf.push_back(msg);
    if (g_optiBuf.size() > g_bufferLen) g_optiBuf.pop_front();
}

// Find buffered OptiTrack sample closest in time to the fanuc sample.
// NOTE: assumes both timestamps are on a comparable clock. If Motive
// time and controller time are NOT aligned, add a clock-offset
// estimation step here (e.g. estimate constant offset at startup).
static bool findNearestOpti(double t, PosePacket& out)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    double best = 1e9;
    for (const auto& p : g_optiBuf) {
        double d = std::fabs(p.timestamp() - t);
        if (d < best) { best = d; out = p; }
    }
    return best <= g_maxMatchGapSec;
}

} // namespace transform_sync

int main(int argc, char** argv)
{
    const auto& cfg = Config::load();

    // ---- validate config up front, before starting anything, so a bad
    // config fails loudly and immediately instead of mid-run ----
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

    transform_sync::g_maxMatchGapSec = cfg["transform_sync"]["max_match_gap_sec"];
    transform_sync::g_bufferLen      = cfg["transform_sync"]["buffer_len"];

    const std::string topicOpti  = cfg["ecal"]["topic_pose_opti"];
    const std::string topicFanuc = cfg["ecal"]["topic_pose_fanuc"];
    const std::string topicOut   = cfg["ecal"]["topic_validation"];

    eCAL::Initialize(argc, argv, "backend");

    // ---- optitrack ----
    // optitrack.required=false skips Motive/NatNet entirely (no attempted
    // connection, no pose_opti data) so backend can run with zero hardware
    // for dev/demo purposes -- transform_sync will then always report
    // valid=false, since there's never a camera sample to match against.
    const bool optitrackRequired = cfg["optitrack"].value("required", true);
    NatNetClient natnetClient;
    eCAL::protobuf::CPublisher<PosePacket> optiPub(topicOpti);
    bool optitrackConnected = false;
    if (optitrackRequired)
    {
        const std::string motiveIp = cfg["optitrack"]["motive_server_ip"];
        const int handRigidBodyId  = cfg["optitrack"]["hand_rigid_body_id"];
        if (!optitrack::start(motiveIp, handRigidBodyId, natnetClient, optiPub))
        {
            eCAL::Finalize();
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
    eCAL::protobuf::CPublisher<PosePacket> fanucPub(topicFanuc);
    const int fanucRateHz = cfg["fanuc"]["publish_rate_hz"];
    std::thread fanucThread(fanuc::runStubLoop, fanucRateHz, std::ref(fanucPub));
    std::printf("[backend/fanuc] mode='stub' — publishing simulated poses @ %d Hz\n", fanucRateHz);

    // ---- transform_sync ----
    eCAL::protobuf::CSubscriber<PosePacket> subOpti(topicOpti);
    subOpti.AddReceiveCallback(
        [](const char*, const PosePacket& msg, long long, long long) { transform_sync::onOptiPose(msg); });

    eCAL::protobuf::CPublisher<ValidationPacket> validationPub(topicOut);

    eCAL::protobuf::CSubscriber<PosePacket> subFanuc(topicFanuc);
    subFanuc.AddReceiveCallback(
        [&validationPub, &calib, fanucIsStub](const char*, const PosePacket& fanucMsg, long long, long long)
        {
            PosePacket opti;
            const bool matched = transform_sync::findNearestOpti(fanucMsg.timestamp(), opti);

            ValidationPacket out;
            out.set_timestamp(fanucMsg.timestamp());
            out.set_fanuc_is_stub(fanucIsStub);
            *out.mutable_pose_fanuc() = fanucMsg;

            if (matched && opti.valid() && fanucMsg.valid())
            {
                double x, y, z;
                calib.apply(opti.pos_x(), opti.pos_y(), opti.pos_z(), x, y, z);

                PosePacket* cam = out.mutable_pose_camera();
                *cam = opti;
                cam->set_pos_x(x); cam->set_pos_y(y); cam->set_pos_z(z);
                cam->set_frame_id("fanuc_base");

                const ErrorResult err = computeError(x, y, z,
                                                      fanucMsg.pos_x(), fanucMsg.pos_y(), fanucMsg.pos_z());
                out.set_error_x_mm(err.x_mm);
                out.set_error_y_mm(err.y_mm);
                out.set_error_z_mm(err.z_mm);
                out.set_error_mm(err.total_mm);
                out.set_valid(true);
            }
            else
            {
                out.set_valid(false);  // occlusion, comm drop, or no time match
            }
            validationPub.Send(out);
        });

    std::printf("[backend] Running (optitrack%s + fanuc[stub] + transform_sync).\n",
                optitrackConnected ? "" : "[disabled]");
    while (eCAL::Ok())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    fanucThread.join();
    if (optitrackConnected) natnetClient.Disconnect();
    eCAL::Finalize();
    return 0;
}
