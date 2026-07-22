#include "optitrack.hpp"

#include "../../nodes/common/time_utils.hpp"

#include <cstdio>

using camera_tracking::PosePacket;

namespace optitrack {
namespace {

int g_handRigidBodyId = 1;
middleware::EcalProtoPublisher<PosePacket>* g_pub = nullptr;

// NatNet frame callback HOT PATH
void NATNET_CALLCONV OnFrameReceived(sFrameOfMocapData* data, void* /*userData*/)
{
    for (int i = 0; i < data->nRigidBodies; ++i)
    {
        const sRigidBodyData& rb = data->RigidBodies[i];
        if (rb.ID != g_handRigidBodyId) continue;

        PosePacket msg;
        // Local arrival time from the SAME steady clock the fanuc node stamps
        // with -- NOT data->fTimestamp. fTimestamp is seconds since Motive
        // started, a different time base entirely, and transform_sync's 20 ms
        // match gate can never pair the two streams across mismatched clocks
        // (the symptom: pose_opti visible in eCAL, nothing on the frontend).
        // Arrival stamping absorbs NatNet transport latency (typically a few
        // ms); if capture-time precision ever matters, convert Motive time to
        // local time via NatNetClient::SecondsSinceHostTimestamp instead.
        msg.set_timestamp(nowSeconds());
        msg.set_pos_x(rb.x);
        msg.set_pos_y(rb.y);
        msg.set_pos_z(rb.z);
        msg.set_quat_w(rb.qw);
        msg.set_quat_x(rb.qx);
        msg.set_quat_y(rb.qy);
        msg.set_quat_z(rb.qz);
        msg.set_valid((rb.params & 0x01) != 0);  // bit 0x01 == tracking valid this frame
        msg.set_frame_id("optitrack_world");
        g_pub->send(msg);
    }
}

} // namespace

bool start(const std::string& motiveIp, int handRigidBodyId, NatNetClient& client,
           middleware::EcalProtoPublisher<PosePacket>& pub)
{
    g_handRigidBodyId = handRigidBodyId;
    g_pub = &pub;

    sNatNetClientConnectParams params;
    params.connectionType = ConnectionType_Unicast;
    params.serverAddress  = motiveIp.c_str();
    params.localAddress   = "127.0.0.1"; 
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
