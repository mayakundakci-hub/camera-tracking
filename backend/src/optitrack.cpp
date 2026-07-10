#include "optitrack.hpp"

#include <cstdio>

using camera_tracking::PosePacket;

namespace optitrack {
namespace {

int g_handRigidBodyId = 1;
mu::middleware::EcalProtoPublisher<PosePacket>* g_pub = nullptr;

// NatNet frame callback — HOT PATH: keep allocation-free, only repackage + publish
void NATNET_CALLCONV OnFrameReceived(sFrameOfMocapData* data, void* /*userData*/)
{
    for (int i = 0; i < data->nRigidBodies; ++i)
    {
        const sRigidBodyData& rb = data->RigidBodies[i];
        if (rb.ID != g_handRigidBodyId) continue;

        PosePacket msg;
        msg.set_timestamp(data->fTimestamp);  // Motive's capture timestamp
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
           mu::middleware::EcalProtoPublisher<PosePacket>& pub)
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
