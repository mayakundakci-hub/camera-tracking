#include "optitrack.hpp"

#include "../../nodes/common/time_utils.hpp"

#include <cstdio>

namespace optitrack {
namespace {

// NatNet's callback is a C function pointer, so the sink lives here. Set once
// in start(), before Connect(), and never reassigned.
FrameCallback g_onFrame;

// NatNet frame callback -- HOT PATH, runs on NatNet's thread.
void NATNET_CALLCONV OnFrameReceived(sFrameOfMocapData* data, void* /*userData*/)
{
    if (!g_onFrame || data == nullptr) return;
    const double stamp = nowSeconds();

    std::vector<placement::BodyObservation> bodies;
    bodies.reserve(static_cast<std::size_t>(data->nRigidBodies));

    for (int i = 0; i < data->nRigidBodies; ++i)
    {
        const sRigidBodyData& rb = data->RigidBodies[i];

        placement::BodyObservation obs;
        obs.asset_id = rb.ID;
        obs.position = {rb.x, rb.y, rb.z};   // Motive streams metres
        obs.rotation = frames::Quat(rb.qw, rb.qx, rb.qy, rb.qz);
        obs.tracked  = (rb.params & 0x01) != 0;   // bit 0x01 == tracking valid

        obs.mean_error = rb.MeanError;
        bodies.push_back(obs);
    }

    g_onFrame(bodies, stamp);
}

}   // namespace

bool start(const std::string& motiveIp, const std::string& localAddress, NatNetClient& client,
           FrameCallback onFrame)
{
    g_onFrame = std::move(onFrame);

    sNatNetClientConnectParams params;
    params.connectionType = ConnectionType_Unicast;
    params.serverAddress  = motiveIp.c_str();
    params.localAddress   = localAddress.c_str();
    client.SetFrameReceivedCallback(OnFrameReceived, nullptr);

    if (client.Connect(params) != ErrorCode_OK)
    {
        std::fprintf(stderr,
                     "[backend/optitrack] FAILED to connect to Motive (server %s, local %s)\n"
                     "  - is Motive running with a project open?\n"
                     "  - Motive's Data Streaming pane: is 'Enable NatNet' ON?\n"
                     "  - does its 'Local Interface' match optitrack.motive_server_ip above?\n"
                     "    (a Motive on a LAN adapter is unreachable from a loopback client)\n"
                     "  - is its Transmission Type 'Unicast'? this client only speaks unicast\n",
                     motiveIp.c_str(), localAddress.c_str());
        return false;
    }
    std::printf("[backend/optitrack] Connected to Motive at %s; forwarding all rigid bodies "
                "(the scene manifest selects which are used)\n",
                motiveIp.c_str());
    return true;
}

}   // namespace optitrack
