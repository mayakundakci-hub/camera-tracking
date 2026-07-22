#include "transform_sync.hpp"

#include <cmath>

using camera_tracking::PosePacket;
using camera_tracking::ValidationPacket;

namespace transform_sync {

Matcher::Matcher(double maxMatchGapSec, std::size_t bufferLen)
    : maxMatchGapSec_(maxMatchGapSec), bufferLen_(bufferLen)
{
}

void Matcher::onOptiPose(const PosePacket& msg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    optiBuf_.push_back(msg);
    if (optiBuf_.size() > bufferLen_) optiBuf_.pop_front();
}

bool Matcher::findNearestOpti(double t, PosePacket& out)
{
    std::lock_guard<std::mutex> lk(mtx_);
    double best = 1e9;
    for (const auto& p : optiBuf_) {
        double d = std::fabs(p.timestamp() - t);
        if (d < best) { best = d; out = p; }
    }
    return best <= maxMatchGapSec_;
}

ValidationPacket Matcher::computeValidation(const PosePacket& fanucMsg, bool fanucIsStub,
                                             const RigidTransform& calib)
{
    PosePacket opti;
    const bool matched = findNearestOpti(fanucMsg.timestamp(), opti);

    ValidationPacket out;
    out.set_timestamp(fanucMsg.timestamp());
    out.set_fanuc_is_stub(fanucIsStub);

    // Express the FANUC-reported point in the OptiTrack (optitrack_world) frame,
    // so both systems are compared in the camera's frame. This needs only the
    // FANUC sample, so it runs whether or not a time-matched OptiTrack sample
    // exists (the match only gates the error comparison below).
    double fx, fy, fz;
    calib.apply(fanucMsg.pos_x(), fanucMsg.pos_y(), fanucMsg.pos_z(), fx, fy, fz);
    PosePacket* fan = out.mutable_pose_fanuc();
    *fan = fanucMsg;
    fan->set_pos_x(fx);
    fan->set_pos_y(fy);
    fan->set_pos_z(fz);
    fan->set_frame_id("optitrack_world");

    if (matched && opti.valid() && fanucMsg.valid())
    {
        PosePacket* cam = out.mutable_pose_camera();
        *cam = opti;   // raw OptiTrack point, already in optitrack_world
        cam->set_frame_id("optitrack_world");

        const ErrorResult err = computeError(opti.pos_x(), opti.pos_y(), opti.pos_z(),
                                              fx, fy, fz);
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
    return out;
}

} // namespace transform_sync
