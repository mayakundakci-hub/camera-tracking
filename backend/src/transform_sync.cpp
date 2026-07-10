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
    return out;
}

} // namespace transform_sync
