// =============================================================
// transform_sync: pose_opti + pose_fanuc -> hand/validation
// =============================================================
#pragma once

#include "camera_tracking.pb.h"
#include "../../nodes/common/transform_math.hpp"

#include <cstddef>
#include <deque>
#include <mutex>

namespace transform_sync {

// Buffers OptiTrack samples and time-matches them against incoming Fanuc
// samples. Not copyable (owns a mutex) -- construct one instance and share
// it by reference/pointer between the opti and fanuc eCAL callbacks.
class Matcher {
public:
    Matcher(double maxMatchGapSec, std::size_t bufferLen);

    void onOptiPose(const camera_tracking::PosePacket& msg);

    // Builds the outgoing ValidationPacket for one fanuc sample: matches it
    // against the buffered opti poses, applies the calibration transform,
    // and computes the tracking error. Sets valid=false on occlusion, a
    // comm drop, or no time match within maxMatchGapSec.
    camera_tracking::ValidationPacket computeValidation(const camera_tracking::PosePacket& fanucMsg,
                                                          bool fanucIsStub,
                                                          const RigidTransform& calib);

private:
    // Find buffered OptiTrack sample closest in time to the fanuc sample.
    // NOTE: assumes both timestamps are on a comparable clock. If Motive
    // time and controller time are NOT aligned, add a clock-offset
    // estimation step here (e.g. estimate constant offset at startup).
    bool findNearestOpti(double t, camera_tracking::PosePacket& out);

    double maxMatchGapSec_;
    std::size_t bufferLen_;
    std::mutex mtx_;
    std::deque<camera_tracking::PosePacket> optiBuf_;
};

} // namespace transform_sync
