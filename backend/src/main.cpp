#include <arena/transport/ecal/EcalTopic.hpp>
#include <ecal/ecal.h>
#include <middleware/EcalProtoTopic.hpp>
#include "camera_tracking.pb.h"
#include "../../nodes/common/comparison.hpp"
#include "../../nodes/common/config.hpp"
#include "../../nodes/common/frames.hpp"
#include "../../nodes/common/joint_projection.hpp"
#include "../../nodes/common/object_placement.hpp"
#include "../../nodes/common/scene_config.hpp"
#include "../../nodes/common/time_utils.hpp"

#include "optitrack.hpp"
#include "fanuc.hpp"

#include <NatNetClient.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

using camera_tracking::ComparisonPacket;
using camera_tracking::JointEstimatePacket;
using camera_tracking::JointStatePacket;
using camera_tracking::ObjectDelta;
using camera_tracking::PlacedPose;
using camera_tracking::PosePacket;
using camera_tracking::ScenePlacementsPacket;

namespace {

// One decimal place, for status strings a human reads once and acts on.
std::string fixed1(double v)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", v);
    return buf;
}

// The controller's most recent joint state, and when it arrived.
class ReportedJoints {
public:
    void update(const JointStatePacket& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_  = msg;
        arrived_ = nowSeconds();
        seen_    = true;
    }

    struct Lookup {
        bool valid{false};
        double value_deg{0.0};
        double age_s{0.0};
        std::string status;
    };

    [[nodiscard]] Lookup find(const std::string& arm, int index, double maxAgeSec) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Lookup out;

        if (!seen_)
        {
            out.status = "no publisher on robot/joint_state (awaiting the robot bridge)";
            return out;
        }

        out.age_s = nowSeconds() - arrived_;
        if (out.age_s > maxAgeSec)
        {
            out.status = "robot/joint_state stale by " + fixed1(out.age_s) + " s";
            return out;
        }
        if (!latest_.valid())
        {
            out.status = "the controller disowns its own sample";
            return out;
        }
        if (!arm.empty() && latest_.arm() != arm)
        {
            out.status = "bridge reports arm '" + latest_.arm() + "'; this joint is configured '" +
                         arm + "'";
            return out;
        }
        if (index < 0 || index >= latest_.hand_joints_size())
        {
            out.status = "hand_joints has " + std::to_string(latest_.hand_joints_size()) +
                         " entries; this joint is index " + std::to_string(index);
            return out;
        }

        out.valid     = true;
        out.value_deg = latest_.hand_joints(index);
        return out;
    }

private:
    mutable std::mutex mutex_;
    JointStatePacket latest_;
    double arrived_{0.0};
    bool seen_{false};
};
void fillPlaced(const std::string& placementId, const std::string& objectId,
                const std::optional<frames::Transform>& t,
                const std::optional<placement::LatchResult>& latch, PlacedPose& out)
{
    out.set_placement_id(placementId);
    out.set_object_id(objectId);
    out.set_valid(t.has_value());

    if (latch)
    {
        out.set_latched(latch->latched);
        out.set_samples_used(static_cast<std::uint32_t>(latch->window - latch->rejected));
        out.set_samples_rejected(static_cast<std::uint32_t>(latch->rejected));
        out.set_spread_mm(frames::convert::mToMm(latch->spread_m));
        out.set_spread_deg(frames::convert::radToDeg(latch->spread_rad));
        out.set_std_err_mm(frames::convert::mToMm(latch->std_err_m));
        out.set_std_err_deg(frames::convert::radToDeg(latch->std_err_rad));
        out.set_mean_marker_error_mm(frames::convert::mToMm(latch->mean_error_m));
    }

    if (!t) return;

    const frames::Vec3 p = t->translation();
    const frames::Quat q = t->rotation();
    out.set_pos_x(p.x());
    out.set_pos_y(p.y());
    out.set_pos_z(p.z());
    out.set_quat_w(q.w());
    out.set_quat_x(q.x());
    out.set_quat_y(q.y());
    out.set_quat_z(q.z());
    out.set_stamp(t->stamp);
}

}   // namespace

int main(int argc, char** argv)
{
    arena::transport::ecal::set_loopback_enabled(true);

    const auto& cfg = Config::load();

    // ---- scene ----
    const std::string sceneManifest = cfg["scene"]["manifest"];
    scene::Scene sceneCfg;
    try
    {
        sceneCfg = scene::load(sceneManifest);
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "[backend] FATAL: %s\n", e.what());
        return 1;
    }
    std::printf("%s", scene::describe(sceneCfg).c_str());

    // ---- latch policy ----
    placement::LatchPolicy latchPolicy;
    {
        const auto& l                   = cfg["latch"];
        latchPolicy.min_samples         = l.value("min_samples", std::size_t{240});
        latchPolicy.max_mean_error_m    = frames::convert::mmToM(l.value("max_mean_error_mm", 2.0));
        latchPolicy.mad_k               = l.value("mad_k", 3.0);
        latchPolicy.max_reject_fraction = l.value("max_reject_fraction", 0.2);
        latchPolicy.max_spread_m        = frames::convert::mmToM(l.value("max_spread_mm", 2.0));
        latchPolicy.max_spread_rad      = frames::convert::degToRad(l.value("max_spread_deg", 0.5));
        latchPolicy.timeout_s           = l.value("timeout_s", 15.0);
        std::printf("[backend/latch] window %zu samples, marker error <= %.2f mm, "
                    "spread <= %.3f mm / %.3f deg, reject <= %.0f%%\n",
                    latchPolicy.min_samples, latchPolicy.max_mean_error_m * 1000.0,
                    latchPolicy.max_spread_m * 1000.0,
                    frames::convert::radToDeg(latchPolicy.max_spread_rad),
                    latchPolicy.max_reject_fraction * 100.0);
    }

    double radialWarnM = 0.002, axialWarnM = 0.002, residualWarnRad = 0.0;
    double reportedMaxAgeSec = 0.5, minConfidence = 0.05;
    bool captureZero = false;
    if (cfg.contains("joint_projection"))
    {
        const auto& j     = cfg["joint_projection"];
        radialWarnM       = frames::convert::mmToM(j.value("radial_warn_mm", 2.0));
        axialWarnM        = frames::convert::mmToM(j.value("axial_warn_mm", 2.0));
        residualWarnRad   = frames::convert::degToRad(j.value("residual_warn_deg", 1.0));
        reportedMaxAgeSec = j.value("reported_max_age_s", 0.5);
        minConfidence     = j.value("min_confidence", 0.05);

        captureZero = j.value("capture_zero", false);
    }
    if (captureZero)
        std::printf("[joint] capture_zero is ON -- this session will PRINT each joint's zero "
                    "rotation for the pose held during it, and change nothing. Set it back to "
                    "false afterwards, or the next session measures against whatever pose it "
                    "happens to start in.\n");

    double chordWarnM = 0.005, outOfPlaneWarnM = 0.005, normalWarnRad = 0.0;
    double expectNormalWarnRad = 0.0;
    if (cfg.contains("construction"))
    {
        const auto& c       = cfg["construction"];
        chordWarnM          = frames::convert::mmToM(c.value("chord_warn_mm", 5.0));
        outOfPlaneWarnM     = frames::convert::mmToM(c.value("out_of_plane_warn_mm", 5.0));
        normalWarnRad       = frames::convert::degToRad(c.value("normal_warn_deg", 0.5));
        expectNormalWarnRad = frames::convert::degToRad(c.value("expect_normal_warn_deg", 5.0));
    }
    else
    {
        normalWarnRad       = frames::convert::degToRad(0.5);
        expectNormalWarnRad = frames::convert::degToRad(5.0);
    }

    frames::Registry registry;
    placement::Placer placer(sceneCfg, registry, latchPolicy);
    const std::string anchorFrame = placer.scene().anchor_frame;

    // Comparison keeps a timestamped history of continuously-tracked
    // placements, because the registry holds only the latest pose and two
    // "latest" poses captured at different instants are not comparable.
    // Registered BEFORE any data flows, so no update is missed.
    comparison::Comparator comparator(sceneCfg, registry, cfg["comparison"]["max_match_gap_sec"],
                                      cfg["comparison"]["buffer_len"]);
    placer.setObserver([&comparator](const std::string& id, const frames::Transform& t) {
        comparator.onPlacementUpdated(id, t);
    });

    const std::string topicPlacements  = cfg["ecal"]["topic_scene_placements"];
    const std::string topicComparisons = cfg["ecal"]["topic_scene_comparisons"];
    const std::string topicFanuc       = cfg["ecal"]["topic_pose_fanuc"];

    // ---- expected_pose subscriptions, one per placement ----
    std::vector<std::unique_ptr<middleware::EcalProtoSubscriber<PosePacket>>> expectedSubs;
    for (const auto* p : placer.scene().placements())
    {
        if (p->source != scene::Source::ExpectedPose) continue;

        const std::string placementId = p->id;
        expectedSubs.push_back(std::make_unique<middleware::EcalProtoSubscriber<PosePacket>>(
            "backend", p->topic, [&placer, placementId](const PosePacket& msg) {
                if (!msg.valid()) return;   // never place from a pose the source disowns
                try
                {
                    placer.onExpectedPose(placementId,
                                          frames::Vec3(msg.pos_x(), msg.pos_y(), msg.pos_z()),
                                          frames::convert::quatFromWxyz(msg.quat_w(), msg.quat_x(),
                                                                        msg.quat_y(), msg.quat_z()),
                                          msg.timestamp());
                }
                catch (const frames::FrameError& e)
                {
                    std::fprintf(stderr, "[backend/placement] %s: %s\n", placementId.c_str(),
                                 e.what());
                }
            }));
        std::printf("[backend/placement] '%s' <- %s\n", placementId.c_str(), p->topic.c_str());
    }

    // ---- optitrack ----
    const bool optitrackRequired = cfg["optitrack"].value("required", true);
    NatNetClient natnetClient;
    bool optitrackConnected = false;

    if (optitrackRequired)
    {
        const std::string motiveIp = cfg["optitrack"]["motive_server_ip"];
        const std::string localAddr =
            cfg["optitrack"].value("local_address", std::string{"127.0.0.1"});
        if (!optitrack::start(motiveIp, localAddr, natnetClient,
                              [&placer](const std::vector<placement::BodyObservation>& bodies,
                                        double stamp) { placer.onMocapFrame(bodies, stamp); }))
        {
            return 1;   // no point running without camera data
        }
        optitrackConnected = true;
    }
    else
    {
        std::printf("[backend/optitrack] optitrack.required=false - skipping Motive/NatNet. "
                    "Only static and expected_pose placements will be positioned.\n");
    }

    // ---- fanuc (stub) ----
    const bool fanucStubEnabled = cfg["fanuc"].value("stub_enabled", true);
    std::unique_ptr<middleware::EcalProtoPublisher<PosePacket>> fanucPub;
    std::unique_ptr<std::thread> fanucThread;
    if (fanucStubEnabled)
    {
        const int fanucRateHz = cfg["fanuc"]["publish_rate_hz"];
        fanucPub =
            std::make_unique<middleware::EcalProtoPublisher<PosePacket>>("backend", topicFanuc);
        fanucThread =
            std::make_unique<std::thread>(fanuc::runStubLoop, fanucRateHz, std::ref(*fanucPub));
        std::printf("[backend/fanuc] stub enabled - simulated poses on %s @ %d Hz\n",
                    topicFanuc.c_str(), fanucRateHz);
    }

    // ---- the controller's side of the joint comparison ----
    ReportedJoints reported;
    const std::string topicJointState =
        cfg["ecal"].value("topic_joint_state", std::string{"robot/joint_state"});
    middleware::EcalProtoSubscriber<JointStatePacket> jointStateSub(
        "backend", topicJointState,
        [&reported](const JointStatePacket& msg) { reported.update(msg); });

    // ---- scene placements + comparisons + joint estimates ----
    middleware::EcalProtoPublisher<ScenePlacementsPacket> placementsPub("backend", topicPlacements);
    middleware::EcalProtoPublisher<ComparisonPacket> comparisonsPub("backend", topicComparisons);
    const std::string topicJointEstimates =
        cfg["ecal"].value("topic_scene_joint_estimates", std::string{"scene/joint_estimates"});
    middleware::EcalProtoPublisher<JointEstimatePacket> jointsPub("backend", topicJointEstimates);

    const std::string topicMeasuredJoints = cfg["ecal"].value(
        "topic_hand_joint_state_measured", std::string{"hand/joint_state_measured"});
    middleware::EcalProtoPublisher<JointStatePacket> measuredJointsPub("backend",
                                                                       topicMeasuredJoints);
    const int placementsRateHz = cfg["scene"].value("publish_rate_hz", 10);

    for (const auto& c : placer.scene().comparisons())
        std::printf("[backend/comparison] %s: %s vs %s\n", c.object_id.c_str(), c.a.c_str(),
                    c.b.c_str());

    for (const auto* p : placer.scene().placements())
    {
        if (p->source != scene::Source::Projected) continue;
        std::printf("[backend/joint] %s: %s projected onto %s through the joint at %s\n",
                    p->object_id.c_str(), p->measured.c_str(), p->parent_frame.c_str(),
                    p->id.c_str());
    }

    for (const auto* p : placer.scene().placements())
    {
        if (p->source != scene::Source::Constructed || p->inputs.size() != 2) continue;
        std::printf("[backend/construct] %s: %s built from the chord %s -> %s\n",
                    p->object_id.c_str(), p->id.c_str(), p->inputs[0].c_str(),
                    p->inputs[1].c_str());
    }

    std::printf("[backend] Running (optitrack%s + placement + comparison).\n",
                optitrackConnected ? "" : "[disabled]");

    bool announcedComplete = false;
    auto lastStatus        = std::chrono::steady_clock::now();
    std::set<std::string> announcedLatches;
    std::set<std::string> announcedJoints;
    std::set<std::string> announcedConstructions;
    std::set<std::string> announcedMountGeometry;
    struct YawAccum {
        double sin{0.0};
        double cos{0.0};
        int n{0};
    };
    std::map<std::string, YawAccum> yawAccum;
    std::set<std::string> announcedYaw;
    constexpr int kYawSamples = 100;   // ~10 s at the 10 Hz tick

    struct ZeroAccum {
        double sin{0.0};
        double cos{0.0};
        frames::Vec3 pos{frames::Vec3::Zero()};
        int n{0};
    };
    std::map<std::string, ZeroAccum> zeroAccum;
    std::set<std::string> announcedZero;

    while (eCAL::Ok())
    {
        ScenePlacementsPacket packet;
        packet.set_timestamp(nowSeconds());
        packet.set_anchor_frame(anchorFrame);
        packet.set_complete(placer.complete());

        for (const auto* p : placer.scene().placements())
            fillPlaced(p->id, p->object_id, registry.lookup(anchorFrame, p->id),
                       placer.latchReport(p->id), *packet.add_placements());

        fillPlaced(anchorFrame, "", registry.lookup(placement::kMocapFrame, anchorFrame),
                   placer.latchReport(anchorFrame), *packet.mutable_anchor_in_mocap());

        placementsPub.send(packet);

        ComparisonPacket deltas;
        deltas.set_timestamp(packet.timestamp());
        for (const comparison::Delta& d : comparator.compute())
        {
            ObjectDelta* out = deltas.add_deltas();
            out->set_object_id(d.object_id);
            out->set_a(d.a);
            out->set_b(d.b);
            out->set_a_source(d.a_source);
            out->set_b_source(d.b_source);
            out->set_dx_mm(d.dx_mm);
            out->set_dy_mm(d.dy_mm);
            out->set_dz_mm(d.dz_mm);
            out->set_distance_mm(d.distance_mm);
            out->set_angle_deg(d.angle_deg);
            out->set_stamp(d.stamp);
            out->set_time_gap_s(d.time_gap_s);
            out->set_valid(d.valid);
            out->set_invalid_reason(d.invalid_reason);
            out->set_review_gated(d.review_gated);
        }
        comparisonsPub.send(deltas);

        // ---- joint estimates ----
        JointEstimatePacket joints;
        joints.set_timestamp(packet.timestamp());
        bool allEstimated = true;

        for (const auto& [id, e] : placer.jointEstimates())
        {
            camera_tracking::JointEstimate* out = joints.add_joints();
            out->set_joint_id(id);
            out->set_object_id(e.object_id);

            out->set_estimated(e.estimated);
            out->set_skip_reason(e.skip_reason);
            out->set_theta_deg(frames::convert::radToDeg(e.theta_rad));
            out->set_age_s(e.stamp > 0.0 ? packet.timestamp() - e.stamp : 0.0);

            out->set_theta_raw_deg(frames::convert::radToDeg(e.theta_raw_rad));
            out->set_lower_deg(frames::convert::radToDeg(e.lower_rad));
            out->set_upper_deg(frames::convert::radToDeg(e.upper_rad));
            out->set_clamped(e.clamped);
            out->set_unconstrained(e.unconstrained);

            out->set_radial_error_mm(frames::convert::mToMm(e.radial_error_m));
            out->set_axial_error_mm(frames::convert::mToMm(e.axial_error_m));
            out->set_residual_mm(frames::convert::mToMm(e.residual_m));
            out->set_residual_deg(frames::convert::radToDeg(e.residual_rad));
            out->set_confidence(e.confidence);
            out->set_stamp(e.stamp);

            const ReportedJoints::Lookup r =
                reported.find(e.reported_arm, e.reported_index, reportedMaxAgeSec);
            out->set_reported_valid(r.valid);
            out->set_reported_status(r.status);
            out->set_reported_deg(r.value_deg);
            out->set_reported_age_s(r.age_s);
            if (r.valid && e.estimated)
            {
                const double reportedRad = frames::convert::degToRad(r.value_deg);
                out->set_error_deg(frames::convert::radToDeg(
                    jointproj::wrapToNear(e.theta_rad, reportedRad) - reportedRad));
            }

            if (!e.estimated) allEstimated = false;
        }
        joints.set_complete(allEstimated);
        jointsPub.send(joints);

        // ---- the same angles, in the shape a URDF poser accepts ----
        {
            JointStatePacket measured;
            measured.set_timestamp(packet.timestamp());
            measured.set_valid(allEstimated);

            int highest = -1;
            for (const auto& [id, e] : placer.jointEstimates())
                highest = std::max(highest, e.reported_index);
            for (int i = 0; i <= highest; ++i)
                measured.add_hand_joints(0.0);

            for (const auto& [id, e] : placer.jointEstimates())
            {
                if (measured.arm().empty()) measured.set_arm(e.reported_arm);
                if (e.reported_index >= 0 && e.reported_index <= highest)
                    measured.set_hand_joints(e.reported_index,
                                             frames::convert::radToDeg(e.theta_rad));
            }
            measuredJointsPub.send(measured);
        }
        for (const auto& [id, e] : placer.jointEstimates())
        {
            if (!e.estimated || !announcedJoints.insert(id).second) continue;

            const bool radialBad = std::fabs(e.radial_error_m) > radialWarnM;
            const bool axialBad  = std::fabs(e.axial_error_m) > axialWarnM;
            std::printf("[joint] %s: theta %.3f deg, radial %+.3f mm, axial %+.3f mm, "
                        "residual %.3f mm / %.4f deg, confidence %.3f%s\n",
                        id.c_str(), frames::convert::radToDeg(e.theta_rad),
                        e.radial_error_m * 1000.0, e.axial_error_m * 1000.0, e.residual_m * 1000.0,
                        frames::convert::radToDeg(e.residual_rad), e.confidence,
                        (radialBad || axialBad) ? "   <-- CHECK GEOMETRY" : "");

            if (radialBad)
                std::printf("[joint] %s: radial error %.3f mm exceeds %.3f mm -- the configured "
                            "axis_point is in the wrong PLACE. This does not vary with the joint "
                            "angle, so it is geometry, not tracking.\n",
                            id.c_str(), e.radial_error_m * 1000.0, radialWarnM * 1000.0);
            if (axialBad)
                std::printf("[joint] %s: axial error %.3f mm exceeds %.3f mm -- the configured "
                            "axis points the wrong WAY.\n",
                            id.c_str(), e.axial_error_m * 1000.0, axialWarnM * 1000.0);
            if (e.residual_rad > residualWarnRad)
                std::printf("[joint] %s: %.4f deg of rotation the joint cannot produce, over the "
                            "%.4f deg limit -- rigid-body identification, not geometry.\n",
                            id.c_str(), frames::convert::radToDeg(e.residual_rad),
                            frames::convert::radToDeg(residualWarnRad));
            if (e.confidence < minConfidence)
                std::printf("[joint] %s: confidence %.3f below %.3f -- the angle was recovered "
                            "from very little signal and should not be believed.\n",
                            id.c_str(), e.confidence, minConfidence);
        }

        // ---- the one rotation a joint's CONFIG still owes ----
        for (const auto& [id, e] : placer.jointEstimates())
        {
            if (!e.estimated || !e.yaw_observable || announcedYaw.count(id)) continue;

            YawAccum& acc = yawAccum[id];
            acc.sin += std::sin(e.yaw_to_align_rad);
            acc.cos += std::cos(e.yaw_to_align_rad);
            if (++acc.n < kYawSamples) continue;
            announcedYaw.insert(id);

            const scene::Placement* p = placer.scene().findPlacement(id);
            if (!p || !p->joint) continue;
            const jointproj::RevoluteJoint& g = p->joint->geometry;

            const double yaw = std::atan2(acc.sin, acc.cos);

            if (std::fabs(yaw) < frames::convert::degToRad(0.05))
            {
                std::printf("[joint] %s: geometry is aligned -- %.4f deg of outstanding turn over "
                            "%d samples, which is nothing to correct.\n",
                            id.c_str(), frames::convert::radToDeg(yaw), kYawSamples);
                continue;
            }

            const frames::Vec3 axis = g.axis.normalized();
            const Eigen::AngleAxisd Y(yaw, axis);
            const frames::Vec3 zero = Y * g.zero_origin_m;
            const frames::Vec3 axpt = Y * g.axis_point_m;

            std::printf("\n[joint] %s: %.3f deg of outstanding turn about this joint's own axis "
                        "(circular mean of %d samples)\n",
                        id.c_str(), frames::convert::radToDeg(yaw), kYawSamples);
            std::printf("[joint] %s: replace the two position_mm arrays in 'joint' --\n"
                        "           \"zero_pose\":  { \"position_mm\": [%.3f, %.3f, %.3f], ... },\n"
                        "           \"axis_point\": { \"position_mm\": [%.3f, %.3f, %.3f] }\n",
                        id.c_str(), frames::convert::mToMm(zero.x()),
                        frames::convert::mToMm(zero.y()), frames::convert::mToMm(zero.z()),
                        frames::convert::mToMm(axpt.x()), frames::convert::mToMm(axpt.y()),
                        frames::convert::mToMm(axpt.z()));

            std::printf("[joint] %s: leave zero_pose's quat_wxyz alone -- this turn corrects the "
                        "POSITION half of the geometry only. theta reads 'axis' and the quaternion "
                        "and neither of the two arrays above, so the angles this joint is already "
                        "reporting do not change.\n",
                        id.c_str());

            const auto along  = [&axis](const frames::Vec3& v) { return v.dot(axis); };
            const auto across = [&axis](const frames::Vec3& v) {
                return (v - v.dot(axis) * axis).norm();
            };
            std::printf("[joint] %s: check the paste -- a turn about the axis preserves both the "
                        "component ALONG it and the distance ACROSS it. zero_pose %.2f / %.2f mm, "
                        "axis_point %.2f / %.2f mm. If either moved, the turn went about the wrong "
                        "axis.\n",
                        id.c_str(), frames::convert::mToMm(along(zero)),
                        frames::convert::mToMm(across(zero)), frames::convert::mToMm(along(axpt)),
                        frames::convert::mToMm(across(axpt)));

            const scene::Placement* parent = placer.scene().findPlacement(p->parent_frame);
            if (parent && parent->source == scene::Source::Projected)
                std::printf("[joint] %s: PARENT FIRST. This hangs off '%s', which is itself a "
                            "projection, so the numbers above were measured against that "
                            "placement's CURRENT geometry. If you are also pasting a correction "
                            "for it in this pass, apply that one, restart, and re-read this joint "
                            "-- correcting the parent moves the frame these numbers live in, and "
                            "this measurement goes stale the moment it does.\n",
                            id.c_str(), p->parent_frame.c_str());

            std::printf("\n");
        }

        // ---- capture_zero: naming the pose that counts as theta = 0 ----
        if (captureZero)
        {
            for (const auto& [id, e] : placer.jointEstimates())
            {
                if (!e.estimated || announcedZero.count(id)) continue;

                ZeroAccum& acc = zeroAccum[id];
                acc.sin += std::sin(e.theta_rad);
                acc.cos += std::cos(e.theta_rad);
                acc.pos += e.measured_origin_m;
                if (++acc.n < kYawSamples) continue;
                announcedZero.insert(id);

                const scene::Placement* p = placer.scene().findPlacement(id);
                if (!p || !p->joint) continue;
                const jointproj::RevoluteJoint& g = p->joint->geometry;

                const double theta      = std::atan2(acc.sin, acc.cos);
                const frames::Vec3 axis = g.axis.normalized();
                const frames::Vec3 pos  = acc.pos / static_cast<double>(acc.n);

                const frames::Quat zero =
                    (frames::Quat(Eigen::AngleAxisd(theta, axis)) * g.zero_rotation.normalized())
                        .normalized();

                std::printf("\n[joint] %s: reads %.3f deg at the pose held right now (mean of %d "
                            "samples).\n",
                            id.c_str(), frames::convert::radToDeg(theta), kYawSamples);

                std::printf("[joint] %s: if THIS pose is the joint's zero, replace BOTH halves --\n"
                            "           \"zero_pose\": {\n"
                            "               \"position_mm\": [%.3f, %.3f, %.3f],\n"
                            "               \"quat_wxyz\":   [%.6f, %.6f, %.6f, %.6f]\n"
                            "           }\n",
                            id.c_str(), frames::convert::mToMm(pos.x()),
                            frames::convert::mToMm(pos.y()), frames::convert::mToMm(pos.z()),
                            zero.w(), zero.x(), zero.y(), zero.z());
                std::printf("[joint] %s: the rotation MOVES every angle this joint reports, by "
                            "%.3f deg -- it is the half theta actually reads. The position is "
                            "MEASURED in the parent's frame, so unlike the CAD number it replaces "
                            "it owes no yaw correction; axis_point still does.\n",
                            id.c_str(), frames::convert::radToDeg(theta));

                if (std::fabs(theta) < frames::convert::degToRad(0.05))
                    std::printf("[joint] %s: the rotation half is already right -- %.4f deg is "
                                "nothing to correct. Paste the position anyway if it moved.\n",
                                id.c_str(), frames::convert::radToDeg(theta));

                const scene::Placement* parent = placer.scene().findPlacement(p->parent_frame);
                if (parent && parent->source == scene::Source::Projected)
                    std::printf("[joint] %s: PARENT FIRST. This hangs off '%s', which is itself a "
                                "projection, so the pose measured above was taken against that "
                                "joint's CURRENT zero. Capture the parent's zero, restart, and "
                                "re-read this one.\n",
                                id.c_str(), p->parent_frame.c_str());

                std::printf("\n");
            }
        }

        // ---- the two things a construction CANNOT get from CAD ----
        for (const auto* p : placer.scene().placements())
        {
            if (p->source != scene::Source::Constructed || !p->construction) continue;
            if (p->inputs.size() != 2 || announcedMountGeometry.count(p->id)) continue;

            const auto ta = registry.lookup(placement::kMocapFrame, p->inputs[0]);
            const auto tb = registry.lookup(placement::kMocapFrame, p->inputs[1]);
            if (!ta || !tb) continue;   // not tracked yet; try again next tick
            announcedMountGeometry.insert(p->id);

            const twomount::Geometry& g = p->construction->geometry;

            frames::Quat qRel = ta->rotation().conjugate() * tb->rotation();
            qRel.normalize();
            const Eigen::AngleAxisd rel(qRel);
            const frames::Vec3 nHat = g.normal_in_part.normalized();

            const double La         = g.mount_a_m.dot(nHat);
            const frames::Vec3 radA = g.mount_a_m - La * nHat;

            const double Lb = g.mount_b_m.dot(nHat);
            const double rb = (g.mount_b_m - Lb * nHat).norm();

            if (radA.norm() < 1e-6)
            {
                std::printf("[mounts] %s: mount_a sits ON the axis, so it has no radial direction "
                            "to turn and the clocking cannot be expressed as an angle from it. "
                            "Give mount_a its real off-axis position first.\n",
                            p->id.c_str());
                continue;
            }
            const frames::Vec3 mountB =
                Lb * nHat + Eigen::AngleAxisd(rel.angle(), nHat) * radA.normalized() * rb;

            std::printf("\n[mounts] %s: '%s' -> '%s' is %.3f deg about [%.4f, %.4f, %.4f]\n",
                        p->id.c_str(), p->inputs[0].c_str(), p->inputs[1].c_str(),
                        frames::convert::radToDeg(rel.angle()), rel.axis().x(), rel.axis().y(),
                        rel.axis().z());
            std::printf("[mounts] %s: paste into 'construction' --\n"
                        "           \"normal_axis\": [%.6f, %.6f, %.6f],\n"
                        "           \"mount_b\": { \"position_mm\": [%.3f, %.3f, %.3f] }\n",
                        p->id.c_str(), rel.axis().x(), rel.axis().y(), rel.axis().z(),
                        frames::convert::mToMm(mountB.x()), frames::convert::mToMm(mountB.y()),
                        frames::convert::mToMm(mountB.z()));

            const double measuredChord  = (tb->translation() - ta->translation()).norm();
            const double predictedChord = (mountB - g.mount_a_m).norm();
            const double gapMm          = (measuredChord - predictedChord) * 1000.0;

            std::printf("[mounts] %s: cross-check -- measured chord %.2f mm against %.2f mm "
                        "predicted from the angle above (%+.2f mm)\n",
                        p->id.c_str(), measuredChord * 1000.0, predictedChord * 1000.0, gapMm);

            if (std::fabs(gapMm) > 5.0)
                std::printf("[mounts] %s: those should agree, and they do not. ONE of these is "
                            "wrong: the radii and axial offsets configured in mount_a and mount_b, "
                            "or the assumption that each mount is rotationally located about its "
                            "own bar. If a clamp can spin freely where it is tightened, its "
                            "orientation records how it was tightened rather than where it sits -- "
                            "and then the chord is the number to trust and the angle above is not. "
                            "Counting teeth between the clamps settles it independently of both.\n",
                            p->id.c_str());

            std::printf("[mounts] %s: if expect_normal_in_parent then reports ~180 deg, the axis "
                        "is the far end of the same line: negate 'normal_axis', negate the "
                        "component of mount_a and mount_b along normal_in_part, and rerun. The "
                        "measurement is the same either way; only which end of it is called "
                        "positive changes.\n\n",
                        p->id.c_str());
        }

        for (const auto& [id, r] : placer.constructionReports())
        {
            if (!r.constructed || !announcedConstructions.insert(id).second) continue;

            const bool chordBad  = std::fabs(r.chord_error_m) > chordWarnM;
            const bool normalBad = r.normal_disagreement_rad > normalWarnRad;
            const bool planeBad  = std::fabs(r.chord_out_of_plane_m) > outOfPlaneWarnM;
            const bool expectBad =
                r.normal_in_parent_checked && r.normal_in_parent_error_rad > expectNormalWarnRad;

            std::printf(
                "[construct] %s: chord %.2f mm measured vs %.2f mm from CAD (%+.2f mm), "
                "normals disagree %.4f deg, chord off plane %+.2f mm%s\n",
                id.c_str(), r.chord_measured_m * 1000.0, r.chord_expected_m * 1000.0,
                r.chord_error_m * 1000.0, frames::convert::radToDeg(r.normal_disagreement_rad),
                r.chord_out_of_plane_m * 1000.0,
                (chordBad || normalBad || planeBad || expectBad) ? "   <-- CHECK GEOMETRY" : "");

            if (chordBad)
                std::printf("[construct] %s: the two bodies are %+.2f mm further apart than CAD "
                            "says, over the %.2f mm limit. One of them is not where mount_a or "
                            "mount_b puts it -- for a rotor, a mount seated in a different groove. "
                            "The part's origin is built from those same two points, so it is "
                            "wrong by a comparable amount.\n",
                            id.c_str(), r.chord_error_m * 1000.0, chordWarnM * 1000.0);
            if (normalBad)
                std::printf("[construct] %s: the two bodies' face normals disagree by %.4f deg, "
                            "over the %.4f deg limit -- one of them is not seated flat.\n",
                            id.c_str(), frames::convert::radToDeg(r.normal_disagreement_rad),
                            frames::convert::radToDeg(normalWarnRad));
            if (planeBad)
                std::printf("[construct] %s: the measured chord sits %+.2f mm off the normal "
                            "relationship CAD describes, over the %.2f mm limit. With the normals "
                            "agreeing, this means 'normal_axis' names the wrong body axis: both "
                            "bodies are wrong the same way, which is the one failure their "
                            "agreement cannot reveal.\n",
                            id.c_str(), r.chord_out_of_plane_m * 1000.0, outOfPlaneWarnM * 1000.0);

            if (r.normal_in_parent_checked)
                std::printf("[construct] %s: part normal lands %.3f deg from where "
                            "expect_normal_in_parent says it must%s\n",
                            id.c_str(), frames::convert::radToDeg(r.normal_in_parent_error_rad),
                            expectBad ? "   <-- CHECK GEOMETRY" : " -- axis and sign confirmed");

            if (expectBad)
                std::printf("[construct] %s: %.2f deg is over the %.2f deg limit, and this check "
                            "does not fail by degrees -- it fails by RIGHT ANGLES. Near 90 means "
                            "'normal_axis' names the wrong body axis (a rail body's 'across' where "
                            "its 'up' was meant); near 180 means the right axis with the wrong "
                            "SIGN. Both leave the three checks above perfectly clean, and on the "
                            "anchor both roll the entire scene. Do not adjust the threshold -- fix "
                            "the axis.\n",
                            id.c_str(), frames::convert::radToDeg(r.normal_in_parent_error_rad),
                            frames::convert::radToDeg(expectNormalWarnRad));

            std::printf("[construct] %s: reach %.1f / %.1f mm from the part's origin -- check both "
                        "against the part.%s\n",
                        id.c_str(), r.reach_a_m * 1000.0, r.reach_b_m * 1000.0,
                        r.normal_in_parent_checked
                            ? ""
                            : " NOT checked by any of the above: the SIGN of normal_axis, and a "
                              "wrong axis sitting at the same angle to the chord as the right one. "
                              "Both roll the part about the chord, and neither is visible here. "
                              "Set 'expect_normal_in_parent' if the room knows which way this "
                              "part's normal points; otherwise only the render or the robot can "
                              "tell you.");
        }

        for (const auto& [id, r] : placer.latchReports())
        {
            if (!r.latched || !announcedLatches.insert(id).second) continue;
            std::printf(
                "[latch] %s: %zu admitted of %zu seen, %zu rejected (%.1f%%); "
                "spread %.3f mm / %.4f deg, std_err %.4f mm / %.5f deg, "
                "marker error %.3f mm -- latched\n",
                id.c_str(), r.admitted, r.seen, r.rejected,
                r.window ? 100.0 * static_cast<double>(r.rejected) / static_cast<double>(r.window)
                         : 0.0,
                r.spread_m * 1000.0, frames::convert::radToDeg(r.spread_rad), r.std_err_m * 1000.0,
                frames::convert::radToDeg(r.std_err_rad), r.mean_error_m * 1000.0);
        }

        const auto now = std::chrono::steady_clock::now();
        if (!placer.complete() && now - lastStatus > std::chrono::seconds(5))
        {
            std::printf("%s\n", placer.status().c_str());
            lastStatus = now;
        }
        else if (placer.complete() && !announcedComplete)
        {
            std::printf("%s\n", placer.status().c_str());
            announcedComplete = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / placementsRateHz));
    }

    if (fanucThread) fanucThread->join();
    if (optitrackConnected) natnetClient.Disconnect();
    return 0;
}
