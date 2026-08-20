#pragma once
// object_placement: turns measurements into frames

#include "frames.hpp"
#include "joint_projection.hpp"
#include "scene_config.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace placement {

inline const frames::FrameId kMocapFrame = "optitrack_world";

// One rigid body as of one mocap frame, already converted to SI.
struct BodyObservation {
    int          asset_id{0};
    frames::Vec3 position{frames::Vec3::Zero()};        // metres, in kMocapFrame
    frames::Quat rotation{frames::Quat::Identity()};
    bool         tracked{false};   // Motive's per-frame tracking-valid flag

    double       mean_error{0.0};
};

struct ConstructionReport {
    std::string placement_id;
    std::string object_id;

    bool        constructed{false};
    std::string skip_reason;

    double chord_measured_m{0.0};
    double chord_expected_m{0.0};
    double chord_error_m{0.0};

    double normal_disagreement_rad{0.0};

    double chord_out_of_plane_m{0.0};
    double normal_in_parent_error_rad{0.0};
    bool   normal_in_parent_checked{false};
    double reach_a_m{0.0};
    double reach_b_m{0.0};

    double stamp{0.0};
};

struct JointEstimate {
    std::string placement_id;
    std::string object_id;
    std::string reported_arm;
    int         reported_index{-1};
    bool        estimated{false};
    std::string skip_reason;

    double theta_rad{0.0};
    double theta_raw_rad{0.0};
    bool   clamped{false};
    bool   unconstrained{false};
    double lower_rad{0.0};
    double upper_rad{0.0};
    double residual_m{0.0};
    double residual_rad{0.0};
    frames::Vec3 measured_origin_m{frames::Vec3::Zero()};
    double radial_error_m{0.0};
    double axial_error_m{0.0};

    double confidence{0.0};
    double yaw_to_align_rad{0.0};
    bool   yaw_observable{false};

    double stamp{0.0};
};

using PlacementObserver =
    std::function<void(const std::string& placementId, const frames::Transform& T_anchor_placement)>;

struct LatchPolicy {
    std::size_t min_samples{240};          // ~2 s at 120 Hz
    double      max_mean_error_m{0.002};   // Motive's mean marker error ceiling
    double      mad_k{3.0};                // outlier cut, in robust sigmas
    double      max_reject_fraction{0.2};
    double      max_spread_m{0.002};       // per-sample dispersion, not std_err
    double      max_spread_rad{0.0087};    // 0.5 deg
    double      timeout_s{15.0};           // REPORTING only; never gives up

    [[nodiscard]] static LatchPolicy immediate()
    {
        LatchPolicy p;
        p.min_samples         = 1;
        p.max_mean_error_m    = std::numeric_limits<double>::infinity();
        p.max_reject_fraction = 1.0;
        p.max_spread_m        = std::numeric_limits<double>::infinity();
        p.max_spread_rad      = std::numeric_limits<double>::infinity();
        return p;
    }
};


struct LatchResult {
    bool        latched{false};
    bool        evaluated{false};   // the gate has run, so the spreads below mean something
    std::size_t seen{0};            // frames the body appeared in at all
    std::size_t admitted{0};        // cumulative, passed the admission test
    std::size_t window{0};          // samples currently held
    std::size_t rejected{0};        // outliers in the most recent evaluation
    double      spread_m{0.0};      // survivor dispersion (stationarity)
    double      spread_rad{0.0};
    double      std_err_m{0.0};     // spread / sqrt(N) -- the estimate's error bar
    double      std_err_rad{0.0};
    double      mean_error_m{0.0};  // mean of the window's marker errors
    double      first_seen{0.0};    // stamp of the first sighting, for timeout
    double      last_seen{0.0};
    std::string reason;             // why not latched; empty once latched
};

namespace detail {

template <typename Get>
[[nodiscard]] frames::Transform meanPose(const frames::FrameId& to, const frames::FrameId& from,
                                         std::size_t n, Get&& get)
{
    frames::Vec3    position = frames::Vec3::Zero();
    Eigen::Vector4d q        = Eigen::Vector4d::Zero();
    double          stamp    = 0.0;

    for (std::size_t i = 0; i < n; ++i)
    {
        const frames::Transform& t = get(i);
        position += t.translation();

        const frames::Quat r = t.rotation();
        Eigen::Vector4d v(r.w(), r.x(), r.y(), r.z());
        if (q.dot(v) < 0.0) v = -v;      // align before summing
        q += v;

        stamp = frames::combineStamps(stamp, t.stamp);
    }

    position /= static_cast<double>(n);
    q.normalize();

    return frames::make(to, from, position, frames::Quat(q(0), q(1), q(2), q(3)), stamp);
}

[[nodiscard]] inline double medianInPlace(std::vector<double>& v)
{
    if (v.empty()) return 0.0;
    const std::size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
    return v[mid];
}

[[nodiscard]] inline double robustSigma(const std::vector<double>& xs, double centre)
{
    std::vector<double> dev;
    dev.reserve(xs.size());
    for (const double x : xs) dev.push_back(std::fabs(x - centre));
    return std::fmax(1.4826 * medianInPlace(dev), 1e-9);
}

[[nodiscard]] inline std::string fixed(double v, int places = 3)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(places) << v;
    return os.str();
}

}  // namespace detail

class LatchAccumulator {
public:
    explicit LatchAccumulator(LatchPolicy policy) : policy_(policy)
    {
        result_.reason = "no sample admitted yet";
    }

    [[nodiscard]] const LatchResult& report() const { return result_; }
    [[nodiscard]] const LatchPolicy& policy() const { return policy_; }

    std::optional<frames::Transform> add(const frames::Transform& sample,
                                         std::optional<double> meanError, double stamp)
    {
        ++result_.seen;
        if (result_.first_seen == 0.0) result_.first_seen = stamp;
        result_.last_seen = stamp;

        if (meanError && *meanError > policy_.max_mean_error_m)
        {
            result_.reason = "marker error " + detail::fixed(*meanError * 1000.0, 2) +
                             " mm over limit " +
                             detail::fixed(policy_.max_mean_error_m * 1000.0, 2) + " mm";
            return std::nullopt;
        }

        ++result_.admitted;
        window_.push_back(sample);
        errors_.push_back(meanError.value_or(0.0));
        while (window_.size() > policy_.min_samples)
        {
            window_.pop_front();
            errors_.pop_front();
        }
        result_.window = window_.size();

        if (window_.size() < policy_.min_samples)
        {
            result_.reason = "collecting " + std::to_string(window_.size()) + "/" +
                             std::to_string(policy_.min_samples) + " samples";
            return std::nullopt;
        }
        return evaluate();
    }

private:

    std::optional<frames::Transform> evaluate()
    {
        const std::size_t n = window_.size();
        result_.evaluated = true;

        frames::Vec3 median;
        std::vector<double> axis(n);
        for (int a = 0; a < 3; ++a)
        {
            for (std::size_t i = 0; i < n; ++i) axis[i] = window_[i].translation()[a];
            median[a] = detail::medianInPlace(axis);
        }
        const frames::Transform seed =
            detail::meanPose(window_.front().to, window_.front().from, n,
                             [this](std::size_t i) -> const frames::Transform& { return window_[i]; });
        const frames::Transform seedInv = frames::inverse(seed);

        std::vector<double> angle(n);
        for (std::size_t i = 0; i < n; ++i)
            angle[i] = frames::magnitudeOf(frames::compose(seedInv, window_[i])).angle_rad;

        // --- reject ---
        frames::Vec3 sigma;
        for (int a = 0; a < 3; ++a)
        {
            for (std::size_t i = 0; i < n; ++i) axis[i] = window_[i].translation()[a];
            sigma[a] = detail::robustSigma(axis, median[a]);
        }
        std::vector<double> angleCopy = angle;
        const double angleMedian = detail::medianInPlace(angleCopy);
        const double angleSigma  = detail::robustSigma(angle, angleMedian);

        std::vector<std::size_t> keep;
        keep.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            const frames::Vec3 d = window_[i].translation() - median;
            bool ok = std::fabs(angle[i] - angleMedian) <= policy_.mad_k * angleSigma;
            for (int a = 0; a < 3 && ok; ++a)
                ok = std::fabs(d[a]) <= policy_.mad_k * sigma[a];
            if (ok) keep.push_back(i);
        }

        result_.rejected = n - keep.size();
        if (keep.empty())
        {
            result_.reason = "every sample rejected as an outlier";
            return std::nullopt;
        }

        // --- statistics of the survivors ---
        const frames::Transform mean =
            detail::meanPose(window_.front().to, window_.front().from, keep.size(),
                             [this, &keep](std::size_t i) -> const frames::Transform& {
                                 return window_[keep[i]];
                             });
        const frames::Transform meanInv = frames::inverse(mean);

        double sumSqPos = 0.0, sumSqAng = 0.0, sumErr = 0.0;
        for (const std::size_t i : keep)
        {
            const frames::Magnitude m = frames::magnitudeOf(frames::compose(meanInv, window_[i]));
            sumSqPos += m.distance_m * m.distance_m;
            sumSqAng += m.angle_rad * m.angle_rad;
            sumErr   += errors_[i];
        }
        const double kept = static_cast<double>(keep.size());
        result_.spread_m     = std::sqrt(sumSqPos / kept);
        result_.spread_rad   = std::sqrt(sumSqAng / kept);
        result_.std_err_m    = result_.spread_m / std::sqrt(kept);
        result_.std_err_rad  = result_.spread_rad / std::sqrt(kept);
        result_.mean_error_m = sumErr / kept;

        // --- gate ---
        const double rejectFraction = static_cast<double>(result_.rejected) / static_cast<double>(n);
        if (rejectFraction > policy_.max_reject_fraction)
        {
            result_.reason = "rejected " + detail::fixed(rejectFraction * 100.0, 1) +
                             "% of samples as outliers, limit " +
                             detail::fixed(policy_.max_reject_fraction * 100.0, 1) + "%";
            return std::nullopt;
        }
        if (result_.spread_m > policy_.max_spread_m)
        {
            result_.reason = "position spread " + detail::fixed(result_.spread_m * 1000.0) +
                             " mm over limit " + detail::fixed(policy_.max_spread_m * 1000.0) +
                             " mm -- is it holding still?";
            return std::nullopt;
        }
        if (result_.spread_rad > policy_.max_spread_rad)
        {
            result_.reason = "angular spread " +
                             detail::fixed(frames::convert::radToDeg(result_.spread_rad)) +
                             " deg over limit " +
                             detail::fixed(frames::convert::radToDeg(policy_.max_spread_rad)) +
                             " deg -- is it holding still?";
            return std::nullopt;
        }

        result_.latched = true;
        result_.reason.clear();
        return mean;
    }

    LatchPolicy                    policy_;
    LatchResult                    result_;
    std::deque<frames::Transform>  window_;
    std::deque<double>             errors_;
};

class Placer {
public:

    Placer(scene::Scene scene, frames::Registry& registry, LatchPolicy defaultPolicy)
        : scene_(std::move(scene)), registry_(registry), defaultPolicy_(defaultPolicy)
    {
        const scene::Placement* anchor = scene_.findPlacement(scene_.anchor_frame);
        if (!anchor)
            throw scene::SceneConfigError("placement: anchor '" + scene_.anchor_frame +
                                          "' is not a placement");

        for (const auto* p : scene_.placements())
        {
            if (p->capture == scene::Capture::Latched &&
                (p->source == scene::Source::Optitrack ||
                 p->source == scene::Source::ExpectedPose))
                latches_.emplace(p->id, LatchAccumulator(policyFor(*p)));

            switch (p->source)
            {
                case scene::Source::Static:
                    registry_.set(p->pose.toTransform(parentOf(*p), p->id));
                    markPlaced(*p);
                    break;

                case scene::Source::JointState:
                    registry_.set(frames::make(parentOf(*p), p->id,
                                               frames::Vec3::Zero(),
                                               frames::Quat::Identity()));
                    markPlaced(*p);
                    break;

                case scene::Source::Optitrack:
                case scene::Source::ExpectedPose:
                case scene::Source::Fused:
                case scene::Source::Projected:
                case scene::Source::Constructed:
                    break;   // awaits a measurement, or awaits its inputs
            }
        }
        buildProjectionOrder();
        seedConstructionReports();
        updateConstructed();
        updateFused();
    }

    void onMocapFrame(const std::vector<BodyObservation>& bodies, double stamp)
    {
        std::map<std::string, frames::Transform> thisFrame;
        std::map<std::string, std::string>       missing;

        for (const auto* p : scene_.placements())
        {
            if (p->source != scene::Source::Optitrack) continue;
            if (isLatchedAndPlaced(*p)) continue;

            const BodyObservation* obs = nullptr;
            for (const auto& b : bodies)
                if (b.asset_id == p->asset_id) { obs = &b; break; }

            if (!obs)
            {
                missing[p->id] = "asset " + std::to_string(p->asset_id) +
                                 " was not streamed this frame";
                continue;
            }
            if (!obs->tracked)
            {
                missing[p->id] = "asset " + std::to_string(p->asset_id) +
                                 " was streamed but is not being tracked";
                continue;
            }

            if (p->capture == scene::Capture::Continuous &&
                obs->mean_error > continuousQualityLimit_)
            {
                missing[p->id] = "asset " + std::to_string(p->asset_id) + " fitted at " +
                                 detail::fixed(obs->mean_error * 1000.0, 2) +
                                 " mm/marker, over the " +
                                 detail::fixed(continuousQualityLimit_ * 1000.0, 2) +
                                 " mm limit -- Motive solved it, but not well enough to believe";
                continue;
            }

            const frames::Transform T_mocap_model =
                frames::make(kMocapFrame, p->id, obs->position, obs->rotation, stamp);

            thisFrame.emplace(p->id, T_mocap_model);

            (void)commit(*p, T_mocap_model, obs->mean_error, stamp);
        }

        projectJoints(thisFrame, missing, stamp);
        updateConstructed();
        updateFused();
    }

    void onExpectedPose(const std::string& placementId, const frames::Vec3& position,
                        const frames::Quat& rotation, double stamp)
    {
        const scene::Placement* p = scene_.findPlacement(placementId);
        if (!p || p->source != scene::Source::ExpectedPose) return;
        if (isLatchedAndPlaced(*p)) return;

        if (!commit(*p, frames::make(parentOf(*p), p->id, position, rotation, stamp),
                    std::nullopt, stamp))
            return;
        updateFused();
    }

    void setObserver(PlacementObserver observer) { observer_ = std::move(observer); }

    [[nodiscard]] bool anchorPlaced() const { return isPlaced(scene_.anchor_frame); }

    [[nodiscard]] bool complete() const { return pending().empty(); }

    [[nodiscard]] std::vector<std::string> pending() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::string> out;
        for (const auto* p : scene_.placements())
            if (!placed_.count(p->id)) out.push_back(p->id);
        return out;
    }

    [[nodiscard]] std::map<std::string, LatchResult> latchReports() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::map<std::string, LatchResult> out;
        for (const auto& [id, acc] : latches_) out.emplace(id, acc.report());
        return out;
    }

    // One placement's latch evidence, or nullopt if it does not latch.
    [[nodiscard]] std::optional<LatchResult> latchReport(const std::string& placementId) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = latches_.find(placementId);
        if (it == latches_.end()) return std::nullopt;
        return it->second.report();
    }

    [[nodiscard]] std::string status() const
    {
        const auto waiting = pending();
        std::ostringstream os;
        os << "[placement] anchor '" << scene_.anchor_frame << "' "
           << (anchorPlaced() ? "placed" : "NOT PLACED");
        if (waiting.empty())
        {
            os << "; all placements positioned";
        }
        else
        {
            os << "; waiting on";
            for (const auto& w : waiting) os << " " << w;
        }

        for (const auto& [id, r] : latchReports())
        {
            if (r.latched) continue;
            os << "\n[latch] " << id << ": " << r.seen << " seen, " << r.admitted
               << " admitted";
            if (r.evaluated)
                os << ", " << r.rejected << " rejected, spread "
                   << detail::fixed(r.spread_m * 1000.0) << " mm / "
                   << detail::fixed(frames::convert::radToDeg(r.spread_rad)) << " deg";
            os << " -- " << r.reason;
        }
        return os.str();
    }

    [[nodiscard]] std::map<std::string, JointEstimate> jointEstimates() const
    {
        std::lock_guard<std::mutex> lock(jointsMutex_);
        return estimates_;
    }

    [[nodiscard]] std::optional<JointEstimate> jointEstimate(const std::string& placementId) const
    {
        std::lock_guard<std::mutex> lock(jointsMutex_);
        const auto it = estimates_.find(placementId);
        if (it == estimates_.end()) return std::nullopt;
        return it->second;
    }

    [[nodiscard]] std::map<std::string, ConstructionReport> constructionReports() const
    {
        std::lock_guard<std::mutex> lock(jointsMutex_);
        return constructions_;
    }

    // One constructed placement's checks, or nullopt if it does not construct.
    [[nodiscard]] std::optional<ConstructionReport>
    constructionReport(const std::string& placementId) const
    {
        std::lock_guard<std::mutex> lock(jointsMutex_);
        const auto it = constructions_.find(placementId);
        if (it == constructions_.end()) return std::nullopt;
        return it->second;
    }

    void setContinuousQualityLimit(double metres)
    {
        continuousQualityLimit_ = metres > 0.0 ? metres
                                               : std::numeric_limits<double>::infinity();
    }

    [[nodiscard]] const scene::Scene& scene() const { return scene_; }

private:
    // One projection, resolved once so the per-frame path does no lookups.
    struct ProjectionStep {
        const scene::Placement*  placement;
        const scene::Placement*  measured;
        frames::FrameId          parentId;
        jointproj::RevoluteJoint joint;
    };

    // Evaluation order, decided once at construction.
    void buildProjectionOrder()
    {
        std::vector<const scene::Placement*> waiting;
        for (const auto* p : scene_.placements())
            if (p->source == scene::Source::Projected) waiting.push_back(p);

        std::set<std::string> ordered;
        bool progress = true;
        while (progress && !waiting.empty())
        {
            progress = false;
            for (auto it = waiting.begin(); it != waiting.end();)
            {
                const scene::Placement* p = *it;
                const scene::Placement* parent = scene_.findPlacement(p->parent_frame);

                const bool parentIsProjection =
                    parent && parent->source == scene::Source::Projected;
                if (parentIsProjection && !ordered.count(p->parent_frame)) { ++it; continue; }

                ProjectionStep step;
                step.placement = p;
                step.measured  = scene_.findPlacement(p->measured);
                step.parentId  = p->parent_frame;
                step.joint     = p->joint->geometry;
                projectionOrder_.push_back(step);

                ordered.insert(p->id);
                it = waiting.erase(it);
                progress = true;
            }
        }

        for (const ProjectionStep& s : projectionOrder_)
        {
            JointEstimate e;
            e.placement_id   = s.placement->id;
            e.object_id      = s.placement->object_id;
            e.reported_arm   = s.placement->joint->reported_arm;
            e.reported_index = s.placement->joint->reported_index;
            e.lower_rad      = s.joint.lower_rad;
            e.upper_rad      = s.joint.upper_rad;
            e.unconstrained  = s.joint.unconstrained();
            e.skip_reason    = "no mocap frame seen yet";
            estimates_.emplace(e.placement_id, e);
        }
    }

    void projectJoints(const std::map<std::string, frames::Transform>& thisFrame,
                       const std::map<std::string, std::string>& missing, double stamp)
    {
        if (projectionOrder_.empty()) return;
        std::map<std::string, frames::Transform> corrected;

        const auto reasonFor = [&](const std::string& id) -> std::string {
            const auto m = missing.find(id);
            if (m != missing.end()) return "'" + id + "': " + m->second;
            return "'" + id + "' produced no pose this frame";
        };

        for (const ProjectionStep& s : projectionOrder_)
        {
            const std::string& id = s.placement->id;

            const frames::Transform* T_mocap_parent = nullptr;
            if (const auto c = corrected.find(s.parentId); c != corrected.end())
                T_mocap_parent = &c->second;
            else if (const auto f = thisFrame.find(s.parentId); f != thisFrame.end())
                T_mocap_parent = &f->second;

            const auto m = thisFrame.find(s.measured->id);
            if (!T_mocap_parent)      { skipJoint(id, reasonFor(s.parentId));      continue; }
            if (m == thisFrame.end()) { skipJoint(id, reasonFor(s.measured->id));  continue; }

            jointproj::Projection pr;
            try {
                pr = jointproj::projectInCommonFrame(*T_mocap_parent, m->second, s.joint, id);
            } catch (const frames::FrameError& e) {

                skipJoint(id, e.what());
                continue;
            }

            if (pr.degenerate)
            {
            
                skipJoint(id, "measured rotation is 180 deg about an axis perpendicular to the "
                              "joint -- every angle fits equally well, so there is no answer",
                          pr.radial_error_m, pr.axial_error_m);
                continue;
            }

        
            registry_.set(pr.corrected);
            corrected.emplace(id, frames::compose(*T_mocap_parent, pr.corrected));

            recordJoint(id, pr, stamp);
            markPlaced(*s.placement);
            notify(id);
        }
    }

    
    void skipJoint(const std::string& id, const std::string& reason,
                   std::optional<double> radial = std::nullopt,
                   std::optional<double> axial = std::nullopt)
    {
        std::lock_guard<std::mutex> lock(jointsMutex_);
        const auto it = estimates_.find(id);
        if (it == estimates_.end()) return;
        it->second.estimated   = false;
        it->second.skip_reason = reason;
        if (radial) it->second.radial_error_m = *radial;
        if (axial)  it->second.axial_error_m  = *axial;
       
    }

    void recordJoint(const std::string& id, const jointproj::Projection& pr, double stamp)
    {
        std::lock_guard<std::mutex> lock(jointsMutex_);
        const auto it = estimates_.find(id);
        if (it == estimates_.end()) return;

        JointEstimate& e  = it->second;
        e.estimated       = true;
        e.skip_reason.clear();
        e.theta_rad       = pr.theta_rad;
        e.theta_raw_rad   = pr.theta_raw_rad;
        e.clamped         = pr.clamped;
        e.residual_m      = pr.residual_m;
        e.residual_rad    = pr.residual_rad;
        e.radial_error_m  = pr.radial_error_m;
        e.axial_error_m   = pr.axial_error_m;
        e.measured_origin_m = pr.measured_origin_m;
        e.confidence      = pr.confidence;
        e.yaw_to_align_rad = pr.yaw_to_align_rad;
        e.yaw_observable   = pr.yaw_observable;
        e.stamp           = stamp;
    }

    [[nodiscard]] frames::FrameId parentOf(const scene::Placement& p) const
    {
        if (p.id == scene_.anchor_frame)  return kMocapFrame;
        if (!p.parent_frame.empty())      return p.parent_frame;
        return scene_.anchor_frame;
    }

    [[nodiscard]] bool isPlaced(const std::string& id) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return placed_.count(id) != 0;
    }

    [[nodiscard]] bool isLatchedAndPlaced(const scene::Placement& p) const
    {
        return p.capture == scene::Capture::Latched && isPlaced(p.id);
    }

    void markPlaced(const scene::Placement& p)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        placed_.insert(p.id);
    }
    [[nodiscard]] LatchPolicy policyFor(const scene::Placement& p) const
    {
        LatchPolicy out = defaultPolicy_;
        const scene::LatchOverride& o = p.latch;
        if (o.min_samples)         out.min_samples         = *o.min_samples;
        if (o.max_mean_error_m)    out.max_mean_error_m    = *o.max_mean_error_m;
        if (o.mad_k)               out.mad_k               = *o.mad_k;
        if (o.max_reject_fraction) out.max_reject_fraction = *o.max_reject_fraction;
        if (o.max_spread_m)        out.max_spread_m        = *o.max_spread_m;
        if (o.max_spread_rad)      out.max_spread_rad      = *o.max_spread_rad;
        if (o.timeout_s)           out.timeout_s           = *o.timeout_s;
        return out;
    }

    bool commit(const scene::Placement& p, const frames::Transform& pose,
                std::optional<double> meanError, double stamp)
    {
        if (p.capture == scene::Capture::Latched)
        {
            std::optional<frames::Transform> settled;
            {
                // Released before markPlaced(), which takes the same mutex.
                std::lock_guard<std::mutex> lock(mutex_);
                const auto it = latches_.find(p.id);
                if (it == latches_.end()) return false;
                settled = it->second.add(pose, meanError, stamp);
            }
            if (!settled) return false;   // still gathering, or the gate failed
            registry_.set(*settled);
        }
        else
        {
            registry_.set(pose);
        }

        markPlaced(p);
        notify(p.id);
        return true;
    }

    void seedConstructionReports()
    {
        for (const auto* p : scene_.placements())
        {
            if (p->source != scene::Source::Constructed || !p->construction) continue;

            const twomount::Geometry& g = p->construction->geometry;

            ConstructionReport r;
            r.placement_id     = p->id;
            r.object_id        = p->object_id;
            r.chord_expected_m = g.chord_length_m();
            r.reach_a_m        = g.reach_a_m();
            r.reach_b_m        = g.reach_b_m();
            r.skip_reason      = "waiting on its rigid bodies";
            constructions_.emplace(r.placement_id, r);
        }
    }

    // Rebuilds every constructed placement whose two mounts are both available.
    void updateConstructed()
    {
        for (const auto* p : scene_.placements())
        {
            if (p->source != scene::Source::Constructed || !p->construction) continue;
            if (isLatchedAndPlaced(*p)) continue;
            if (p->inputs.size() != 2) continue;   // refused at load; belt and braces

            const frames::FrameId parent = parentOf(*p);

            const auto a = registry_.lookup(parent, p->inputs[0]);
            const auto b = registry_.lookup(parent, p->inputs[1]);
            if (!a || !b)
            {
                recordConstructionSkip(p->id, "waiting on " +
                                              (!a ? p->inputs[0] : p->inputs[1]));
                continue;
            }

            const twomount::Construction c =
                twomount::construct(*a, *b, p->construction->geometry, p->id);

            recordConstruction(p->id, c, p->construction->geometry);
            if (!c.constructed) continue;

            registry_.set(c.pose);
            markPlaced(*p);
            notify(p->id);
        }
    }

    void recordConstructionSkip(const std::string& id, const std::string& reason)
    {
        std::lock_guard<std::mutex> lock(jointsMutex_);
        const auto it = constructions_.find(id);
        if (it == constructions_.end()) return;
        it->second.constructed = false;
        it->second.skip_reason = reason;
    }

    void recordConstruction(const std::string& id, const twomount::Construction& c,
                            const twomount::Geometry& g)
    {
        std::lock_guard<std::mutex> lock(jointsMutex_);
        const auto it = constructions_.find(id);
        if (it == constructions_.end()) return;

        ConstructionReport& r      = it->second;
        r.constructed              = c.constructed;
        r.skip_reason              = c.skip_reason;
        r.chord_expected_m         = g.chord_length_m();
        r.chord_error_m            = c.chord_error_m;
        r.chord_measured_m         = g.chord_length_m() + c.chord_error_m;
        r.normal_disagreement_rad  = c.normal_disagreement_rad;
        r.chord_out_of_plane_m     = c.chord_out_of_plane_m;
        r.normal_in_parent_error_rad = c.normal_in_parent_error_rad;
        r.normal_in_parent_checked   = c.normal_in_parent_checked;
        r.reach_a_m                = g.reach_a_m();
        r.reach_b_m                = g.reach_b_m();
        if (c.constructed) r.stamp = c.pose.stamp;
    }

    // Recomputes every fused placement whose inputs are all available.
    void updateFused()
    {
        for (const auto* p : scene_.placements())
        {
            if (p->source != scene::Source::Fused) continue;
            if (isLatchedAndPlaced(*p)) continue;

            const frames::FrameId parent = parentOf(*p);

            std::vector<frames::Transform> in;
            in.reserve(p->inputs.size());
            for (const auto& id : p->inputs)
            {
                auto t = registry_.lookup(parent, id);
                if (!t) break;          // an input is not placed yet; try again next update
                in.push_back(*t);
            }
            if (in.size() != p->inputs.size()) continue;

            registry_.set(detail::meanPose(
                parent, p->id, in.size(),
                [&in](std::size_t i) -> const frames::Transform& { return in[i]; }));
            markPlaced(*p);
            notify(p->id);
        }
    }

    void notify(const std::string& placementId)
    {
        if (!observer_) return;
        if (auto t = registry_.lookup(scene_.anchor_frame, placementId))
            observer_(placementId, *t);
        
    }

    scene::Scene            scene_;
    frames::Registry&       registry_;
    LatchPolicy             defaultPolicy_;
    PlacementObserver       observer_;

    double continuousQualityLimit_{std::numeric_limits<double>::infinity()};

    mutable std::mutex      mutex_;
    std::set<std::string>   placed_;

    
    std::map<std::string, LatchAccumulator> latches_;

    std::vector<ProjectionStep> projectionOrder_;

    mutable std::mutex                        jointsMutex_;
    std::map<std::string, JointEstimate>      estimates_;
    std::map<std::string, ConstructionReport> constructions_;
};

}  // namespace placement
