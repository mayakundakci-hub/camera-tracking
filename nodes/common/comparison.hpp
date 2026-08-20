#pragma once
// comparison: how far apart two claims about one object are

#include "frames.hpp"
#include "scene_config.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace comparison {

// One object, two claims, and the disagreement between them.
struct Delta {
    std::string object_id;
    std::string a;          // placement id: the pose being described
    std::string b;          // placement id: the pose it is described FROM
    std::string a_source;   // e.g. "optitrack" -- which claim is which
    std::string b_source;

    double dx_mm{0.0};   // b's origin expressed in a's frame
    double dy_mm{0.0};
    double dz_mm{0.0};
    double distance_mm{0.0};
    double angle_deg{0.0};

    double stamp{0.0};        // oldest measurement in the pair
    double time_gap_s{0.0};   // 0 for latched pairs; the match error otherwise

    bool valid{false};
    std::string invalid_reason;   // populated when valid == false, never empty then
    bool review_gated{false};
};

class Comparator {
public:
    Comparator(scene::Scene scene, const frames::Registry& registry, double maxMatchGapSec,
               std::size_t bufferLen)
        : scene_(std::move(scene)), registry_(registry), maxMatchGapSec_(maxMatchGapSec),
          bufferLen_(bufferLen)
    {
        for (const auto* p : scene_.placements())
            if (isTimeVarying(*p)) buffers_[p->id];   // create the (empty) deque
    }

    [[nodiscard]] static bool isTimeVarying(const scene::Placement& p)
    {
        return p.capture == scene::Capture::Continuous &&
               (p.source == scene::Source::Optitrack || p.source == scene::Source::ExpectedPose ||
                p.source == scene::Source::Projected);
    }

    void onPlacementUpdated(const std::string& placementId,
                            const frames::Transform& T_anchor_placement)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = buffers_.find(placementId);
        if (it == buffers_.end()) return;   // latched or static; nothing to remember

        it->second.push_back(T_anchor_placement);
        if (it->second.size() > bufferLen_) it->second.pop_front();
    }

    [[nodiscard]] std::vector<Delta> compute() const
    {
        std::vector<Delta> out;
        for (const auto& c : scene_.comparisons())
            out.push_back(evaluate(c));
        return out;
    }

    [[nodiscard]] const scene::Scene& scene() const { return scene_; }

private:
    [[nodiscard]] Delta evaluate(const scene::Comparison& c) const
    {
        Delta d;
        d.object_id = c.object_id;
        d.a         = c.a;
        d.b         = c.b;

        const scene::Placement* pa = scene_.findPlacement(c.a);
        const scene::Placement* pb = scene_.findPlacement(c.b);
        if (!pa || !pb)
        {
            d.invalid_reason = "placement missing from scene";
            return d;
        }
        d.a_source = scene::toString(pa->source);
        d.b_source = scene::toString(pb->source);
        const bool bothLatched =
            pa->capture == scene::Capture::Latched && pb->capture == scene::Capture::Latched;
        const scene::Object* obj = scene_.findObject(c.object_id);
        d.review_gated           = (obj && obj->review_gate) ? *obj->review_gate : bothLatched;

        const bool bothContinuous = isTimeVarying(*pa) && isTimeVarying(*pb);

        std::optional<frames::Transform> Ta, Tb;
        if (bothContinuous)
        {
            if (!matchInTime(c.a, c.b, Ta, Tb, d)) return d;   // matchInTime filled in the reason
        }
        else
        {
            Ta = registry_.lookup(scene_.anchor_frame, c.a);
            Tb = registry_.lookup(scene_.anchor_frame, c.b);
            if (!Ta)
            {
                d.invalid_reason = "'" + c.a + "' has not been placed yet";
                return d;
            }
            if (!Tb)
            {
                d.invalid_reason = "'" + c.b + "' has not been placed yet";
                return d;
            }
        }

        const frames::Transform T_a_b = frames::compose(frames::inverse(*Ta), *Tb);
        const frames::Vec3 t          = T_a_b.translation();
        const frames::Magnitude m     = frames::magnitudeOf(T_a_b);

        d.dx_mm       = frames::convert::mToMm(t.x());
        d.dy_mm       = frames::convert::mToMm(t.y());
        d.dz_mm       = frames::convert::mToMm(t.z());
        d.distance_mm = frames::convert::mToMm(m.distance_m);
        d.angle_deg   = frames::convert::radToDeg(m.angle_rad);
        d.stamp       = T_a_b.stamp;
        d.valid       = true;
        return d;
    }

    bool matchInTime(const std::string& a, const std::string& b,
                     std::optional<frames::Transform>& Ta, std::optional<frames::Transform>& Tb,
                     Delta& d) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto ia = buffers_.find(a);
        const auto ib = buffers_.find(b);
        if (ia == buffers_.end() || ib == buffers_.end())
        {
            d.invalid_reason = "no history buffer (placement is not continuous)";
            return false;
        }
        if (ia->second.empty())
        {
            d.invalid_reason = "'" + a + "' has no samples yet";
            return false;
        }
        if (ib->second.empty())
        {
            d.invalid_reason = "'" + b + "' has no samples yet";
            return false;
        }

        const frames::Transform& newestA = ia->second.back();

        const frames::Transform* best = nullptr;
        double bestGap                = 0.0;
        for (const auto& cand : ib->second)
        {
            const double gap = std::fabs(cand.stamp - newestA.stamp);
            if (!best || gap < bestGap)
            {
                best    = &cand;
                bestGap = gap;
            }
        }

        if (bestGap > maxMatchGapSec_)
        {

            d.time_gap_s     = bestGap;
            d.invalid_reason = "no time match: nearest sample is " + std::to_string(bestGap) +
                               "s away, limit " + std::to_string(maxMatchGapSec_) + "s";
            return false;
        }

        d.time_gap_s = bestGap;
        Ta           = newestA;
        Tb           = *best;
        return true;
    }

    scene::Scene scene_;
    const frames::Registry& registry_;
    double maxMatchGapSec_;
    std::size_t bufferLen_;

    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::deque<frames::Transform>> buffers_;
};

}   // namespace comparison
