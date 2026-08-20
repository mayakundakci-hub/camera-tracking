#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace frames {

using FrameId = std::string;

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;
using Iso  = Eigen::Isometry3d;

class FrameError : public std::runtime_error {
public:
    explicit FrameError(const std::string& what) : std::runtime_error("frames: " + what) {}
};

struct Point {
    FrameId frame;
    Vec3 p{Vec3::Zero()};
};

struct Transform {
    FrameId to;                 // the frame this transform produces
    FrameId from;               // the frame it consumes
    Iso iso{Iso::Identity()};   // to <- from. Eigen owns all arithmetic.

    double stamp{0.0};

    [[nodiscard]] Point apply(const Point& in) const
    {
        if (in.frame != from)
            throw FrameError("apply: T_" + to + "_" + from + " cannot consume a point in '" +
                             in.frame + "' (expected '" + from + "')");
        return {to, iso * in.p};
    }

    // For callers looping over many points in a frame already established.
    [[nodiscard]] Vec3 applyUnchecked(const Vec3& p) const { return iso * p; }

    [[nodiscard]] Vec3 translation() const { return iso.translation(); }
    [[nodiscard]] Quat rotation() const { return Quat(iso.rotation()); }
};

inline Transform identity(const FrameId& frame) { return {frame, frame, Iso::Identity(), 0.0}; }

inline Transform make(FrameId to, FrameId from, const Vec3& translation_m, const Quat& rotation,
                      double stamp = 0.0)
{
    if (rotation.norm() < 1e-9)
        throw FrameError("make: T_" + to + "_" + from + " was given a zero-length quaternion");

    Iso iso           = Iso::Identity();
    iso.linear()      = rotation.normalized().toRotationMatrix();
    iso.translation() = translation_m;
    return {std::move(to), std::move(from), iso, stamp};
}

// T_a_b  ->  T_b_a
inline Transform inverse(const Transform& t) { return {t.from, t.to, t.iso.inverse(), t.stamp}; }

inline double combineStamps(double a, double b)
{
    if (a == 0.0) return b;
    if (b == 0.0) return a;
    return std::fmin(a, b);
}

// compose(T_a_b, T_b_c) -> T_a_c. The inner-frame check catches a reversed
// transform where it is used rather than three modules downstream.
inline Transform compose(const Transform& a_b, const Transform& b_c)
{
    if (a_b.from != b_c.to)
        throw FrameError("compose: T_" + a_b.to + "_" + a_b.from + " o T_" + b_c.to + "_" +
                         b_c.from + " -- inner frames do not meet ('" + a_b.from + "' vs '" +
                         b_c.to + "'); one of them is probably reversed");

    return {a_b.to, b_c.from, a_b.iso * b_c.iso, combineStamps(a_b.stamp, b_c.stamp)};
}

// How far apart two frames are -- the entire output of a comparison.
struct Magnitude {
    double distance_m{0.0};
    double angle_rad{0.0};
};

inline Magnitude magnitudeOf(const Transform& t)
{
    // Eigen's AngleAxis conversion already folds q/-q and clamps, so the
    // angle comes back in [0, pi] without special-casing.
    return {t.iso.translation().norm(), Eigen::AngleAxisd(t.iso.rotation()).angle()};
}

class Registry {
public:
    void set(const Transform& t)
    {
        if (t.to == t.from)
            throw FrameError("set: refusing a self-edge on frame '" + t.to +
                             "' -- an identity edge is never useful, and usually means two "
                             "placements in the manifest were given the same name");

        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& e : edges_)
        {
            if ((e.to == t.to && e.from == t.from) || (e.to == t.from && e.from == t.to))
            {
                e = t;
                return;
            }
        }
        edges_.push_back(t);
    }

    // For when an object loses tracking and a stale pose is worse than none.
    bool erase(const FrameId& a, const FrameId& b)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = edges_.begin(); it != edges_.end(); ++it)
        {
            if ((it->to == a && it->from == b) || (it->to == b && it->from == a))
            {
                edges_.erase(it);
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] bool has(const FrameId& to, const FrameId& from) const
    {
        return lookup(to, from).has_value();
    }

    // Resolve T_to_from, or nullopt if the frames are not connected.
    [[nodiscard]] std::optional<Transform> lookup(const FrameId& to, const FrameId& from) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (to == from) return identity(to);

        // Breadth-first from `from`; reached[f] holds T_f_from.
        std::unordered_map<FrameId, Transform> reached;
        reached.emplace(from, identity(from));
        std::deque<FrameId> queue{from};

        while (!queue.empty())
        {
            const FrameId cur = queue.front();
            queue.pop_front();
            const Transform T_cur_from = reached.at(cur);   // copied: emplace below may rehash

            for (const auto& e : edges_)
            {
                // Every stored edge is usable in both directions.
                FrameId next;
                Transform T_next_cur;
                if (e.from == cur)
                {
                    next       = e.to;
                    T_next_cur = e;
                }
                else if (e.to == cur)
                {
                    next       = e.from;
                    T_next_cur = inverse(e);
                }
                else
                    continue;

                if (reached.count(next)) continue;

                Transform T_next_from = compose(T_next_cur, T_cur_from);
                if (next == to) return T_next_from;

                reached.emplace(next, std::move(T_next_from));
                queue.push_back(next);
            }
        }
        return std::nullopt;
    }

    // Throws naming the frames it does know, which is what turns a misspelled
    // manifest entry into a five-second diagnosis.
    [[nodiscard]] Transform require(const FrameId& to, const FrameId& from) const
    {
        if (auto t = lookup(to, from)) return *t;

        std::string known;
        for (const auto& f : knownFrames())
            known += (known.empty() ? "" : ", ") + f;
        throw FrameError("require: no route from '" + from + "' to '" + to + "'. Known frames: " +
                         (known.empty() ? "(none -- nothing has been placed yet)" : known));
    }

    [[nodiscard]] std::vector<FrameId> knownFrames() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<FrameId> out;
        for (const auto& e : edges_)
        {
            for (const FrameId& f : {e.to, e.from})
            {
                bool seen = false;
                for (const auto& o : out)
                    seen = seen || (o == f);
                if (!seen) out.push_back(f);
            }
        }
        return out;
    }

    [[nodiscard]] std::size_t edgeCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return edges_.size();
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        edges_.clear();
    }

private:
    mutable std::mutex mutex_;
    std::vector<Transform> edges_;
};

// The only place units and layouts change.
namespace convert {

inline constexpr double kMmPerM = 1000.0;

inline double mmToM(double mm) { return mm / kMmPerM; }
inline double mToMm(double m) { return m * kMmPerM; }
inline double degToRad(double d) { return d * (3.14159265358979323846 / 180.0); }
inline double radToDeg(double r) { return r * (180.0 / 3.14159265358979323846); }

// Motive, PosePacket and ludus::Transform all store (w,x,y,z), as does Eigen's
// Quaterniond CONSTRUCTOR -- but its coeffs() vector is (x,y,z,w).
inline Quat quatFromWxyz(double w, double x, double y, double z)
{
    const Quat q(w, x, y, z);
    if (q.norm() < 1e-9)
        throw FrameError("quatFromWxyz: zero-length quaternion (source reported no rotation "
                         "at all -- check the tracking-valid flag before converting)");
    return q.normalized();
}

// Pairs with ludus::Transform and PosePacket without knowing either type.
inline std::array<double, 4> quatToWxyz(const Quat& q)
{
    const Quat n = q.normalized();
    return {n.w(), n.x(), n.y(), n.z()};
}

inline std::array<double, 3> vecToArray(const Vec3& v) { return {v.x(), v.y(), v.z()}; }

inline Vec3 vecFromArray(const std::array<double, 3>& a) { return {a[0], a[1], a[2]}; }

// For the config path, which is authored in the units a drawing reports.
inline Vec3 vecFromMm(double x_mm, double y_mm, double z_mm)
{
    return {mmToM(x_mm), mmToM(y_mm), mmToM(z_mm)};
}

}   // namespace convert
}   // namespace frames
