#pragma once


#include "frames.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace twomount {

inline constexpr double kDegenerateTol = 1e-9;
struct Geometry {

    frames::Vec3 normal_axis{frames::Vec3::UnitZ()};
    frames::Vec3 normal_in_part{frames::Vec3::UnitZ()};
    frames::Vec3 mount_a_m{frames::Vec3::Zero()};
    frames::Vec3 mount_b_m{frames::Vec3::Zero()};
    std::optional<frames::Vec3> expect_normal_in_parent;

    [[nodiscard]] frames::Vec3 chord() const { return mount_b_m - mount_a_m; }
    [[nodiscard]] double chord_length_m() const { return chord().norm(); }
    [[nodiscard]] double reach_a_m() const { return mount_a_m.norm(); }
    [[nodiscard]] double reach_b_m() const { return mount_b_m.norm(); }
};

struct Construction {
    bool        constructed{false};
    std::string skip_reason;

    // T_parent_part.
    frames::Transform pose;
    double chord_error_m{0.0};
    double normal_disagreement_rad{0.0};
    double chord_out_of_plane_m{0.0};
    double normal_in_parent_error_rad{0.0};
    bool   normal_in_parent_checked{false};
};

namespace detail {
[[nodiscard]] inline double angleBetween(const frames::Vec3& a, const frames::Vec3& b)
{
    const frames::Vec3 ua = a.normalized();
    const frames::Vec3 ub = b.normalized();
    return 2.0 * std::atan2((ua - ub).norm(), (ua + ub).norm());
}

}  // namespace detail

[[nodiscard]] inline Construction construct(const frames::Transform& T_parent_a,
                                            const frames::Transform& T_parent_b,
                                            const Geometry&          geometry,
                                            const frames::FrameId&   partFrame)
{
    if (T_parent_a.to != T_parent_b.to)
        throw frames::FrameError("twomount::construct: the two bodies are in different frames ('" +
                                 T_parent_a.to + "' vs " + T_parent_b.to +
                                 "'); both must be resolved into the constructed placement's "
                                 "parent frame first");

    Construction out;

    // --- the CAD side ---
    if (!geometry.normal_axis.allFinite() || geometry.normal_axis.norm() < kDegenerateTol)
    {
        out.skip_reason = "normal_axis is zero or non-finite";
        return out;
    }
    if (!geometry.normal_in_part.allFinite() || geometry.normal_in_part.norm() < kDegenerateTol)
    {
        out.skip_reason = "normal_in_part is zero or non-finite";
        return out;
    }

    const frames::Vec3 chordCad     = geometry.chord();
    const double       chordCadNorm = chordCad.norm();
    if (chordCadNorm < kDegenerateTol)
    {
        out.skip_reason = "mount_a and mount_b are the same point in CAD; there is no chord to "
                          "build on";
        return out;
    }
    const frames::Vec3 cCad     = chordCad / chordCadNorm;
    const frames::Vec3 nCad     = geometry.normal_in_part.normalized();
    const double       dCad     = cCad.dot(nCad);
    const frames::Vec3 upCadRaw = nCad - dCad * cCad;
    if (upCadRaw.norm() < kDegenerateTol)
    {
        out.skip_reason = "normal_in_part is parallel to the CAD chord, so it cannot pin the roll "
                          "about it -- the two mount points and the normal are describing the same "
                          "line";
        return out;
    }

    // --- the measured side ---
    const frames::Vec3 nUnit = geometry.normal_axis.normalized();
    const frames::Vec3 nA    = T_parent_a.rotation() * nUnit;
    const frames::Vec3 nB    = T_parent_b.rotation() * nUnit;

    out.normal_disagreement_rad = detail::angleBetween(nA, nB);

    const frames::Vec3 nSum = nA + nB;
    if (nSum.norm() < kDegenerateTol)
    {
        out.skip_reason = "the two bodies' face normals are antiparallel, so they cannot be "
                          "seated the same way on one part -- check that both Motive bodies use "
                          "the same axis convention";
        return out;
    }
    const frames::Vec3 nBar = nSum.normalized();

    const frames::Vec3 pA        = T_parent_a.translation();
    const frames::Vec3 pB        = T_parent_b.translation();
    const frames::Vec3 chord     = pB - pA;
    const double       chordNorm = chord.norm();
    if (chordNorm < kDegenerateTol)
    {
        out.skip_reason = "the two bodies are at the same point; there is no chord to build on";
        return out;
    }
    const frames::Vec3 c    = chord / chordNorm;
    const double       dMes = c.dot(nBar);

    out.chord_error_m        = chordNorm - chordCadNorm;
    out.chord_out_of_plane_m = chordNorm * (dMes - dCad);

    const frames::Vec3 upRaw = nBar - dMes * c;
    if (upRaw.norm() < kDegenerateTol)
    {
        out.skip_reason = "the averaged face normal is parallel to the measured chord, so it "
                          "cannot pin the roll about it -- normal_axis almost certainly names the "
                          "wrong body axis";
        return out;
    }

    Eigen::Matrix3d measured;
    measured.col(0) = c;
    measured.col(1) = upRaw.normalized();
    measured.col(2) = measured.col(0).cross(measured.col(1));

    Eigen::Matrix3d cad;
    cad.col(0) = cCad;
    cad.col(1) = upCadRaw.normalized();
    cad.col(2) = cad.col(0).cross(cad.col(1));

    const Eigen::Matrix3d R = measured * cad.transpose();

    if (geometry.expect_normal_in_parent)
    {
        const frames::Vec3& want = *geometry.expect_normal_in_parent;
        if (want.allFinite() && want.norm() > kDegenerateTol)
        {
            out.normal_in_parent_error_rad = detail::angleBetween(R * nCad, want);
            out.normal_in_parent_checked = true;
        }
    }

    const frames::Vec3 midMeasured = 0.5 * (pA + pB);
    const frames::Vec3 midCad      = 0.5 * (geometry.mount_a_m + geometry.mount_b_m);
    const frames::Vec3 origin      = midMeasured - R * midCad;

    out.pose = frames::make(T_parent_a.to, partFrame, origin, frames::Quat(R),
                            frames::combineStamps(T_parent_a.stamp, T_parent_b.stamp));
    out.constructed = true;
    return out;
}

}  // namespace twomount
