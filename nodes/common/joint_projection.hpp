#pragma once

#include "frames.hpp"

#include <cmath>
#include <string>

namespace jointproj {

inline constexpr double kPi    = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

inline constexpr double kDegenerateTol = 1e-9;

struct RevoluteJoint {
    frames::Vec3 axis_point_m{frames::Vec3::Zero()};
    frames::Vec3 axis{frames::Vec3::UnitZ()};
    frames::Vec3 zero_origin_m{frames::Vec3::Zero()};
    frames::Quat zero_rotation{frames::Quat::Identity()};

    double lower_rad{-kPi};
    double upper_rad{kPi};

    [[nodiscard]] frames::Transform at(const frames::FrameId& parent, const frames::FrameId& child,
                                       double theta_rad, double stamp = 0.0) const;

    [[nodiscard]] double radius_m() const;

    [[nodiscard]] bool unconstrained() const { return (upper_rad - lower_rad) >= kTwoPi; }
};

struct Projection {
    double theta_rad{0.0};

    double theta_raw_rad{0.0};

    bool clamped{false};
    bool degenerate{false};
    frames::Transform corrected;
    double residual_m{0.0};
    double residual_rad{0.0};
    frames::Vec3 measured_origin_m{frames::Vec3::Zero()};
    double radial_error_m{0.0};
    double axial_error_m{0.0};

    // sqrt(w^2 + (a.v)^2), in [0, 1]. How much signal theta was recovered from.
    double confidence{0.0};
    double yaw_to_align_rad{0.0};
    bool yaw_observable{false};
};

[[nodiscard]] inline double wrapToNear(double theta_rad, double centre_rad)
{
    return theta_rad + kTwoPi * std::round((centre_rad - theta_rad) / kTwoPi);
}

[[nodiscard]] inline double clampToLimits(double theta_star, double lower, double upper,
                                          bool& clamped)
{
    clamped = false;
    if ((upper - lower) >= kTwoPi) return theta_star;   // constrains nothing

    const double centre  = 0.5 * (lower + upper);
    const double wrapped = wrapToNear(theta_star, centre);

    if (wrapped < lower)
    {
        clamped = true;
        return lower;
    }
    if (wrapped > upper)
    {
        clamped = true;
        return upper;
    }
    return wrapped;
}

namespace detail {

[[nodiscard]] inline frames::Vec3 unitAxis(const frames::Vec3& axis)
{
    const double n = axis.norm();
    if (!(n > 1e-9))
        throw frames::FrameError("joint_projection: joint axis has zero length; it must name a "
                                 "direction (the URDF <axis> of the joint)");
    return axis / n;
}

}   // namespace detail

inline frames::Transform RevoluteJoint::at(const frames::FrameId& parent,
                                           const frames::FrameId& child, double theta_rad,
                                           double stamp) const
{
    const frames::Vec3 a = detail::unitAxis(axis);
    const Eigen::AngleAxisd turn(theta_rad, a);
    const frames::Vec3 origin   = axis_point_m + turn * (zero_origin_m - axis_point_m);
    const frames::Quat rotation = frames::Quat(turn) * zero_rotation.normalized();

    return frames::make(parent, child, origin, rotation, stamp);
}

inline double RevoluteJoint::radius_m() const
{
    const frames::Vec3 a = detail::unitAxis(axis);
    const frames::Vec3 d = zero_origin_m - axis_point_m;
    return (d - d.dot(a) * a).norm();
}

[[nodiscard]] inline Projection project(const frames::Transform& T_parent_measured,
                                        const RevoluteJoint& joint,
                                        const frames::FrameId& correctedChildFrame)
{
    if (correctedChildFrame.empty())
        throw frames::FrameError("project: correctedChildFrame must be named -- the corrected "
                                 "pose is a separate claim from the measurement and needs its "
                                 "own frame");
    if (correctedChildFrame == T_parent_measured.to)
        throw frames::FrameError("project: correctedChildFrame '" + correctedChildFrame +
                                 "' is the parent frame; that would make the joint a self-edge");

    const frames::Vec3 axis = detail::unitAxis(joint.axis);

    Projection out;
    out.measured_origin_m = T_parent_measured.translation();

    {
        const frames::Vec3 dMeas = T_parent_measured.translation() - joint.axis_point_m;
        const frames::Vec3 dZero = joint.zero_origin_m - joint.axis_point_m;

        const double axialMeas = dMeas.dot(axis);
        const double axialZero = dZero.dot(axis);

        out.axial_error_m  = axialMeas - axialZero;
        out.radial_error_m = (dMeas - axialMeas * axis).norm() - (dZero - axialZero * axis).norm();
    }

    frames::Quat q = T_parent_measured.rotation() * joint.zero_rotation.normalized().conjugate();
    q.normalize();

    if (q.w() < 0.0) q.coeffs() = -q.coeffs();

    const double w = q.w();
    const double d = axis.dot(q.vec());

    out.confidence = std::sqrt(w * w + d * d);
    if (out.confidence < kDegenerateTol)
    {
        out.degenerate = true;
        return out;
    }

    out.theta_raw_rad = 2.0 * std::atan2(d, w);
    out.theta_rad = clampToLimits(out.theta_raw_rad, joint.lower_rad, joint.upper_rad, out.clamped);

    out.corrected =
        joint.at(T_parent_measured.to, correctedChildFrame, out.theta_rad, T_parent_measured.stamp);

    out.residual_m = (T_parent_measured.translation() - out.corrected.translation()).norm();

    const frames::Quat onManifold(Eigen::AngleAxisd(out.theta_rad, axis));
    out.residual_rad = Eigen::AngleAxisd(onManifold.conjugate() * q).angle();

    {
        const auto perpendicular = [&axis](const frames::Vec3& v) {
            return v - v.dot(axis) * axis;
        };
        const frames::Vec3 predicted = perpendicular(out.corrected.translation());
        const frames::Vec3 measured  = perpendicular(T_parent_measured.translation());

        if (predicted.norm() > kDegenerateTol && measured.norm() > kDegenerateTol)
        {
            out.yaw_to_align_rad =
                std::atan2(axis.dot(predicted.cross(measured)), predicted.dot(measured));
            out.yaw_observable = true;
        }
    }

    return out;
}

[[nodiscard]] inline Projection projectInCommonFrame(const frames::Transform& T_x_parent,
                                                     const frames::Transform& T_x_measured,
                                                     const RevoluteJoint& joint,
                                                     const frames::FrameId& correctedChildFrame)
{
    if (T_x_parent.to != T_x_measured.to)
        throw frames::FrameError("projectInCommonFrame: the parent pose is in '" + T_x_parent.to +
                                 "' but the measured pose is in '" + T_x_measured.to +
                                 "'; both must be expressed in the same frame");

    return project(frames::compose(frames::inverse(T_x_parent), T_x_measured), joint,
                   correctedChildFrame);
}

}   // namespace jointproj
