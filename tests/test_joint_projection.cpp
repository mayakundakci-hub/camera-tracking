// Tests for nodes/common/joint_projection.hpp
//
// The feature this file backs is one function, so these tests are the feature's
// correctness argument. Three of them carry most of the weight:
//
//   ProjectionIsOptimal      -- brute force. The ONLY test that pins "closed-form
//                               optimal"; every other round-trip test passes just
//                               as happily with a plausible-but-suboptimal formula.
//   ThetaIsIndependentOfPosition
//                            -- pins the geometric claim the whole design rests on.
//   DegenerateCaseIsRefused  -- pins that an undefined angle is refused rather than
//                               reported as a confident zero.
//
// Eigen's arithmetic is not retested here; it is not ours to verify.

#include <gtest/gtest.h>

#include "joint_projection.hpp"

#include <cmath>
#include <random>

using namespace jointproj;
using frames::Quat;
using frames::Transform;
using frames::Vec3;

namespace {

constexpr double kTight = 1e-12;
constexpr double kLoose = 1e-9;

// The hand's J9 in the URDF's own convention: the axis runs through the child
// frame's origin, so the child does not orbit. radius_m() is zero here.
RevoluteJoint j9()
{
    RevoluteJoint j;
    j.axis_point_m  = Vec3(0.0, 0.270, 0.0);
    j.zero_origin_m = Vec3(0.0, 0.270, 0.0);   // ON the axis
    j.axis          = Vec3::UnitZ();
    j.lower_rad     = -1.5707;
    j.upper_rad     = 1.5707;
    return j;
}

RevoluteJoint j10()
{
    RevoluteJoint j;
    j.axis_point_m  = Vec3(0.0, -0.040958, -0.055129);
    j.zero_origin_m = Vec3(0.0, -0.040958, -0.055129);
    j.axis          = Vec3::UnitY();
    j.lower_rad     = -6.2831;
    j.upper_rad     = 6.2831;
    return j;
}

// The case the real hardware presents: the tracked body's frame is a machined
// fiducial on a plate corner, a long way OFF the axis, so it orbits as the joint
// turns. 120 mm of radius, which over a 90 degree sweep is 170 mm of travel --
// far too much to charge to measurement error.
RevoluteJoint offAxis()
{
    RevoluteJoint j;
    j.axis_point_m  = Vec3(0.0, 0.270, 0.0);
    j.zero_origin_m = Vec3(0.120, 0.270, 0.045);   // 120 mm off in X, 45 mm along Z
    j.axis          = Vec3::UnitZ();
    j.lower_rad     = -1.5707;
    j.upper_rad     = 1.5707;
    return j;
}

// Chosen to break anything that assumes a coordinate-aligned axis, an identity
// zero rotation, or an axis through the child origin -- none of which the real
// geometry above exercises all at once.
RevoluteJoint awkward()
{
    RevoluteJoint j;
    j.axis_point_m  = Vec3(0.031, -0.204, 0.087);
    j.zero_origin_m = Vec3(0.075, -0.150, 0.140);
    j.zero_rotation = Quat(Eigen::AngleAxisd(0.83, Vec3(-0.2, 0.7, 0.68).normalized()));
    j.axis          = Vec3(1.0, 2.0, 3.0).normalized();
    j.lower_rad     = -kPi;
    j.upper_rad     = kPi;
    return j;
}

// An arbitrary parent pose. Deliberately not near identity, so a dropped or
// misordered transform cannot coincidentally pass.
Transform parentPose(const frames::FrameId& to, const frames::FrameId& from, double stamp = 0.0)
{
    const Quat q(Eigen::AngleAxisd(1.13, Vec3(0.41, -0.62, 0.67).normalized()));
    return frames::make(to, from, Vec3(1.84, -0.37, 1.62), q, stamp);
}

// The measured pose of a child sitting exactly at `theta` on `joint`.
Transform onManifold(const RevoluteJoint& joint, double theta, const frames::FrameId& parent,
                     const frames::FrameId& child)
{
    return joint.at(parent, child, theta);
}

// The matrix form of the same projection, written out independently so the two
// derivations can be cross-checked against each other.
Vec3 vee(const Eigen::Matrix3d& s) { return {s(2, 1), s(0, 2), s(1, 0)}; }

double thetaViaMatrixForm(const Eigen::Matrix3d& r, const Vec3& axis)
{
    const double a = r.trace() - axis.dot(r * axis);
    const double b = axis.dot(vee(r - r.transpose()));
    return std::atan2(b, a);
}

}   // namespace

// ---------------------------------------------------------------
// round trip -- a pose built at theta must give theta back
// ---------------------------------------------------------------

TEST(JointProjection, RecoversAngleOnRealGeometry)
{
    const RevoluteJoint joint = j9();

    for (const double theta : {-1.5, -0.7, 0.0, 0.3, 1.5})
    {
        const Projection p = project(onManifold(joint, theta, "j8", "j9"), joint, "j9_proj");

        EXPECT_FALSE(p.degenerate);
        EXPECT_FALSE(p.clamped);
        EXPECT_NEAR(p.theta_rad, theta, kTight) << "theta = " << theta;
        EXPECT_NEAR(p.residual_m, 0.0, kTight);
        EXPECT_NEAR(p.residual_rad, 0.0, kLoose);
        EXPECT_NEAR(p.confidence, 1.0, kLoose);
    }
}

TEST(JointProjection, RecoversAngleWithSkewAxisOffsetOriginAndZeroRotation)
{
    // All three awkward features at once: a non-coordinate axis, a non-identity
    // zero rotation, AND a child origin off the axis. Each is individually
    // invisible on a URDF-shaped joint, and each is wrong in a different way.
    const RevoluteJoint joint = awkward();

    for (const double theta : {-2.4, -0.9, 0.0, 1.1, 2.9})
    {
        const Projection p = project(onManifold(joint, theta, "a", "b"), joint, "b_proj");

        EXPECT_FALSE(p.degenerate);
        EXPECT_NEAR(p.theta_rad, theta, kTight) << "theta = " << theta;
        EXPECT_NEAR(p.residual_m, 0.0, kTight);
        EXPECT_NEAR(p.residual_rad, 0.0, kLoose);
    }
}

// ---------------------------------------------------------------
// optimality -- the test that actually pins the closed form
// ---------------------------------------------------------------

TEST(JointProjection, ProjectionIsOptimal)
{
    // Everything above passes with a formula that is merely close. This does not:
    // it perturbs off the manifold, then checks by exhaustive scan that no other
    // angle explains the measurement better.
    const RevoluteJoint joint = awkward();
    const Vec3 axis           = joint.axis.normalized();

    std::mt19937 rng(20260810);
    std::uniform_real_distribution<double> angle(-0.35, 0.35);
    std::uniform_real_distribution<double> comp(-1.0, 1.0);

    for (int trial = 0; trial < 25; ++trial)
    {
        const double thetaTrue = std::uniform_real_distribution<double>(-2.8, 2.8)(rng);

        // A small, deliberately mixed perturbation -- not about the joint axis,
        // not about a single perpendicular axis either.
        const Vec3 wobbleAxis = Vec3(comp(rng), comp(rng), comp(rng)).normalized();
        const Quat wobble(Eigen::AngleAxisd(angle(rng), wobbleAxis));

        const Transform clean = onManifold(joint, thetaTrue, "a", "b");
        const Transform measured =
            frames::make("a", "b", clean.translation(), clean.rotation() * wobble);

        const Projection p = project(measured, joint, "b_proj");
        ASSERT_FALSE(p.degenerate);

        // Cost = geodesic angle between the measurement and the manifold point.
        // Minimising this is equivalent to minimising Frobenius distance. Note
        // the zero rotation divides out on the RIGHT: the axis is fixed in the
        // PARENT frame, so the turn premultiplies.
        const Quat qRel = measured.rotation() * joint.zero_rotation.normalized().conjugate();

        const auto costAt = [&](double theta) {
            const Quat candidate(Eigen::AngleAxisd(theta, axis));
            return Eigen::AngleAxisd(candidate.conjugate() * qRel).angle();
        };

        const double best = costAt(p.theta_rad);
        EXPECT_NEAR(best, p.residual_rad, kLoose);

        constexpr int kSteps = 100000;
        for (int i = 0; i <= kSteps; ++i)
        {
            const double theta = -kPi + (kTwoPi * i) / kSteps;
            // Tolerance covers the grid's own resolution; a wrong closed form
            // loses by far more than this.
            EXPECT_GE(costAt(theta), best - 1e-9)
                << "grid theta " << theta << " beat the closed form " << p.theta_rad;
        }
    }
}

TEST(JointProjection, QuaternionAndMatrixFormsAgree)
{
    // The implementation uses the quaternion form; the header claims it is
    // algebraically identical to the matrix form. If anyone ever "optimises" one
    // of them, this is what notices.
    const Vec3 axis = Vec3(1.0, 2.0, 3.0).normalized();

    std::mt19937 rng(7);
    std::uniform_real_distribution<double> comp(-1.0, 1.0);

    RevoluteJoint joint;
    joint.axis      = axis;
    joint.lower_rad = -kPi;
    joint.upper_rad = kPi;

    for (int trial = 0; trial < 200; ++trial)
    {
        const Quat q             = Quat(comp(rng), comp(rng), comp(rng), comp(rng)).normalized();
        const Transform measured = frames::make("a", "b", Vec3::Zero(), q);

        const Projection p = project(measured, joint, "b_proj");
        ASSERT_FALSE(p.degenerate);

        EXPECT_NEAR(p.theta_raw_rad, thetaViaMatrixForm(q.toRotationMatrix(), axis), kLoose);
    }
}

TEST(JointProjection, QuaternionDoubleCoverDoesNotShiftTheta)
{
    // q and -q are the same rotation. Without the w >= 0 canonicalisation the
    // atan2 branch flips and theta moves by a full turn, so which representative
    // the tracker happened to send would change the reported angle.
    const RevoluteJoint joint = j9();
    const Transform clean     = onManifold(joint, 0.62, "j8", "j9");

    const Quat q = clean.rotation();
    const Quat negated(-q.w(), -q.x(), -q.y(), -q.z());

    const Projection a = project(clean, joint, "j9_proj");
    const Projection b =
        project(frames::make("j8", "j9", clean.translation(), negated), joint, "j9_proj");

    EXPECT_NEAR(a.theta_rad, b.theta_rad, kTight);
    EXPECT_NEAR(b.theta_rad, 0.62, kTight);
}

// ---------------------------------------------------------------
// what the residuals actually measure
// ---------------------------------------------------------------

TEST(JointProjection, ErrorAboutTheJointAxisIsAbsorbedNotReported)
{
    // A rotation about the joint's own axis IS the joint moving. It must land in
    // theta, not in the residual -- otherwise the residual would grow simply
    // because the hand was being used.
    const RevoluteJoint joint = j9();
    const Transform clean     = onManifold(joint, 0.4, "j8", "j9");

    const Quat extra(Eigen::AngleAxisd(0.05, Vec3::UnitZ()));
    const Projection p = project(
        frames::make("j8", "j9", clean.translation(), clean.rotation() * extra), joint, "j9_proj");

    EXPECT_NEAR(p.theta_rad, 0.45, kLoose);
    EXPECT_NEAR(p.residual_rad, 0.0, kLoose);
}

TEST(JointProjection, ErrorPerpendicularToTheAxisIsReportedInFull)
{
    // A rotation the joint cannot produce must survive into residual_rad at its
    // full size -- this is the rigid-body identification error, isolated.
    const RevoluteJoint joint = j9();
    const Transform clean     = onManifold(joint, 0.4, "j8", "j9");

    const double tilt = frames::convert::degToRad(0.5);
    const Quat extra(Eigen::AngleAxisd(tilt, Vec3::UnitX()));   // perpendicular to Z

    const Projection p = project(
        frames::make("j8", "j9", clean.translation(), clean.rotation() * extra), joint, "j9_proj");

    EXPECT_NEAR(p.theta_rad, 0.4, kLoose);       // theta is untouched
    EXPECT_NEAR(p.residual_rad, tilt, kLoose);   // and the tilt is reported whole
}

TEST(JointProjection, ThetaIsIndependentOfPosition)
{
    // theta is recovered from the relative ROTATION alone -- position never
    // enters the solve, whether or not the axis runs through the child origin.
    // Move the measurement 5 mm and theta must not budge by one bit, while
    // residual_m picks up exactly the 5 mm.
    const RevoluteJoint joint = j9();
    const Transform clean     = onManifold(joint, 0.77, "j8", "j9");

    const Vec3 shift(0.003, -0.004, 0.0);   // 5 mm
    const Transform moved = frames::make("j8", "j9", clean.translation() + shift, clean.rotation());

    const Projection a = project(clean, joint, "j9_proj");
    const Projection b = project(moved, joint, "j9_proj");

    EXPECT_EQ(a.theta_rad, b.theta_rad);   // bit-identical, not merely close
    EXPECT_NEAR(a.residual_m, 0.0, kTight);
    EXPECT_NEAR(b.residual_m, 0.005, kLoose);
}

TEST(JointProjection, GeometryChecksSurviveADegenerateAngle)
{
    // The radial and axial errors never depended on theta, so an unrecoverable
    // angle must not cost us the geometry check -- which is the more important
    // of the two outputs. residual_m and residual_rad ARE lost, because both are
    // measured against a corrected pose that does not exist.
    const RevoluteJoint joint = j9();   // axis through the origin, so radius 0

    const Quat flip(Eigen::AngleAxisd(kPi, Vec3::UnitX()));   // 180 deg, perpendicular to Z
    const Projection p =
        project(frames::make("j8", "j9", Vec3(0.0, 0.280, 0.006), flip), joint, "j9_proj");

    ASSERT_TRUE(p.degenerate);
    EXPECT_NEAR(p.radial_error_m, 0.010, kLoose);   // 10 mm out from the axis
    EXPECT_NEAR(p.axial_error_m, 0.006, kLoose);    // 6 mm along it
}

// ---------------------------------------------------------------
// an off-axis child origin -- the real hardware's case
// ---------------------------------------------------------------

TEST(JointProjection, OffAxisOriginOrbitsRatherThanHoldingStill)
{
    // The bug the generalisation exists to prevent. A tracked body whose frame
    // is a fiducial 120 mm off its own joint's axis travels 170 mm over a 90
    // degree sweep. The OLD model predicted a fixed offset, so every millimetre
    // of that would have been charged to measurement error.
    const RevoluteJoint joint = offAxis();
    EXPECT_NEAR(joint.radius_m(), 0.120, kLoose);

    const Transform atZero   = joint.at("j8", "j9", 0.0);
    const Transform atNinety = joint.at("j8", "j9", kPi / 2.0);

    const double travelled = (atNinety.translation() - atZero.translation()).norm();
    EXPECT_NEAR(travelled, 2.0 * 0.120 * std::sin(kPi / 4.0), kLoose);   // 2 r sin(theta/2)
    EXPECT_GT(travelled, 0.16);
}

TEST(JointProjection, RecoversAngleWithAnOffAxisOrigin)
{
    const RevoluteJoint joint = offAxis();

    for (const double theta : {-1.5, -0.7, 0.0, 0.3, 1.5})
    {
        const Projection p = project(onManifold(joint, theta, "j8", "j9"), joint, "j9_proj");

        ASSERT_FALSE(p.degenerate);
        EXPECT_NEAR(p.theta_rad, theta, kTight) << "theta = " << theta;

        // A pose built exactly on the manifold has no error of any kind.
        EXPECT_NEAR(p.residual_m, 0.0, kLoose) << "theta = " << theta;
        EXPECT_NEAR(p.radial_error_m, 0.0, kLoose) << "theta = " << theta;
        EXPECT_NEAR(p.axial_error_m, 0.0, kLoose) << "theta = " << theta;
    }
}

TEST(JointProjection, GeometryChecksAreIndependentOfTheAngle)
{
    // The property that makes radial/axial the geometry check: sweep the joint
    // through its whole range with a FIXED error applied, and both must report
    // that same error at every angle. If either moved with theta it could not
    // distinguish a misplaced axis from the joint simply being used.
    const RevoluteJoint joint = offAxis();
    const Vec3 axis           = joint.axis;

    for (const double theta : {-1.5, -0.9, -0.2, 0.0, 0.4, 1.1, 1.5})
    {
        const Transform clean = onManifold(joint, theta, "j8", "j9");

        // Push 4 mm further from the axis and 3 mm along it -- in the frame the
        // errors are defined in, so the expected values are exact.
        const Vec3 d             = clean.translation() - joint.axis_point_m;
        const Vec3 radialDir     = (d - d.dot(axis) * axis).normalized();
        const Transform measured = frames::make(
            "j8", "j9", clean.translation() + 0.004 * radialDir + 0.003 * axis, clean.rotation());

        const Projection p = project(measured, joint, "j9_proj");

        ASSERT_FALSE(p.degenerate);
        EXPECT_NEAR(p.radial_error_m, 0.004, kLoose) << "theta = " << theta;
        EXPECT_NEAR(p.axial_error_m, 0.003, kLoose) << "theta = " << theta;
        EXPECT_NEAR(p.theta_rad, theta, kLoose) << "theta = " << theta;
    }
}

TEST(JointProjection, RadialAndAxialErrorsAreDistinguishable)
{
    // They diagnose different faults -- a misplaced axis versus a mis-directed
    // one -- so a purely radial error must not leak into the axial figure or
    // vice versa. A single blended residual would lose this.
    const RevoluteJoint joint = offAxis();
    const Transform clean     = onManifold(joint, 0.5, "j8", "j9");

    const Vec3 d         = clean.translation() - joint.axis_point_m;
    const Vec3 radialDir = (d - d.dot(joint.axis) * joint.axis).normalized();

    const Projection pureRadial =
        project(frames::make("j8", "j9", clean.translation() + 0.007 * radialDir, clean.rotation()),
                joint, "j9_proj");
    EXPECT_NEAR(pureRadial.radial_error_m, 0.007, kLoose);
    EXPECT_NEAR(pureRadial.axial_error_m, 0.0, kLoose);

    const Projection pureAxial = project(
        frames::make("j8", "j9", clean.translation() + 0.007 * joint.axis, clean.rotation()), joint,
        "j9_proj");
    EXPECT_NEAR(pureAxial.radial_error_m, 0.0, kLoose);
    EXPECT_NEAR(pureAxial.axial_error_m, 0.007, kLoose);
}

TEST(JointProjection, AnyPointOnTheAxisGivesTheSameAnswer)
{
    // Sliding axis_point along the axis must change nothing -- a rotation fixes
    // its own axis. This is what lets a user pick whichever point is easiest to
    // measure in CAD without it mattering which one they chose.
    RevoluteJoint a = offAxis();
    RevoluteJoint b = offAxis();
    b.axis_point_m += 0.35 * b.axis;   // 350 mm along the axis

    const Transform measured =
        frames::make("j8", "j9", Vec3(0.131, 0.244, 0.051),
                     Quat(Eigen::AngleAxisd(0.42, Vec3(0.1, -0.2, 0.97).normalized())));

    const Projection pa = project(measured, a, "j9_proj");
    const Projection pb = project(measured, b, "j9_proj");

    EXPECT_NEAR(pa.theta_rad, pb.theta_rad, kLoose);
    EXPECT_NEAR(pa.residual_m, pb.residual_m, kLoose);
    EXPECT_NEAR(pa.radial_error_m, pb.radial_error_m, kLoose);
    EXPECT_NEAR(pa.axial_error_m, pb.axial_error_m, kLoose);
}

TEST(JointProjection, AMisplacedAxisShowsUpAsRadialError)
{
    // The realistic failure: the CAD axis point is 8 mm off where the axis
    // really is. Build poses from the TRUE joint, project with the WRONG one,
    // and confirm the error surfaces rather than hiding in theta.
    const RevoluteJoint truth = offAxis();
    RevoluteJoint believed    = offAxis();
    believed.axis_point_m += Vec3(0.008, 0.0, 0.0);

    bool sawRadial = false;
    for (const double theta : {-1.2, -0.4, 0.6, 1.3})
    {
        const Projection p = project(onManifold(truth, theta, "j8", "j9"), believed, "j9_proj");
        ASSERT_FALSE(p.degenerate);
        if (std::fabs(p.radial_error_m) > 1e-4) sawRadial = true;
    }
    EXPECT_TRUE(sawRadial) << "an 8 mm axis error produced no radial signal";
}

// ---------------------------------------------------------------
// the degenerate case
// ---------------------------------------------------------------

TEST(JointProjection, DegenerateCaseIsRefused)
{
    // Exactly 180 degrees about an axis perpendicular to the joint axis. Every
    // theta fits equally well and atan2(0,0) would return a confident 0.0.
    const RevoluteJoint joint = j9();
    const Quat flip(Eigen::AngleAxisd(kPi, Vec3::UnitX()));

    const Projection p =
        project(frames::make("j8", "j9", joint.zero_origin_m, flip), joint, "j9_proj");

    EXPECT_TRUE(p.degenerate);
    EXPECT_NEAR(p.confidence, 0.0, 1e-12);
}

TEST(JointProjection, DegenerateResultCannotBeInstalledAsAFrame)
{
    // Ignoring `degenerate` must be loud. The corrected transform is left unset,
    // so its frame names are empty and the registry refuses it as a self-edge
    // rather than installing a fabricated pose.
    const RevoluteJoint joint = j9();
    const Quat flip(Eigen::AngleAxisd(kPi, Vec3::UnitY()));

    const Projection p =
        project(frames::make("j8", "j9", joint.zero_origin_m, flip), joint, "j9_proj");

    ASSERT_TRUE(p.degenerate);
    EXPECT_TRUE(p.corrected.to.empty());

    frames::Registry reg;
    EXPECT_THROW(reg.set(p.corrected), frames::FrameError);
}

TEST(JointProjection, ConfidenceFallsAsTheSignalDoes)
{
    // Approaching the singularity, theta is still defined but is being recovered
    // from less and less. confidence is what says so.
    const RevoluteJoint joint = j9();

    double previous = 1.1;
    for (const double tilt : {0.5, 1.5, 2.5, 3.0, 3.1})
    {
        const Quat q(Eigen::AngleAxisd(tilt, Vec3::UnitX()));
        const Projection p =
            project(frames::make("j8", "j9", joint.zero_origin_m, q), joint, "j9_proj");

        ASSERT_FALSE(p.degenerate) << "tilt = " << tilt;
        EXPECT_LT(p.confidence, previous) << "tilt = " << tilt;
        previous = p.confidence;
    }
}

// ---------------------------------------------------------------
// limits
// ---------------------------------------------------------------

TEST(JointProjection, ClampPicksTheAngularlyNearestLimit)
{
    // The case std::clamp gets wrong. With limits [2.0, 3.0], theta = -3.0 is
    // 0.28 rad from the UPPER limit and 1.28 from the lower; std::clamp would
    // return the lower and report the wrong end of the joint's travel.
    bool clamped = false;
    EXPECT_NEAR(clampToLimits(-3.0, 2.0, 3.0, clamped), 3.0, kLoose);
    EXPECT_TRUE(clamped);

    // Sanity: this is genuinely a case where the naive answer differs.
    EXPECT_NE(3.0, std::max(2.0, std::min(3.0, -3.0)));
}

TEST(JointProjection, ClampLeavesInRangeAnglesAlone)
{
    bool clamped = true;
    EXPECT_NEAR(clampToLimits(1.4, -1.5707, 1.5707, clamped), 1.4, kTight);
    EXPECT_FALSE(clamped);
}

TEST(JointProjection, CentredLimitsAgreeWithNaiveClamping)
{
    // For an interval centred on zero -- all three hand joints -- the wrap-aware
    // clamp and std::clamp agree for every principal value. Recorded so nobody
    // "simplifies" the wrap-aware version back out on the evidence of J8/J9/J10.
    for (const double theta : {-3.1, -1.6, -1.5, 0.0, 1.5, 1.6, 3.1})
    {
        bool clamped     = false;
        const double got = clampToLimits(theta, -1.5707, 1.5707, clamped);
        EXPECT_NEAR(got, std::max(-1.5707, std::min(1.5707, theta)), kLoose) << "theta = " << theta;
    }
}

TEST(JointProjection, WideLimitsNeverClamp)
{
    // J10's +/-360 deg span is wider than the (-pi, pi] the projection can
    // return, so `clamped` is structurally always false there. Reading "not
    // clamped" as "in range" is therefore wrong for this joint.
    const RevoluteJoint joint = j10();
    EXPECT_TRUE(joint.unconstrained());

    std::mt19937 rng(99);
    std::uniform_real_distribution<double> comp(-1.0, 1.0);

    for (int i = 0; i < 100; ++i)
    {
        const Quat q = Quat(comp(rng), comp(rng), comp(rng), comp(rng)).normalized();
        const Projection p =
            project(frames::make("j9", "j10", joint.zero_origin_m, q), joint, "j10_proj");

        ASSERT_FALSE(p.degenerate);
        EXPECT_FALSE(p.clamped);
        EXPECT_LE(p.theta_rad, kPi + kLoose);
        EXPECT_GE(p.theta_rad, -kPi - kLoose);
    }
}

TEST(JointProjection, ClampedResidualIsMeasuredAgainstWhatIsPublished)
{
    // A clamped projection publishes the limit, so the residual must describe
    // the limit -- not the discarded unclamped pose, which nothing downstream
    // ever sees.
    RevoluteJoint joint = j9();
    joint.lower_rad     = -0.2;
    joint.upper_rad     = 0.2;

    const Transform clean = onManifold(joint, 0.0, "j8", "j9");
    const Quat extra(Eigen::AngleAxisd(0.9, Vec3::UnitZ()));

    const Projection p = project(
        frames::make("j8", "j9", clean.translation(), clean.rotation() * extra), joint, "j9_proj");

    EXPECT_TRUE(p.clamped);
    EXPECT_NEAR(p.theta_raw_rad, 0.9, kLoose);
    EXPECT_NEAR(p.theta_rad, 0.2, kLoose);
    EXPECT_NEAR(p.residual_rad, 0.7, kLoose);   // 0.9 requested, 0.2 delivered
}

// ---------------------------------------------------------------
// wrapping, for comparison against a controller
// ---------------------------------------------------------------

TEST(JointProjection, WrapToNearPicksTheRightBranch)
{
    EXPECT_NEAR(wrapToNear(0.17, 6.28), 0.17 + kTwoPi, kLoose);
    EXPECT_NEAR(wrapToNear(3.2, 0.0), 3.2 - kTwoPi, kLoose);
    EXPECT_NEAR(wrapToNear(0.5, 0.4), 0.5, kLoose);   // already nearest
    EXPECT_NEAR(wrapToNear(-3.1, 3.1), -3.1 + kTwoPi, kLoose);
}

TEST(JointProjection, WrapToNearMakesJ10ComparableWithAControllerReading)
{
    // The concrete reason wrapToNear exists: a controller reporting 370 degrees
    // against a projection that necessarily produces 10 must read as 0 error.
    const double estimated = frames::convert::degToRad(10.0);
    const double reported  = frames::convert::degToRad(370.0);

    const double error = wrapToNear(estimated, reported) - reported;
    EXPECT_NEAR(frames::convert::radToDeg(error), 0.0, 1e-9);
}

// ---------------------------------------------------------------
// frame discipline
// ---------------------------------------------------------------

TEST(JointProjection, CorrectedPoseCarriesItsOwnFrameName)
{
    // The corrected pose is a SECOND claim about the child, not a replacement
    // for the measurement. Sharing a frame name would let one silently overwrite
    // the other, and the residual between them is the feature's output.
    const RevoluteJoint joint = j9();
    const Projection p        = project(onManifold(joint, 0.3, "j8", "j9"), joint, "j9_proj");

    EXPECT_EQ(p.corrected.to, "j8");
    EXPECT_EQ(p.corrected.from, "j9_proj");
}

TEST(JointProjection, RejectsAnUnnamedOrCollidingCorrectedFrame)
{
    const RevoluteJoint joint = j9();
    const Transform measured  = onManifold(joint, 0.3, "j8", "j9");

    EXPECT_THROW((void)project(measured, joint, ""), frames::FrameError);
    EXPECT_THROW((void)project(measured, joint, "j8"), frames::FrameError);
}

TEST(JointProjection, RejectsAZeroLengthAxis)
{
    RevoluteJoint joint = j9();
    joint.axis          = Vec3::Zero();

    EXPECT_THROW((void)joint.at("a", "b", 0.0), frames::FrameError);
}

TEST(JointProjection, NormalisesANonUnitAxis)
{
    RevoluteJoint joint = j9();
    joint.axis          = Vec3(0.0, 0.0, 4.0);   // right direction, wrong length

    const Projection p = project(onManifold(j9(), 0.55, "j8", "j9"), joint, "j9_proj");
    EXPECT_NEAR(p.theta_rad, 0.55, kLoose);
}

TEST(JointProjection, CommonFrameRejectsPosesInDifferentFrames)
{
    const RevoluteJoint joint = j9();
    const Transform a         = parentPose("mocap", "j8");
    const Transform b         = parentPose("some_other_world", "j9");

    EXPECT_THROW((void)projectInCommonFrame(a, b, joint, "j9_proj"), frames::FrameError);
}

TEST(JointProjection, CommonFrameRejectsAReversedInput)
{
    const RevoluteJoint joint = j9();
    const Transform a         = parentPose("mocap", "j8");

    // Same `to` frame, but this one is inverted -- the classic reversed-transform
    // mistake. compose() inside must catch it.
    EXPECT_THROW((void)projectInCommonFrame(a, frames::inverse(a), joint, "j9_proj"),
                 frames::FrameError);
}

// ---------------------------------------------------------------
// the whole chain, on the real hand
// ---------------------------------------------------------------

TEST(JointProjection, CommonFrameNeedsNoAnchor)
{
    // theta depends only on the RELATIVE rotation, so the same answer comes out
    // whatever frame the two bodies are measured in. This is what lets the
    // projection run before the scene anchor has latched.
    const RevoluteJoint joint  = j9();
    const Transform T_mocap_j8 = parentPose("mocap", "j8", 12.5);
    const Transform T_mocap_j9 = frames::compose(T_mocap_j8, onManifold(joint, 0.83, "j8", "j9"));

    const Projection p = projectInCommonFrame(T_mocap_j8, T_mocap_j9, joint, "j9_proj");

    EXPECT_NEAR(p.theta_rad, 0.83, kLoose);
    EXPECT_EQ(p.corrected.to, "j8");
    EXPECT_EQ(p.corrected.from, "j9_proj");
}

TEST(JointProjection, IsInvariantToTheMocapWorldOrigin)
{
    // Motive can be recalibrated between sessions, moving its world origin by an
    // unknown M. Both bodies move with it, so M cancels -- the same argument
    // object_placement.hpp makes for the anchor, restated for angles.
    const RevoluteJoint joint  = j9();
    const Transform T_mocap_j8 = parentPose("mocap", "j8");
    const Transform T_mocap_j9 = frames::compose(T_mocap_j8, onManifold(joint, -1.02, "j8", "j9"));

    const Transform M =
        frames::make("mocap2", "mocap", Vec3(-7.3, 2.9, 0.44),
                     Quat(Eigen::AngleAxisd(2.4, Vec3(0.5, 0.5, 0.7071).normalized())));

    const Projection before = projectInCommonFrame(T_mocap_j8, T_mocap_j9, joint, "j9_proj");
    const Projection after = projectInCommonFrame(frames::compose(M, T_mocap_j8),
                                                  frames::compose(M, T_mocap_j9), joint, "j9_proj");

    EXPECT_NEAR(before.theta_rad, after.theta_rad, kLoose);
    EXPECT_NEAR(before.residual_m, after.residual_m, kLoose);
    EXPECT_NEAR(before.residual_rad, after.residual_rad, kLoose);
}

TEST(JointProjection, RecoversBothAnglesOfTheRealTwoJointChain)
{
    // End to end on the actual hand geometry, in the order the placer will run
    // it: J9 from the raw J8 body, then J10 from the CORRECTED J9 -- not from
    // the J9 measurement.
    const RevoluteJoint jointA = j9();
    const RevoluteJoint jointB = j10();
    constexpr double kTheta9   = 0.61;
    constexpr double kTheta10  = -2.35;

    const Transform T_mocap_j8 = parentPose("mocap", "j8", 3.5);
    const Transform T_mocap_j9 =
        frames::compose(T_mocap_j8, onManifold(jointA, kTheta9, "j8", "j9"));
    const Transform T_mocap_j10 =
        frames::compose(T_mocap_j9, onManifold(jointB, kTheta10, "j9", "j10"));

    const Projection p9 = projectInCommonFrame(T_mocap_j8, T_mocap_j9, jointA, "j9_proj");
    ASSERT_FALSE(p9.degenerate);
    EXPECT_NEAR(p9.theta_rad, kTheta9, kLoose);

    const Transform T_mocap_j9proj = frames::compose(T_mocap_j8, p9.corrected);
    const Projection p10 = projectInCommonFrame(T_mocap_j9proj, T_mocap_j10, jointB, "j10_proj");

    ASSERT_FALSE(p10.degenerate);
    EXPECT_NEAR(p10.theta_rad, kTheta10, kLoose);
    EXPECT_NEAR(p10.residual_m, 0.0, kLoose);
    EXPECT_NEAR(p10.residual_rad, 0.0, kLoose);

    // The corrected chain reproduces the pose it was built from.
    const Transform T_mocap_j10proj = frames::compose(T_mocap_j9proj, p10.corrected);
    EXPECT_NEAR((T_mocap_j10proj.translation() - T_mocap_j10.translation()).norm(), 0.0, kLoose);
}

TEST(JointProjection, StampTravelsWithTheCorrectedPose)
{
    // Staleness has to survive the projection, because a corrected pose built
    // from an old measurement is exactly as old as that measurement.
    const RevoluteJoint joint = j9();
    const Transform measured =
        frames::make("j8", "j9", joint.zero_origin_m, Quat::Identity(), 42.0);

    EXPECT_EQ(project(measured, joint, "j9_proj").corrected.stamp, 42.0);
}

// ---------------------------------------------------------------
// recovering the one rotation CAD cannot supply
// ---------------------------------------------------------------
//
// zero_origin_m and axis_point_m are read off CAD, but they have to be expressed
// in a tracked body's frame -- and that frame is whatever Motive froze at
// creation. The two differ by a turn about the joint's own axis, which is the
// single component no drawing knows and no amount of measuring in CAD can fix.
//
// project() measures it. These pin that it is right, that it is the SAME at any
// joint angle (which is what makes it worth trusting), and that applying it
// actually closes the geometry.

namespace {

// The config as it would be written from CAD alone: correct except for one
// unknown turn about the joint's axis.
RevoluteJoint turnedAboutItsAxis(const RevoluteJoint& truth, double yaw)
{
    const Eigen::AngleAxisd Y(yaw, truth.axis.normalized());
    RevoluteJoint out = truth;
    out.zero_origin_m = Y.inverse() * truth.zero_origin_m;
    out.axis_point_m  = Y.inverse() * truth.axis_point_m;
    return out;
}

}   // namespace

TEST(JointProjection, RecoversTheYawTheConfigOwes)
{
    const RevoluteJoint truth        = offAxis();
    constexpr double kYaw            = 0.6;   // ~34 degrees of unknown turn
    const RevoluteJoint asConfigured = turnedAboutItsAxis(truth, kYaw);

    // A real measurement, generated from the TRUE geometry.
    const double theta       = 0.4;
    const Transform measured = truth.at("parent", "child", theta);

    const Projection p = project(measured, asConfigured, "child");
    ASSERT_FALSE(p.degenerate);
    ASSERT_TRUE(p.yaw_observable);

    EXPECT_NEAR(p.yaw_to_align_rad, kYaw, 1e-9);

    // theta is untouched by the mis-set geometry, because it never reads either
    // of the two fields the yaw corrects. This is the claim the whole "angles
    // are correct as shipped" position rests on.
    EXPECT_NEAR(p.theta_rad, theta, 1e-9);

    // ...and the position half is wrong until it is applied, by a lot.
    EXPECT_GT(p.residual_m, 0.05);
}

TEST(JointProjection, TheSameYawComesBackAtEveryJointAngle)
{
    // The property that makes this measurable at all: the unknown turn commutes
    // with the joint's own rotation, so it factors out of the model and does not
    // depend on where the joint happens to be sitting. Without this you could
    // not tell a mis-set config from a moving joint.
    const RevoluteJoint truth        = offAxis();
    constexpr double kYaw            = -0.9;
    const RevoluteJoint asConfigured = turnedAboutItsAxis(truth, kYaw);

    for (double theta : {-1.4, -0.7, -0.05, 0.0, 0.3, 1.1, 1.5})
    {
        const Projection p = project(truth.at("parent", "child", theta), asConfigured, "child");
        ASSERT_TRUE(p.yaw_observable) << "theta = " << theta;
        EXPECT_NEAR(p.yaw_to_align_rad, kYaw, 1e-9) << "theta = " << theta;
    }
}

TEST(JointProjection, ApplyingTheYawClosesTheGeometry)
{
    // End to end: measure the yaw, rotate the two fields by it, and the residual
    // collapses to nothing. That is the whole commissioning loop -- run it, read
    // the number, paste it back.
    const RevoluteJoint truth = offAxis();
    constexpr double kYaw     = 1.25;
    RevoluteJoint cfg         = turnedAboutItsAxis(truth, kYaw);

    const double theta       = -0.55;
    const Transform measured = truth.at("parent", "child", theta);

    const double recovered = project(measured, cfg, "child").yaw_to_align_rad;

    const Eigen::AngleAxisd Y(recovered, cfg.axis.normalized());
    cfg.zero_origin_m = Y * cfg.zero_origin_m;
    cfg.axis_point_m  = Y * cfg.axis_point_m;

    const Projection closed = project(measured, cfg, "child");
    EXPECT_NEAR(closed.residual_m, 0.0, 1e-9);
    EXPECT_NEAR(closed.theta_rad, theta, 1e-9);
    EXPECT_NEAR(closed.yaw_to_align_rad, 0.0, 1e-9);   // nothing left to correct
}

TEST(JointProjection, YawIsNotObservableWhenTheChildSitsStraightUpTheAxis)
{
    // The one arrangement that carries no information about the turn: the child
    // frame lies on the line through the PARENT's origin in the axis direction,
    // so its position has no component across the axis for a turn about the axis
    // to move. Reported as unobservable rather than as a confident zero -- the
    // same discipline as the degenerate angle.
    //
    // Note this is NOT the same as "the body sits on its own hinge". A body on
    // the hinge does not ORBIT, but its offset from the parent origin still
    // turns, so the yaw stays perfectly observable -- which is worth knowing,
    // because it is the hand's J9 case.
    RevoluteJoint j;
    j.axis          = Vec3::UnitZ();
    j.axis_point_m  = Vec3(0.0, 0.0, 0.0);
    j.zero_origin_m = Vec3(0.0, 0.0, 0.100);   // straight up the axis
    j.lower_rad     = -1.5707;
    j.upper_rad     = 1.5707;

    const Projection p = project(j.at("parent", "child", 0.3), j, "child");
    EXPECT_FALSE(p.yaw_observable);
    EXPECT_DOUBLE_EQ(p.yaw_to_align_rad, 0.0);
}

TEST(JointProjection, YawStaysObservableForABodyOnItsOwnHinge)
{
    // J9's real shape: zero_origin_m sits ON the axis, so the body does not orbit
    // -- and the yaw is still fully recoverable, because what turns is its offset
    // from the PARENT's origin, not its offset from the hinge.
    const RevoluteJoint truth = j9();
    constexpr double kYaw     = 0.42;

    const Projection p =
        project(truth.at("parent", "child", 0.2), turnedAboutItsAxis(truth, kYaw), "child");

    ASSERT_TRUE(p.yaw_observable);
    EXPECT_NEAR(p.yaw_to_align_rad, kYaw, 1e-9);
}
