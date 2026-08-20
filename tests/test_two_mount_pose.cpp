// Tests for nodes/common/two_mount_pose.hpp
//
// The construction is one function, so these are its correctness argument. It
// serves two parts whose geometry is arranged completely differently -- the rail
// (chord ALONG the axis, 10 m of it) and the rotor (chord ACROSS the spin axis)
// -- so both are exercised here, and neither is allowed to be a special case in
// the code.
//
// The ones carrying the weight:
//
//   RotorRoundTrip / RailRoundTrip
//       the whole thing, end to end, on real-scale geometry. Every sign
//       convention in the file is wrong if either fails.
//   EachBodySpinAboutItsOwnNormalIsIrrelevant
//       pins the claim that only the NORMAL of each body is used, never its
//       full orientation -- which is why this needs one assertion, not three.
//   ChordFixesEverythingButTheRollAboutIt / RollAboutTheChordSurvives
//       the pair that pins exactly which part of the orientation the chord
//       determines and which part it cannot. Either alone would pass with the
//       Gram-Schmidt step deleted or applied backwards.
//   AWrongBodyAxisAtTheSAMEAngleToTheChordIsInvisible, and the ExpectedNormal
//   group that answers it
//       the limit of what the geometry can check itself, then the one field that
//       reaches past it. Read them together: the first is the failure, the rest
//       are the detection, and neither means much alone.
//
// Eigen's arithmetic is not retested here; it is not ours to verify.

#include <gtest/gtest.h>

#include "two_mount_pose.hpp"

#include <algorithm>
#include <cmath>

using namespace twomount;
using frames::Quat;
using frames::Transform;
using frames::Vec3;

namespace {

constexpr double kPi    = 3.14159265358979323846;
constexpr double kLoose = 1e-9;

// A part pose aligned with nothing, so no sign error can hide behind a zero.
Transform partTruth()
{
    return frames::make("parent", "part", Vec3(1.234, -0.567, 2.345),
                        Quat(Eigen::AngleAxisd(0.7, Vec3(0.3, -0.5, 0.81).normalized())));
}

// A rig is: the CAD the config would carry, the truth it is being asked to
// recover, and the two body poses Motive would report.
struct Rig {
    Geometry  geometry;
    Transform truth;
    Transform a;
    Transform b;
};

// Places two bodies on a part at the given CAD points, each oriented so that its
// OWN +Z points along `normalInPart`, plus an arbitrary spin about that -- which
// is the part of a Motive body's orientation nobody controls.
Rig rig(const Vec3& mountA, const Vec3& mountB, const Vec3& normalInPart,
        double spinA = 0.0, double spinB = 0.0)
{
    Rig r;
    r.geometry.normal_axis    = Vec3::UnitZ();
    r.geometry.normal_in_part = normalInPart;
    r.geometry.mount_a_m      = mountA;
    r.geometry.mount_b_m      = mountB;
    r.truth                   = partTruth();

    // Body +Z onto normalInPart, then spin about it.
    const Quat align =
        Quat::FromTwoVectors(Vec3::UnitZ(), normalInPart.normalized());
    const auto bodyRot = [&](double spin) {
        return Quat(Eigen::AngleAxisd(spin, normalInPart.normalized())) * align;
    };

    r.a = frames::compose(r.truth, frames::make("part", "a", mountA, bodyRot(spinA)));
    r.b = frames::compose(r.truth, frames::make("part", "b", mountB, bodyRot(spinB)));
    return r;
}

// The rotor: two mounts in blade grooves on one disk face, 90 degrees apart at a
// 700 mm radius, on the face 1.83 m out from the axial midpoint. Part frame is
// +X down the spin axis, per the 'Making a Site' convention, so the mounts'
// shared face normal points along +X.
Rig rotorRig(double phi = kPi / 2.0, double spinA = 0.0, double spinB = 0.0)
{
    constexpr double r = 0.7, faceX = 1.83;
    return rig({faceX, 0.0, r},
               {faceX, -r * std::sin(phi), r * std::cos(phi)},
               Vec3::UnitX(), spinA, spinB);
}

// The rail: one body at each end, both on the centreline seen from above, 50 mm
// above the rail's own origin plane. Part frame is +X along the rail, +Z up, so
// the bodies' shared face normal points along +Z.
Rig railRig(double spinA = 0.0, double spinB = 0.0)
{
    return rig({-5.0, 0.0, 0.05}, {5.0, 0.0, 0.05}, Vec3::UnitZ(), spinA, spinB);
}

void expectSamePose(const Transform& got, const Transform& want, double tol)
{
    EXPECT_NEAR((got.translation() - want.translation()).norm(), 0.0, tol);
    EXPECT_NEAR(got.rotation().angularDistance(want.rotation()), 0.0, tol);
}

// Rotate a body's ORIENTATION in place, leaving its position untouched. This is
// what a body not seated flat looks like, and what solve noise looks like.
Transform tilted(const Transform& t, double angle, const Vec3& axis)
{
    return frames::make(t.to, t.from, t.translation(),
                        Quat(Eigen::AngleAxisd(angle, axis.normalized())) * t.rotation(),
                        t.stamp);
}

}  // namespace

// ---------------------------------------------------------------
// round trips -- both parts, same code path
// ---------------------------------------------------------------

TEST(TwoMountPose, RotorRoundTrip)
{
    // Chord ACROSS the spin axis. The part's origin is ~1.9 m from either mount
    // and on neither side of the chord by accident.
    const Rig r = rotorRig();
    const Construction c = construct(r.a, r.b, r.geometry, "part");

    ASSERT_TRUE(c.constructed) << c.skip_reason;
    expectSamePose(c.pose, r.truth, kLoose);

    EXPECT_EQ(c.pose.to, "parent");
    EXPECT_EQ(c.pose.from, "part");

    EXPECT_NEAR(c.chord_error_m, 0.0, kLoose);
    EXPECT_NEAR(c.normal_disagreement_rad, 0.0, kLoose);
    EXPECT_NEAR(c.chord_out_of_plane_m, 0.0, kLoose);
}

TEST(TwoMountPose, RailRoundTrip)
{
    // Chord ALONG the axis, 10 m of it. Same function, no special case.
    const Rig r = railRig();
    const Construction c = construct(r.a, r.b, r.geometry, "part");

    ASSERT_TRUE(c.constructed) << c.skip_reason;
    expectSamePose(c.pose, r.truth, kLoose);

    EXPECT_NEAR(c.chord_error_m, 0.0, kLoose);
    EXPECT_NEAR(c.normal_disagreement_rad, 0.0, kLoose);
    EXPECT_NEAR(c.chord_out_of_plane_m, 0.0, kLoose);
    EXPECT_NEAR(r.geometry.chord_length_m(), 10.0, kLoose);
}

TEST(TwoMountPose, EachBodySpinAboutItsOwnNormalIsIrrelevant)
{
    // Motive picks each body's orientation at creation time and nobody controls
    // it. Only the face normal is used, so spinning either body about its own
    // normal must change nothing -- which is the whole reason this construction
    // asserts one axis rather than a full rotation it could not check.
    for (const Rig& spun : {rotorRig(kPi / 2.0, 1.9, -2.7), railRig(2.4, -0.8)})
    {
        const Construction c = construct(spun.a, spun.b, spun.geometry, "part");
        ASSERT_TRUE(c.constructed) << c.skip_reason;
        expectSamePose(c.pose, spun.truth, kLoose);
    }
}

TEST(TwoMountPose, MountsNeedNotBeSymmetricAboutThePartOrigin)
{
    // Nothing requires the two bodies to be at equal reach or arranged
    // symmetrically -- the CAD points carry whatever arrangement is real. Here
    // one body is four times further from the origin than the other.
    const Rig r = rig({0.2, 0.0, 0.05}, {3.0, 0.0, 0.05}, Vec3::UnitZ());
    const Construction c = construct(r.a, r.b, r.geometry, "part");

    ASSERT_TRUE(c.constructed) << c.skip_reason;
    expectSamePose(c.pose, r.truth, kLoose);
}

TEST(TwoMountPose, WorksWhenTheNormalIsNotPerpendicularToTheChord)
{
    // The construction compares the MEASURED chord-to-normal angle against the
    // CAD one rather than assuming ninety degrees, so a part whose bodies are on
    // a sloped face is handled without a special case -- and its
    // chord_out_of_plane still reads zero, because it is measuring a
    // disagreement, not an angle.
    const Rig r = rig({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, Vec3(0.5, 0.0, 1.0));
    const Construction c = construct(r.a, r.b, r.geometry, "part");

    ASSERT_TRUE(c.constructed) << c.skip_reason;
    expectSamePose(c.pose, r.truth, kLoose);
    EXPECT_NEAR(c.chord_out_of_plane_m, 0.0, kLoose);
}

// ---------------------------------------------------------------
// what the chord fixes, and what it cannot
// ---------------------------------------------------------------

TEST(TwoMountPose, ChordFixesEverythingButTheRollAboutIt)
{
    // Tilt BOTH bodies' normals in the plane containing the chord, by far more
    // than any real seat error. The chord's direction is taken as authoritative,
    // so this entire error is removable -- and must be removed, recovering the
    // part pose exactly.
    const Rig r = railRig();

    const Vec3 chordHat = (r.b.translation() - r.a.translation()).normalized();
    const Vec3 up       = r.truth.rotation() * Vec3::UnitZ();
    const Vec3 across   = chordHat.cross(up);          // tilts `up` toward the chord

    constexpr double kTilt = 0.02;   // ~1.1 degrees, ~20x a real seat error
    const Construction c = construct(tilted(r.a, kTilt, across), tilted(r.b, kTilt, across),
                                     r.geometry, "part");

    ASSERT_TRUE(c.constructed) << c.skip_reason;
    expectSamePose(c.pose, r.truth, kLoose);

    // Visible while being corrected: chord * sin(tilt).
    EXPECT_NEAR(std::abs(c.chord_out_of_plane_m),
                r.geometry.chord_length_m() * std::sin(kTilt), 1e-6);
}

TEST(TwoMountPose, RollAboutTheChordSurvives)
{
    // The complement, and the honest half. Rotating both normals ABOUT the chord
    // leaves their relationship to it unchanged, so the chord sees nothing and
    // the error passes straight through into the part's orientation.
    const Rig r = railRig();
    const Vec3 chordHat = (r.b.translation() - r.a.translation()).normalized();

    constexpr double kTilt = 0.02;
    const Construction c = construct(tilted(r.a, kTilt, chordHat), tilted(r.b, kTilt, chordHat),
                                     r.geometry, "part");

    ASSERT_TRUE(c.constructed) << c.skip_reason;

    // Invisible to the checks that catch the other half.
    EXPECT_NEAR(c.chord_out_of_plane_m, 0.0, kLoose);
    EXPECT_NEAR(c.normal_disagreement_rad, 0.0, kLoose);

    // And it lands as exactly that much roll about the chord.
    const Eigen::AngleAxisd err(c.pose.rotation() * r.truth.rotation().conjugate());
    EXPECT_NEAR(err.angle(), kTilt, 1e-9);
    EXPECT_NEAR(std::abs(err.axis().dot(chordHat)), 1.0, 1e-9);
}

TEST(TwoMountPose, SurvivingRollCostsTheOriginItsOffsetFromTheChord)
{
    // What that roll is worth in millimetres. The origin swings about the chord
    // by its own perpendicular distance from it -- which for the rail is the
    // 50 mm the bodies sit above the origin plane, and for the rotor is the
    // ~1.9 m to the axial midpoint. Pinned here so the two are not confused:
    // the lever arm that matters is the distance to the CHORD, not to a body.
    const double tilt = 0.1 * kPi / 180.0;

    const auto rollErrorFor = [&](const Rig& r) {
        const Vec3 chordHat = (r.b.translation() - r.a.translation()).normalized();
        const Construction c = construct(tilted(r.a, tilt, chordHat), tilted(r.b, tilt, chordHat),
                                         r.geometry, "part");
        EXPECT_TRUE(c.constructed);
        return (c.pose.translation() - r.truth.translation()).norm();
    };

    // Rail: the origin is 50 mm off the chord, so 0.1 degrees costs ~0.09 mm.
    EXPECT_LT(rollErrorFor(railRig()), 0.0002);

    // Rotor: the origin is ~1.9 m off the chord, so the same tilt costs ~3.3 mm.
    const double rotorErr = rollErrorFor(rotorRig());
    EXPECT_GT(rotorErr, 0.0030);
    EXPECT_LT(rotorErr, 0.0035);
}

// ---------------------------------------------------------------
// the checks
// ---------------------------------------------------------------

TEST(TwoMountPose, ChordErrorCatchesAMiscountedGroove)
{
    // The failure this construction exposes on the rotor: the CAD mount points
    // depend on WHICH grooves the mounts sit in, so a miscount corrupts the
    // origin. It has to be caught, and this is what catches it.
    constexpr double kPhi  = kPi / 2.0;
    constexpr double kSlip = 10.0 * kPi / 180.0;   // one groove over

    const Rig believed = rotorRig(kPhi);
    const Rig actual   = rotorRig(kPhi + kSlip);   // where mount B really is

    // Measured from the real hardware, checked against the believed CAD.
    const Construction c = construct(actual.a, actual.b, believed.geometry, "part");
    ASSERT_TRUE(c.constructed) << c.skip_reason;

    EXPECT_NEAR(c.chord_error_m,
                actual.geometry.chord_length_m() - believed.geometry.chord_length_m(), 1e-12);
    EXPECT_GT(std::abs(c.chord_error_m), 0.07);   // 70+ mm: unmissable

    // The other two stay clean, which is why three are reported rather than one:
    // both mounts are still seated flat on the same face. Only the chord knows.
    EXPECT_NEAR(c.normal_disagreement_rad, 0.0, kLoose);
    EXPECT_NEAR(c.chord_out_of_plane_m, 0.0, kLoose);

    // And the pose it produces is smooth, stable and wrong.
    EXPECT_GT((c.pose.translation() - actual.truth.translation()).norm(), 0.02);
}

TEST(TwoMountPose, NormalDisagreementCatchesABodyNotSeatedFlat)
{
    const Rig r = railRig();
    const Vec3 chordHat = (r.b.translation() - r.a.translation()).normalized();

    constexpr double kRock = 0.5 * kPi / 180.0;
    const Construction c = construct(r.a, tilted(r.b, kRock, chordHat), r.geometry, "part");

    ASSERT_TRUE(c.constructed) << c.skip_reason;
    EXPECT_NEAR(c.normal_disagreement_rad, kRock, 1e-9);
    EXPECT_NEAR(c.chord_error_m, 0.0, kLoose);   // it tilted; it did not MOVE
}

TEST(TwoMountPose, NormalDisagreementResolvesAnglesFarBelowWhatAcosCan)
{
    // A CONDITIONING test, not a geometry one, and it exists because the obvious
    // implementation of "angle between two directions" silently cannot do this.
    //
    // acos(a.b) has an unbounded derivative at 1. At the tilt below the dot
    // product is 1 - 5e-19, which rounds to exactly 1.0 in double, so acos
    // returns exactly ZERO -- a check reporting perfect agreement between two
    // bodies that genuinely disagree. The same rounding runs the other way too:
    // two identical directions that differ by one ulp of representation come
    // back as ~3e-8 rad, a floor beneath every reading.
    //
    // Both are asserted here, so a future simplification back to acos fails
    // rather than quietly reintroducing the floor.
    constexpr double kTiny = 1e-9;

    const Rig r = railRig();

    // About an axis perpendicular to the normal, so the normal moves by the full
    // tilt rather than some cosine of it.
    const Vec3 perp = r.truth.rotation() * Vec3::UnitX();
    const Transform tiltedB = tilted(r.b, kTiny, perp);

    const Construction c = construct(r.a, tiltedB, r.geometry, "part");
    ASSERT_TRUE(c.constructed) << c.skip_reason;
    EXPECT_NEAR(c.normal_disagreement_rad, kTiny, kTiny * 0.01);

    // The form this replaced, on the very same inputs. Not a hypothetical.
    const Vec3 nA = r.a.rotation() * Vec3::UnitZ();
    const Vec3 nB = tiltedB.rotation() * Vec3::UnitZ();
    EXPECT_EQ(std::acos(std::clamp(nA.dot(nB), -1.0, 1.0)), 0.0)
        << "acos was expected to lose this angle entirely; if it no longer does, this "
           "test has stopped demonstrating anything and needs a smaller tilt";
}

TEST(TwoMountPose, PerfectlySeatedBodiesReportExactlyNoDisagreement)
{
    // The other half. The rotor rig orients its bodies through a real quaternion
    // (its normal is the part's +X, so FromTwoVectors is not identity), which
    // leaves the two computed normals differing by an ulp or so. Through acos
    // that came back as 2.98e-8 rad -- indistinguishable from a mount genuinely
    // out of flat by that much, and the reason three tests in this file used to
    // fail on the rotor while passing on the rail.
    for (const Rig& r : {rotorRig(), railRig(), rotorRig(kPi / 3.0)})
    {
        const Construction c = construct(r.a, r.b, r.geometry, "part");
        ASSERT_TRUE(c.constructed) << c.skip_reason;
        EXPECT_LT(c.normal_disagreement_rad, 1e-12);
    }
}

TEST(TwoMountPose, ChordOutOfPlaneCatchesAWrongBodyAxisAtADifferentAngle)
{
    // The case normal_disagreement_rad structurally cannot see: both bodies name
    // the same wrong body axis, so they agree with each other perfectly and are
    // both wrong. On the rotor the wrong axis sits 45 degrees off the chord where
    // the right one sits at 90, so the disagreement is 0.7 m and unmissable.
    const Rig r = rotorRig();

    Geometry wrong    = r.geometry;
    wrong.normal_axis = Vec3::UnitX();   // the body's own X, not its face normal

    const Construction c = construct(r.a, r.b, wrong, "part");

    ASSERT_TRUE(c.constructed) << c.skip_reason;
    EXPECT_NEAR(c.normal_disagreement_rad, 0.0, kLoose);   // agreeing, and useless
    EXPECT_GT(std::abs(c.chord_out_of_plane_m), 0.5);
}

TEST(TwoMountPose, AWrongBodyAxisAtTheSAMEAngleToTheChordIsInvisible)
{
    // THE LIMIT OF THE CHECKS, and it has to be written down rather than
    // discovered. chord_out_of_plane compares the measured chord-to-normal angle
    // against the CAD one, so a wrong axis is caught only when it sits at a
    // DIFFERENT angle to the chord than the right one.
    //
    // The rail is exactly where that fails. Its bodies' +Z (up) and +Y (across)
    // are both perpendicular to the chord, so naming +Y instead of +Z leaves all
    // three checks reading clean while rolling the whole rail 90 degrees about
    // its own length -- and the rail is the anchor, so that rolls the scene.
    //
    // No INTERNAL check can catch this, and that is a property of internal
    // checks rather than a gap in these three: all of them compare measured
    // geometry against CAD geometry, and a wrong axis named the same way on both
    // bodies is wrong on both sides, so it cancels. Catching it needs a fact from
    // outside the part -- see the ExpectedNormal tests below, which is the same
    // rig with expect_normal_in_parent set.
    const Rig r = railRig();

    Geometry wrong    = r.geometry;
    wrong.normal_axis = Vec3::UnitY();

    const Construction c = construct(r.a, r.b, wrong, "part");
    ASSERT_TRUE(c.constructed) << c.skip_reason;

    // Every check clean...
    EXPECT_NEAR(c.chord_error_m, 0.0, kLoose);
    EXPECT_NEAR(c.normal_disagreement_rad, 0.0, kLoose);
    EXPECT_NEAR(c.chord_out_of_plane_m, 0.0, kLoose);

    // ...and nothing was checked, which must not read as nothing being wrong.
    EXPECT_FALSE(c.normal_in_parent_checked);

    // ...and the pose rolled by exactly a right angle about the rail's length.
    const Eigen::AngleAxisd err(c.pose.rotation() * r.truth.rotation().conjugate());
    EXPECT_NEAR(err.angle(), kPi / 2.0, 1e-9);
    const Vec3 chordHat = (r.b.translation() - r.a.translation()).normalized();
    EXPECT_NEAR(std::abs(err.axis().dot(chordHat)), 1.0, 1e-9);
}

// ---------------------------------------------------------------
// expect_normal_in_parent -- the one check that looks outside the part
// ---------------------------------------------------------------
//
// These are the companions to the test above. Same rail rig, same wrong axis,
// and now an expectation about where the part's normal must point in the parent
// frame -- which the room supplies for free whenever the part's attitude is
// already known (a rail is level; Motive's ground plane says which way is up).
//
// The two failures it catches are not small. They are 90 degrees and 180
// degrees, which is why the threshold that judges it can afford to be loose.

TEST(TwoMountPose, ExpectedNormalIsCleanWhenTheAxisIsRight)
{
    Rig r = railRig();
    r.geometry.expect_normal_in_parent = r.truth.rotation() * Vec3::UnitZ();

    const Construction c = construct(r.a, r.b, r.geometry, "part");
    ASSERT_TRUE(c.constructed) << c.skip_reason;

    EXPECT_TRUE(c.normal_in_parent_checked);
    EXPECT_NEAR(c.normal_in_parent_error_rad, 0.0, kLoose);

    // And it did not disturb anything it has no business touching.
    expectSamePose(c.pose, r.truth, kLoose);
}

TEST(TwoMountPose, ExpectedNormalCatchesTheWrongBodyAxisOnARail)
{
    // THE POINT OF THE WHOLE FIELD. This is the exact configuration that
    // AWrongBodyAxisAtTheSAMEAngleToTheChordIsInvisible leaves undetected: a rail
    // body's 'across' named where its 'up' was meant. All three geometry checks
    // read clean there. Here it comes back as a right angle.
    const Rig r = railRig();

    Geometry wrong                   = r.geometry;
    wrong.normal_axis                = Vec3::UnitY();
    wrong.expect_normal_in_parent    = r.truth.rotation() * Vec3::UnitZ();

    const Construction c = construct(r.a, r.b, wrong, "part");
    ASSERT_TRUE(c.constructed) << c.skip_reason;   // reported, never fatal

    EXPECT_NEAR(c.chord_error_m, 0.0, kLoose);
    EXPECT_NEAR(c.normal_disagreement_rad, 0.0, kLoose);
    EXPECT_NEAR(c.chord_out_of_plane_m, 0.0, kLoose);

    EXPECT_TRUE(c.normal_in_parent_checked);
    EXPECT_NEAR(c.normal_in_parent_error_rad, kPi / 2.0, 1e-9);
}

TEST(TwoMountPose, ExpectedNormalCatchesAFlippedSign)
{
    // The other documented blind spot, and the reason the error is reported as an
    // unsigned angle rather than a dot product: a flipped normal_axis lands at
    // 180 degrees, which is as far from agreement as it is possible to be and
    // would read as a large positive alignment if it were signed the other way.
    const Rig r = railRig();

    Geometry flipped                = r.geometry;
    flipped.normal_axis             = -Vec3::UnitZ();
    flipped.expect_normal_in_parent = r.truth.rotation() * Vec3::UnitZ();

    const Construction c = construct(r.a, r.b, flipped, "part");
    ASSERT_TRUE(c.constructed) << c.skip_reason;

    // Once again the three internal checks see nothing: both bodies flipped
    // together, so they still agree with each other and with the CAD chord.
    EXPECT_NEAR(c.chord_error_m, 0.0, kLoose);
    EXPECT_NEAR(c.normal_disagreement_rad, 0.0, kLoose);
    EXPECT_NEAR(c.chord_out_of_plane_m, 0.0, kLoose);

    EXPECT_TRUE(c.normal_in_parent_checked);
    EXPECT_NEAR(c.normal_in_parent_error_rad, kPi, 1e-9);
}

TEST(TwoMountPose, ExpectedNormalWorksOnTheRotorToo)
{
    // Not a rail-only feature. A rotor whose spin axis attitude the room happens
    // to know gets the same protection, and the rotor's own blind spot -- the
    // SIGN -- is the one chord_out_of_plane could never see either.
    const Rig r = rotorRig();

    Geometry flipped                = r.geometry;
    flipped.normal_axis             = -Vec3::UnitZ();
    flipped.expect_normal_in_parent = r.truth.rotation() * Vec3::UnitX();

    const Construction c = construct(r.a, r.b, flipped, "part");
    ASSERT_TRUE(c.constructed) << c.skip_reason;

    EXPECT_TRUE(c.normal_in_parent_checked);
    EXPECT_NEAR(c.normal_in_parent_error_rad, kPi, 1e-9);
}

TEST(TwoMountPose, AnUnsetExpectationReportsUncheckedRatherThanZero)
{
    // The distinction the bool exists for. A construction with no expectation
    // configured has an error of 0.0, and a construction that passed also has an
    // error of 0.0 -- so the number alone cannot tell them apart, and anything
    // reading it without the flag would report an unchecked axis as a verified
    // one.
    const Rig r = railRig();
    ASSERT_FALSE(r.geometry.expect_normal_in_parent.has_value());

    const Construction c = construct(r.a, r.b, r.geometry, "part");
    ASSERT_TRUE(c.constructed) << c.skip_reason;

    EXPECT_FALSE(c.normal_in_parent_checked);
    EXPECT_NEAR(c.normal_in_parent_error_rad, 0.0, kLoose);
}

TEST(TwoMountPose, ADegenerateExpectationIsAnAbsentCheckNotAFailure)
{
    // A zero-length expectation names no direction, so there is nothing to
    // compare against. It leaves the check absent rather than refusing the
    // construction: the pose is unaffected by this field, and withholding a good
    // pose over a bad diagnostic would trade a real output for a cosmetic one.
    Rig r = railRig();
    r.geometry.expect_normal_in_parent = Vec3::Zero();

    const Construction c = construct(r.a, r.b, r.geometry, "part");
    ASSERT_TRUE(c.constructed) << c.skip_reason;

    EXPECT_FALSE(c.normal_in_parent_checked);
    expectSamePose(c.pose, r.truth, kLoose);
}

TEST(TwoMountPose, ChordErrorIsNotFudgedIntoThePose)
{
    // With a chord error the CAD cannot be satisfied exactly. The discrepancy is
    // split symmetrically about the midpoint rather than charged entirely to
    // whichever body happens to be named second -- so which body is `a` and which
    // is `b` cannot change the answer.
    const Rig believed = railRig();

    Rig stretched = railRig();
    stretched.geometry = believed.geometry;                     // CAD unchanged
    stretched.b = frames::make(stretched.b.to, stretched.b.from,
                               stretched.b.translation() +
                                   (stretched.truth.rotation() * Vec3(0.01, 0, 0)),
                               stretched.b.rotation());

    const Construction ab = construct(stretched.a, stretched.b, believed.geometry, "part");
    ASSERT_TRUE(ab.constructed);
    EXPECT_NEAR(ab.chord_error_m, 0.01, 1e-9);

    // Swap both the bodies and the CAD points, which describes the identical
    // hardware. The pose must be bit-comparable.
    Geometry swapped     = believed.geometry;
    swapped.mount_a_m    = believed.geometry.mount_b_m;
    swapped.mount_b_m    = believed.geometry.mount_a_m;

    const Construction ba = construct(stretched.b, stretched.a, swapped, "part");
    ASSERT_TRUE(ba.constructed);
    expectSamePose(ba.pose, ab.pose, 1e-12);
}

// ---------------------------------------------------------------
// refusals
// ---------------------------------------------------------------

TEST(TwoMountPose, ZeroNormalAxisIsRefused)
{
    const Rig r = railRig();
    Geometry g  = r.geometry;
    g.normal_axis = Vec3::Zero();

    const Construction c = construct(r.a, r.b, g, "part");
    EXPECT_FALSE(c.constructed);
    EXPECT_NE(c.skip_reason.find("normal_axis"), std::string::npos);

    // The pose is left UNSET so it cannot be installed by accident.
    EXPECT_TRUE(c.pose.to.empty());
    EXPECT_TRUE(c.pose.from.empty());
}

TEST(TwoMountPose, ZeroNormalInPartIsRefused)
{
    const Rig r = railRig();
    Geometry g  = r.geometry;
    g.normal_in_part = Vec3::Zero();

    const Construction c = construct(r.a, r.b, g, "part");
    EXPECT_FALSE(c.constructed);
    EXPECT_NE(c.skip_reason.find("normal_in_part"), std::string::npos);
}

TEST(TwoMountPose, CoincidentCadPointsAreRefused)
{
    const Rig r = railRig();
    Geometry g  = r.geometry;
    g.mount_b_m = g.mount_a_m;

    const Construction c = construct(r.a, r.b, g, "part");
    EXPECT_FALSE(c.constructed);
    EXPECT_NE(c.skip_reason.find("same point in CAD"), std::string::npos);
}

TEST(TwoMountPose, NormalParallelToTheCadChordIsRefused)
{
    const Rig r = railRig();
    Geometry g  = r.geometry;
    g.normal_in_part = Vec3::UnitX();   // the rail's chord runs along X

    const Construction c = construct(r.a, r.b, g, "part");
    EXPECT_FALSE(c.constructed);
    EXPECT_NE(c.skip_reason.find("parallel to the CAD chord"), std::string::npos);
}

TEST(TwoMountPose, AntiparallelNormalsAreRefused)
{
    const Rig r = railRig();
    const Construction c =
        construct(r.a, tilted(r.b, kPi, (r.b.translation() - r.a.translation()).normalized()),
                  r.geometry, "part");

    EXPECT_FALSE(c.constructed);
    EXPECT_NE(c.skip_reason.find("antiparallel"), std::string::npos);
}

TEST(TwoMountPose, CoincidentBodiesAreRefused)
{
    const Rig r = railRig();
    const Transform bOnA =
        frames::make(r.b.to, r.b.from, r.a.translation(), r.b.rotation());

    const Construction c = construct(r.a, bOnA, r.geometry, "part");
    EXPECT_FALSE(c.constructed);
    EXPECT_NE(c.skip_reason.find("same point"), std::string::npos);
}

TEST(TwoMountPose, BodiesInDifferentFramesThrow)
{
    const Rig r = railRig();
    const Transform elsewhere =
        frames::make("somewhere_else", r.b.from, r.b.translation(), r.b.rotation());

    EXPECT_THROW((void)construct(r.a, elsewhere, r.geometry, "part"), frames::FrameError);
}

// ---------------------------------------------------------------
// stamps
// ---------------------------------------------------------------

TEST(TwoMountPose, ConstructedPoseCarriesTheOlderStamp)
{
    // Same rule as frames::compose: a derived quantity is only as fresh as its
    // stalest input, so a body that stopped updating drags the stamp back and
    // the comparison layer's time gate can see it.
    const Rig r = railRig();

    const Transform a =
        frames::make(r.a.to, r.a.from, r.a.translation(), r.a.rotation(), 100.0);
    const Transform b =
        frames::make(r.b.to, r.b.from, r.b.translation(), r.b.rotation(), 97.5);

    const Construction c = construct(a, b, r.geometry, "part");
    ASSERT_TRUE(c.constructed);
    EXPECT_DOUBLE_EQ(c.pose.stamp, 97.5);
}
