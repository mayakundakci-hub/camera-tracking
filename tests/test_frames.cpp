// Tests for nodes/common/frames.hpp
//
// Two things are under test, and only one of them is arithmetic:
//   1. that the frame bookkeeping composes/inverts/routes correctly, and
//   2. that a MISUSE throws instead of returning a plausible wrong answer.
//
// (2) is the reason the file exists, so it gets as much attention as (1).
// Eigen's own arithmetic is not retested here -- it is not ours to verify.

#include <gtest/gtest.h>

#include "frames.hpp"

#include <algorithm>

using namespace frames;

namespace {

// A deliberately awkward transform: rotation about a non-axis-aligned vector
// plus a translation, so a wrong composition order cannot coincidentally pass.
Transform sample(const FrameId& to, const FrameId& from, double stamp = 0.0)
{
    const Quat q(Eigen::AngleAxisd(0.7, Vec3(0.3, -0.5, 0.81).normalized()));
    return make(to, from, Vec3(0.12, -0.34, 1.05), q, stamp);
}

constexpr double kTol = 1e-12;

void expectSameTransform(const Transform& a, const Transform& b)
{
    EXPECT_EQ(a.to, b.to);
    EXPECT_EQ(a.from, b.from);
    EXPECT_NEAR((a.translation() - b.translation()).norm(), 0.0, 1e-9);
    EXPECT_NEAR(magnitudeOf(compose(inverse(a), b)).angle_rad, 0.0, 1e-9);
}

}   // namespace

// ---------------------------------------------------------------
// direction discipline -- the bug class this file exists to prevent
// ---------------------------------------------------------------

TEST(Frames, ApplyRejectsPointFromWrongFrame)
{
    const Transform T_rail_opti = sample("rail", "opti");
    const Point p_in_rail{"rail", Vec3(1, 2, 3)};

    // The point is already in the frame this transform PRODUCES. Applying it
    // is the reversed-transform mistake, and it must not silently succeed.
    EXPECT_THROW((void)T_rail_opti.apply(p_in_rail), FrameError);
}

TEST(Frames, ComposeRejectsMismatchedInnerFrames)
{
    const Transform T_a_b = sample("a", "b");
    const Transform T_b_c = sample("b", "c");

    EXPECT_NO_THROW((void)compose(T_a_b, T_b_c));            // inner frames meet: b == b
    EXPECT_THROW((void)compose(T_b_c, T_a_b), FrameError);   // reversed: c != a
}

TEST(Frames, ComposeProducesOuterFrames)
{
    const Transform T_a_c = compose(sample("a", "b"), sample("b", "c"));
    EXPECT_EQ(T_a_c.to, "a");
    EXPECT_EQ(T_a_c.from, "c");
}

// ---------------------------------------------------------------
// algebra
// ---------------------------------------------------------------

TEST(Frames, InverseRoundTripsToIdentity)
{
    const Transform t     = sample("rail", "opti");
    const Transform round = compose(t, inverse(t));

    EXPECT_EQ(round.to, "rail");
    EXPECT_EQ(round.from, "rail");
    const Magnitude m = magnitudeOf(round);
    EXPECT_NEAR(m.distance_m, 0.0, 1e-12);
    EXPECT_NEAR(m.angle_rad, 0.0, 1e-12);
}

TEST(Frames, ApplyMatchesComposedTransform)
{
    const Transform T_a_b = sample("a", "b");
    const Transform T_b_c = sample("b", "c");
    const Point p_c{"c", Vec3(0.4, -1.1, 0.25)};

    const Point viaSteps    = T_a_b.apply(T_b_c.apply(p_c));
    const Point viaComposed = compose(T_a_b, T_b_c).apply(p_c);

    EXPECT_EQ(viaSteps.frame, "a");
    EXPECT_NEAR((viaSteps.p - viaComposed.p).norm(), 0.0, kTol);
}

TEST(Frames, MakeNormalizesQuaternion)
{
    // A quaternion scaled by 3: if it is not normalized, the rotation matrix
    // scales the point by 9 and every downstream distance is wrong.
    const Quat raw(3.0, 0.0, 0.0, 0.0);
    const Transform t = make("a", "b", Vec3::Zero(), raw);
    const Point out   = t.apply({"b", Vec3(1, 0, 0)});
    EXPECT_NEAR(out.p.norm(), 1.0, kTol);
}

TEST(Frames, MakeRejectsZeroQuaternion)
{
    EXPECT_THROW((void)make("a", "b", Vec3::Zero(), Quat(0, 0, 0, 0)), FrameError);
}

TEST(Frames, MagnitudeOfPureTranslation)
{
    const Transform t = make("a", "b", Vec3(0.3, 0.4, 0.0), Quat::Identity());
    const Magnitude m = magnitudeOf(t);
    EXPECT_NEAR(m.distance_m, 0.5, kTol);
    EXPECT_NEAR(m.angle_rad, 0.0, kTol);
}

TEST(Frames, MagnitudeOfPureRotation)
{
    const Quat q(Eigen::AngleAxisd(0.42, Vec3::UnitZ()));
    const Magnitude m = magnitudeOf(make("a", "b", Vec3::Zero(), q));
    EXPECT_NEAR(m.distance_m, 0.0, kTol);
    EXPECT_NEAR(m.angle_rad, 0.42, 1e-12);
}

// ---------------------------------------------------------------
// registry routing
// ---------------------------------------------------------------

TEST(Frames, RegistryResolvesDirectEdge)
{
    Registry reg;
    const Transform T_rail_opti = sample("rail", "opti");
    reg.set(T_rail_opti);

    expectSameTransform(reg.require("rail", "opti"), T_rail_opti);
}

TEST(Frames, RegistryInvertsEdgeAutomatically)
{
    Registry reg;
    const Transform T_rail_opti = sample("rail", "opti");
    reg.set(T_rail_opti);

    // Only one direction was stored; the other must still resolve.
    expectSameTransform(reg.require("opti", "rail"), inverse(T_rail_opti));
}

TEST(Frames, RegistryRoutesThroughIntermediateFrame)
{
    // The comparison case: two independent estimates of one object, each
    // anchored to the rail, with no edge between them.
    Registry reg;
    const Transform T_rail_measured = sample("rail", "rotor_opti");
    const Transform T_rail_expected = sample("rail", "rotor_expected", 5.0);
    reg.set(T_rail_measured);
    reg.set(T_rail_expected);

    const Transform delta = reg.require("rotor_opti", "rotor_expected");
    EXPECT_EQ(delta.to, "rotor_opti");
    EXPECT_EQ(delta.from, "rotor_expected");

    expectSameTransform(delta, compose(inverse(T_rail_measured), T_rail_expected));
}

TEST(Frames, RegistryIdentityForSameFrame)
{
    Registry reg;
    const Magnitude m = magnitudeOf(reg.require("rail", "rail"));
    EXPECT_NEAR(m.distance_m, 0.0, kTol);
    EXPECT_NEAR(m.angle_rad, 0.0, kTol);
}

TEST(Frames, RegistryReportsUnreachableFrames)
{
    Registry reg;
    reg.set(sample("rail", "opti"));

    EXPECT_FALSE(reg.lookup("rail", "rotor").has_value());
    EXPECT_THROW((void)reg.require("rail", "rotor"), FrameError);
}

TEST(Frames, RegistryReplacesEdgeInEitherDirection)
{
    Registry reg;
    reg.set(sample("rail", "hand"));
    EXPECT_EQ(reg.edgeCount(), 1u);

    // A continuously-tracked object re-sets its edge every frame; that must
    // overwrite, not accumulate -- including when written the other way round.
    reg.set(sample("rail", "hand"));
    reg.set(sample("hand", "rail"));
    EXPECT_EQ(reg.edgeCount(), 1u);
}

TEST(Frames, RegistryRejectsSelfEdge)
{
    Registry reg;
    Transform bad = sample("rail", "rail");   // two manifest entries, same name
    bad.to = bad.from = "rail";
    EXPECT_THROW(reg.set(bad), FrameError);
}

TEST(Frames, RegistryEraseRemovesRoute)
{
    Registry reg;
    reg.set(sample("rail", "hand"));
    EXPECT_TRUE(reg.has("rail", "hand"));

    EXPECT_TRUE(reg.erase("hand", "rail"));   // either order identifies the edge
    EXPECT_FALSE(reg.has("rail", "hand"));
    EXPECT_FALSE(reg.erase("hand", "rail"));
}

TEST(Frames, RegistryKnownFramesListsEveryFrameOnce)
{
    Registry reg;
    reg.set(sample("rail", "opti"));
    reg.set(sample("rail", "rotor_opti"));

    auto known = reg.knownFrames();
    std::sort(known.begin(), known.end());
    EXPECT_EQ(known, (std::vector<FrameId>{"opti", "rail", "rotor_opti"}));
}

// ---------------------------------------------------------------
// staleness provenance
// ---------------------------------------------------------------

TEST(Frames, ComposeKeepsOldestStamp)
{
    EXPECT_EQ(compose(sample("a", "b", 10.0), sample("b", "c", 4.0)).stamp, 4.0);
    EXPECT_EQ(compose(sample("a", "b", 0.0), sample("b", "c", 4.0)).stamp, 4.0);
    EXPECT_EQ(compose(sample("a", "b", 7.0), sample("b", "c", 0.0)).stamp, 7.0);
    EXPECT_EQ(compose(sample("a", "b", 0.0), sample("b", "c", 0.0)).stamp, 0.0);
}

// ---------------------------------------------------------------
// boundary conversions
// ---------------------------------------------------------------

TEST(Frames, QuaternionWxyzLayoutSurvivesRoundTrip)
{
    // Guards the coeffs()-is-xyzw trap: if the layout were mixed up anywhere,
    // these components come back permuted.
    const Quat q    = convert::quatFromWxyz(0.5, 0.5, -0.5, 0.5);
    const auto wxyz = convert::quatToWxyz(q);
    EXPECT_NEAR(wxyz[0], 0.5, kTol);
    EXPECT_NEAR(wxyz[1], 0.5, kTol);
    EXPECT_NEAR(wxyz[2], -0.5, kTol);
    EXPECT_NEAR(wxyz[3], 0.5, kTol);
}

TEST(Frames, QuaternionFromWxyzRejectsZero)
{
    EXPECT_THROW((void)convert::quatFromWxyz(0, 0, 0, 0), FrameError);
}

TEST(Frames, UnitConversionsRoundTrip)
{
    EXPECT_NEAR(convert::mmToM(1234.5), 1.2345, kTol);
    EXPECT_NEAR(convert::mToMm(convert::mmToM(987.6)), 987.6, 1e-9);
    EXPECT_NEAR(convert::radToDeg(convert::degToRad(37.5)), 37.5, 1e-9);
    EXPECT_NEAR(convert::vecFromMm(1000.0, -500.0, 250.0).y(), -0.5, kTol);
}
