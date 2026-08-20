// Tests for nodes/common/object_placement.hpp
//
// The behaviours worth pinning are the ones that would otherwise fail
// silently: latching on a bad frame, a dropout wiping an object out of the
// scene, and -- most importantly -- the claim that the mocap world origin
// does not matter. That last one is the argument for anchoring to the rail at
// all, so it gets a test rather than a comment.

#include <gtest/gtest.h>

#include "object_placement.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <random>

using namespace placement;

namespace {

class TempScene {
public:
    TempScene()
    {
        static int counter = 0;
        const auto stamp   = std::chrono::steady_clock::now().time_since_epoch().count();
        root_ = scene::fs::temp_directory_path() /
                ("ct_place_test_" + std::to_string(stamp) + "_" + std::to_string(counter++));
        scene::fs::remove_all(root_);
        scene::fs::create_directories(root_ / "config" / "objects");
    }

    ~TempScene()
    {
        std::error_code ec;
        scene::fs::remove_all(root_, ec);
    }
    TempScene(const TempScene&)            = delete;
    TempScene& operator=(const TempScene&) = delete;

    void manifest(const std::string& b) const { write(root_ / "config" / "scene.json", b); }
    void object(const std::string& n, const std::string& b) const
    {
        write(root_ / "config" / "objects" / (n + ".json"), b);
    }
    // The loader only checks that referenced assets exist, so a placeholder
    // is enough to exercise a joint_state placement without a real URDF.
    void asset(const std::string& relPath) const
    {
        const scene::fs::path p = root_ / relPath;
        scene::fs::create_directories(p.parent_path());
        write(p, "<robot name=\"placeholder\"/>\n");
    }
    [[nodiscard]] scene::Scene load() const { return scene::load(root_ / "config" / "scene.json"); }

private:
    static void write(const scene::fs::path& p, const std::string& b)
    {
        std::ofstream f(p);
        f << b;
    }
    scene::fs::path root_;
};

// MSVC does not define M_PI without _USE_MATH_DEFINES; spell it locally.
constexpr double kPi = 3.14159265358979323846;

frames::Quat quat(double angleRad, const frames::Vec3& axis)
{
    return frames::Quat(Eigen::AngleAxisd(angleRad, axis.normalized()));
}

// A rail measured by OptiTrack, plus a rotor measured two ways: by the
// cameras, and as reported by the robot. The shape of the real scene.
scene::Scene railAndRotor(const TempScene& t)
{
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "capture": "latched", "asset_id": 1}]
    })");
    t.object("rotor_a", R"({
        "id": "rotor",
        "placements": [
            {"id": "rotor_opti", "source": "optitrack", "capture": "latched", "asset_id": 2},
            {"id": "rotor_expected", "source": "expected_pose", "capture": "latched",
             "topic": "rotor/pose_expected"}
        ]})");
    return t.load();
}

BodyObservation body(int id, const frames::Vec3& p, const frames::Quat& q, bool tracked = true,
                     double meanError = 0.0)
{
    BodyObservation o;
    o.asset_id   = id;
    o.position   = p;
    o.rotation   = q;
    o.tracked    = tracked;
    o.mean_error = meanError;
    return o;
}

// A real gate, but over a window short enough for a test to fill by hand.
// Everything else keeps the production default, so these tests exercise the
// thresholds the backend actually runs with.
LatchPolicy gated(std::size_t window = 20)
{
    LatchPolicy p;
    p.min_samples = window;
    return p;
}

// Drives `count` frames of asset 1 at `centre`, displaced by `offset(i)`. Keeps
// the latch tests to their point rather than to their loops.
void feed(Placer& placer, std::size_t count, const frames::Vec3& centre,
          const std::function<frames::Vec3(std::size_t)>& offset, double meanError = 0.0,
          int assetId = 1)
{
    for (std::size_t i = 0; i < count; ++i)
        placer.onMocapFrame(
            {body(assetId, centre + offset(i), frames::Quat::Identity(), true, meanError)},
            1.0 + static_cast<double>(i));
}

// Sub-millimetre jitter, deterministic and non-zero. Non-zero matters: the
// robust sigma of a bit-identical window floors near zero, and then any
// deviation at all reads as an outlier.
frames::Vec3 jitter(std::size_t i)
{
    const double s = 1e-4;   // 0.1 mm
    return {s * std::sin(static_cast<double>(i) * 1.7), s * std::sin(static_cast<double>(i) * 2.3),
            s * std::sin(static_cast<double>(i) * 3.1)};
}

}   // namespace

// ---------------------------------------------------------------
// construction
// ---------------------------------------------------------------

TEST(Placement, StaticPlacementsAreKnownImmediately)
{
    // The bring-up scene: a static anchor and nothing measured. It must be
    // usable the instant it loads, with no mocap running at all.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");

    frames::Registry reg;
    Placer placer(t.load(), reg, LatchPolicy::immediate());

    EXPECT_TRUE(placer.anchorPlaced());
    EXPECT_TRUE(placer.complete());
    EXPECT_TRUE(reg.has("optitrack_world", "rail_origin"));
}

TEST(Placement, JointStateRootSitsAtTheAnchor)
{
    // The robot's BASE is fixed at the rail origin -- the joints move the
    // links, not the base -- so its root edge is known without any measurement
    // and is identity-valued.
    TempScene t;
    t.asset("Rendering/p2d2.urdf");
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "p2d2"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("p2d2", R"({
        "id": "p2d2",
        "placements": [{"id": "p2d2_fanuc", "source": "joint_state",
                        "urdf": "Rendering/p2d2.urdf", "topic": "robot/joint_state"}]})");

    frames::Registry reg;
    Placer placer(t.load(), reg, LatchPolicy::immediate());

    EXPECT_TRUE(placer.complete());
    const auto m = frames::magnitudeOf(reg.require("rail_origin", "p2d2_fanuc"));
    EXPECT_NEAR(m.distance_m, 0.0, 1e-12);
    EXPECT_NEAR(m.angle_rad, 0.0, 1e-12);
}

TEST(Placement, AwaitsMeasurementForTrackedPlacements)
{
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, LatchPolicy::immediate());

    EXPECT_FALSE(placer.anchorPlaced());
    EXPECT_FALSE(placer.complete());

    const auto waiting = placer.pending();
    EXPECT_EQ(waiting.size(), 3u);   // rail_origin, rotor_opti, rotor_expected
}

// ---------------------------------------------------------------
// latching
// ---------------------------------------------------------------

TEST(Placement, DoesNotLatchOnAnUntrackedBody)
{
    // The one failure mode the simplified latch can still have: capturing an
    // occluded frame and anchoring the whole session on garbage.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, LatchPolicy::immediate());

    placer.onMocapFrame({body(1, {9, 9, 9}, frames::Quat::Identity(), /*tracked=*/false)}, 1.0);
    EXPECT_FALSE(placer.anchorPlaced());

    placer.onMocapFrame({body(1, {1, 2, 3}, frames::Quat::Identity(), /*tracked=*/true)}, 2.0);
    EXPECT_TRUE(placer.anchorPlaced());

    // And it took the good pose, not the bad one.
    const auto T = reg.require(kMocapFrame, "rail_origin");
    EXPECT_NEAR(T.translation().x(), 1.0, 1e-12);   // the body's pose, taken as-is
}

TEST(Placement, LatchedPlacementIgnoresLaterFrames)
{
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, LatchPolicy::immediate());

    placer.onMocapFrame({body(1, {1, 0, 0}, frames::Quat::Identity())}, 1.0);
    placer.onMocapFrame({body(1, {5, 5, 5}, frames::Quat::Identity())}, 2.0);

    const auto T = reg.require(kMocapFrame, "rail_origin");
    EXPECT_NEAR(T.translation().x(), 1.0, 1e-12);   // still the first capture
    EXPECT_NEAR(T.translation().y(), 0.0, 1e-12);
}

TEST(Placement, ContinuousPlacementKeepsUpdating)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "hand"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("hand", R"({
        "id": "hand",
        "placements": [{
            "id": "hand_opti", "source": "optitrack", "capture": "continuous", "asset_id": 5}]
    })");

    frames::Registry reg;
    Placer placer(t.load(), reg, LatchPolicy::immediate());

    placer.onMocapFrame({body(5, {1, 0, 0}, frames::Quat::Identity())}, 1.0);
    EXPECT_NEAR(reg.require(kMocapFrame, "hand_opti").translation().x(), 1.0, 1e-12);

    placer.onMocapFrame({body(5, {2, 0, 0}, frames::Quat::Identity())}, 2.0);
    EXPECT_NEAR(reg.require(kMocapFrame, "hand_opti").translation().x(), 2.0, 1e-12);
    EXPECT_EQ(reg.require(kMocapFrame, "hand_opti").stamp, 2.0);
}

TEST(Placement, DropoutKeepsLastGoodPoseRatherThanErasingIt)
{
    // Erasing would make objects flicker out of the scene on a brief
    // occlusion. The stale pose survives; its stamp is what says it is old.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "hand"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("hand", R"({
        "id": "hand",
        "placements": [{
            "id": "hand_opti", "source": "optitrack", "capture": "continuous", "asset_id": 5}]
    })");

    frames::Registry reg;
    Placer placer(t.load(), reg, LatchPolicy::immediate());

    placer.onMocapFrame({body(5, {1, 0, 0}, frames::Quat::Identity())}, 1.0);
    placer.onMocapFrame({body(5, {7, 7, 7}, frames::Quat::Identity(), /*tracked=*/false)}, 2.0);

    ASSERT_TRUE(reg.has(kMocapFrame, "hand_opti"));
    EXPECT_NEAR(reg.require(kMocapFrame, "hand_opti").translation().x(), 1.0, 1e-12);
    EXPECT_EQ(reg.require(kMocapFrame, "hand_opti").stamp, 1.0);   // visibly stale
}

TEST(Placement, IgnoresBodiesNoPlacementAsksFor)
{
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, LatchPolicy::immediate());

    placer.onMocapFrame({body(99, {1, 1, 1}, frames::Quat::Identity())}, 1.0);
    EXPECT_FALSE(placer.anchorPlaced());
    EXPECT_EQ(reg.edgeCount(), 0u);
}

// ---------------------------------------------------------------
// the latch quality gate
//
// These are the tests that matter most in this file. A bad latch is silent by
// construction -- it produces a plausible pose that biases every pose in the
// session consistently, which reads as systematic robot error rather than as a
// mocap glitch. Nothing downstream can detect it, so it has to be caught here.
// ---------------------------------------------------------------

TEST(Latch, WaitsForAFullWindowRatherThanTakingTheFirstFrame)
{
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    feed(placer, 19, {1.0, 0.0, 0.0}, jitter);
    EXPECT_FALSE(placer.anchorPlaced()) << "latched before the window was full";

    feed(placer, 1, {1.0, 0.0, 0.0}, jitter);
    EXPECT_TRUE(placer.anchorPlaced());
}

TEST(Latch, CommitsTheMeanOfTheWindowNotOneSampleFromIt)
{
    // Alternating +/-0.5 mm about the truth: every individual sample is wrong,
    // and only the average is right. First-frame-wins scores 0.5 mm here.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    const double half = 5e-4;
    feed(placer, 20, {1.0, 0.0, 0.0},
         [half](std::size_t i) { return frames::Vec3((i % 2 == 0) ? half : -half, 0.0, 0.0); });

    ASSERT_TRUE(placer.anchorPlaced());
    EXPECT_NEAR(reg.require(kMocapFrame, "rail_origin").translation().x(), 1.0, 1e-12);

    const auto r = placer.latchReport("rail_origin");
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(r->latched);
    EXPECT_EQ(r->rejected, 0u);
    EXPECT_NEAR(r->spread_m, half, 1e-9);                      // dispersion: unchanged by N
    EXPECT_NEAR(r->std_err_m, half / std::sqrt(20.0), 1e-9);   // error bar: shrinks as sqrt(N)
}

TEST(Latch, AveragingActuallyBuysAccuracy)
{
    // The claim behind the whole window: the committed pose is closer to truth
    // than a typical single sample. Gaussian noise, fixed seed, so the bound
    // below is deterministic rather than flaky.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(200));

    constexpr double kSigma = 3e-4;   // 0.3 mm per axis, typical OptiTrack jitter
    std::mt19937 rng(12345);
    std::normal_distribution<double> noise(0.0, kSigma);

    const frames::Vec3 truth(1.0, 0.0, 0.0);
    for (int i = 0; i < 200; ++i)
        placer.onMocapFrame({body(1, truth + frames::Vec3(noise(rng), noise(rng), noise(rng)),
                                  frames::Quat::Identity())},
                            1.0 + i);

    ASSERT_TRUE(placer.anchorPlaced());
    const auto r = placer.latchReport("rail_origin");
    ASSERT_TRUE(r.has_value());

    // Per-sample dispersion is the 3D radius, so ~sigma*sqrt(3).
    EXPECT_NEAR(r->spread_m, kSigma * std::sqrt(3.0), 0.25 * kSigma);

    // The error that matters: distance from the committed pose to the truth,
    // which must sit inside a few standard errors -- and well inside one sigma.
    const frames::Vec3 got = reg.require(kMocapFrame, "rail_origin").translation();
    const double error     = (got - truth).norm();
    EXPECT_LT(error, 4.0 * r->std_err_m);
    EXPECT_LT(error, kSigma) << "averaging did not beat a single sample";
}

TEST(Latch, RejectsAnIsolatedOutlierRatherThanAveragingItIn)
{
    // A mislabeled marker for one frame. Under first-frame-wins this could BE
    // the latch; under a plain mean it drags it.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    feed(placer, 20, {1.0, 0.0, 0.0}, [](std::size_t i) {
        return i == 7 ? frames::Vec3(0.05, 0.0, 0.0) : jitter(i);   // one 50 mm glitch
    });

    ASSERT_TRUE(placer.anchorPlaced());
    const auto r = placer.latchReport("rail_origin");
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->rejected, 1u);

    // Had the outlier survived it would have pulled the mean 2.5 mm out.
    EXPECT_NEAR(reg.require(kMocapFrame, "rail_origin").translation().x(), 1.0, 5e-5);
}

TEST(Latch, RefusesToLatchWhenTooManySamplesAreOutliers)
{
    // Intermittent marker swapping. The outlier pass alone would discard these
    // happily and forever, reporting a beautiful spread over the 70% that was
    // real -- so the reject FRACTION has to be a gate of its own.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    feed(placer, 20, {1.0, 0.0, 0.0}, [](std::size_t i) {
        return (i % 10 < 3) ? frames::Vec3(0.05, 0.0, 0.0) : jitter(i);   // 30% glitched
    });

    EXPECT_FALSE(placer.anchorPlaced());
    const auto r = placer.latchReport("rail_origin");
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->rejected, 6u);
    EXPECT_NE(r->reason.find("outliers"), std::string::npos) << r->reason;
}

TEST(Latch, RefusesToLatchAnObjectThatIsStillMoving)
{
    // The case plain averaging silently gets wrong: average across a movement
    // and the result is a pose the object never occupied. MAD cannot save this
    // -- a moving object moves TOGETHER, so there is no minority to reject.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    feed(placer, 20, {1.0, 0.0, 0.0},
         [](std::size_t i) { return frames::Vec3(1e-3 * static_cast<double>(i), 0.0, 0.0); });

    EXPECT_FALSE(placer.anchorPlaced());
    const auto r = placer.latchReport("rail_origin");
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->rejected, 0u) << "a steady ramp has no outliers -- the SPREAD is the signal";
    EXPECT_GT(r->spread_m, 0.002);
    EXPECT_NE(r->reason.find("holding still"), std::string::npos) << r->reason;
}

TEST(Latch, LatchesAfterMotionStopsWithoutNeedingARestart)
{
    // Why the window slides. Placer has no re-arm API, so an operator who was
    // still tightening a bolt when the backend started must be able to walk away
    // and have it latch -- with no reset step and no relaunch.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    feed(placer, 40, {1.0, 0.0, 0.0},
         [](std::size_t i) { return frames::Vec3(1e-3 * static_cast<double>(i), 0.0, 0.0); });
    ASSERT_FALSE(placer.anchorPlaced());

    // Settled, at a pose unrelated to where it was during the move.
    feed(placer, 20, {2.0, 0.0, 0.0}, jitter);

    ASSERT_TRUE(placer.anchorPlaced());
    EXPECT_NEAR(reg.require(kMocapFrame, "rail_origin").translation().x(), 2.0, 1e-4)
        << "latched on a window that still contained the movement";
}

TEST(Latch, NeverAdmitsAFrameWhoseMarkerErrorIsTooHigh)
{
    // "Tracked" and "solved well" are different claims. Motive reports a body as
    // tracked while fitting it badly, and that is the frame a latch must not
    // take -- which is why mean_error is forwarded from NatNet at all.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    feed(placer, 50, {1.0, 0.0, 0.0}, jitter, /*meanError=*/0.005);   // 5 mm, limit is 2 mm

    EXPECT_FALSE(placer.anchorPlaced());
    const auto r = placer.latchReport("rail_origin");
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->seen, 50u);
    EXPECT_EQ(r->admitted, 0u);
    EXPECT_NE(r->reason.find("marker error"), std::string::npos) << r->reason;

    // And it recovers the moment Motive solves the body properly again.
    feed(placer, 20, {1.0, 0.0, 0.0}, jitter, /*meanError=*/0.0003);
    EXPECT_TRUE(placer.anchorPlaced());
}

TEST(Latch, MeasuresSpreadWhereMotivePutTheBody)
{
    // What the gate can and cannot see now that a body's pose is taken as-is.
    // Spread is measured at Motive's rigid-body frame, so a body that rocks
    // WITHOUT translating passes on position however far its pivot sits from
    // its markers. The angular limit is the only thing standing under that
    // case, which is why it is not slack, and why the rotor is built from two
    // clamps and a chord rather than offset from one distant body.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "capture": "latched", "asset_id": 1}]
    })");

    frames::Registry reg;
    const LatchPolicy policy = gated(20);
    Placer placer(t.load(), reg, policy);

    // The body is at the origin and does not translate at all. Only its
    // orientation wanders, by +/-0.115 deg.
    constexpr double kWobbleRad = 2e-3;
    for (std::size_t i = 0; i < 20; ++i)
        placer.onMocapFrame(
            {body(1, frames::Vec3::Zero(),
                  quat(kWobbleRad * ((i % 2 == 0) ? 1.0 : -1.0), frames::Vec3::UnitZ()))},
            1.0 + static_cast<double>(i));

    const auto r = placer.latchReport("rail_origin");
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->rejected, 0u) << "a symmetric wobble has no minority to reject";

    // Position spread is exactly zero: the frame the gate watches did not move.
    EXPECT_NEAR(r->spread_m, 0.0, 1e-12);
    EXPECT_LT(r->spread_m, policy.max_spread_m);

    // So the angular spread is what has to carry it, and here it is inside the
    // limit -- 0.115 deg against 0.5 -- so this latches.
    EXPECT_LT(r->spread_rad, policy.max_spread_rad);
    EXPECT_TRUE(placer.anchorPlaced());
}

TEST(Latch, GatesALatchedExpectedPoseToo)
{
    // Latch quality is a property of LATCHING, not of OptiTrack. A pose the
    // robot reports carries no marker residual, and nothing else changes.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    for (std::size_t i = 0; i < 20; ++i)
        placer.onExpectedPose("rotor_expected",
                              frames::Vec3(0.5 + 1e-3 * static_cast<double>(i), 0.0, 0.0),
                              frames::Quat::Identity(), 1.0 + static_cast<double>(i));

    const auto moving = placer.latchReport("rotor_expected");
    ASSERT_TRUE(moving.has_value());
    EXPECT_FALSE(moving->latched);
    EXPECT_EQ(moving->admitted, 20u) << "absent mean_error must not block admission";

    for (std::size_t i = 0; i < 20; ++i)
        placer.onExpectedPose("rotor_expected", frames::Vec3(0.5, 0.0, 0.0) + jitter(i),
                              frames::Quat::Identity(), 100.0 + static_cast<double>(i));

    const auto settled = placer.latchReport("rotor_expected");
    ASSERT_TRUE(settled.has_value());
    EXPECT_TRUE(settled->latched);
}

TEST(Latch, ContinuousPlacementsAreNotGatedAndHaveNoReport)
{
    // A continuous placement is never committed once, so there is nothing to
    // gate. Gating one would freeze a moving object at its first quiet moment.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "hand"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("hand", R"({
        "id": "hand",
        "placements": [{
            "id": "hand_opti", "source": "optitrack", "capture": "continuous", "asset_id": 5}]
    })");

    frames::Registry reg;
    Placer placer(t.load(), reg, gated(20));

    placer.onMocapFrame({body(5, {1, 0, 0}, frames::Quat::Identity(), true, 0.05)}, 1.0);
    EXPECT_NEAR(reg.require(kMocapFrame, "hand_opti").translation().x(), 1.0, 1e-12);
    EXPECT_FALSE(placer.latchReport("hand_opti").has_value());
    EXPECT_TRUE(placer.latchReports().empty());
}

TEST(Latch, AnUnlatchedAnchorLeavesEveryOtherPlacementUnreachable)
{
    // How a failed anchor latch reaches the comparison output: with no anchor
    // edge there is no anchor-relative route to anything, so every delta comes
    // back invalid rather than being computed against a wrong origin.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    // The rotor latches cleanly; the anchor never holds still.
    for (std::size_t i = 0; i < 20; ++i)
        placer.onMocapFrame(
            {body(1, frames::Vec3(1e-2 * static_cast<double>(i), 0, 0), frames::Quat::Identity()),
             body(2, frames::Vec3(3.0, 0.0, 0.0) + jitter(i), frames::Quat::Identity())},
            1.0 + static_cast<double>(i));

    ASSERT_FALSE(placer.anchorPlaced());
    EXPECT_TRUE(reg.has(kMocapFrame, "rotor_opti"));                     // measured
    EXPECT_FALSE(reg.lookup("rail_origin", "rotor_opti").has_value());   // but not placeable
    EXPECT_FALSE(placer.complete());
}

TEST(Latch, StatusExplainsWhyAPlacementHasNotLatched)
{
    // "Waiting on rail_origin" and "rail_origin has been moving for 40 seconds"
    // are different problems with different fixes, and the operator can only see
    // the difference if the reason is printed.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, gated(20));

    EXPECT_NE(placer.status().find("no sample admitted yet"), std::string::npos);

    feed(placer, 5, {1.0, 0.0, 0.0}, jitter);
    EXPECT_NE(placer.status().find("collecting 5/20"), std::string::npos) << placer.status();

    feed(placer, 20, {1.0, 0.0, 0.0},
         [](std::size_t i) { return frames::Vec3(1e-3 * static_cast<double>(i), 0.0, 0.0); });
    const std::string s = placer.status();
    EXPECT_NE(s.find("[latch] rail_origin"), std::string::npos) << s;
    EXPECT_NE(s.find("holding still"), std::string::npos) << s;
}

// ---------------------------------------------------------------
// tracked bodies and expected poses
// ---------------------------------------------------------------

TEST(Placement, TakesTheBodyPoseExactlyAsMotiveReportsIt)
{
    // There is no offset between a rigid body and its placement -- the pivot is
    // set in Motive instead. So a body rotated 90 degrees about Z at the origin
    // places its object at the origin, rotated 90 degrees about Z, and nothing
    // is displaced by anything.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, LatchPolicy::immediate());

    const frames::Quat q = quat(kPi / 2.0, frames::Vec3::UnitZ());
    placer.onMocapFrame({body(1, frames::Vec3::Zero(), q)}, 1.0);

    const auto T = reg.require(kMocapFrame, "rail_origin");
    EXPECT_NEAR(T.translation().norm(), 0.0, 1e-12);
    EXPECT_NEAR(T.rotation().angularDistance(q), 0.0, 1e-12);
}

TEST(Placement, ExpectedPoseLandsInTheAnchorFrame)
{
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, LatchPolicy::immediate());

    placer.onExpectedPose("rotor_expected", {0.5, 0.0, 0.0}, frames::Quat::Identity(), 3.0);

    const auto T = reg.require("rail_origin", "rotor_expected");
    EXPECT_NEAR(T.translation().x(), 0.5, 1e-12);
}

TEST(Placement, ExpectedPoseIgnoresUnknownPlacementIds)
{
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, LatchPolicy::immediate());

    placer.onExpectedPose("not_in_the_scene", {1, 1, 1}, frames::Quat::Identity(), 1.0);
    EXPECT_EQ(reg.edgeCount(), 0u);
}

// ---------------------------------------------------------------
// the claim that justifies anchoring to the rail
// ---------------------------------------------------------------

TEST(Placement, MocapWorldOriginCancelsOutOfAnchorRelativeQueries)
{
    // Recalibrating Motive moves its world origin by some unknown M. Because
    // the rail is MEASURED rather than stored, every edge shifts by M and the
    // two cancel in any anchor-relative query. This is the whole reason the
    // system has no stored world origin, so it is worth a test.
    const frames::Vec3 railPos(1.0, 2.0, 0.0);
    const frames::Vec3 rotorPos(3.0, 2.0, 0.5);
    const frames::Quat railRot  = quat(0.3, frames::Vec3::UnitZ());
    const frames::Quat rotorRot = quat(-0.2, frames::Vec3(1, 1, 0));

    // M is T_mocapNew_mocapOld: the shift a recalibration applies to Motive's
    // world frame. Every pose Motive reports afterwards is M composed onto
    // what it used to report.
    auto measure = [&](const frames::Transform& M) {
        TempScene t;
        frames::Registry reg;
        Placer placer(railAndRotor(t), reg, LatchPolicy::immediate());

        const frames::Transform railInM =
            frames::compose(M, frames::make("mocapOld", "rail_marker", railPos, railRot));
        const frames::Transform rotorInM =
            frames::compose(M, frames::make("mocapOld", "rotor_marker", rotorPos, rotorRot));

        placer.onMocapFrame({body(1, railInM.translation(), railInM.rotation()),
                             body(2, rotorInM.translation(), rotorInM.rotation())},
                            1.0);

        return reg.require("rail_origin", "rotor_opti");
    };

    const frames::Transform none =
        frames::make("mocapNew", "mocapOld", frames::Vec3::Zero(), frames::Quat::Identity());
    const frames::Transform shifted = frames::make("mocapNew", "mocapOld", {12.0, -7.0, 3.0},
                                                   quat(1.1, frames::Vec3(0.2, 0.9, -0.4)));

    const frames::Transform a = measure(none);
    const frames::Transform b = measure(shifted);

    EXPECT_NEAR((a.translation() - b.translation()).norm(), 0.0, 1e-9);
    EXPECT_NEAR(frames::magnitudeOf(frames::compose(frames::inverse(a), b)).angle_rad, 0.0, 1e-9);
}

// ---------------------------------------------------------------
// fused placements
// ---------------------------------------------------------------

namespace {

// Two measured mounts on one object, plus their average.
scene::Scene fusedRotor(const TempScene& t)
{
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("rotor_a", R"({
        "id": "rotor",
        "placements": [
            {"id": "rotor_opti_a", "source": "optitrack", "capture": "latched", "asset_id": 1,
             "visible": false},
            {"id": "rotor_opti_b", "source": "optitrack", "capture": "latched", "asset_id": 2,
             "visible": false},
            {"id": "rotor_opti", "source": "fused", "capture": "latched",
             "inputs": ["rotor_opti_a", "rotor_opti_b"]}
        ]})");
    return t.load();
}

}   // namespace

TEST(Placement, FusedPlacementAveragesItsInputs)
{
    TempScene t;
    frames::Registry reg;
    Placer placer(fusedRotor(t), reg, LatchPolicy::immediate());

    placer.onMocapFrame({body(1, {1.000, 0, 0}, frames::Quat::Identity()),
                         body(2, {1.010, 0, 0}, frames::Quat::Identity())},
                        1.0);

    ASSERT_TRUE(reg.has("rail_origin", "rotor_opti"));
    EXPECT_NEAR(reg.require("rail_origin", "rotor_opti").translation().x(), 1.005, 1e-9);
}

TEST(Placement, FusedPlacementWaitsForEveryInput)
{
    // A fused pose computed from half its inputs would be silently biased
    // toward whichever mount happened to be visible first.
    TempScene t;
    frames::Registry reg;
    Placer placer(fusedRotor(t), reg, LatchPolicy::immediate());

    placer.onMocapFrame({body(1, {1.0, 0, 0}, frames::Quat::Identity())}, 1.0);
    EXPECT_FALSE(reg.has("rail_origin", "rotor_opti"));
    EXPECT_FALSE(placer.complete());

    placer.onMocapFrame({body(2, {1.02, 0, 0}, frames::Quat::Identity())}, 2.0);
    EXPECT_TRUE(reg.has("rail_origin", "rotor_opti"));
    EXPECT_TRUE(placer.complete());
    EXPECT_NEAR(reg.require("rail_origin", "rotor_opti").translation().x(), 1.01, 1e-9);
}

TEST(Placement, FusedInputsMayLatchOnDifferentFrames)
{
    // The rotor is stationary and each placement latches independently, so the
    // two mounts never have to be visible in the same mocap frame -- which is
    // what makes a camera layout that cannot see both at once acceptable.
    TempScene t;
    frames::Registry reg;
    Placer placer(fusedRotor(t), reg, LatchPolicy::immediate());

    placer.onMocapFrame({body(1, {2.0, 0, 0}, frames::Quat::Identity())}, 1.0);
    for (int i = 0; i < 50; ++i)
        placer.onMocapFrame({body(1, {2.0, 0, 0}, frames::Quat::Identity())}, 1.0 + i);
    placer.onMocapFrame({body(2, {2.2, 0, 0}, frames::Quat::Identity())}, 99.0);

    EXPECT_NEAR(reg.require("rail_origin", "rotor_opti").translation().x(), 2.1, 1e-9);
}

TEST(Placement, FusedRotationAveragesWithoutQuaternionSignCancellation)
{
    // q and -q are the same rotation. Summing them unaligned cancels toward
    // zero and yields garbage, so the average must sign-align before adding.
    TempScene t;
    frames::Registry reg;
    Placer placer(fusedRotor(t), reg, LatchPolicy::immediate());

    const frames::Quat qa = quat(0.10, frames::Vec3::UnitZ());
    frames::Quat qb       = quat(0.20, frames::Vec3::UnitZ());
    qb = frames::Quat(-qb.w(), -qb.x(), -qb.y(), -qb.z());   // same rotation, flipped sign

    placer.onMocapFrame({body(1, frames::Vec3::Zero(), qa), body(2, frames::Vec3::Zero(), qb)},
                        1.0);

    const auto T = reg.require("rail_origin", "rotor_opti");
    EXPECT_NEAR(frames::magnitudeOf(T).angle_rad, 0.15, 1e-6);   // midway, not cancelled
}

TEST(Placement, FusedPlacementIsLatchedOnceComputed)
{
    TempScene t;
    frames::Registry reg;
    Placer placer(fusedRotor(t), reg, LatchPolicy::immediate());

    placer.onMocapFrame({body(1, {1.0, 0, 0}, frames::Quat::Identity()),
                         body(2, {1.0, 0, 0}, frames::Quat::Identity())},
                        1.0);
    placer.onMocapFrame({body(1, {9.0, 0, 0}, frames::Quat::Identity()),
                         body(2, {9.0, 0, 0}, frames::Quat::Identity())},
                        2.0);

    EXPECT_NEAR(reg.require("rail_origin", "rotor_opti").translation().x(), 1.0, 1e-9);
}

TEST(Latch, FusedPlacementInheritsItsInputsGate)
{
    // No new math for fusion: it latches once its inputs do, so gating the
    // inputs gates the result. Nothing in the fused path knows the gate exists.
    TempScene t;
    frames::Registry reg;
    Placer placer(fusedRotor(t), reg, gated(20));

    // Mount A settled, mount B still moving.
    for (std::size_t i = 0; i < 20; ++i)
        placer.onMocapFrame({body(1, frames::Vec3(1.0, 0, 0) + jitter(i), frames::Quat::Identity()),
                             body(2, frames::Vec3(1.0 + 1e-2 * static_cast<double>(i), 0, 0),
                                  frames::Quat::Identity())},
                            1.0 + static_cast<double>(i));

    EXPECT_TRUE(reg.has("rail_origin", "rotor_opti_a"));
    EXPECT_FALSE(reg.has("rail_origin", "rotor_opti_b"));
    EXPECT_FALSE(reg.has("rail_origin", "rotor_opti")) << "fused from half its inputs";

    for (std::size_t i = 0; i < 20; ++i)
        placer.onMocapFrame(
            {body(2, frames::Vec3(1.02, 0, 0) + jitter(i), frames::Quat::Identity())},
            100.0 + static_cast<double>(i));

    ASSERT_TRUE(reg.has("rail_origin", "rotor_opti"));
    EXPECT_NEAR(reg.require("rail_origin", "rotor_opti").translation().x(), 1.01, 1e-4);
}

// ---------------------------------------------------------------
// reporting
// ---------------------------------------------------------------

TEST(Placement, StatusNamesWhatItIsWaitingFor)
{
    // "Nothing is rendering" and "one asset id is wrong" look identical from
    // the outside unless the waiting list is printed.
    TempScene t;
    frames::Registry reg;
    Placer placer(railAndRotor(t), reg, LatchPolicy::immediate());

    const std::string s = placer.status();
    EXPECT_NE(s.find("NOT PLACED"), std::string::npos);
    EXPECT_NE(s.find("rotor_opti"), std::string::npos);

    placer.onMocapFrame({body(1, {1, 0, 0}, frames::Quat::Identity()),
                         body(2, {2, 0, 0}, frames::Quat::Identity())},
                        1.0);
    placer.onExpectedPose("rotor_expected", {2, 0, 0}, frames::Quat::Identity(), 1.0);

    EXPECT_TRUE(placer.complete());
    EXPECT_NE(placer.status().find("all placements positioned"), std::string::npos);
}

// ===============================================================
// joint projection
// ===============================================================
//
// The projection pass runs on the mocap thread, inside onMocapFrame, from THAT
// FRAME's observations. Most of what is worth testing here is not the arithmetic
// -- test_joint_projection.cpp covers that -- but the plumbing around it, whose
// failures are quiet: a stale input silently paired with a fresh one, a chain
// evaluated in manifest order rather than dependency order, a dropout that
// blanks a number instead of greying it.

namespace {

// The hand chain from config/objects/hand_left_*.json: base -> J8 -> J9, three
// tracked bodies and two revolute joints, continuous throughout.
//
// J8's body frame sits 120 mm OFF its own axis -- the real hardware's case, where
// the marker body's frame is a machined plate corner rather than a kinematic
// datum, so it ORBITS as the joint turns. J9's sits ON its axis, so one fixture
// covers both shapes.
jointproj::RevoluteJoint j8Joint()
{
    jointproj::RevoluteJoint j;
    j.axis_point_m  = {0.0, 0.0805, 0.0};
    j.zero_origin_m = {0.120, 0.0805, 0.0};   // 120 mm off the axis
    j.axis          = frames::Vec3::UnitZ();
    j.lower_rad     = -1.5707;
    j.upper_rad     = 1.5707;
    return j;
}

jointproj::RevoluteJoint j9Joint()
{
    jointproj::RevoluteJoint j;
    j.axis_point_m  = {0.0, 0.270, 0.0};
    j.zero_origin_m = {0.0, 0.270, 0.0};   // on the axis
    j.axis          = frames::Vec3::UnitZ();
    j.lower_rad     = -1.5707;
    j.upper_rad     = 1.5707;
    return j;
}

scene::Scene handChain(const TempScene& t, bool listJ9First = false)
{
    t.manifest(listJ9First ? R"({"world_anchor": "rail_origin",
              "objects": ["rail", "hand_j9", "hand_j8", "hand_base"]})"
                           : R"({"world_anchor": "rail_origin",
              "objects": ["rail", "hand_base", "hand_j8", "hand_j9"]})");

    t.object("rail", R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "static"}]})");

    t.object("hand_base", R"({
        "id": "hand_base",
        "placements": [{
            "id": "base_opti", "source": "optitrack", "capture": "continuous", "asset_id": 1}]
    })");

    t.object("hand_j8", R"({
        "id": "hand_j8",
        "placements": [
            {"id": "j8_opti", "source": "optitrack", "capture": "continuous", "asset_id": 2},
            {"id": "j8_projected", "source": "projected",
             "parent_frame": "base_opti", "measured": "j8_opti",
             "joint": {
                 "zero_pose":  {"position_mm": [120.0, 80.5, 0.0]},
                 "axis_point": {"position_mm": [0.0, 80.5, 0.0]},
                 "axis": [0.0, 0.0, 1.0],
                 "lower_deg": -89.994481, "upper_deg": 89.994481,
                 "reported_arm": "Left", "reported_index": 0}}
        ]})");

    t.object("hand_j9", R"({
        "id": "hand_j9",
        "placements": [
            {"id": "j9_opti", "source": "optitrack", "capture": "continuous", "asset_id": 3},
            {"id": "j9_projected", "source": "projected",
             "parent_frame": "j8_projected", "measured": "j9_opti",
             "joint": {
                 "zero_pose":  {"position_mm": [0.0, 270.0, 0.0]},
                 "axis_point": {"position_mm": [0.0, 270.0, 0.0]},
                 "axis": [0.0, 0.0, 1.0],
                 "lower_deg": -89.994481, "upper_deg": 89.994481,
                 "reported_arm": "Left", "reported_index": 1}}
        ]})");

    return t.load();
}

// Where each body physically is, for a hand at (theta8, theta9) with its base
// plate at `T_mocap_base`. Built through the same joint model the placer will
// invert, so a passing test says the round trip closes.
struct HandPose {
    frames::Transform base, j8, j9;
};

HandPose handAt(double theta8, double theta9, const frames::Transform& T_mocap_base)
{
    HandPose h;
    h.base = T_mocap_base;
    h.j8   = frames::compose(h.base, j8Joint().at(T_mocap_base.from, "j8", theta8));
    h.j9   = frames::compose(h.j8, j9Joint().at("j8", "j9", theta9));
    return h;
}

frames::Transform basePose(double stamp = 1.0)
{
    return frames::make("optitrack_world", "base", {1.84, -0.37, 1.62},
                        quat(1.13, {0.41, -0.62, 0.67}), stamp);
}

std::vector<BodyObservation> handBodies(const HandPose& h)
{
    return {body(1, h.base.translation(), h.base.rotation()),
            body(2, h.j8.translation(), h.j8.rotation()),
            body(3, h.j9.translation(), h.j9.rotation())};
}

}   // namespace

TEST(Placement, ProjectionRecoversBothJointAngles)
{
    TempScene t;
    frames::Registry reg;
    Placer placer(handChain(t), reg, LatchPolicy::immediate());

    const HandPose h = handAt(0.61, -0.42, basePose());
    placer.onMocapFrame(handBodies(h), 1.0);

    const auto est = placer.jointEstimates();
    ASSERT_EQ(est.size(), 2u);

    EXPECT_TRUE(est.at("j8_projected").estimated);
    EXPECT_NEAR(est.at("j8_projected").theta_rad, 0.61, 1e-9);
    EXPECT_TRUE(est.at("j9_projected").estimated);
    EXPECT_NEAR(est.at("j9_projected").theta_rad, -0.42, 1e-9);

    // A pose built exactly on the model has no error of any kind.
    for (const char* id : {"j8_projected", "j9_projected"})
    {
        EXPECT_NEAR(est.at(id).residual_m, 0.0, 1e-9) << id;
        EXPECT_NEAR(est.at(id).radial_error_m, 0.0, 1e-9) << id;
        EXPECT_NEAR(est.at(id).axial_error_m, 0.0, 1e-9) << id;
    }

    // And the corrected chain resolves against the anchor, through two hops.
    const auto placed = reg.lookup("rail_origin", "j9_projected");
    ASSERT_TRUE(placed.has_value());
    EXPECT_NEAR((placed->translation() - h.j9.translation()).norm(), 0.0, 1e-9);
    EXPECT_TRUE(placer.complete());
}

TEST(Placement, ProjectionCarriesJointIdentityFromConfig)
{
    // reported_arm / reported_index are the only place an arm is named. If they
    // did not travel with the estimate, every consumer would have to re-open the
    // scene to find out which controller value to compare against.
    TempScene t;
    frames::Registry reg;
    Placer placer(handChain(t), reg, LatchPolicy::immediate());

    const auto est = placer.jointEstimates();
    EXPECT_EQ(est.at("j8_projected").reported_arm, "Left");
    EXPECT_EQ(est.at("j8_projected").reported_index, 0);
    EXPECT_EQ(est.at("j9_projected").reported_index, 1);
    EXPECT_EQ(est.at("j9_projected").object_id, "hand_j9");
}

TEST(Placement, EstimatesExistBeforeAnyMocapFrame)
{
    // "No estimate yet" and "an estimate of zero" must not look alike. A joint
    // absent from the map would be indistinguishable from one that does not
    // exist in the scene at all.
    TempScene t;
    frames::Registry reg;
    Placer placer(handChain(t), reg, LatchPolicy::immediate());

    const auto est = placer.jointEstimates();
    ASSERT_EQ(est.size(), 2u);
    for (const auto& [id, e] : est)
    {
        EXPECT_FALSE(e.estimated) << id;
        EXPECT_FALSE(e.skip_reason.empty()) << id;
    }
}

TEST(Placement, ProjectionUsesThisFrameNotTheLatestPose)
{
    // THE HEADLINE TEST. The registry holds each placement's LATEST pose, so a
    // projection that read from it would pair a dropped-out body with a fresh
    // parent as though the two were simultaneous. The hand moves all session;
    // 500 mm/s x 20 ms is 10 mm, the same size as the error being measured.
    TempScene t;
    frames::Registry reg;
    Placer placer(handChain(t), reg, LatchPolicy::immediate());

    placer.onMocapFrame(handBodies(handAt(0.61, -0.42, basePose(1.0))), 1.0);

    const auto after1 = reg.lookup("rail_origin", "j9_projected");
    ASSERT_TRUE(after1.has_value());

    // Frame 2: the hand has MOVED and J9's body is gone.
    const HandPose moved = handAt(0.90, 0.15, basePose(2.0));
    placer.onMocapFrame({body(1, moved.base.translation(), moved.base.rotation()),
                         body(2, moved.j8.translation(), moved.j8.rotation())},
                        2.0);

    const auto est = placer.jointEstimates();

    // J8 still has both its inputs, so it updates.
    EXPECT_TRUE(est.at("j8_projected").estimated);
    EXPECT_NEAR(est.at("j8_projected").theta_rad, 0.90, 1e-9);

    // J9 does not, and must say so rather than silently recomputing against the
    // fresh J8 -- which would have produced a plausible, entirely wrong angle.
    EXPECT_FALSE(est.at("j9_projected").estimated);
    EXPECT_NE(est.at("j9_projected").skip_reason.find("j9_opti"), std::string::npos);

    // The last good angle is retained, so a UI greys it rather than zeroing it.
    EXPECT_NEAR(est.at("j9_projected").theta_rad, -0.42, 1e-9);
    EXPECT_EQ(est.at("j9_projected").stamp, 1.0);

    // And its registry edge is untouched: stale, not wrong. Staleness travels on
    // the stamp, which composition propagates as the oldest input, so a consumer
    // that cares can still tell.
    const auto after2 = reg.lookup("j8_projected", "j9_projected");
    ASSERT_TRUE(after2.has_value());
    EXPECT_EQ(after2->stamp, 1.0);
}

TEST(Placement, ProjectionDistinguishesTheTwoDropoutKinds)
{
    // optitrack.cpp forwards untracked bodies rather than dropping them,
    // precisely so this distinction survives to here: "never arrived" is a wrong
    // asset id or a dead stream, "arrived untracked" is an occlusion.
    TempScene t;
    frames::Registry reg;
    Placer placer(handChain(t), reg, LatchPolicy::immediate());

    const HandPose h = handAt(0.2, 0.3, basePose());

    placer.onMocapFrame({body(1, h.base.translation(), h.base.rotation()),
                         body(2, h.j8.translation(), h.j8.rotation(), /*tracked=*/false)},
                        1.0);
    const std::string untracked = placer.jointEstimate("j8_projected")->skip_reason;

    placer.onMocapFrame({body(1, h.base.translation(), h.base.rotation())}, 2.0);
    const std::string absent = placer.jointEstimate("j8_projected")->skip_reason;

    EXPECT_FALSE(untracked.empty());
    EXPECT_FALSE(absent.empty());
    EXPECT_NE(untracked, absent);
    EXPECT_NE(untracked.find("not being tracked"), std::string::npos);
    EXPECT_NE(absent.find("not streamed"), std::string::npos);
}

TEST(Placement, AMissingInputCascadesDownTheChain)
{
    // J9 hangs off the CORRECTED J8, so losing J8's body must take J9 with it --
    // and say which one caused it. Projecting J9 against a stale J8 instead
    // would be exactly the silent error this design exists to avoid.
    TempScene t;
    frames::Registry reg;
    Placer placer(handChain(t), reg, LatchPolicy::immediate());

    const HandPose h = handAt(0.2, 0.3, basePose());
    placer.onMocapFrame({body(1, h.base.translation(), h.base.rotation()),
                         body(3, h.j9.translation(), h.j9.rotation())},
                        1.0);

    const auto est = placer.jointEstimates();
    EXPECT_FALSE(est.at("j8_projected").estimated);
    EXPECT_FALSE(est.at("j9_projected").estimated);
    EXPECT_NE(est.at("j9_projected").skip_reason.find("j8_projected"), std::string::npos);
}

TEST(Placement, ProjectionOrderDoesNotFollowTheManifest)
{
    // The manifest is a description, not a program. Listing J9 before J8 must
    // give bit-identical results -- otherwise the evaluation order is an
    // accident of file ordering, and a chain would break on a reordering that
    // looks purely cosmetic.
    TempScene a, b;
    frames::Registry regA, regB;
    Placer inOrder(handChain(a, /*listJ9First=*/false), regA, LatchPolicy::immediate());
    Placer reversed(handChain(b, /*listJ9First=*/true), regB, LatchPolicy::immediate());

    const auto bodies = handBodies(handAt(-0.33, 1.02, basePose()));
    inOrder.onMocapFrame(bodies, 1.0);
    reversed.onMocapFrame(bodies, 1.0);

    for (const char* id : {"j8_projected", "j9_projected"})
    {
        const auto x = inOrder.jointEstimate(id);
        const auto y = reversed.jointEstimate(id);
        ASSERT_TRUE(x.has_value()) << id;
        ASSERT_TRUE(y.has_value()) << id;
        ASSERT_TRUE(x->estimated) << id;
        ASSERT_TRUE(y->estimated) << id;
        EXPECT_EQ(x->theta_rad, y->theta_rad) << id;
        EXPECT_EQ(x->residual_m, y->residual_m) << id;
    }
}

TEST(Placement, ProjectedAnglesDoNotDependOnTheMocapWorldOrigin)
{
    // The same argument the anchor rests on, restated for angles: theta comes
    // from the RELATIVE rotation of two bodies, so recalibrating Motive -- which
    // premultiplies every body by some unknown M -- cannot change it.
    TempScene a, b;
    frames::Registry regA, regB;
    Placer plain(handChain(a), regA, LatchPolicy::immediate());
    Placer moved(handChain(b), regB, LatchPolicy::immediate());

    const HandPose h          = handAt(0.77, -1.10, basePose());
    const frames::Transform M = frames::make("optitrack_world", "optitrack_world_old",
                                             {-7.3, 2.9, 0.44}, quat(2.4, {0.5, 0.5, 0.7071}));

    std::vector<BodyObservation> shifted;
    for (const auto& o : handBodies(h))
    {
        const frames::Transform t =
            frames::compose(M, frames::make("optitrack_world_old", "b", o.position, o.rotation));
        shifted.push_back(body(o.asset_id, t.translation(), t.rotation()));
    }

    plain.onMocapFrame(handBodies(h), 1.0);
    moved.onMocapFrame(shifted, 1.0);

    for (const char* id : {"j8_projected", "j9_projected"})
    {
        const auto a = plain.jointEstimate(id);
        const auto b = moved.jointEstimate(id);
        ASSERT_TRUE(a.has_value() && b.has_value()) << id;
        EXPECT_NEAR(a->theta_rad, b->theta_rad, 1e-9) << id;
        EXPECT_NEAR(a->radial_error_m, b->radial_error_m, 1e-9) << id;
    }
}

TEST(Placement, ProjectionNeedsNoAnchor)
{
    // theta is a relative quantity, so it must be available before the scene
    // anchor is placed -- during bring-up there may be no rail body at all.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "hand_base", "hand_j8"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "optitrack", "capture": "latched",
                        "asset_id": 9}]})");
    t.object("hand_base", R"({
        "id": "hand_base",
        "placements": [{"id": "base_opti", "source": "optitrack", "capture": "continuous",
                        "asset_id": 1}]})");
    t.object("hand_j8", R"({
        "id": "hand_j8",
        "placements": [
            {"id": "j8_opti", "source": "optitrack", "capture": "continuous", "asset_id": 2},
            {"id": "j8_projected", "source": "projected",
             "parent_frame": "base_opti", "measured": "j8_opti",
             "joint": {
                 "zero_pose":  {"position_mm": [120.0, 80.5, 0.0]},
                 "axis_point": {"position_mm": [0.0, 80.5, 0.0]},
                 "axis": [0.0, 0.0, 1.0],
                 "reported_arm": "Left", "reported_index": 0}}
        ]})");

    frames::Registry reg;
    Placer placer(t.load(), reg, LatchPolicy::immediate());

    const HandPose h = handAt(0.44, 0.0, basePose());
    // Asset 9 is never streamed, so the anchor is never placed.
    placer.onMocapFrame({body(1, h.base.translation(), h.base.rotation()),
                         body(2, h.j8.translation(), h.j8.rotation())},
                        1.0);

    const auto e = placer.jointEstimate("j8_projected");
    ASSERT_TRUE(e.has_value());
    EXPECT_FALSE(placer.anchorPlaced());
    EXPECT_TRUE(e->estimated);
    EXPECT_NEAR(e->theta_rad, 0.44, 1e-9);
}

TEST(Placement, GeometryErrorSurvivesTheJointBeingUsed)
{
    // The property that makes radial/axial the geometry check rather than just
    // more residual: a fixed misplacement must read the SAME at every angle. If
    // it moved with theta, a misplaced axis and a working joint would be
    // indistinguishable.
    TempScene t;
    frames::Registry reg;
    Placer placer(handChain(t), reg, LatchPolicy::immediate());

    double stamp = 1.0;

    for (const double theta8 : {-1.2, -0.4, 0.0, 0.5, 1.3})
    {
        const HandPose h = handAt(theta8, 0.0, basePose());

        // 4 mm along the joint axis -- which is +Z in the BASE BODY's frame, not
        // in mocap's. Pushing along mocap Z instead would land partly radial and
        // the expected values below would be wrong for a reason unrelated to the
        // code under test.
        const frames::Vec3 push = 0.004 * (h.base.rotation() * frames::Vec3::UnitZ());

        placer.onMocapFrame({body(1, h.base.translation(), h.base.rotation()),
                             body(2, h.j8.translation() + push, h.j8.rotation()),
                             body(3, h.j9.translation(), h.j9.rotation())},
                            stamp += 1.0);

        const auto e = placer.jointEstimate("j8_projected");
        ASSERT_TRUE(e.has_value()) << "theta8 = " << theta8;
        ASSERT_TRUE(e->estimated) << "theta8 = " << theta8;
        EXPECT_NEAR(e->axial_error_m, 0.004, 1e-9) << "theta8 = " << theta8;
        EXPECT_NEAR(e->radial_error_m, 0.0, 1e-9) << "theta8 = " << theta8;
        EXPECT_NEAR(e->theta_rad, theta8, 1e-9) << "theta8 = " << theta8;
    }
}

// ---------------------------------------------------------------
// constructed placements
// ---------------------------------------------------------------
//
// Two rigid bodies on one part, and the part's own pose out the other side. The
// rail and the rotor both work this way and both are exercised here, because
// they stress opposite things: the rail's chord runs ALONG its axis over 10 m,
// the rotor's runs ACROSS its axis and its origin is ~1.9 m off that chord.
//
// What these pin is that NOTHING about a constructed pose looks wrong when the
// CAD behind it is wrong -- it comes out smooth, stable and confidently
// misplaced -- so the checks have to be what fails, and for the right reason.

namespace {

constexpr double kRotorRadius = 0.7;    // spin axis to a mount's body origin
constexpr double kRotorFaceX  = 1.83;   // part origin to the mounted face

// A rotor mount's body origin in the PART's frame: +X down the spin axis,
// clocking measured about +X starting from +Z.
frames::Vec3 rotorMount(double clocking)
{
    return {kRotorFaceX, -kRotorRadius * std::sin(clocking), kRotorRadius * std::cos(clocking)};
}

// A bare JSON array. `normal_in_part` is a direction, so it carries no units and
// gets no conversion -- unlike the mount points, which are authored in mm.
std::string vec3(const frames::Vec3& v)
{
    return "[" + detail::fixed(v.x(), 6) + ", " + detail::fixed(v.y(), 6) + ", " +
           detail::fixed(v.z(), 6) + "]";
}

std::string mm3(const frames::Vec3& v)
{
    return "[" + detail::fixed(v.x() * 1000.0, 4) + ", " + detail::fixed(v.y() * 1000.0, 4) + ", " +
           detail::fixed(v.z() * 1000.0, 4) + "]";
}

// A part carrying two bodies, with a static identity anchor so that anchor
// coordinates and mocap coordinates coincide and the arithmetic stays readable.
scene::Scene twoBodyPart(const TempScene& t, const frames::Vec3& mountA, const frames::Vec3& mountB,
                         const frames::Vec3& normalInPart, const char* capture = "continuous")
{
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "part"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("part", std::string(R"({
        "id": "part",
        "placements": [
            {"id": "body_a", "source": "optitrack", "capture": ")") +
                         capture + R"(",
             "asset_id": 2, "compare": false, "visible": false},
            {"id": "body_b", "source": "optitrack", "capture": ")" +
                         capture + R"(",
             "asset_id": 3, "compare": false, "visible": false},
            {"id": "part_opti", "source": "constructed", "capture": ")" +
                         capture + R"(",
             "inputs": ["body_a", "body_b"],
             "construction": {
                "normal_axis": [0.0, 0.0, 1.0],
                "normal_in_part": )" +
                         vec3(normalInPart) + R"(,
                "mount_a": {"position_mm": )" +
                         mm3(mountA) + R"(},
                "mount_b": {"position_mm": )" +
                         mm3(mountB) + R"(}}}
        ]})");
    return t.load();
}

// A part pose aligned with nothing, so no sign error can hide behind a zero.
frames::Transform partTruth()
{
    return frames::make("mocap", "part", frames::Vec3(1.234, -0.567, 2.345),
                        quat(0.7, frames::Vec3(0.3, -0.5, 0.81)));
}

// The two bodies Motive would stream: each seated with its own +Z along
// `normalInPart`, plus an arbitrary spin about that which nothing may depend on.
std::vector<BodyObservation> partBodies(const frames::Transform& truth, const frames::Vec3& mountA,
                                        const frames::Vec3& mountB,
                                        const frames::Vec3& normalInPart, double spinA = 0.0,
                                        double spinB = 0.0)
{
    const frames::Quat align =
        frames::Quat::FromTwoVectors(frames::Vec3::UnitZ(), normalInPart.normalized());
    const auto rot = [&](double spin) { return quat(spin, normalInPart) * align; };

    const frames::Transform a =
        frames::compose(truth, frames::make(truth.from, "a", mountA, rot(spinA)));
    const frames::Transform b =
        frames::compose(truth, frames::make(truth.from, "b", mountB, rot(spinB)));
    return {body(2, a.translation(), a.rotation()), body(3, b.translation(), b.rotation())};
}

void expectMatchesTruth(const frames::Transform& got, const frames::Transform& truth, double tol)
{
    EXPECT_NEAR((got.translation() - truth.translation()).norm(), 0.0, tol);
    // Compared as quaternions rather than by composing: `got` is tagged with the
    // anchor frame and `truth` with the mocap frame, and here those coincide
    // numerically without the frame tags knowing it.
    EXPECT_NEAR(got.rotation().angularDistance(truth.rotation()), 0.0, tol);
}

}   // namespace

TEST(Placement, ConstructionRecoversARotorPoseFromTwoMounts)
{
    // Chord ACROSS the spin axis. The part's origin is ~1.9 m from that chord and
    // on neither side of it by accident.
    TempScene t;
    frames::Registry reg;
    const frames::Vec3 a = rotorMount(0.0), b = rotorMount(kPi / 2.0);
    Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitX()), reg, LatchPolicy::immediate());

    const frames::Transform truth = partTruth();
    placer.onMocapFrame(partBodies(truth, a, b, frames::Vec3::UnitX()), 1.0);

    const auto got = reg.lookup("rail_origin", "part_opti");
    ASSERT_TRUE(got.has_value());
    expectMatchesTruth(*got, truth, 1e-9);
}

TEST(Placement, ConstructionRecoversARailPoseFromTwoEndBodies)
{
    // Chord ALONG the axis, 10 m of it, with the bodies 50 mm above the rail's
    // own origin plane. Same code path, no special case.
    TempScene t;
    frames::Registry reg;
    const frames::Vec3 a(-5.0, 0.0, 0.05), b(5.0, 0.0, 0.05);
    Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitZ()), reg, LatchPolicy::immediate());

    const frames::Transform truth = partTruth();
    placer.onMocapFrame(partBodies(truth, a, b, frames::Vec3::UnitZ()), 1.0);

    const auto got = reg.lookup("rail_origin", "part_opti");
    ASSERT_TRUE(got.has_value());
    expectMatchesTruth(*got, truth, 1e-9);
}

TEST(Placement, ConstructionIgnoresEachBodysArbitrarySpin)
{
    // Motive picks each body's orientation at creation time and nobody controls
    // it. Only the face normal is used, so spinning either body about it must
    // change nothing -- which is why this construction asserts one axis rather
    // than a full rotation it could not check.
    TempScene t;
    frames::Registry reg;
    const frames::Vec3 a(-5.0, 0.0, 0.05), b(5.0, 0.0, 0.05);
    Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitZ()), reg, LatchPolicy::immediate());

    const frames::Transform truth = partTruth();
    placer.onMocapFrame(partBodies(truth, a, b, frames::Vec3::UnitZ(), 1.9, -2.7), 1.0);

    const auto got = reg.lookup("rail_origin", "part_opti");
    ASSERT_TRUE(got.has_value());
    expectMatchesTruth(*got, truth, 1e-9);
}

TEST(Placement, ConstructionReportIsSeededBeforeAnyFrame)
{
    // "waiting on its bodies" and "built, and the geometry checks out" must not
    // both render as silence.
    TempScene t;
    frames::Registry reg;
    const frames::Vec3 a(-5.0, 0.0, 0.05), b(5.0, 0.0, 0.05);
    Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitZ()), reg, LatchPolicy::immediate());

    const auto r = placer.constructionReport("part_opti");
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(r->constructed);
    EXPECT_FALSE(r->skip_reason.empty());
    EXPECT_NEAR(r->chord_expected_m, 10.0, 1e-6);   // known from config alone
}

TEST(Placement, ConstructionWaitsForBothBodiesAndNamesTheMissingOne)
{
    TempScene t;
    frames::Registry reg;
    const frames::Vec3 a(-5.0, 0.0, 0.05), b(5.0, 0.0, 0.05);
    Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitZ()), reg, LatchPolicy::immediate());

    auto bodies = partBodies(partTruth(), a, b, frames::Vec3::UnitZ());
    bodies.pop_back();   // body B never arrives
    placer.onMocapFrame(bodies, 1.0);

    EXPECT_FALSE(reg.lookup("rail_origin", "part_opti").has_value());

    const auto r = placer.constructionReport("part_opti");
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(r->constructed);
    EXPECT_NE(r->skip_reason.find("body_b"), std::string::npos) << r->skip_reason;
}

TEST(Placement, ConstructionChecksAreCleanOnGoodGeometry)
{
    TempScene t;
    frames::Registry reg;
    const frames::Vec3 a(-5.0, 0.0, 0.05), b(5.0, 0.0, 0.05);
    Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitZ()), reg, LatchPolicy::immediate());

    placer.onMocapFrame(partBodies(partTruth(), a, b, frames::Vec3::UnitZ()), 1.0);

    const auto r = placer.constructionReport("part_opti");
    ASSERT_TRUE(r.has_value());
    ASSERT_TRUE(r->constructed) << r->skip_reason;
    EXPECT_NEAR(r->chord_error_m, 0.0, 1e-9);
    EXPECT_NEAR(r->normal_disagreement_rad, 0.0, 1e-9);
    EXPECT_NEAR(r->chord_out_of_plane_m, 0.0, 1e-9);
    EXPECT_NEAR(r->chord_measured_m, r->chord_expected_m, 1e-9);
    EXPECT_DOUBLE_EQ(r->stamp, 1.0);
}

TEST(Placement, AMiscountedGrooveMovesTheOriginAndOnlyTheChordCatchesIt)
{
    // THE test for this feature. The two configured mount points encode which
    // grooves the rotor's mounts occupy, so a miscount corrupts the part's origin
    // -- while producing a pose that is perfectly smooth and stable. This is the
    // cost of the construction, and this is what pays it.
    TempScene t;
    frames::Registry reg;

    // What the config believes.
    const frames::Vec3 a = rotorMount(0.0), b = rotorMount(kPi / 2.0);
    Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitX()), reg, LatchPolicy::immediate());

    // Where mount B actually sits: ten degrees further round.
    const frames::Vec3 bReal = rotorMount(kPi / 2.0 + 10.0 * kPi / 180.0);

    const frames::Transform truth = partTruth();
    placer.onMocapFrame(partBodies(truth, a, bReal, frames::Vec3::UnitX()), 1.0);

    const auto got = reg.lookup("rail_origin", "part_opti");
    ASSERT_TRUE(got.has_value());

    // The origin is out by centimetres...
    EXPECT_GT((got->translation() - truth.translation()).norm(), 0.02);

    // ...and nothing about the pose says so. The chord does.
    const auto r = placer.constructionReport("part_opti");
    ASSERT_TRUE(r.has_value());
    ASSERT_TRUE(r->constructed);
    EXPECT_GT(std::abs(r->chord_error_m), 0.07);

    // The other two stay clean: both mounts are still seated flat on one face.
    // Reporting three separate checks is what makes that distinction sayable
    // instead of blending it into a single unexplained number.
    EXPECT_NEAR(r->normal_disagreement_rad, 0.0, 1e-9);
    EXPECT_NEAR(r->chord_out_of_plane_m, 0.0, 1e-9);
}

TEST(Placement, AnUnseatedBodyShowsInTheNormalDisagreement)
{
    TempScene t;
    frames::Registry reg;
    const frames::Vec3 a(-5.0, 0.0, 0.05), b(5.0, 0.0, 0.05);
    Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitZ()), reg, LatchPolicy::immediate());

    auto bodies = partBodies(partTruth(), a, b, frames::Vec3::UnitZ());

    // Rock body B about the chord by half a degree, leaving its position alone.
    constexpr double kRock      = 0.5 * kPi / 180.0;
    const frames::Vec3 chordHat = (bodies[1].position - bodies[0].position).normalized();
    bodies[1].rotation          = quat(kRock, chordHat) * bodies[1].rotation;

    placer.onMocapFrame(bodies, 1.0);

    const auto r = placer.constructionReport("part_opti");
    ASSERT_TRUE(r.has_value());
    ASSERT_TRUE(r->constructed);
    EXPECT_NEAR(r->normal_disagreement_rad, kRock, 1e-9);
    EXPECT_NEAR(r->chord_error_m, 0.0, 1e-9);   // it tilted; it did not MOVE
}

TEST(Placement, ConstructionIsLatchedOnceComputed)
{
    // A latched construction is settled once and then holds, like any other
    // latched placement. Later frames must not quietly recompute it -- which for
    // the rail matters most of all, since it is the anchor.
    TempScene t;
    frames::Registry reg;
    const frames::Vec3 a(-5.0, 0.0, 0.05), b(5.0, 0.0, 0.05);
    Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitZ(), "latched"), reg,
                  LatchPolicy::immediate());

    const frames::Transform truth = partTruth();
    placer.onMocapFrame(partBodies(truth, a, b, frames::Vec3::UnitZ()), 1.0);

    const auto first = reg.lookup("rail_origin", "part_opti");
    ASSERT_TRUE(first.has_value());
    expectMatchesTruth(*first, truth, 1e-9);

    // Now move the whole part half a metre. Its bodies are latched, so nothing
    // downstream of them may move either.
    const frames::Transform moved = frames::make(
        "mocap", "part", truth.translation() + frames::Vec3(0.5, 0.0, 0.0), truth.rotation());
    placer.onMocapFrame(partBodies(moved, a, b, frames::Vec3::UnitZ()), 2.0);

    const auto again = reg.lookup("rail_origin", "part_opti");
    ASSERT_TRUE(again.has_value());
    EXPECT_NEAR((again->translation() - first->translation()).norm(), 0.0, 1e-12);
}

TEST(Placement, ConstructedPoseDoesNotDependOnTheMocapWorldOrigin)
{
    // Same claim the rest of the anchoring scheme rests on. The construction is
    // built entirely from measured directions and distances, so recalibrating
    // Motive must cancel out of any placement-relative query.
    const frames::Transform M = frames::make("mocap2", "mocap", frames::Vec3(-3.0, 7.5, 1.25),
                                             quat(1.1, frames::Vec3(0.2, 0.9, -0.3)));
    const frames::Vec3 a(-5.0, 0.0, 0.05), b(5.0, 0.0, 0.05);

    auto run = [&](bool shifted) {
        TempScene t;
        frames::Registry reg;
        Placer placer(twoBodyPart(t, a, b, frames::Vec3::UnitZ()), reg, LatchPolicy::immediate());

        auto bodies = partBodies(partTruth(), a, b, frames::Vec3::UnitZ());
        if (shifted)
            for (auto& obs : bodies)
            {
                const frames::Transform out =
                    frames::compose(M, frames::make("mocap", "b", obs.position, obs.rotation));
                obs.position = out.translation();
                obs.rotation = out.rotation();
            }
        placer.onMocapFrame(bodies, 1.0);

        return reg.require("body_a", "part_opti");
    };

    const frames::Transform plain = run(false);
    const frames::Transform moved = run(true);

    EXPECT_NEAR((plain.translation() - moved.translation()).norm(), 0.0, 1e-12);
    EXPECT_NEAR(frames::magnitudeOf(frames::compose(frames::inverse(plain), moved)).angle_rad, 0.0,
                1e-12);
}
