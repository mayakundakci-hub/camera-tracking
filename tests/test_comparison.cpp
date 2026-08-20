// Tests for nodes/common/comparison.hpp
//
// This module produces the project's headline numbers, so the tests care about
// two things above all:
//
//   1. that a delta is RIGHT when it is reported, and
//   2. that it is REFUSED rather than fudged when the inputs do not support it.
//
// (2) covers the timing case specifically. Comparing the latest pose of two
// continuously-tracked placements captured 20 ms apart, with the object moving
// at 500 mm/s, yields 10 mm of pure timing error that looks exactly like a
// tracking error. Silently reporting it would be the worst outcome available.

#include <gtest/gtest.h>

#include "comparison.hpp"
#include "object_placement.hpp"

#include <chrono>
#include <fstream>
#include <optional>

using namespace comparison;

namespace {

class TempScene {
public:
    TempScene()
    {
        static int counter = 0;
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        root_ = scene::fs::temp_directory_path() /
                ("ct_cmp_test_" + std::to_string(stamp) + "_" + std::to_string(counter++));
        scene::fs::remove_all(root_);
        scene::fs::create_directories(root_ / "config" / "objects");
    }
    ~TempScene() { std::error_code ec; scene::fs::remove_all(root_, ec); }
    TempScene(const TempScene&) = delete;
    TempScene& operator=(const TempScene&) = delete;

    void manifest(const std::string& b) const { write(root_ / "config" / "scene.json", b); }
    void object(const std::string& n, const std::string& b) const
    {
        write(root_ / "config" / "objects" / (n + ".json"), b);
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

// A rotor located two ways, both LATCHED -- the bolted-down case, where no
// timing relationship exists to get wrong.
scene::Scene latchedPair(const TempScene& t)
{
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("rotor_a", R"({
        "id": "rotor",
        "placements": [
            {"id": "rotor_opti", "source": "optitrack", "capture": "latched", "asset_id": 2},
            {"id": "rotor_expected", "source": "expected_pose", "capture": "latched",
             "topic": "rotor/pose_expected"}
        ]})");
    return t.load();
}

// A hand tracked two ways, both CONTINUOUS -- the case that needs time-matching.
scene::Scene continuousPair(const TempScene& t)
{
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "hand"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("hand", R"({
        "id": "hand",
        "placements": [
            {"id": "hand_opti", "source": "optitrack", "capture": "continuous", "asset_id": 5},
            {"id": "hand_expected", "source": "expected_pose", "capture": "continuous",
             "topic": "robot/tcp"}
        ]})");
    return t.load();
}

frames::Transform at(const std::string& id, const frames::Vec3& p, double stamp)
{
    return frames::make("rail_origin", id, p, frames::Quat::Identity(), stamp);
}

// Returns a COPY, deliberately. compute() hands back a fresh vector by value,
// so a helper returning a pointer into it dangles the moment the full
// expression ends -- which is easy to write and produces garbage rather than
// a crash. Copying a Delta is cheap and makes the misuse impossible.
std::optional<Delta> find(const std::vector<Delta>& ds, const std::string& a)
{
    for (const auto& d : ds)
        if (d.a == a) return d;
    return std::nullopt;
}

}  // namespace

// ---------------------------------------------------------------
// latched pairs -- a plain registry lookup
// ---------------------------------------------------------------

TEST(Comparison, LatchedPairMeasuresTheDistanceBetweenClaims)
{
    TempScene t;
    frames::Registry reg;
    scene::Scene s = latchedPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    // Two claims about one rotor, 3 mm apart in X.
    reg.set(at("rotor_opti", {1.000, 0.0, 0.0}, 1.0));
    reg.set(at("rotor_expected", {1.003, 0.0, 0.0}, 1.0));

    const auto deltas = cmp.compute();
    ASSERT_EQ(deltas.size(), 1u);
    const Delta& d = deltas[0];

    EXPECT_TRUE(d.valid) << d.invalid_reason;
    EXPECT_EQ(d.object_id, "rotor");
    EXPECT_NEAR(d.distance_mm, 3.0, 1e-9);
    EXPECT_NEAR(d.dx_mm, 3.0, 1e-9);
    EXPECT_NEAR(d.angle_deg, 0.0, 1e-9);
    EXPECT_EQ(d.time_gap_s, 0.0);      // latched: no matching involved
}

TEST(Comparison, LatchedPairReportsOrientationDisagreement)
{
    TempScene t;
    frames::Registry reg;
    scene::Scene s = latchedPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    const frames::Quat spun(Eigen::AngleAxisd(0.05, frames::Vec3::UnitZ()));
    reg.set(frames::make("rail_origin", "rotor_opti", frames::Vec3::Zero(),
                          frames::Quat::Identity(), 1.0));
    reg.set(frames::make("rail_origin", "rotor_expected", frames::Vec3::Zero(), spun, 1.0));

    const auto deltas = cmp.compute();
    ASSERT_EQ(deltas.size(), 1u);
    const Delta& d = deltas[0];
    EXPECT_TRUE(d.valid) << d.invalid_reason;
    EXPECT_NEAR(d.distance_mm, 0.0, 1e-9);
    EXPECT_NEAR(d.angle_deg, frames::convert::radToDeg(0.05), 1e-9);
}

TEST(Comparison, UnplacedPairIsInvalidWithAReasonRatherThanOmitted)
{
    // "The rotor delta is missing" and "the rotor delta is 0 mm" must not look
    // the same to anything downstream.
    TempScene t;
    frames::Registry reg;
    scene::Scene s = latchedPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    const auto deltas = cmp.compute();
    ASSERT_EQ(deltas.size(), 1u);
    EXPECT_FALSE(deltas[0].valid);
    EXPECT_FALSE(deltas[0].invalid_reason.empty());
    EXPECT_NE(deltas[0].invalid_reason.find("rotor_opti"), std::string::npos);
}

// ---------------------------------------------------------------
// continuous pairs -- the timing problem
// ---------------------------------------------------------------

TEST(Comparison, ContinuousPairMatchesSamplesInTime)
{
    TempScene t;
    frames::Registry reg;
    scene::Scene s = continuousPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    // A hand sweeping in X. The two streams arrive interleaved and at
    // different rates, as they really do.
    cmp.onPlacementUpdated("hand_expected", at("hand_expected", {0.900, 0, 0}, 0.990));
    cmp.onPlacementUpdated("hand_expected", at("hand_expected", {0.950, 0, 0}, 1.000));
    cmp.onPlacementUpdated("hand_expected", at("hand_expected", {1.000, 0, 0}, 1.010));
    cmp.onPlacementUpdated("hand_opti", at("hand_opti", {0.952, 0, 0}, 1.002));

    auto d = find(cmp.compute(), "hand_opti");
    ASSERT_TRUE(d);
    EXPECT_TRUE(d->valid) << d->invalid_reason;
    EXPECT_NEAR(d->distance_mm, 2.0, 1e-6);
    EXPECT_NEAR(d->time_gap_s, 0.002, 1e-9);
}

TEST(Comparison, ContinuousPairIsRefusedWhenNoSampleIsCloseEnough)
{
    // The failure this module exists to prevent: an object that moved between
    // the two samples produces a number indistinguishable from tracking error.
    TempScene t;
    frames::Registry reg;
    scene::Scene s = continuousPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    cmp.onPlacementUpdated("hand_expected", at("hand_expected", {0.900, 0, 0}, 1.000));
    cmp.onPlacementUpdated("hand_opti", at("hand_opti", {1.400, 0, 0}, 1.500));  // 500 ms later

    auto d = find(cmp.compute(), "hand_opti");
    ASSERT_TRUE(d);
    EXPECT_FALSE(d->valid);
    EXPECT_NE(d->invalid_reason.find("no time match"), std::string::npos);
    EXPECT_NEAR(d->time_gap_s, 0.5, 1e-9);
}

TEST(Comparison, ContinuousPairWaitsForBothStreams)
{
    TempScene t;
    frames::Registry reg;
    scene::Scene s = continuousPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    cmp.onPlacementUpdated("hand_opti", at("hand_opti", {1.0, 0, 0}, 1.0));

    auto d = find(cmp.compute(), "hand_opti");
    ASSERT_TRUE(d);
    EXPECT_FALSE(d->valid);
    EXPECT_NE(d->invalid_reason.find("no samples yet"), std::string::npos);
}

TEST(Comparison, BufferIsBoundedAndKeepsTheNewestSamples)
{
    TempScene t;
    frames::Registry reg;
    scene::Scene s = continuousPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, /*bufferLen=*/4);

    for (int i = 0; i < 50; ++i)
        cmp.onPlacementUpdated("hand_expected",
                               at("hand_expected", {0.001 * i, 0, 0}, 1.0 + 0.001 * i));
    cmp.onPlacementUpdated("hand_opti", at("hand_opti", {0.049, 0, 0}, 1.049));

    auto d = find(cmp.compute(), "hand_opti");
    ASSERT_TRUE(d);
    EXPECT_TRUE(d->valid) << d->invalid_reason;
    EXPECT_NEAR(d->distance_mm, 0.0, 1e-6);
}

// ---------------------------------------------------------------
// it does not know what a rotor is
// ---------------------------------------------------------------

TEST(Comparison, ThreePlacementsProduceThreeDeltas)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("rotor_a", R"({
        "id": "rotor",
        "placements": [
            {"id": "r1", "source": "expected_pose", "capture": "latched", "topic": "a"},
            {"id": "r2", "source": "expected_pose", "capture": "latched", "topic": "b"},
            {"id": "r3", "source": "expected_pose", "capture": "latched", "topic": "c"}
        ]})");

    frames::Registry reg;
    scene::Scene s = t.load();
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    reg.set(at("r1", {0.0, 0, 0}, 1.0));
    reg.set(at("r2", {0.001, 0, 0}, 1.0));
    reg.set(at("r3", {0.002, 0, 0}, 1.0));

    const auto deltas = cmp.compute();
    EXPECT_EQ(deltas.size(), 3u);      // adding a third system is config, not code
    for (const auto& d : deltas)
        EXPECT_TRUE(d.valid) << d.invalid_reason;
}

TEST(Comparison, StaticVersusContinuousDoesNotAttemptTimeMatching)
{
    // `capture` defaults to continuous, so a static placement LOOKS continuous
    // by that field alone -- but its pose is a constant set once at
    // construction and is never buffered. Time-matching against it would fail
    // forever with "no samples yet".
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "demo"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("demo", R"({
        "id": "demo",
        "placements": [
            {"id": "demo_moving", "source": "expected_pose", "capture": "continuous",
             "topic": "pose_fanuc"},
            {"id": "demo_reference", "source": "static",
             "pose": {"position_mm": [0, 0, 1000]}}
        ]})");

    frames::Registry reg;
    scene::Scene s = t.load();
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    // The static side was placed by the Placer's constructor; only the moving
    // side streams. This must still produce a delta.
    placer.onExpectedPose("demo_moving", {0.5, 0.0, 1.0}, frames::Quat::Identity(), 1.0);
    cmp.onPlacementUpdated("demo_moving", at("demo_moving", {0.5, 0.0, 1.0}, 1.0));

    const auto d = find(cmp.compute(), "demo_moving");
    ASSERT_TRUE(d);
    EXPECT_TRUE(d->valid) << d->invalid_reason;
    EXPECT_NEAR(d->distance_mm, 500.0, 1e-6);   // the stub's circle radius
    EXPECT_EQ(d->time_gap_s, 0.0);              // no matching was attempted
}

TEST(Comparison, SinglePlacementObjectsProduceNoDeltas)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");

    frames::Registry reg;
    scene::Scene s = t.load();
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    EXPECT_TRUE(cmp.compute().empty());
}

// ---------------------------------------------------------------
// the startup review gate
//
// review_gated is what the frontend's modal is driven by. It has to select
// exactly the one-shot comparisons and nothing else: too broad and the app
// blocks on a stream that will never "settle", too narrow and the rotor is
// never reviewed at all.
// ---------------------------------------------------------------

TEST(Comparison, LatchedPairIsReviewGated)
{
    // The rotor: decided once, never improves, so it is the case the gate exists
    // for. Nothing in the comparator knows it is a rotor -- only that both sides
    // are latched.
    TempScene t;
    frames::Registry reg;
    scene::Scene s = latchedPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    const auto d = find(cmp.compute(), "rotor_opti");
    ASSERT_TRUE(d);
    EXPECT_TRUE(d->review_gated);
}

TEST(Comparison, ContinuousPairIsNotReviewGated)
{
    // The hand. Gating it would hold the session on a number that never stops
    // changing -- and it needs no gate, because a stream across many arm
    // configurations announces its own problems.
    TempScene t;
    frames::Registry reg;
    scene::Scene s = continuousPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    const auto d = find(cmp.compute(), "hand_opti");
    ASSERT_TRUE(d);
    EXPECT_FALSE(d->review_gated);
}

TEST(Comparison, StaticPairIsNotReviewGatedByDefault)
{
    // The render-test scenes are pairs of config constants. `capture` defaults to
    // continuous for a static placement, which is what keeps them out -- there is
    // no measurement to review, and a modal on every launch of a bring-up scene
    // with no hardware attached would be pure obstruction.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "pair"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("pair", R"({
        "id": "pair",
        "placements": [
            {"id": "left",  "source": "static", "pose": {"position_mm": [0, 0, 0]}},
            {"id": "right", "source": "static", "pose": {"position_mm": [10, 0, 0]}}
        ]})");

    frames::Registry reg;
    scene::Scene s = t.load();
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    const auto d = find(cmp.compute(), "left");
    ASSERT_TRUE(d);
    EXPECT_TRUE(d->valid) << d->invalid_reason;
    EXPECT_FALSE(d->review_gated);
}

TEST(Comparison, ReviewGateTrueOverridesTheAutomaticRule)
{
    // The escape hatch that makes the gate rehearsable. Without it the workflow
    // can only be exercised with Motive streaming and the robot bridge running,
    // because those are the only ways to get a latched pair -- so the one thing
    // that cannot be tested is the screen a human has to act on.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "pair"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("pair", R"({
        "id": "pair",
        "review_gate": true,
        "placements": [
            {"id": "left",  "source": "static", "pose": {"position_mm": [0, 0, 0]}},
            {"id": "right", "source": "static", "pose": {"position_mm": [10, 0, 0]}}
        ]})");

    frames::Registry reg;
    scene::Scene s = t.load();
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    const auto d = find(cmp.compute(), "left");
    ASSERT_TRUE(d);
    EXPECT_TRUE(d->valid) << d->invalid_reason;
    EXPECT_TRUE(d->review_gated);
    EXPECT_NEAR(d->distance_mm, 10.0, 1e-9);
}

TEST(Comparison, ReviewGateFalseSuppressesAnOtherwiseGatedPair)
{
    // The other direction: a latched object the operator does not want to be
    // stopped by. Both sides are latched, so the rule would gate it.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("rotor_a", R"({
        "id": "rotor",
        "review_gate": false,
        "placements": [
            {"id": "rotor_opti", "source": "optitrack", "capture": "latched", "asset_id": 2},
            {"id": "rotor_expected", "source": "expected_pose", "capture": "latched",
             "topic": "rotor/pose_expected"}
        ]})");

    frames::Registry reg;
    scene::Scene s = t.load();
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    const auto d = find(cmp.compute(), "rotor_opti");
    ASSERT_TRUE(d);
    EXPECT_FALSE(d->review_gated);
}

TEST(Comparison, RejectsNonBooleanReviewGate)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "review_gate": "yes",
        "placements": [{"id": "rail_origin", "source": "static"}]})");

    EXPECT_THROW((void)t.load(), scene::SceneConfigError);
}

TEST(Comparison, MixedLatchedAndContinuousPairIsNotReviewGated)
{
    // Only one side settles, so the pair keeps producing new information and
    // there is no moment at which it is "decided". Reviewing it would mean
    // freezing a judgement on a number still in motion.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "thing"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("thing", R"({
        "id": "thing",
        "placements": [
            {"id": "thing_opti", "source": "optitrack", "capture": "latched", "asset_id": 7},
            {"id": "thing_expected", "source": "expected_pose", "capture": "continuous",
             "topic": "thing/pose"}
        ]})");

    frames::Registry reg;
    scene::Scene s = t.load();
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    const auto d = find(cmp.compute(), "thing_opti");
    ASSERT_TRUE(d);
    EXPECT_FALSE(d->review_gated);
}

TEST(Comparison, ReviewGatedIsSetEvenWhenTheDeltaHasNoReading)
{
    // The frontend needs to know a gated comparison EXISTS before it is valid, so
    // it can tell "waiting for the rotor" from "there is no rotor". If this were
    // only populated on success the gate would open the instant the first delta
    // arrived, having never known it was waiting.
    TempScene t;
    frames::Registry reg;
    scene::Scene s = latchedPair(t);   // nothing placed: no poses fed
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    const auto d = find(cmp.compute(), "rotor_opti");
    ASSERT_TRUE(d);
    EXPECT_FALSE(d->valid);
    EXPECT_TRUE(d->review_gated);
}

// ---------------------------------------------------------------
// integration with the placer
// ---------------------------------------------------------------

TEST(Comparison, PlacerObserverFeedsTheHistoryBuffer)
{
    // The wiring in main.cpp: every placement update must reach the
    // comparator, already resolved into the anchor frame.
    TempScene t;
    frames::Registry reg;
    scene::Scene s = continuousPair(t);
    placement::Placer placer(s, reg, placement::LatchPolicy::immediate());
    Comparator cmp(s, reg, 0.020, 256);

    placer.setObserver([&cmp](const std::string& id, const frames::Transform& tr) {
        cmp.onPlacementUpdated(id, tr);
    });

    placement::BodyObservation obs;
    obs.asset_id = 5;
    obs.position = {1.0, 0.0, 0.0};
    obs.rotation = frames::Quat::Identity();
    obs.tracked  = true;
    placer.onMocapFrame({obs}, 1.000);

    placer.onExpectedPose("hand_expected", {1.004, 0.0, 0.0}, frames::Quat::Identity(), 1.001);

    auto d = find(cmp.compute(), "hand_opti");
    ASSERT_TRUE(d);
    EXPECT_TRUE(d->valid) << d->invalid_reason;
    EXPECT_NEAR(d->distance_mm, 4.0, 1e-6);
}
