// Tests for nodes/common/scene_config.hpp
//
// The loader's job is to REFUSE bad configuration, so most of these assert
// that something throws. A scene config that loads with a silent default is
// the failure mode that matters: it renders an incomplete scene that looks
// fine, and every number downstream is quietly wrong.
//
// Each test builds a throwaway config tree on disk, because path resolution
// and file-existence checks are half of what is being tested.

#include <gtest/gtest.h>

#include "scene_config.hpp"

#include <chrono>
#include <fstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace scene;

namespace {

// Mirrors the real layout: <root>/config/scene.json + <root>/config/objects/*.json,
// with assets resolved against <root>.
class TempScene {
public:
    TempScene()
    {
        static int counter = 0;
        const auto stamp   = std::chrono::steady_clock::now().time_since_epoch().count();
        root_ = fs::temp_directory_path() /
                ("ct_scene_test_" + std::to_string(stamp) + "_" + std::to_string(counter++));
        fs::remove_all(root_);
        fs::create_directories(root_ / "config" / "objects");
    }

    ~TempScene()
    {
        std::error_code ec;
        fs::remove_all(root_, ec);
    }

    TempScene(const TempScene&)            = delete;
    TempScene& operator=(const TempScene&) = delete;

    void manifest(const std::string& body) const { write(root_ / "config" / "scene.json", body); }

    void object(const std::string& name, const std::string& body) const
    {
        write(root_ / "config" / "objects" / (name + ".json"), body);
    }

    // Creates an asset file so visual_mesh / urdf existence checks can pass.
    void asset(const std::string& relPath) const
    {
        const fs::path p = root_ / relPath;
        fs::create_directories(p.parent_path());
        write(p, "solid placeholder\nendsolid placeholder\n");
    }

    [[nodiscard]] fs::path manifestPath() const { return root_ / "config" / "scene.json"; }
    [[nodiscard]] Scene load() const { return scene::load(manifestPath()); }

private:
    static void write(const fs::path& p, const std::string& body)
    {
        std::ofstream f(p);
        f << body;
    }

    fs::path root_;
};

// A minimal valid scene: one static anchor, nothing else.
void writeAnchorOnly(const TempScene& t)
{
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "static"}]})");
}

}   // namespace

// ---------------------------------------------------------------
// the happy path
// ---------------------------------------------------------------

TEST(SceneConfig, LoadsMinimalScene)
{
    TempScene t;
    writeAnchorOnly(t);

    const Scene s = t.load();
    EXPECT_EQ(s.anchor_frame, "rail_origin");
    ASSERT_EQ(s.objects.size(), 1u);
    EXPECT_EQ(s.objects[0].id, "rail");
    ASSERT_TRUE(s.findPlacement("rail_origin"));
    EXPECT_EQ(s.findPlacement("rail_origin")->source, Source::Static);
    // No mesh is legitimate: the anchor contributes a frame, not geometry.
    EXPECT_TRUE(s.objects[0].visual_mesh.empty());
}

TEST(SceneConfig, ResolvesAssetPathsAgainstRepoRoot)
{
    TempScene t;
    t.asset("Rendering/Rotor/RotorA.stl");
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("rotor_a", R"({
        "id": "rotor",
        "visual_mesh": "Rendering/Rotor/RotorA.stl",
        "placements": [{"id": "rotor_expected", "source": "expected_pose", "topic": "rotor/pose"}]})");

    const Scene s       = t.load();
    const Object* rotor = s.findObject("rotor");
    ASSERT_TRUE(rotor);
    EXPECT_TRUE(fs::exists(rotor->visual_mesh));
}

TEST(SceneConfig, ConvertsMountOffsetToSi)
{
    // The mm -> m boundary, on the offset that is left: a static placement's
    // own pose. Same parser as visual_offset, zero_pose and axis_point, so one
    // test covers the conversion for all four.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "static",
            "pose": {
                "position_mm": [1000.0, -250.0, 0.0],
                "quat_wxyz": [0.0, 1.0, 0.0, 0.0]}
        }]})");

    const Scene s      = t.load();
    const Placement* p = s.findPlacement("rail_origin");
    ASSERT_TRUE(p);
    // Authored in mm, stored in metres -- everything downstream is SI.
    EXPECT_NEAR(p->pose.translation.x(), 1.0, 1e-12);
    EXPECT_NEAR(p->pose.translation.y(), -0.25, 1e-12);

    // And it hands back a properly framed transform.
    const frames::Transform T = p->pose.toTransform("mount_stage", "rail_origin");
    EXPECT_EQ(T.to, "mount_stage");
    EXPECT_EQ(T.from, "rail_origin");
    EXPECT_NEAR(T.translation().x(), 1.0, 1e-12);
}

TEST(SceneConfig, UnlistedObjectFilesAreIgnored)
{
    // The swap-a-rotor workflow: rotor_b.json can sit in the tree, broken and
    // unvalidated, until it is named in the manifest.
    TempScene t;
    writeAnchorOnly(t);
    t.object("rotor_b", R"({ "this": "is not even a valid object" })");

    EXPECT_NO_THROW((void)t.load());
    const Scene s = t.load();
    EXPECT_FALSE(s.findObject("rotor_b"));
}

// ---------------------------------------------------------------
// comparisons are generated, never configured
// ---------------------------------------------------------------

TEST(SceneConfig, ComparisonsEnumeratePlacementPairs)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("rotor_a", R"({
        "id": "rotor",
        "placements": [
            {"id": "rotor_opti", "source": "optitrack", "capture": "latched", "asset_id": 11},
            {"id": "rotor_expected", "source": "expected_pose", "topic": "rotor/pose_expected"}
        ]})");

    const auto cmps = t.load().comparisons();
    ASSERT_EQ(cmps.size(), 1u);   // the single-placement rail contributes none
    EXPECT_EQ(cmps[0].object_id, "rotor");
    EXPECT_EQ(cmps[0].a, "rotor_opti");
    EXPECT_EQ(cmps[0].b, "rotor_expected");
}

TEST(SceneConfig, ThreePlacementsGiveThreeComparisons)
{
    // A third measurement system is config, not code.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("rotor_a", R"({
        "id": "rotor",
        "placements": [
            {"id": "rotor_a1", "source": "expected_pose", "topic": "a"},
            {"id": "rotor_a2", "source": "expected_pose", "topic": "b"},
            {"id": "rotor_a3", "source": "expected_pose", "topic": "c"}
        ]})");

    EXPECT_EQ(t.load().comparisons().size(), 3u);
}

// ---------------------------------------------------------------
// refusals -- the reason the loader exists
// ---------------------------------------------------------------

TEST(SceneConfig, RejectsDuplicatePlacementId)
{
    // Placement ids are frame names. A duplicate would silently overwrite
    // another object's frame in the registry.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("rotor_a", R"({
        "id": "rotor",
        "placements": [{"id": "rail_origin", "source": "expected_pose", "topic": "x"}]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsAnchorThatIsNotAPlacement)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "not_a_placement", "objects": ["rail"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsMissingObjectFile)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "ghost"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsMissingMesh)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "visual_mesh": "Rendering/does_not_exist.stl",
        "placements": [{"id": "rail_origin", "source": "static"}]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsUnknownSource)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "vicon"}]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsOptitrackPlacementWithoutAssetId)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "capture": "latched"}]
    })");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, OptitrackPlacementNeedsNothingBeyondItsAssetId)
{
    // Motive's rigid-body frame IS the placement's frame. There is no offset to
    // state, because the offset lives in Motive as a hand-set pivot.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "capture": "latched", "asset_id": 7}]
    })");

    const Scene s      = t.load();
    const Placement* p = s.findPlacement("rail_origin");
    ASSERT_TRUE(p);
    EXPECT_EQ(p->asset_id, 7);
}

TEST(SceneConfig, RejectsAPlacementStillCarryingAMountOffset)
{
    // The migration guard. A file written against the old schema is REFUSED
    // rather than ignored: if its offset was not identity, that pose has to be
    // moved into Motive's pivot, and silently dropping it would move the object
    // by exactly the amount someone once measured.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "capture": "latched", "asset_id": 7,
            "model_in_marker_frame": {"position_mm": [100.0, 0.0, 0.0]}}]
    })");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsOptitrackPlacementWithoutCapture)
{
    // Guessing latched-vs-continuous either freezes a moving object or lets a
    // bolted one drift with tracking noise.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "asset_id": 7}]
    })");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsExpectedPosePlacementWithoutTopic)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "expected_pose"}]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsJointStatePlacementWithMissingUrdf)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "p2d2"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("p2d2", R"({
        "id": "p2d2",
        "placements": [{
            "id": "p2d2_fanuc", "source": "joint_state",
            "urdf": "Rendering/nope.urdf", "topic": "robot/joint_state"}]
    })");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, InputPlacementsAreExcludedFromComparisons)
{
    // The real rotor shape: two mounts feeding a fused origin, and only the
    // fused result compared against the robot's claim. Marking the mounts
    // compare=false must collapse six pairs down to one.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor_a"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("rotor_a", R"({
        "id": "rotor",
        "placements": [
            {"id": "rotor_opti_a", "source": "optitrack", "capture": "latched", "asset_id": 1,
             "compare": false},
            {"id": "rotor_opti_b", "source": "optitrack", "capture": "latched", "asset_id": 2,
             "compare": false},
            {"id": "rotor_opti", "source": "fused", "capture": "latched",
             "inputs": ["rotor_opti_a", "rotor_opti_b"]},
            {"id": "rotor_expected", "source": "expected_pose", "capture": "latched",
             "topic": "rotor/pose_expected"}
        ]})");

    const auto cmps = t.load().comparisons();
    ASSERT_EQ(cmps.size(), 1u);
    EXPECT_EQ(cmps[0].a, "rotor_opti");
    EXPECT_EQ(cmps[0].b, "rotor_expected");
}

TEST(SceneConfig, InputPlacementsStillExistAsPlacements)
{
    // compare=false must not remove them from the scene -- they are still
    // measured, framed, and available to the fusion that consumes them.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [
            {"id": "rail_origin", "source": "static"},
            {"id": "helper", "source": "static", "compare": false}
        ]})");

    const Scene s = t.load();
    EXPECT_EQ(s.placements().size(), 2u);
    ASSERT_TRUE(s.findPlacement("helper"));
    EXPECT_FALSE(s.findPlacement("helper")->compare);
    EXPECT_TRUE(s.comparisons().empty());
}

TEST(SceneConfig, RejectsFusedPlacementWithUnknownInput)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [
            {"id": "rail_origin", "source": "static"},
            {"id": "f", "source": "fused", "inputs": ["rail_origin", "typo"]}
        ]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsFusedPlacementWithFewerThanTwoInputs)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [
            {"id": "rail_origin", "source": "static"},
            {"id": "f", "source": "fused", "inputs": ["rail_origin"]}
        ]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsChainedFusion)
{
    // Fusing a fused placement would need cycle detection and a defined
    // evaluation order; it is refused rather than half-supported.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [
            {"id": "rail_origin", "source": "static"},
            {"id": "s2", "source": "static"},
            {"id": "f1", "source": "fused", "inputs": ["rail_origin", "s2"]},
            {"id": "f2", "source": "fused", "inputs": ["f1", "s2"]}
        ]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsObjectWithNoPlacements)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "empty"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("empty", R"({"id": "empty", "placements": []})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsBadColor)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "static", "color": "blue"}]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsUnknownParentFrame)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "static", "parent_frame": "nowhere"}]})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsMalformedJson)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", "{ not json at all ");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

// ---------------------------------------------------------------
// error reporting
// ---------------------------------------------------------------

TEST(SceneConfig, ReportsEveryProblemAtOnce)
{
    // One round-trip per typo is the thing that makes strict validation
    // unpleasant enough to get disabled. All problems come back together.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "visual_mesh": "Rendering/missing.stl",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "color": "octarine"}]
    })");

    try
    {
        (void)t.load();
        FAIL() << "expected SceneConfigError";
    }
    catch (const SceneConfigError& e)
    {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("missing.stl"), std::string::npos);   // bad mesh
        EXPECT_NE(msg.find("octarine"), std::string::npos);      // bad colour
        EXPECT_NE(msg.find("asset_id"), std::string::npos);      // absent asset id
        EXPECT_NE(msg.find("capture"), std::string::npos);       // absent capture
    }
}

// ---------------------------------------------------------------
// per-placement latch thresholds
// ---------------------------------------------------------------

TEST(SceneConfig, ParsesLatchOverridesAndConvertsToSi)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "capture": "latched", "asset_id": 4,

            "latch": {"min_samples": 480, "max_spread_mm": 4.0, "max_spread_deg": 0.25}}]
    })");

    const scene::Scene s      = t.load();
    const scene::Placement* p = s.findPlacement("rail_origin");
    ASSERT_NE(p, nullptr);

    ASSERT_TRUE(p->latch.min_samples.has_value());
    EXPECT_EQ(*p->latch.min_samples, 480u);
    ASSERT_TRUE(p->latch.max_spread_m.has_value());
    EXPECT_NEAR(*p->latch.max_spread_m, 0.004, 1e-12);   // mm -> m
    ASSERT_TRUE(p->latch.max_spread_rad.has_value());
    EXPECT_NEAR(*p->latch.max_spread_rad, 0.25 * 3.14159265358979323846 / 180.0, 1e-12);

    // Unmentioned fields stay empty so the global policy shows through.
    EXPECT_FALSE(p->latch.mad_k.has_value());
    EXPECT_FALSE(p->latch.max_mean_error_m.has_value());
}

TEST(SceneConfig, RejectsUnknownLatchKey)
{
    // A typo'd threshold silently inheriting the global default reads as a
    // threshold that is being honoured, which is worse than none at all.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "capture": "latched", "asset_id": 4,

            "latch": {"max_spred_mm": 4.0}}]
    })");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsLatchBlockOnAContinuousPlacement)
{
    // Thresholds that cannot apply must not look like thresholds that do.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "hand"]})");
    t.object("rail", R"({"id":"rail","placements":[{"id":"rail_origin","source":"static"}]})");
    t.object("hand", R"({
        "id": "hand",
        "placements": [{
            "id": "hand_opti", "source": "optitrack", "capture": "continuous", "asset_id": 5,

            "latch": {"min_samples": 100}}]
    })");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsNonsensicalLatchThresholds)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "capture": "latched", "asset_id": 4,

            "latch": {"min_samples": 0, "max_spread_mm": -1.0, "max_reject_fraction": 1.5}}]
    })");

    try
    {
        (void)t.load();
        FAIL() << "expected SceneConfigError";
    }
    catch (const SceneConfigError& e)
    {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("min_samples"), std::string::npos);
        EXPECT_NE(msg.find("max_spread_mm"), std::string::npos);
        EXPECT_NE(msg.find("max_reject_fraction"), std::string::npos);
    }
}

TEST(SceneConfig, DescribeMentionsAnchorAndMissingProvenance)
{
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{
            "id": "rail_origin", "source": "optitrack", "capture": "latched", "asset_id": 4}]
    })");

    const std::string d = describe(t.load());
    EXPECT_NE(d.find("rail_origin"), std::string::npos);
    // An uncalibrated mount offset must be visible in the run log, not silent.
    EXPECT_NE(d.find("none recorded"), std::string::npos);
}

// ---------------------------------------------------------------
// projected placements
// ---------------------------------------------------------------
//
// A projection's geometry is measured CAD written straight into config -- so
// unlike a mesh path or an asset id, a wrong number here produces a plausible
// angle and a plausible pose rather than an obvious failure. The loader cannot
// tell a wrong number from a right one, but it CAN refuse the structural
// mistakes, and these pin that it does.

namespace {

// A base body and one joint projected off it. `jointBody` replaces the whole
// "joint" block so a test can corrupt exactly one thing.
void writeProjection(const TempScene& t, const std::string& jointBody,
                     const std::string& parentFrame = "base_opti",
                     const std::string& measured    = "j8_opti")
{
    t.manifest(R"({"world_anchor": "rail_origin",
                   "objects": ["rail", "hand_base", "hand_j8"]})");
    t.object("rail", R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("hand_base", R"({
        "id": "hand_base",
        "placements": [{"id": "base_opti", "source": "optitrack", "capture": "continuous",
                        "asset_id": 1}]})");
    t.object("hand_j8", R"({
        "id": "hand_j8",
        "placements": [
            {"id": "j8_opti", "source": "optitrack", "capture": "continuous", "asset_id": 2},
            {"id": "j8_projected", "source": "projected",
             "parent_frame": ")" +
                            parentFrame + R"(", "measured": ")" + measured + R"(",
             "joint": )" + jointBody +
                            R"(}
        ]})");
}

// A complete, valid joint block. 120 mm off-axis, like the real hardware.
const char* kGoodJoint = R"({
    "zero_pose":  {"position_mm": [120.0, 80.5, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]},
    "axis_point": {"position_mm": [0.0, 80.5, 0.0]},
    "axis": [0.0, 0.0, 1.0],
    "lower_deg": -90.0, "upper_deg": 90.0,
    "reported_arm": "Left", "reported_index": 0})";

}   // namespace

TEST(SceneConfig, ParsesProjectedPlacementAndConvertsToSi)
{
    TempScene t;
    writeProjection(t, kGoodJoint);

    const Scene s      = t.load();
    const Placement* p = s.findPlacement("j8_projected");
    ASSERT_TRUE(p);
    EXPECT_EQ(p->source, Source::Projected);
    EXPECT_EQ(p->measured, "j8_opti");
    EXPECT_EQ(p->parent_frame, "base_opti");
    ASSERT_TRUE(p->joint.has_value());

    // Millimetres in the file, metres in the struct -- the units trap, pinned.
    EXPECT_DOUBLE_EQ(p->joint->geometry.zero_origin_m.x(), 0.120);
    EXPECT_DOUBLE_EQ(p->joint->geometry.zero_origin_m.y(), 0.0805);
    EXPECT_DOUBLE_EQ(p->joint->geometry.axis_point_m.y(), 0.0805);

    // Degrees in the file, radians in the struct.
    EXPECT_NEAR(p->joint->geometry.lower_rad, -1.5707963267948966, 1e-12);

    // 120 mm off the axis: the body's frame ORBITS as the joint turns, which is
    // the case the whole axis_point field exists for.
    EXPECT_NEAR(p->joint->geometry.radius_m(), 0.120, 1e-12);

    EXPECT_EQ(p->joint->reported_arm, "Left");
    EXPECT_EQ(p->joint->reported_index, 0);
    EXPECT_EQ(toString(Source::Projected), "projected");
}

TEST(SceneConfig, NormalisesTheJointAxis)
{
    TempScene t;
    writeProjection(t, R"({
        "zero_pose": {"position_mm": [0.0, 80.5, 0.0]},
        "axis_point": {"position_mm": [0.0, 80.5, 0.0]},
        "axis": [0.0, 0.0, 4.0],
        "reported_arm": "Left", "reported_index": 0})");

    auto scene = t.load();

    EXPECT_NEAR(scene.findPlacement("j8_projected")->joint->geometry.axis.norm(), 1.0, 1e-12);
}

TEST(SceneConfig, ProjectedPlacementGeneratesTheResidualComparison)
{
    // The raw and corrected poses are two placements of one object, so the
    // residual comparison falls out of comparisons() with nothing configured.
    TempScene t;
    writeProjection(t, kGoodJoint);

    const Scene s    = t.load();
    const auto pairs = s.comparisons();
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0].object_id, "hand_j8");
}

TEST(SceneConfig, RejectsProjectedPlacementMissingItsParts)
{
    // Each of the three is separately load-bearing, and each must name itself.
    for (const auto& [body, parent, measured, needle] :
         std::vector<std::tuple<std::string, std::string, std::string, std::string>>{
             {kGoodJoint, "", "j8_opti", "parent_frame"},
             {kGoodJoint, "base_opti", "", "measured"},
         })
    {
        TempScene t;
        writeProjection(t, body, parent, measured);
        try
        {
            (void)t.load();
            ADD_FAILURE() << "expected a throw for missing " << needle;
        }
        catch (const SceneConfigError& e)
        {
            EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << e.what();
        }
    }
}

TEST(SceneConfig, RejectsJointBlockMissingItsGeometry)
{
    const std::pair<const char*, const char*> cases[] = {
        {R"({"axis_point": {"position_mm": [0,0,0]}, "axis": [0,0,1],
             "reported_arm": "Left", "reported_index": 0})",
         "zero_pose"},
        {R"({"zero_pose": {"position_mm": [0,0,0]}, "axis": [0,0,1],
             "reported_arm": "Left", "reported_index": 0})",
         "axis_point"},
        {R"({"zero_pose": {"position_mm": [0,0,0]}, "axis_point": {"position_mm": [0,0,0]},
             "reported_arm": "Left", "reported_index": 0})",
         "axis"},
        {R"({"zero_pose": {"position_mm": [0,0,0]}, "axis_point": {"position_mm": [0,0,0]},
             "axis": [0,0,1], "reported_index": 0})",
         "reported_arm"},
        {R"({"zero_pose": {"position_mm": [0,0,0]}, "axis_point": {"position_mm": [0,0,0]},
             "axis": [0,0,1], "reported_arm": "Left"})",
         "reported_index"},
    };

    for (const auto& [body, needle] : cases)
    {
        TempScene t;
        writeProjection(t, body);
        try
        {
            (void)t.load();
            ADD_FAILURE() << "expected a throw for missing " << needle;
        }
        catch (const SceneConfigError& e)
        {
            EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << e.what();
        }
    }
}

TEST(SceneConfig, RejectsADegenerateJointAxis)
{
    // A zero axis is a catastrophic pose, not a small error -- Eigen would hand
    // back NaN and every pose downstream would follow it.
    for (const char* axis : {"[0.0, 0.0, 0.0]", "[1e-12, 0.0, 0.0]"})
    {
        TempScene t;
        writeProjection(t, std::string(R"({
            "zero_pose": {"position_mm": [0,0,0]}, "axis_point": {"position_mm": [0,0,0]},
            "axis": )") + axis +
                               R"(, "reported_arm": "Left", "reported_index": 0})");
        EXPECT_THROW((void)t.load(), SceneConfigError) << axis;
    }
}

TEST(SceneConfig, RejectsAnAxisPointCarryingARotation)
{
    // A point has no orientation. Accepting a quaternion here would let someone
    // believe they had specified the axis DIRECTION twice, in two places that
    // disagree.
    TempScene t;
    writeProjection(t, R"({
        "zero_pose": {"position_mm": [0,0,0]},
        "axis_point": {"position_mm": [0,0,0], "quat_wxyz": [1,0,0,0]},
        "axis": [0,0,1], "reported_arm": "Left", "reported_index": 0})");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsUnknownKeyInsideAJointBlock)
{
    // Placement-level keys are NOT swept, so without the sweep inside `joint` a
    // typo would vanish silently and then report itself as a MISSING key --
    // sending you to look in entirely the wrong place.
    TempScene t;
    writeProjection(t, R"({
        "zero_pose": {"position_mm": [0,0,0]}, "axis_point": {"position_mm": [0,0,0]},
        "axis": [0,0,1], "axsi": [1,0,0],
        "reported_arm": "Left", "reported_index": 0})");

    try
    {
        (void)t.load();
        ADD_FAILURE() << "expected a throw for the typo'd key";
    }
    catch (const SceneConfigError& e)
    {
        EXPECT_NE(std::string(e.what()).find("axsi"), std::string::npos) << e.what();
    }
}

TEST(SceneConfig, RejectsJointOrMeasuredOnAPlacementThatDoesNotProject)
{
    // Ignored rather than refused, these would read as honoured -- the same
    // reason a `latch` block on a continuous placement is an error.
    for (const char* extra : {R"("measured": "base_opti")", R"("joint": {"axis": [0,0,1]})"})
    {
        TempScene t;
        t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "hand_base"]})");
        t.object("rail", R"({"id": "rail",
                             "placements": [{"id": "rail_origin", "source": "static"}]})");
        t.object("hand_base", std::string(R"({
            "id": "hand_base",
            "placements": [{"id": "base_opti", "source": "optitrack", "capture": "continuous",
                            "asset_id": 1, )") +
                                  extra + "}]}");

        EXPECT_THROW((void)t.load(), SceneConfigError) << extra;
    }
}

TEST(SceneConfig, RejectsALatchedProjection)
{
    // A projection is recomputed from scratch every mocap frame. "Latched" would
    // read as "hold this pose", which is not something this source can do.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin",
                   "objects": ["rail", "hand_base", "hand_j8"]})");
    t.object("rail", R"({"id": "rail",
                         "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("hand_base", R"({"id": "hand_base",
        "placements": [{"id": "base_opti", "source": "optitrack", "capture": "continuous",
                        "asset_id": 1}]})");
    t.object("hand_j8", std::string(R"({
        "id": "hand_j8",
        "placements": [
            {"id": "j8_opti", "source": "optitrack", "capture": "continuous", "asset_id": 2},
            {"id": "j8_projected", "source": "projected", "capture": "latched",
             "parent_frame": "base_opti", "measured": "j8_opti", "joint": )") +
                            kGoodJoint + "}]}");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsAMeasuredThatIsNotARawContinuousMeasurement)
{
    // A projection corrects a raw per-frame measurement. A latched pose has no
    // per-frame sample; a static or projected one is not a measurement at all.
    const std::pair<const char*, const char*> cases[] = {
        {"j8_projected", "itself"},   // its own output
        {"rail_origin", "static"},    // not a measurement
        {"nonexistent_body", "not a placement id"},
    };

    for (const auto& [measured, why] : cases)
    {
        TempScene t;
        writeProjection(t, kGoodJoint, "base_opti", measured);
        EXPECT_THROW((void)t.load(), SceneConfigError) << measured << " (" << why << ")";
    }
}

TEST(SceneConfig, RejectsALatchedParentForAProjection)
{
    // The projection reads its parent from the CURRENT mocap frame, so the
    // parent has to produce a sample every frame. Refused rather than
    // half-supported: a latched parent WOULD work, via the registry, but that is
    // a different code path and nothing needs it yet.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin",
                   "objects": ["rail", "hand_base", "hand_j8"]})");
    t.object("rail", R"({"id": "rail",
                         "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("hand_base", R"({"id": "hand_base",
        "placements": [{"id": "base_opti", "source": "optitrack", "capture": "latched",
                        "asset_id": 1}]})");
    t.object("hand_j8", std::string(R"({
        "id": "hand_j8",
        "placements": [
            {"id": "j8_opti", "source": "optitrack", "capture": "continuous", "asset_id": 2},
            {"id": "j8_projected", "source": "projected",
             "parent_frame": "base_opti", "measured": "j8_opti", "joint": )") +
                            kGoodJoint + "}]}");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsAProjectionParentedToItself)
{
    TempScene t;
    writeProjection(t, kGoodJoint, "j8_projected");
    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, RejectsACycleInTheProjectionChain)
{
    // Placer resolves the evaluation order once, by topological sort. A loop
    // leaves it nothing to start from, and every placement in the loop is simply
    // never computed -- silently, because each one has a perfectly valid parent.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "hand_base", "loop"]})");
    t.object("rail", R"({"id": "rail",
                         "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("hand_base", R"({"id": "hand_base",
        "placements": [{"id": "base_opti", "source": "optitrack", "capture": "continuous",
                        "asset_id": 1}]})");

    const std::string joint = kGoodJoint;
    t.object("loop", std::string(R"({
        "id": "loop",
        "placements": [
            {"id": "raw", "source": "optitrack", "capture": "continuous", "asset_id": 2},
            {"id": "a", "source": "projected", "parent_frame": "b", "measured": "raw",
             "joint": )") +
                         joint + R"(},
            {"id": "b", "source": "projected", "parent_frame": "a", "measured": "raw",
             "joint": )" +
                         joint + R"(}
        ]})");

    try
    {
        (void)t.load();
        ADD_FAILURE() << "expected a throw for the cycle";
    }
    catch (const SceneConfigError& e)
    {
        EXPECT_NE(std::string(e.what()).find("cycle"), std::string::npos) << e.what();
    }
}

TEST(SceneConfig, AcceptsAChainOfProjections)
{
    // The real shape: base -> J8 -> J9, each hanging off the CORRECTED pose
    // before it. Not a cycle, and must not be mistaken for one.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin",
                   "objects": ["rail", "hand_base", "chain"]})");
    t.object("rail", R"({"id": "rail",
                         "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("hand_base", R"({"id": "hand_base",
        "placements": [{"id": "base_opti", "source": "optitrack", "capture": "continuous",
                        "asset_id": 1}]})");

    const std::string joint = kGoodJoint;
    t.object("chain", std::string(R"({
        "id": "chain",
        "placements": [
            {"id": "j8_opti", "source": "optitrack", "capture": "continuous", "asset_id": 2,
             "compare": false},
            {"id": "j9_opti", "source": "optitrack", "capture": "continuous", "asset_id": 3,
             "compare": false},
            {"id": "j8_projected", "source": "projected", "parent_frame": "base_opti",
             "measured": "j8_opti", "joint": )") +
                          joint + R"(},
            {"id": "j9_projected", "source": "projected", "parent_frame": "j8_projected",
             "measured": "j9_opti", "joint": )" +
                          joint + R"(}
        ]})");

    EXPECT_NO_THROW((void)t.load());
}

TEST(SceneConfig, RejectsAProjectedWorldAnchor)
{
    // The anchor must be locatable without reference to anything else. A
    // projection is defined relative to its parent, so anchoring to one is
    // circular.
    TempScene t;
    t.manifest(R"({"world_anchor": "j8_projected",
                   "objects": ["rail", "hand_base", "hand_j8"]})");
    t.object("rail", R"({"id": "rail",
                         "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("hand_base", R"({"id": "hand_base",
        "placements": [{"id": "base_opti", "source": "optitrack", "capture": "continuous",
                        "asset_id": 1}]})");
    t.object("hand_j8", std::string(R"({
        "id": "hand_j8",
        "placements": [
            {"id": "j8_opti", "source": "optitrack", "capture": "continuous", "asset_id": 2},
            {"id": "j8_projected", "source": "projected", "parent_frame": "base_opti",
             "measured": "j8_opti", "joint": )") +
                            kGoodJoint + "}]}");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, DescribeRecordsTheJointGeometry)
{
    // The geometry is measured CAD, and a wrong number in it is invisible in the
    // output -- it yields a plausible angle and a plausible pose. So the run log
    // is the only record of what a session was standing on, the same argument
    // the `calibrated` block makes for mount offsets.
    TempScene t;
    writeProjection(t, kGoodJoint);

    const std::string d = describe(t.load());
    EXPECT_NE(d.find("j8_projected"), std::string::npos);
    EXPECT_NE(d.find("axis through"), std::string::npos);
    EXPECT_NE(d.find("radius"), std::string::npos);   // the orbit, in mm
    EXPECT_NE(d.find("hand_joints[0]"), std::string::npos);
}

// ---------------------------------------------------------------
// source: constructed
// ---------------------------------------------------------------

namespace {

// The rotor's two mounts: 90 degrees apart at a 700 mm radius, on the disk face
// 1830 mm out from the axial midpoint. The part frame is +X down the spin axis,
// so the mounts' shared face normal points along +X.
constexpr const char* kGoodConstruction = R"({
    "normal_axis": [0.0, 0.0, 1.0],
    "normal_in_part": [1.0, 0.0, 0.0],
    "mount_a": {"position_mm": [1830.0, 0.0, 700.0]},
    "mount_b": {"position_mm": [1830.0, -700.0, 0.0]}})";

// Two latched bodies and one constructed part. `construction` is injected so
// each test can corrupt exactly one thing.
void writeConstruction(const TempScene& t, const std::string& construction,
                       const std::string& inputs = R"(["mount_a", "mount_b"])")
{
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor"]})");
    t.object("rail", R"({"id": "rail",
                         "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("rotor", std::string(R"({
        "id": "rotor",
        "placements": [
            {"id": "mount_a", "source": "optitrack", "capture": "latched", "asset_id": 1,
             "compare": false, "visible": false},
            {"id": "mount_b", "source": "optitrack", "capture": "latched", "asset_id": 2,
             "compare": false, "visible": false},
            {"id": "rotor_opti", "source": "constructed", "capture": "latched",
             "inputs": )") +
                          inputs + R"(, "construction": )" + construction + R"(}
        ]})");
}

}   // namespace

TEST(SceneConfig, LoadsAConstructedPlacement)
{
    TempScene t;
    writeConstruction(t, kGoodConstruction);

    const Scene s      = t.load();
    const Placement* p = s.findPlacement("rotor_opti");
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->source, Source::Constructed);
    EXPECT_EQ(toString(Source::Constructed), "constructed");
    ASSERT_TRUE(p->construction.has_value());

    const auto& g = p->construction->geometry;
    EXPECT_NEAR(g.mount_a_m.x(), 1.830, 1e-9);   // mm -> m, once, at the boundary
    EXPECT_NEAR(g.mount_a_m.z(), 0.700, 1e-9);
    EXPECT_NEAR(g.mount_b_m.y(), -0.700, 1e-9);
    EXPECT_NEAR(g.normal_axis.z(), 1.0, 1e-12);
    EXPECT_NEAR(g.normal_in_part.x(), 1.0, 1e-12);

    // Everything else is derived from those two points rather than configured:
    // the chord at 90 degrees on a 700 mm radius, and each body's reach from the
    // part's origin.
    EXPECT_NEAR(g.chord_length_m(), 0.989949, 1e-6);
    EXPECT_NEAR(g.reach_a_m(), std::hypot(1.830, 0.700), 1e-9);
    EXPECT_NEAR(g.reach_b_m(), std::hypot(1.830, 0.700), 1e-9);
}

TEST(SceneConfig, LoadsARailStyleConstruction)
{
    // The other arrangement the same schema has to serve: the chord runs ALONG
    // the part rather than across it, and the normal is up rather than axial.
    // Nothing in the parser distinguishes the two cases.
    TempScene t;
    writeConstruction(t, R"({
        "normal_axis": [0.0, 0.0, 1.0],
        "normal_in_part": [0.0, 0.0, 1.0],
        "mount_a": {"position_mm": [-5000.0, 0.0, 50.0]},
        "mount_b": {"position_mm": [5000.0, 0.0, 50.0]}})");

    const Scene s      = t.load();
    const Placement* p = s.findPlacement("rotor_opti");
    ASSERT_NE(p, nullptr);
    ASSERT_TRUE(p->construction.has_value());
    EXPECT_NEAR(p->construction->geometry.chord_length_m(), 10.0, 1e-9);
}

// ---------------------------------------------------------------
// expect_normal_in_parent -- optional, and unset must not read as passing
// ---------------------------------------------------------------

TEST(SceneConfig, ExpectNormalInParentIsOptionalAndAbsentByDefault)
{
    // kGoodConstruction does not set it, and that must survive into the geometry
    // as an EMPTY optional rather than as a zero vector. A zero vector would be a
    // direction naming nothing, and the construction would have to guess whether
    // it meant "check against nothing" or "check against the origin".
    TempScene t;
    writeConstruction(t, kGoodConstruction);

    const Scene s      = t.load();
    const Placement* p = s.findPlacement("rotor_opti");
    ASSERT_NE(p, nullptr);
    ASSERT_TRUE(p->construction.has_value());
    EXPECT_FALSE(p->construction->geometry.expect_normal_in_parent.has_value());
}

TEST(SceneConfig, ExpectNormalInParentIsParsedAndNormalised)
{
    // Normalised on load like every other direction key, so nothing downstream
    // has to care whether it was typed as a unit vector -- and the angle the
    // construction reports is an angle either way.
    TempScene t;
    writeConstruction(t, R"({
        "normal_axis": [0.0, 0.0, 1.0],
        "normal_in_part": [1.0, 0.0, 0.0],
        "expect_normal_in_parent": [0.0, 3.0, 0.0],
        "mount_a": {"position_mm": [1830.0, 0.0, 700.0]},
        "mount_b": {"position_mm": [1830.0, -700.0, 0.0]}})");

    const Scene s      = t.load();
    const Placement* p = s.findPlacement("rotor_opti");
    ASSERT_NE(p, nullptr);
    ASSERT_TRUE(p->construction.has_value());

    const auto& e = p->construction->geometry.expect_normal_in_parent;
    ASSERT_TRUE(e.has_value());
    EXPECT_NEAR(e->norm(), 1.0, 1e-12);
    EXPECT_NEAR(e->y(), 1.0, 1e-12);
}

TEST(SceneConfig, ExpectNormalInParentRejectsANonDirection)
{
    // Present but meaningless is an ERROR, not a silent fallback to absent.
    // Omitting the key is how you say "do not check"; typing a zero into it is a
    // mistake, and treating the two the same would turn a typo into a check that
    // quietly never ran.
    for (const char* bad : {"[0.0, 0.0, 0.0]", "[0.0, 1.0]", R"("up")"})
    {
        TempScene t;
        writeConstruction(t, std::string(R"({
            "normal_axis": [0.0, 0.0, 1.0],
            "normal_in_part": [1.0, 0.0, 0.0],
            "expect_normal_in_parent": )") +
                                 bad + R"(,
            "mount_a": {"position_mm": [1830.0, 0.0, 700.0]},
            "mount_b": {"position_mm": [1830.0, -700.0, 0.0]}})");

        EXPECT_THROW((void)t.load(), SceneConfigError) << "accepted " << bad;
    }
}

TEST(SceneConfig, ConstructedNeedsExactlyTwoInputs)
{
    // The construction is built on the line between two bodies. One input has no
    // line; three have three, and nothing says which to use.
    for (const char* inputs : {R"(["mount_a"])", R"(["mount_a", "mount_b", "mount_a"])"})
    {
        TempScene t;
        writeConstruction(t, kGoodConstruction, inputs);
        EXPECT_THROW((void)t.load(), SceneConfigError) << inputs;
    }
}

TEST(SceneConfig, ConstructedRefusesNonOptitrackInputs)
{
    // A construction reads each input's measured face NORMAL. An expected_pose
    // carries an orientation too, and it would mean nothing here.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor"]})");
    t.object("rail", R"({"id": "rail",
                         "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("rotor", std::string(R"({
        "id": "rotor",
        "placements": [
            {"id": "mount_a", "source": "optitrack", "capture": "latched", "asset_id": 1,
             "compare": false},
            {"id": "mount_b", "source": "expected_pose", "capture": "latched",
             "topic": "rotor/pose_expected", "compare": false},
            {"id": "rotor_opti", "source": "constructed", "capture": "latched",
             "inputs": ["mount_a", "mount_b"], "construction": )") +
                          kGoodConstruction + "}]}");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, ConstructedRefusesMixedCapture)
{
    // The line between the two bodies is only a line if both ends describe the
    // same instant. A latched body is a pose from startup; a continuous one is a
    // pose from now.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor"]})");
    t.object("rail", R"({"id": "rail",
                         "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("rotor", std::string(R"({
        "id": "rotor",
        "placements": [
            {"id": "mount_a", "source": "optitrack", "capture": "latched", "asset_id": 1,
             "compare": false},
            {"id": "mount_b", "source": "optitrack", "capture": "continuous", "asset_id": 2,
             "compare": false},
            {"id": "rotor_opti", "source": "constructed", "capture": "latched",
             "inputs": ["mount_a", "mount_b"], "construction": )") +
                          kGoodConstruction + "}]}");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, ConstructedRefusesEveryMissingGeometryKey)
{
    // None of these can be inferred, and every one is worth metres if guessed.
    // There are deliberately no defaults, so each absence is an error.
    const std::vector<std::pair<const char*, const char*>> partial = {
        {"normal_axis", R"({"normal_in_part": [1,0,0],
                            "mount_a": {"position_mm": [1830,0,700]},
                            "mount_b": {"position_mm": [1830,-700,0]}})"},
        {"normal_in_part", R"({"normal_axis": [0,0,1],
                               "mount_a": {"position_mm": [1830,0,700]},
                               "mount_b": {"position_mm": [1830,-700,0]}})"},
        {"mount_a", R"({"normal_axis": [0,0,1], "normal_in_part": [1,0,0],
                        "mount_b": {"position_mm": [1830,-700,0]}})"},
        {"mount_b", R"({"normal_axis": [0,0,1], "normal_in_part": [1,0,0],
                        "mount_a": {"position_mm": [1830,0,700]}})"},
    };

    for (const auto& entry : partial)
    {
        TempScene t;
        writeConstruction(t, entry.second);
        EXPECT_THROW((void)t.load(), SceneConfigError) << "missing " << entry.first;
    }
}

TEST(SceneConfig, ConstructedRefusesBadGeometryValues)
{
    const std::vector<std::pair<const char*, const char*>> bad = {
        {"zero-length normal_axis",
         R"({"normal_axis": [0,0,0], "normal_in_part": [1,0,0],
             "mount_a": {"position_mm": [1830,0,700]},
             "mount_b": {"position_mm": [1830,-700,0]}})"},
        {"zero-length normal_in_part",
         R"({"normal_axis": [0,0,1], "normal_in_part": [0,0,0],
             "mount_a": {"position_mm": [1830,0,700]},
             "mount_b": {"position_mm": [1830,-700,0]}})"},
        {"coincident mount points",
         R"({"normal_axis": [0,0,1], "normal_in_part": [1,0,0],
             "mount_a": {"position_mm": [1830,0,700]},
             "mount_b": {"position_mm": [1830,0,700]}})"},
        {"a rotation on a point",
         R"({"normal_axis": [0,0,1], "normal_in_part": [1,0,0],
             "mount_a": {"position_mm": [1830,0,700], "quat_wxyz": [1,0,0,0]},
             "mount_b": {"position_mm": [1830,-700,0]}})"},
        {"unknown key",
         R"({"normal_axis": [0,0,1], "normal_in_part": [1,0,0],
             "mount_a": {"position_mm": [1830,0,700]},
             "mount_b": {"position_mm": [1830,-700,0]},
             "chord_length_mm": 990})"},
    };

    for (const auto& entry : bad)
    {
        TempScene t;
        writeConstruction(t, entry.second);
        EXPECT_THROW((void)t.load(), SceneConfigError) << entry.first;
    }
}

TEST(SceneConfig, ConstructionBlockOnANonConstructedPlacementIsAnError)
{
    // Placement-level keys are not swept anywhere, so without this a
    // `construction` block on the wrong placement sits there looking honoured.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", std::string(R"({
        "id": "rail",
        "placements": [{"id": "rail_origin", "source": "static", "construction": )") +
                         kGoodConstruction + "}]}");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, ConstructedRefusesADerivedInput)
{
    // Chaining derived placements would need cycle detection and an evaluation
    // order. Nothing needs it, so it is refused rather than half-supported.
    TempScene t;
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail", "rotor"]})");
    t.object("rail", R"({"id": "rail",
                         "placements": [{"id": "rail_origin", "source": "static"}]})");
    t.object("rotor", std::string(R"({
        "id": "rotor",
        "placements": [
            {"id": "mount_a", "source": "optitrack", "capture": "latched", "asset_id": 1,
             "compare": false},
            {"id": "mount_b", "source": "optitrack", "capture": "latched", "asset_id": 2,
             "compare": false},
            {"id": "mount_avg", "source": "fused", "capture": "latched",
             "inputs": ["mount_a", "mount_b"], "compare": false},
            {"id": "rotor_opti", "source": "constructed", "capture": "latched",
             "inputs": ["mount_a", "mount_avg"], "construction": )") +
                          kGoodConstruction + "}]}");

    EXPECT_THROW((void)t.load(), SceneConfigError);
}

// ---------------------------------------------------------------
// a constructed world anchor -- which is how the rail works
// ---------------------------------------------------------------

namespace {

// The rail: two bodies at the ends, and the anchor built from them.
void writeConstructedAnchor(const TempScene& t, const char* bodyCapture = "latched")
{
    t.manifest(R"({"world_anchor": "rail_origin", "objects": ["rail"]})");
    t.object("rail", std::string(R"({
        "id": "rail",
        "placements": [
            {"id": "rail_opti_left", "source": "optitrack", "capture": ")") +
                         bodyCapture + R"(",
             "asset_id": 1, "compare": false, "visible": false},
            {"id": "rail_opti_right", "source": "optitrack", "capture": ")" +
                         bodyCapture + R"(",
             "asset_id": 2, "compare": false, "visible": false},
            {"id": "rail_origin", "source": "constructed", "capture": "latched",
             "inputs": ["rail_opti_left", "rail_opti_right"],
             "construction": {
                "normal_axis": [0.0, 0.0, 1.0],
                "normal_in_part": [0.0, 0.0, 1.0],
                "mount_a": {"position_mm": [-5000.0, 0.0, 50.0]},
                "mount_b": {"position_mm": [5000.0, 0.0, 50.0]}}}
        ]})");
}

}   // namespace

TEST(SceneConfig, AcceptsAConstructedWorldAnchor)
{
    // The rail's real shape. The anchor may be built from measured bodies -- what
    // it may not be is defined RELATIVE to something, which is why expected_pose
    // and projected are still refused.
    TempScene t;
    writeConstructedAnchor(t);

    const Scene s = t.load();
    EXPECT_EQ(s.anchor_frame, "rail_origin");
    ASSERT_NE(s.findPlacement("rail_origin"), nullptr);
    EXPECT_EQ(s.findPlacement("rail_origin")->source, Source::Constructed);
}

TEST(SceneConfig, RejectsAConstructedAnchorBuiltFromContinuousBodies)
{
    // The anchor has to be settled once. A continuous input would rebuild it on
    // every mocap frame, and every object in the scene would swim with the
    // anchor's tracking noise -- which looks like everything else moving.
    TempScene t;
    writeConstructedAnchor(t, "continuous");
    EXPECT_THROW((void)t.load(), SceneConfigError);
}

TEST(SceneConfig, DescribeRecordsTheConstructionGeometry)
{
    // Every number in a construction is CAD read by hand, none can be inferred,
    // and a wrong one yields a pose that is confidently metres out while looking
    // perfectly stable. The run log is the only record of what it stood on.
    TempScene t;
    writeConstruction(t, kGoodConstruction);

    const std::string d = describe(t.load());
    EXPECT_NE(d.find("rotor_opti"), std::string::npos);
    EXPECT_NE(d.find("inputs=mount_a,mount_b"), std::string::npos);
    EXPECT_NE(d.find("body normal"), std::string::npos);
    EXPECT_NE(d.find("mount_a at"), std::string::npos);
    EXPECT_NE(d.find("mount_b at"), std::string::npos);
    EXPECT_NE(d.find("reach"), std::string::npos);
}
