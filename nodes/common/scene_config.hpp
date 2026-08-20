#pragma once
// scene_config: the scene is declared in config, not compiled in

#include "frames.hpp"
#include "joint_projection.hpp"
#include "two_mount_pose.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace scene {

namespace fs = std::filesystem;
using nlohmann::json;

class SceneConfigError : public std::runtime_error {
public:
    explicit SceneConfigError(const std::string& what) : std::runtime_error(what) {}
};

enum class Source {
    Optitrack,      // tracked rigid body, taken as-is from Motive -- asset_id
    ExpectedPose,   // pose the robot reports, already in the parent frame -- topic
    JointState,     // joint angles, posed through the URDF by ludus FK
    Static,         // constant from this file; nothing measures it
    Fused,          // the average of two or more other placements -- inputs
    Projected,      // another placement's measurement, forced onto a revolute joint
                    // hanging off parent_frame -- measured + joint
    Constructed,    // one pose built GEOMETRICALLY from two mounts on a common
                    // face -- inputs + construction
};


enum class Capture { Latched, Continuous };

std::string toString(Source s);
std::string toString(Capture c);

struct MountOffset {
    frames::Vec3 translation{frames::Vec3::Zero()};   // metres (authored in mm)
    frames::Quat rotation{frames::Quat::Identity()};

    [[nodiscard]] frames::Transform toTransform(const frames::FrameId& to,
                                                const frames::FrameId& from,
                                                double stamp = 0.0) const
    {
        return frames::make(to, from, translation, rotation, stamp);
    }
};

struct Calibration {
    bool        present{false};
    std::string date;
    double      rms_mm{-1.0};
    std::string motive_asset_rev;
    std::string note;
};
struct LatchOverride {
    std::optional<std::size_t> min_samples;
    std::optional<double>      max_mean_error_m;
    std::optional<double>      mad_k;
    std::optional<double>      max_reject_fraction;
    std::optional<double>      max_spread_m;
    std::optional<double>      max_spread_rad;
    std::optional<double>      timeout_s;
};

struct JointSpec {
    jointproj::RevoluteJoint geometry;
    std::string reported_arm;
    int         reported_index{-1};
};

struct ConstructionSpec {
    twomount::Geometry geometry;
};

struct Placement {
    std::string id;          // frame name; unique across the whole scene
    std::string object_id;   // owner; filled in by the loader
    Source      source{Source::Static};
    Capture     capture{Capture::Continuous};
    LatchOverride latch;

    int         asset_id{0};
    std::string topic;
    std::string urdf;   // resolved absolute path
    std::vector<std::string> inputs;
    std::string                measured;
    std::optional<JointSpec>   joint;
    std::optional<ConstructionSpec> construction;
    MountOffset pose;
    std::string parent_frame;
    bool        compare{true};
    std::string color{"#cccccc"};
    bool        visible{true};
};

struct Object {
    std::string            id;
    std::string            display_name;
    std::string            visual_mesh;      // resolved absolute; empty = frame-only
    double                 mesh_scale{1.0};

    MountOffset            visual_offset;
    std::vector<Placement> placements;
    Calibration            calibrated;
    fs::path               source_file;      // for error messages

    std::optional<bool>    review_gate;
};

struct Comparison {
    std::string object_id;
    std::string a;   // placement id
    std::string b;   // placement id
};

struct Scene {
    std::string         anchor_frame;   // a placement id; the render frame
    fs::path            root;           // paths in object files resolve against this
    fs::path            manifest_file;
    std::vector<Object> objects;

    [[nodiscard]] const Object* findObject(const std::string& id) const&
    {
        for (const auto& o : objects)
            if (o.id == id) return &o;
        return nullptr;
    }
    const Object* findObject(const std::string&) const&& = delete;

    [[nodiscard]] const Placement* findPlacement(const std::string& id) const&
    {
        for (const auto& o : objects)
            for (const auto& p : o.placements)
                if (p.id == id) return &p;
        return nullptr;
    }
    const Placement* findPlacement(const std::string&) const&& = delete;

    [[nodiscard]] std::vector<const Placement*> placements() const&
    {
        std::vector<const Placement*> out;
        for (const auto& o : objects)
            for (const auto& p : o.placements)
                out.push_back(&p);
        return out;
    }
    std::vector<const Placement*> placements() const&& = delete;

    [[nodiscard]] std::vector<Comparison> comparisons() const
    {
        std::vector<Comparison> out;
        for (const auto& o : objects)
        {
            std::vector<const Placement*> comparable;
            for (const auto& p : o.placements)
                if (p.compare) comparable.push_back(&p);

            for (std::size_t i = 0; i < comparable.size(); ++i)
                for (std::size_t j = i + 1; j < comparable.size(); ++j)
                    out.push_back({o.id, comparable[i]->id, comparable[j]->id});
        }
        return out;
    }
};

Scene load(const fs::path& manifest_path);

std::string describe(const Scene& scene);

// implementation

inline std::string toString(Source s)
{
    switch (s) {
        case Source::Optitrack:    return "optitrack";
        case Source::ExpectedPose: return "expected_pose";
        case Source::JointState:   return "joint_state";
        case Source::Static:       return "static";
        case Source::Fused:        return "fused";
        case Source::Projected:    return "projected";
        case Source::Constructed:  return "constructed";
    }
    return "?";
}

inline std::string toString(Capture c)
{
    return c == Capture::Latched ? "latched" : "continuous";
}

namespace detail {

// Accumulates problems so load() can report every one of them at once.
class Errors {
public:
    void add(const std::string& where, const std::string& what)
    {
        items_.push_back("  " + where + ": " + what);
    }

    [[nodiscard]] bool empty() const { return items_.empty(); }

    void throwIfAny(const fs::path& manifest) const
    {
        if (items_.empty()) return;
        std::ostringstream os;
        os << "scene config invalid (" << items_.size() << " problem"
           << (items_.size() == 1 ? "" : "s") << ") loading " << manifest.string() << ":\n";
        for (const auto& i : items_) os << i << "\n";
        throw SceneConfigError(os.str());
    }

private:
    std::vector<std::string> items_;
};

inline json readJson(const fs::path& p, const std::string& role)
{
    std::ifstream f(p);
    if (!f.is_open())
        throw SceneConfigError("scene config: cannot open " + role + " '" + p.string() + "'");
    try {
        json j;
        f >> j;
        return j;
    } catch (const std::exception& e) {
        throw SceneConfigError("scene config: " + role + " '" + p.string() +
                               "' is not valid JSON -- " + e.what());
    }
}

inline fs::path resolvePath(const fs::path& root, const std::string& p)
{
    const fs::path given(p);
    return given.is_absolute() ? given : (root / given);
}

inline bool isHexColor(const std::string& s)
{
    if (s.size() != 7 || s[0] != '#') return false;
    return std::all_of(s.begin() + 1, s.end(),
                       [](unsigned char c) { return std::isxdigit(c) != 0; });
}

inline MountOffset parseOffset(const json& j, const std::string& where, Errors& errs)
{
    MountOffset out;

    if (j.contains("position_mm")) {
        const auto& a = j.at("position_mm");
        if (!a.is_array() || a.size() != 3) {
            errs.add(where, "'position_mm' must be an array of 3 numbers");
        } else {
            out.translation = frames::convert::vecFromMm(a[0].get<double>(),
                                                         a[1].get<double>(),
                                                         a[2].get<double>());
        }
    }

    if (j.contains("quat_wxyz")) {
        const auto& a = j.at("quat_wxyz");
        if (!a.is_array() || a.size() != 4) {
            errs.add(where, "'quat_wxyz' must be an array of 4 numbers [w,x,y,z]");
        } else {
            try {
                out.rotation = frames::convert::quatFromWxyz(a[0].get<double>(),
                                                             a[1].get<double>(),
                                                             a[2].get<double>(),
                                                             a[3].get<double>());
            } catch (const frames::FrameError& e) {
                errs.add(where, e.what());
            }
        }
    }

    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        if (k != "position_mm" && k != "quat_wxyz" && k.rfind('_', 0) != 0)
            errs.add(where, "unknown key '" + k + "' (expected position_mm, quat_wxyz)");
    }
    return out;
}

inline LatchOverride parseLatch(const json& j, const std::string& where, Errors& errs)
{
    LatchOverride out;

    const auto positive = [&](const char* key, double& sink) {
        const double v = j.at(key).get<double>();
        if (v <= 0.0) { errs.add(where, std::string("'") + key + "' must be positive"); return false; }
        sink = v;
        return true;
    };

    if (j.contains("min_samples")) {
        const auto v = j.at("min_samples").get<long long>();
        if (v < 1) errs.add(where, "'min_samples' must be at least 1");
        else       out.min_samples = static_cast<std::size_t>(v);
    }
    if (j.contains("max_mean_error_mm")) {
        double mm = 0.0;
        if (positive("max_mean_error_mm", mm)) out.max_mean_error_m = frames::convert::mmToM(mm);
    }
    if (j.contains("mad_k")) {
        double k = 0.0;
        if (positive("mad_k", k)) out.mad_k = k;
    }
    if (j.contains("max_reject_fraction")) {
        const double v = j.at("max_reject_fraction").get<double>();
        if (v < 0.0 || v > 1.0) errs.add(where, "'max_reject_fraction' must be in [0, 1]");
        else                    out.max_reject_fraction = v;
    }
    if (j.contains("max_spread_mm")) {
        double mm = 0.0;
        if (positive("max_spread_mm", mm)) out.max_spread_m = frames::convert::mmToM(mm);
    }
    if (j.contains("max_spread_deg")) {
        double deg = 0.0;
        if (positive("max_spread_deg", deg)) out.max_spread_rad = frames::convert::degToRad(deg);
    }
    if (j.contains("timeout_s")) {
        double s = 0.0;
        if (positive("timeout_s", s)) out.timeout_s = s;
    }

    static const char* kKnown[] = {"min_samples",   "max_mean_error_mm", "mad_k",
                                   "max_reject_fraction", "max_spread_mm", "max_spread_deg",
                                   "timeout_s"};
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        if (k.rfind('_', 0) == 0) continue;
        if (std::find(std::begin(kKnown), std::end(kKnown), k) == std::end(kKnown))
            errs.add(where, "unknown key '" + k + "' in 'latch'");
    }
    return out;
}

inline JointSpec parseJoint(const json& j, const std::string& where, Errors& errs)
{
    JointSpec out;

    if (!j.contains("zero_pose")) {
        errs.add(where, "a joint needs a 'zero_pose' -- where the CHILD body's frame sits in the "
                        "PARENT body's frame at theta = 0. Read it off the running system with "
                        "the joint at a known angle, or off CAD");
    } else {
        const MountOffset zero = parseOffset(j.at("zero_pose"), where + " zero_pose", errs);
        out.geometry.zero_origin_m = zero.translation;
        out.geometry.zero_rotation = zero.rotation;
    }

    if (!j.contains("axis_point")) {
        errs.add(where, "a joint needs an 'axis_point' -- any point its rotation axis passes "
                        "through, in the parent placement's frame. ANY point on the axis works, "
                        "since sliding along it changes nothing, so pick whichever is easiest to "
                        "measure. Set it equal to zero_pose's position if the body's frame really "
                        "is on the axis");
    } else {
        const MountOffset ap = parseOffset(j.at("axis_point"), where + " axis_point", errs);
        out.geometry.axis_point_m = ap.translation;
        if (j.at("axis_point").contains("quat_wxyz"))
            errs.add(where, "'axis_point' is a point, not a pose -- drop its 'quat_wxyz'. The "
                            "axis's direction is the 'axis' key");
    }

    if (!j.contains("axis")) {
        errs.add(where, "a joint needs an 'axis' -- the direction it turns about, in the parent "
                        "placement's frame. There is no default: a zero axis is a catastrophic "
                        "pose, not a small error");
    } else if (const auto& a = j.at("axis"); !a.is_array() || a.size() != 3) {
        errs.add(where, "'axis' must be an array of 3 numbers");
    } else {
        const frames::Vec3 axis(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
        if (!axis.allFinite())          errs.add(where, "'axis' has a non-finite component");
        else if (!(axis.norm() > 1e-9)) errs.add(where, "'axis' has zero length; it must name a "
                                                        "direction");
        else                            out.geometry.axis = axis.normalized();
    }

    const bool hasLower = j.contains("lower_deg");
    const bool hasUpper = j.contains("upper_deg");
    if (hasLower != hasUpper) {
        errs.add(where, "'lower_deg' and 'upper_deg' must be given together");
    } else if (hasLower) {
        const double lo = j.at("lower_deg").get<double>();
        const double hi = j.at("upper_deg").get<double>();
        if (!(lo < hi)) errs.add(where, "'lower_deg' must be less than 'upper_deg'");
        else {
            out.geometry.lower_rad = frames::convert::degToRad(lo);
            out.geometry.upper_rad = frames::convert::degToRad(hi);
        }
    }
    out.reported_arm = j.value("reported_arm", std::string{});
    if (out.reported_arm.empty())
        errs.add(where, "a joint needs 'reported_arm' ('Left' or 'Right') to know which "
                        "JointStatePacket to compare its estimate against");

    if (!j.contains("reported_index")) {
        errs.add(where, "a joint needs 'reported_index' -- which entry of the controller's "
                        "hand_joints array this joint is");
    } else {
        out.reported_index = j.at("reported_index").get<int>();
        if (out.reported_index < 0)
            errs.add(where, "'reported_index' must be zero or greater");
    }

    static const char* kKnown[] = {"zero_pose", "axis_point",   "axis",         "lower_deg",
                                   "upper_deg", "reported_arm", "reported_index"};
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        if (k.rfind('_', 0) == 0) continue;
        if (std::find(std::begin(kKnown), std::end(kKnown), k) == std::end(kKnown))
            errs.add(where, "unknown key '" + k + "' in 'joint'");
    }
    return out;
}

inline ConstructionSpec parseConstruction(const json& j, const std::string& where, Errors& errs)
{
    ConstructionSpec out;

    const auto direction = [&](const char* key, bool required,
                               const char* what) -> std::optional<frames::Vec3> {
        if (!j.contains(key)) {
            if (required)
                errs.add(where, std::string("a construction needs '") + key + "' -- " + what);
            return std::nullopt;
        }
        const auto& a = j.at(key);
        if (!a.is_array() || a.size() != 3) {
            errs.add(where, std::string("'") + key + "' must be an array of 3 numbers");
            return std::nullopt;
        }
        const frames::Vec3 v(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
        if (!v.allFinite()) {
            errs.add(where, std::string("'") + key + "' has a non-finite component");
            return std::nullopt;
        }
        if (!(v.norm() > 1e-9)) {
            errs.add(where, std::string("'") + key + "' has zero length; it must name a direction");
            return std::nullopt;
        }
        return v.normalized();
    };

    if (const auto v = direction(
            "normal_axis", true,
            "which axis of EACH BODY'S OWN frame points out of the mounted face. This is the "
            "one assertion a construction rests on. Create both bodies with that face against a "
            "known plane so the answer is defined by the procedure rather than estimated, and set "
            "'expect_normal_in_parent' so a wrong answer is caught"))
        out.geometry.normal_axis = *v;

    if (const auto v = direction(
            "normal_in_part", true,
            "where that same physical direction points in the PART's own frame -- the spin axis "
            "for a rotor, up for a rail. It need not be perpendicular to the chord; whatever "
            "CAD says it is, is what the measured geometry is checked against"))
        out.geometry.normal_in_part = *v;

    // OPTIONAL, and the only key here that is not a fact about the part.
    out.geometry.expect_normal_in_parent = direction(
        "expect_normal_in_parent", false,
        "where 'normal_in_part' must end up pointing in the PARENT frame -- the one check that "
        "can catch a 'normal_axis' naming the wrong body axis or the right one with the wrong "
        "sign, because it is the only one that is not the part checking itself. Omit it for a "
        "part whose attitude the room does not already know");

    const auto point = [&](const char* key, const char* what, frames::Vec3& dst) {
        if (!j.contains(key)) {
            errs.add(where, std::string("a construction needs '") + key + "' -- " + what);
            return;
        }
        dst = parseOffset(j.at(key), where + " " + key, errs).translation;
        if (j.at(key).contains("quat_wxyz"))
            errs.add(where, std::string("'") + key + "' is a point, not a pose -- drop its "
                            "'quat_wxyz'. Which way the bodies face is 'normal_axis'");
    };

    point("mount_a", "where the FIRST body listed in 'inputs' sits, in the PART's frame. This is "
                     "the Motive pivot's location, not the marker centroid and not the seat",
          out.geometry.mount_a_m);
    point("mount_b", "where the SECOND body listed in 'inputs' sits, in the PART's frame",
          out.geometry.mount_b_m);

    if (j.contains("mount_a") && j.contains("mount_b") &&
        out.geometry.chord_length_m() < 1e-9)
        errs.add(where, "'mount_a' and 'mount_b' are the same point; the construction is built on "
                        "the line between the two bodies, and there is none");

    static const char* kKnown[] = {"normal_axis", "normal_in_part", "mount_a", "mount_b",
                                   "expect_normal_in_parent"};
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        if (k.rfind('_', 0) == 0) continue;
        if (std::find(std::begin(kKnown), std::end(kKnown), k) == std::end(kKnown))
            errs.add(where, "unknown key '" + k + "' in 'construction'");
    }
    return out;
}

inline std::optional<Source> parseSource(const std::string& s)
{
    if (s == "optitrack")     return Source::Optitrack;
    if (s == "expected_pose") return Source::ExpectedPose;
    if (s == "joint_state")   return Source::JointState;
    if (s == "static")        return Source::Static;
    if (s == "fused")         return Source::Fused;
    if (s == "projected")     return Source::Projected;
    if (s == "constructed")   return Source::Constructed;
    return std::nullopt;
}

inline Placement parsePlacement(const json& j, const Object& owner, const fs::path& root,
                                Errors& errs)
{
    Placement p;
    p.object_id = owner.id;

    const std::string where0 = owner.source_file.filename().string();
    p.id = j.value("id", std::string{});
    if (p.id.empty()) {
        errs.add(where0, "a placement is missing required key 'id' (it becomes the frame name)");
        p.id = "<unnamed>";
    }
    const std::string where = where0 + " placement '" + p.id + "'";

    const std::string srcStr = j.value("source", std::string{});
    if (auto s = parseSource(srcStr)) {
        p.source = *s;
    } else {
        errs.add(where, "'source' is '" + srcStr +
                        "'; expected one of optitrack, expected_pose, joint_state, static, "
                        "fused, projected, constructed");
        return p;   // every check below is source-specific
    }

    if (j.contains("capture")) {
        const std::string c = j.at("capture").get<std::string>();
        if (c == "latched")         p.capture = Capture::Latched;
        else if (c == "continuous") p.capture = Capture::Continuous;
        else errs.add(where, "'capture' is '" + c + "'; expected 'latched' or 'continuous'");
    } else if (p.source == Source::Optitrack) {
        // No default: latched-vs-continuous is a real semantic difference, and
        // guessing it wrong either freezes a moving object or lets a bolted
        // one drift with tracking noise.
        errs.add(where, "optitrack placements must state 'capture' ('latched' or 'continuous')");
    }

    if (j.contains("measured") && p.source != Source::Projected)
        errs.add(where, "has 'measured' but source is '" + toString(p.source) +
                        "'; only 'projected' placements project another placement's measurement");
    if (j.contains("joint") && p.source != Source::Projected)
        errs.add(where, "has a 'joint' block but source is '" + toString(p.source) +
                        "'; only 'projected' placements hang off a joint");
    if (j.contains("construction") && p.source != Source::Constructed)
        errs.add(where, "has a 'construction' block but source is '" + toString(p.source) +
                        "'; only 'constructed' placements build a pose from two mounts");

    if (j.contains("latch")) {

        if (p.capture != Capture::Latched)
            errs.add(where, "has a 'latch' block but capture is '" + toString(p.capture) +
                            "'; latch thresholds only apply to 'latched' captures");
        else if (p.source != Source::Optitrack && p.source != Source::ExpectedPose)
            errs.add(where, "has a 'latch' block but source is '" + toString(p.source) +
                            "'; only measured sources (optitrack, expected_pose) latch");
        else
            p.latch = parseLatch(j.at("latch"), where + " latch", errs);
    }

    p.parent_frame = j.value("parent_frame", std::string{});
    p.color        = j.value("color", std::string{"#cccccc"});
    p.visible      = j.value("visible", true);
    p.compare      = j.value("compare", true);
    if (!isHexColor(p.color))
        errs.add(where, "'color' is '" + p.color + "'; expected #RRGGBB");

    switch (p.source)
    {
        case Source::Optitrack:
            p.asset_id = j.value("asset_id", 0);
            if (p.asset_id <= 0)
                errs.add(where, "optitrack placements need a positive 'asset_id' "
                                "(the Motive rigid-body streaming id)");
    
            if (j.contains("model_in_marker_frame"))
                errs.add(where, "'model_in_marker_frame' no longer exists -- Motive's rigid-body "
                                "frame IS the placement's frame. Set the pivot in Motive at the "
                                "CAD point instead, and record it in 'calibrated'. If this offset "
                                "was NOT identity, that pose has to be moved into Motive before "
                                "this file is worth loading");
            break;

        case Source::ExpectedPose:
            p.topic = j.value("topic", std::string{});
            if (p.topic.empty())
                errs.add(where, "expected_pose placements need a 'topic' to subscribe to");
            break;

        case Source::JointState:
            p.topic = j.value("topic", std::string{});
            if (p.topic.empty())
                errs.add(where, "joint_state placements need a 'topic' carrying joint state");
            if (const std::string u = j.value("urdf", std::string{}); u.empty()) {
                errs.add(where, "joint_state placements need a 'urdf'");
            } else {
                const fs::path resolved = resolvePath(root, u);
                if (!fs::exists(resolved))
                    errs.add(where, "urdf '" + u + "' not found (resolved to " +
                                    resolved.string() + ")");
                p.urdf = resolved.string();
            }
            break;

        case Source::Static:
            if (j.contains("pose"))
                p.pose = parseOffset(j.at("pose"), where + " pose", errs);
            // A static placement with no pose is identity, which is a
            // legitimate and useful thing to say -- notably for the anchor
            // during bring-up, before any marker is installed.
            break;

        case Source::Fused:
            if (!j.contains("inputs") || !j.at("inputs").is_array())
            {
                errs.add(where, "fused placements need an 'inputs' array naming the "
                                "placements to average");
                break;
            }
            for (const auto& in : j.at("inputs"))
                p.inputs.push_back(in.get<std::string>());
            if (p.inputs.size() < 2)
                errs.add(where, "fused placements need at least 2 inputs; averaging one "
                                "placement would just duplicate it under a second name");
            break;

        case Source::Projected:
            if (j.contains("capture") && p.capture == Capture::Latched)
                errs.add(where, "projected placements cannot be 'latched' -- a projection is "
                                "recomputed every frame from its inputs, so there is nothing "
                                "to capture once");

            p.measured = j.value("measured", std::string{});
            if (p.measured.empty())
                errs.add(where, "projected placements need 'measured' naming the raw optitrack "
                                "placement whose pose is being forced onto the joint");

            if (p.parent_frame.empty())
                errs.add(where, "projected placements need 'parent_frame' naming the pose the "
                                "joint hangs off -- for a chain, that is the PROJECTED placement "
                                "before this one, not its raw measurement");

            if (!j.contains("joint"))
                errs.add(where, "projected placements need a 'joint' block naming which URDF "
                                "joint they ride on");
            else
                p.joint = parseJoint(j.at("joint"), where + " joint", errs);
            break;

        case Source::Constructed:
            if (!j.contains("inputs") || !j.at("inputs").is_array())
            {
                errs.add(where, "constructed placements need an 'inputs' array naming the two "
                                "mount placements the pose is built from");
            }
            else
            {
                for (const auto& in : j.at("inputs"))
                    p.inputs.push_back(in.get<std::string>());
                
                if (p.inputs.size() != 2)
                    errs.add(where, "constructed placements need exactly 2 inputs, not " +
                                    std::to_string(p.inputs.size()) +
                                    ". The construction is built on the chord between two mounts "
                                    "on one face; it has no meaning for any other count");
            }

            if (!j.contains("construction"))
                errs.add(where, "constructed placements need a 'construction' block carrying the "
                                "CAD geometry that relates the two mounts to the part's own "
                                "origin");
            else
                p.construction =
                    parseConstruction(j.at("construction"), where + " construction", errs);
            break;
    }
    return p;
}

inline Object parseObject(const std::string& name, const fs::path& file, const fs::path& root,
                          Errors& errs)
{
    const json j = readJson(file, "object file");

    Object o;
    o.source_file  = file;
    o.id           = j.value("id", name);
    o.display_name = j.value("display_name", o.id);
    o.mesh_scale   = j.value("mesh_scale", 1.0);

    const std::string where = file.filename().string();

    if (const std::string m = j.value("visual_mesh", std::string{}); !m.empty()) {
        const fs::path resolved = resolvePath(root, m);
        if (!fs::exists(resolved))
            errs.add(where, "visual_mesh '" + m + "' not found (resolved to " +
                            resolved.string() + ")");
        o.visual_mesh = resolved.string();
    }

    if (j.contains("visual_offset")) {
       
        if (o.visual_mesh.empty())
            errs.add(where, "has a 'visual_offset' but no 'visual_mesh'; the offset positions a "
                            "mesh relative to its placement, so without one it does nothing");
        else
            o.visual_offset = parseOffset(j.at("visual_offset"), where + " visual_offset", errs);
    }

    if (j.contains("review_gate")) {
        if (!j.at("review_gate").is_boolean())
            errs.add(where, "'review_gate' must be true or false");
        else
            o.review_gate = j.at("review_gate").get<bool>();
    }

    if (j.contains("calibrated")) {
        const auto& c = j.at("calibrated");
        o.calibrated.present          = true;
        o.calibrated.date             = c.value("date", std::string{});
        o.calibrated.rms_mm           = c.value("rms_mm", -1.0);
        o.calibrated.motive_asset_rev = c.value("motive_asset_rev", std::string{});
        o.calibrated.note             = c.value("note", std::string{});
    }

    if (!j.contains("placements") || !j.at("placements").is_array() ||
        j.at("placements").empty())
    {
        errs.add(where, "'placements' must be a non-empty array -- an object with no "
                        "placement has no position and cannot be rendered or compared");
        return o;
    }

    for (const auto& pj : j.at("placements"))
        o.placements.push_back(parsePlacement(pj, o, root, errs));

    return o;
}

}  // namespace detail

inline Scene load(const fs::path& manifest_path)
{
    detail::Errors errs;

    Scene scene;
    scene.manifest_file = fs::absolute(manifest_path);
    
    scene.root = scene.manifest_file.parent_path().parent_path();

    const json manifest = detail::readJson(scene.manifest_file, "manifest");

    scene.anchor_frame = manifest.value("world_anchor", std::string{});
    if (scene.anchor_frame.empty())
        errs.add("scene.json", "missing 'world_anchor' -- it names the PLACEMENT whose frame "
                               "the whole scene is expressed in");

    if (!manifest.contains("objects") || !manifest.at("objects").is_array())
    {
        errs.add("scene.json", "missing 'objects' array");
        errs.throwIfAny(scene.manifest_file);
    }

    const fs::path objectsDir = scene.manifest_file.parent_path() / "objects";
    for (const auto& entry : manifest.at("objects"))
    {
        const std::string name = entry.get<std::string>();
        const fs::path file = objectsDir / (name + ".json");
        if (!fs::exists(file)) {
            errs.add("scene.json", "object '" + name + "' listed in the manifest but " +
                                   file.string() + " does not exist");
            continue;
        }
        scene.objects.push_back(detail::parseObject(name, file, scene.root, errs));
    }
    std::vector<std::string> objectIds, placementIds;
    for (const auto& o : scene.objects)
    {
        if (std::find(objectIds.begin(), objectIds.end(), o.id) != objectIds.end())
            errs.add(o.source_file.filename().string(), "duplicate object id '" + o.id + "'");
        objectIds.push_back(o.id);

        for (const auto& p : o.placements)
        {
            if (std::find(placementIds.begin(), placementIds.end(), p.id) != placementIds.end())
                errs.add(o.source_file.filename().string(),
                         "duplicate placement id '" + p.id + "'. Placement ids are FRAME NAMES "
                         "and must be unique across the whole scene -- a duplicate silently "
                         "overwrites the other object's frame");
            placementIds.push_back(p.id);
        }
    }

    if (!scene.anchor_frame.empty())
    {
        const Placement* anchor = scene.findPlacement(scene.anchor_frame);
        if (!anchor)
        {
            std::string known;
            for (const auto& p : placementIds) known += (known.empty() ? "" : ", ") + p;
            errs.add("scene.json", "world_anchor '" + scene.anchor_frame +
                                   "' is not a placement id. Known placements: " +
                                   (known.empty() ? "(none)" : known));
        }
        else if (anchor->source != Source::Optitrack && anchor->source != Source::Static &&
                 anchor->source != Source::Constructed)
        {
            
            errs.add("scene.json", "world_anchor '" + scene.anchor_frame + "' has source '" +
                                   toString(anchor->source) +
                                   "'; the anchor must be 'optitrack', 'static' or 'constructed'");
        }
        else if (anchor->source == Source::Optitrack && anchor->capture != Capture::Latched)
        {
            errs.add("scene.json", "world_anchor '" + scene.anchor_frame +
                                   "' is continuous; the frame the whole scene is measured "
                                   "against must be 'latched' or every object appears to "
                                   "move whenever the anchor's tracking jitters");
        }
        else if (anchor->source == Source::Constructed)
        {
            for (const auto& in : anchor->inputs)
            {
                const Placement* src = scene.findPlacement(in);
                if (src && src->capture != Capture::Latched)
                    errs.add("scene.json", "world_anchor '" + scene.anchor_frame +
                                           "' is built from '" + in +
                                           "', which is continuous. The frame the whole scene is "
                                           "measured against must be settled once, or every "
                                           "object appears to move whenever the anchor's bodies "
                                           "jitter");
            }
        }
    }

    for (const auto& o : scene.objects)
    {
        for (const auto& p : o.placements)
        {
            const std::string where = o.source_file.filename().string();

            if (!p.parent_frame.empty() && !scene.findPlacement(p.parent_frame))
                errs.add(where, "placement '" + p.id + "' names parent_frame '" + p.parent_frame +
                                "', which is not a placement id");

            const std::string kind = toString(p.source);   // "fused" or "constructed"
            for (const auto& in : p.inputs)
            {
                const Placement* src = scene.findPlacement(in);
                if (!src)
                {
                    errs.add(where, kind + " placement '" + p.id + "' names input '" + in +
                                    "', which is not a placement id");
                }
                else if (in == p.id)
                {
                    errs.add(where, kind + " placement '" + p.id + "' lists itself as an input");
                }
                else if (src->source == Source::Fused || src->source == Source::Constructed)
                {
                    
                    errs.add(where, kind + " placement '" + p.id + "' takes input '" + in +
                                    "', which is itself " + toString(src->source) +
                                    "; deriving one derived placement from another is not "
                                    "supported");
                }
                else if (p.source == Source::Constructed && src->source != Source::Optitrack)
                {
                    errs.add(where, "constructed placement '" + p.id + "' takes input '" + in +
                                    "', whose source is '" + toString(src->source) +
                                    "'. A construction reads each mount's measured face NORMAL, "
                                    "so both inputs must be 'optitrack' placements");
                }
            }

            if (p.source == Source::Constructed && p.inputs.size() == 2)
            {
                const Placement* a = scene.findPlacement(p.inputs[0]);
                const Placement* b = scene.findPlacement(p.inputs[1]);
                if (a && b && a->capture != b->capture)
                    errs.add(where, "constructed placement '" + p.id + "' mixes a " +
                                    toString(a->capture) + " input ('" + p.inputs[0] +
                                    "') with a " + toString(b->capture) + " one ('" + p.inputs[1] +
                                    "'). The chord between the two mounts is only a chord if both "
                                    "ends describe the same instant");
            }

            if (p.source != Source::Projected) continue;

            // --- what a projection may correct ---
            const Placement* m = p.measured.empty() ? nullptr : scene.findPlacement(p.measured);
            if (p.measured.empty()) {
                // already reported by parsePlacement
            } else if (!m) {
                errs.add(where, "projected placement '" + p.id + "' names measured '" +
                                p.measured + "', which is not a placement id");
            } else if (p.measured == p.id) {
                errs.add(where, "projected placement '" + p.id + "' names itself as its own "
                                "measurement");
            } else if (m->source != Source::Optitrack) {
                errs.add(where, "projected placement '" + p.id + "' takes measured '" + p.measured +
                                "', whose source is '" + toString(m->source) + "'. A projection "
                                "corrects a RAW measurement, so it must name an 'optitrack' "
                                "placement");
            } else if (m->capture != Capture::Continuous) {
                errs.add(where, "projected placement '" + p.id + "' takes measured '" + p.measured +
                                "', which is latched. A latched pose is captured once and has no "
                                "per-frame sample to project");
            }

            // --- what a projection may hang off ---
            const Placement* par =
                p.parent_frame.empty() ? nullptr : scene.findPlacement(p.parent_frame);
            if (p.parent_frame == p.id) {
                errs.add(where, "projected placement '" + p.id + "' names itself as its "
                                "parent_frame");
            } else if (par && par->source != Source::Projected &&
                       !(par->source == Source::Optitrack &&
                         par->capture == Capture::Continuous)) {
                errs.add(where, "projected placement '" + p.id + "' hangs off parent_frame '" +
                                p.parent_frame + "', whose source is '" + toString(par->source) +
                                "' (" + toString(par->capture) + "). A projection reads its parent "
                                "from the current mocap frame, so the parent must be a continuous "
                                "optitrack placement or another projection");
            }
        }
    }

    // --- cycles in the projection chain ---
    {
        std::map<std::string, std::string> parentOf;
        for (const auto& o : scene.objects)
            for (const auto& p : o.placements)
                if (p.source == Source::Projected && !p.parent_frame.empty())
                    parentOf.emplace(p.id, p.parent_frame);

        for (const auto& entry : parentOf)
        {
            const std::string& start = entry.first;
            std::vector<std::string> path{start};

            for (std::string cur = entry.second;;)
            {
                const auto it = parentOf.find(cur);
                if (it == parentOf.end()) break;          // reached a measured root: fine

                if (cur == start)
                {
                    // Report once per cycle rather than once per member.
                    if (*std::min_element(path.begin(), path.end()) == start)
                    {
                        std::string chain;
                        for (const auto& n : path) chain += n + " -> ";
                        errs.add(scene.findPlacement(start)->object_id,
                                 "projection chain is a cycle: " + chain + start +
                                 ". Each projection must ultimately hang off a measured pose");
                    }
                    break;
                }
                if (std::find(path.begin(), path.end(), cur) != path.end()) break;  // other cycle

                path.push_back(cur);
                cur = it->second;
            }
        }
    }

    errs.throwIfAny(scene.manifest_file);
    return scene;
}

inline std::string describe(const Scene& scene)
{
    std::ostringstream os;
    os << "[scene] " << scene.manifest_file.string() << "\n"
       << "[scene] anchor frame: " << scene.anchor_frame << "\n";

    for (const auto& o : scene.objects)
    {
        os << "[scene] " << o.display_name << " (id=" << o.id << ")";
        if (!o.visual_mesh.empty()) os << " mesh=" << fs::path(o.visual_mesh).filename().string();
        os << "\n";

        for (const auto& p : o.placements)
        {
            os << "[scene]     " << p.id << "  source=" << toString(p.source);
            if (p.source == Source::Optitrack || p.source == Source::ExpectedPose)
                os << " capture=" << toString(p.capture);
            if (p.source == Source::Optitrack)   os << " asset_id=" << p.asset_id;
            if (!p.topic.empty())                os << " topic=" << p.topic;
            if (!p.measured.empty())             os << " measured=" << p.measured;
            if (!p.inputs.empty())
            {
                os << " inputs=";
                for (std::size_t i = 0; i < p.inputs.size(); ++i)
                    os << (i ? "," : "") << p.inputs[i];
            }
            if (!p.parent_frame.empty())         os << " parent=" << p.parent_frame;
            os << "\n";

            if (p.joint)
            {
                const jointproj::RevoluteJoint& g = p.joint->geometry;
                const auto mm = [](double m) { return frames::convert::mToMm(m); };
                os << "[scene]         joint in "
                   << (p.parent_frame.empty() ? scene.anchor_frame : p.parent_frame)
                   << ": axis through " << mm(g.axis_point_m.x()) << ", "
                   << mm(g.axis_point_m.y()) << ", " << mm(g.axis_point_m.z()) << " mm"
                   << "  along " << g.axis.x() << ", " << g.axis.y() << ", " << g.axis.z() << "\n"
                   << "[scene]         zero pose at " << mm(g.zero_origin_m.x()) << ", "
                   << mm(g.zero_origin_m.y()) << ", " << mm(g.zero_origin_m.z()) << " mm"
                   // The orbit radius. Zero means the body's frame is ON the axis
                   // and its position does not move with the joint; large means it
                   // swings, and by how much.
                   << "  radius " << mm(g.radius_m()) << " mm"
                   << "  limits " << frames::convert::radToDeg(g.lower_rad) << " to "
                   << frames::convert::radToDeg(g.upper_rad) << " deg\n"
                   << "[scene]         compares against hand_joints[" << p.joint->reported_index
                   << "] of arm " << p.joint->reported_arm << "\n";
            }

            if (p.construction)
            {
                const twomount::Geometry& g = p.construction->geometry;
                const auto mm  = [](double m) { return frames::convert::mToMm(m); };
                const auto vec = [&](const frames::Vec3& v) {
                    return std::to_string(mm(v.x())) + ", " + std::to_string(mm(v.y())) + ", " +
                           std::to_string(mm(v.z())) + " mm";
                };
                os << "[scene]         construction: body normal " << g.normal_axis.x() << ", "
                   << g.normal_axis.y() << ", " << g.normal_axis.z() << " (in each body's frame)"
                   << " = " << g.normal_in_part.x() << ", " << g.normal_in_part.y() << ", "
                   << g.normal_in_part.z() << " in the part\n"
                   << "[scene]         mount_a at " << vec(g.mount_a_m) << "\n"
                   << "[scene]         mount_b at " << vec(g.mount_b_m) << "\n"
                   << "[scene]         chord " << mm(g.chord_length_m()) << " mm"
                   << ", reach " << mm(g.reach_a_m()) << " / " << mm(g.reach_b_m())
                   << " mm -- check both against the part\n";
            }
        }

        if (o.calibrated.present)
        {
            os << "[scene]     mount set up " << o.calibrated.date
               << " motive_rev=" << o.calibrated.motive_asset_rev;
            if (o.calibrated.rms_mm >= 0.0) os << " rms=" << o.calibrated.rms_mm << "mm";
            if (!o.calibrated.note.empty()) os << " -- " << o.calibrated.note;
            os << "\n";
        }
        else if (std::any_of(o.placements.begin(), o.placements.end(),
                             [](const Placement& p) { return p.source == Source::Optitrack; }))
            os << "[scene]     calibrated: (none recorded -- mount offset has no provenance)\n";
    }

    for (const auto& c : scene.comparisons())
        os << "[scene] compare " << c.object_id << ": " << c.a << " vs " << c.b << "\n";

    return os.str();
}

}  // namespace scene
