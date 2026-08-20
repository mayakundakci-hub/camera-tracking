// logger_node: one CSV row per comparison sample, joined against the arm
// configuration and the anchor that produced it.

#include <middleware/EcalProtoTopic.hpp>
#include "camera_tracking.pb.h"
#include "config.hpp"

#include <arena/transport/ecal/EcalTopic.hpp>
#include <ecal/ecal.h>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

using camera_tracking::ComparisonPacket;
using camera_tracking::JointStatePacket;
using camera_tracking::ScenePlacementsPacket;
using camera_tracking::SessionControlPacket;

namespace {

// CSV fields are free text from config, so a stray comma or quote would shift
// every following column. Quote and escape rather than trusting the input.
std::string csvField(const std::string& raw)
{
    std::string out = "\"";
    for (const char c : raw)
    {
        if (c == '"')
            out += "\"\"";
        else if (c == '\n' || c == '\r')
            out += ' ';
        else
            out += c;
    }
    return out + "\"";
}

}   // namespace

int main()
{
    // See backend/src/main.cpp for why. Not strictly needed here -- this process
    // only subscribes -- but kept uniform so no future publisher added to it
    // fails silently.
    arena::transport::ecal::set_loopback_enabled(true);

    const auto& cfg = Config::load();   // also cd's to the repo root (where config.json lives)
    const std::string topicComparisons = cfg["ecal"]["topic_scene_comparisons"];
    const std::string topicPlacements  = cfg["ecal"]["topic_scene_placements"];
    const std::string topicJointState  = cfg["ecal"]["topic_joint_state"];
    const std::string topicSession     = cfg["ecal"]["topic_session_control"];
    const std::string outputDir        = cfg["logger"]["output_dir"];
    const int flushEveryNRows          = cfg["logger"]["flush_every_n_rows"];

    std::error_code ec;
    std::filesystem::create_directories(outputDir, ec);

    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::tm tmBuf{};
    localtime_s(&tmBuf, &now);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tmBuf);
    const auto csvPath =
        std::filesystem::path(outputDir) / ("comparisons_" + std::string(stamp) + ".csv");

    std::ofstream csv(csvPath);
    if (!csv.is_open())
    {
        std::fprintf(stderr, "[logger_node] FATAL: could not open %s for writing\n",
                     csvPath.string().c_str());
        return 1;
    }

    csv << "timestamp,object_id,a,b,a_source,b_source,"
           "distance_mm,angle_deg,dx_mm,dy_mm,dz_mm,time_gap_s,valid,invalid_reason,"
           // Arm configuration at the time of the sample. Without these the
           // constant-vs-configuration-dependent test cannot be run at all.
           "arm,rail_position_mm,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,"
           "active_tool_frame,joints_valid,joints_timestamp,"
           // What the session was anchored on.
           "anchor_frame,anchor_valid,anchor_x_m,anchor_y_m,anchor_z_m,"
           // The error bars on the two claims this row compares. Without them
           // distance_mm is uninterpretable: a 4 mm delta between two estimates
           // that are each +/-4 mm is not evidence of anything. `spread` is
           // per-sample dispersion (was it holding still?), `std_err` is the
           "a_latched,a_spread_mm,a_std_err_mm,a_samples_used,a_samples_rejected,"
           "b_latched,b_spread_mm,b_std_err_mm,b_samples_used,b_samples_rejected,"
           // And on the anchor, which biases BOTH sides of every row equally --
           // the one error that cannot show up in a delta.
           "anchor_spread_mm,anchor_std_err_mm,"
           // Whether this row is a settled one-shot claim (the reviewed rotor) or
           // a live stream (the hand). They mean different things and averaging
           // them together would be a category error: the rotor rows are all the
           // same reading repeated, so they must not be weighted as samples.
           "review_gated,"
           // Which workflow screen the operator was on. Makes the run boundary
           // readable from the file itself rather than only from the console.
           "phase\n";

    std::mutex mtx;   // eCAL delivers callbacks from its own threads
    long long rowCount = 0;
    JointStatePacket joints;   // latest, joined into each row
    ScenePlacementsPacket placements;

    // Off until the frontend says otherwise. Defaulting to ON would mean a logger
    // started before the frontend recorded the setup phase as though it were the
    // run -- rows that look like validation data and are not.
    bool logEnabled   = false;
    std::string phase = "(no frontend)";

    middleware::EcalProtoSubscriber<SessionControlPacket> subSession(
        "logger_node", topicSession, [&](const SessionControlPacket& msg) {
            std::scoped_lock lock(mtx);
            // Transitions are announced, because the boundary between "the run"
            // and "everything else" is the one thing a reader of the CSV cannot
            // reconstruct from the CSV.
            if (msg.log_enabled() != logEnabled)
                std::printf("[logger_node] logging %s (phase: %s) after %lld rows\n",
                            msg.log_enabled() ? "STARTED" : "STOPPED", msg.phase().c_str(),
                            rowCount);
            logEnabled = msg.log_enabled();
            phase      = msg.phase();
        });

    middleware::EcalProtoSubscriber<JointStatePacket> subJoints("logger_node", topicJointState,
                                                                [&](const JointStatePacket& msg) {
                                                                    std::scoped_lock lock(mtx);
                                                                    joints = msg;
                                                                });

    middleware::EcalProtoSubscriber<ScenePlacementsPacket> subPlacements(
        "logger_node", topicPlacements, [&](const ScenePlacementsPacket& msg) {
            std::scoped_lock lock(mtx);
            placements = msg;
        });

    // Comparisons drive row emission; the other two topics are joined in.
    middleware::EcalProtoSubscriber<ComparisonPacket> subComparisons(
        "logger_node", topicComparisons, [&](const ComparisonPacket& msg) {
            std::scoped_lock lock(mtx);

            // The gate. Deltas keep arriving throughout; only the run is written.
            if (!logEnabled) return;

            const auto& anchor = placements.anchor_in_mocap();
            const auto jointAt = [&](int i) {
                return i < joints.robot_joints_size() ? joints.robot_joints(i) : 0.0;
            };

            // The delta names its two placements by id; their latch quality
            // lives on the placements packet. A missing one yields all zeros,
            // which is also what a non-latching placement reports -- both mean
            // "no error bar available here".
            static const camera_tracking::PlacedPose kNoPose;
            const auto poseFor = [&](const std::string& id) -> const camera_tracking::PlacedPose& {
                for (const auto& p : placements.placements())
                    if (p.placement_id() == id) return p;
                return kNoPose;
            };

            for (const auto& d : msg.deltas())
            {
                // Invalid deltas are logged too. A gap in the record is
                // indistinguishable from a period that was never sampled,
                // whereas a row carrying its own reason is diagnosable.
                csv << msg.timestamp() << "," << csvField(d.object_id()) << "," << csvField(d.a())
                    << "," << csvField(d.b()) << "," << csvField(d.a_source()) << ","
                    << csvField(d.b_source()) << "," << d.distance_mm() << "," << d.angle_deg()
                    << "," << d.dx_mm() << "," << d.dy_mm() << "," << d.dz_mm() << ","
                    << d.time_gap_s() << "," << d.valid() << "," << csvField(d.invalid_reason())
                    << ","

                    << csvField(joints.arm()) << "," << joints.rail_position() << "," << jointAt(0)
                    << "," << jointAt(1) << "," << jointAt(2) << "," << jointAt(3) << ","
                    << jointAt(4) << "," << jointAt(5) << "," << joints.active_tool_frame() << ","
                    << joints.valid() << "," << joints.timestamp() << ","

                    << csvField(placements.anchor_frame()) << "," << anchor.valid() << ","
                    << anchor.pos_x() << "," << anchor.pos_y() << "," << anchor.pos_z() << ",";

                for (const auto* side : {&poseFor(d.a()), &poseFor(d.b())})
                    csv << side->latched() << "," << side->spread_mm() << "," << side->std_err_mm()
                        << "," << side->samples_used() << "," << side->samples_rejected() << ",";

                csv << anchor.spread_mm() << "," << anchor.std_err_mm() << "," << d.review_gated()
                    << "," << csvField(phase) << "\n";
                ++rowCount;
            }

            if (flushEveryNRows <= 0 || rowCount % flushEveryNRows == 0) csv.flush();
        });

    std::printf("[logger_node] logging %s (joined with %s + %s) -> %s (flush every %d rows)\n",
                topicComparisons.c_str(), topicJointState.c_str(), topicPlacements.c_str(),
                csvPath.string().c_str(), flushEveryNRows);
    // Stated explicitly because a file with a header and no rows looks like a
    // broken logger, when in fact it is a logger correctly waiting to be told
    // that a run has begun.
    std::printf("[logger_node] HOLDING -- no rows until %s reports logging enabled, which the "
                "frontend does once the operator continues to position tracking\n",
                topicSession.c_str());

    while (eCAL::Ok())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    {
        std::scoped_lock lock(mtx);
        csv.flush();
        std::printf("[logger_node] %lld rows written to %s\n", rowCount, csvPath.string().c_str());
    }
    return 0;
}
