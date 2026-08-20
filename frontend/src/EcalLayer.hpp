#pragma once

#include <middleware/EcalProtoTopic.hpp>

#include "AppState.hpp"
#include "config.hpp"

#include <string>

class EcalLayer {
public:
    explicit EcalLayer(AppState& state)
        : placements_("frontend", topic("topic_scene_placements"),
                      [&state](const ScenePlacementsPacket& msg) { state.updatePlacements(msg); }),
          comparisons_("frontend", topic("topic_scene_comparisons"),
                       [&state](const ComparisonPacket& msg) { state.updateComparisons(msg); }),
          joints_("frontend", topic("topic_joint_state"),
                  [&state](const JointStatePacket& msg) { state.updateJoints(msg); }),
          measuredJoints_(
              "frontend", topic("topic_hand_joint_state_measured"),
              [&state](const JointStatePacket& msg) { state.updateMeasuredJoints(msg); }),
          jointEstimates_(
              "frontend", topic("topic_scene_joint_estimates"),
              [&state](const JointEstimatePacket& msg) { state.updateJointEstimates(msg); }),
          sessionControl_("frontend", topic("topic_session_control"))
    {}

    void publishSessionControl(bool logEnabled, const std::string& phase, double stamp)
    {
        SessionControlPacket pkt;
        pkt.set_timestamp(stamp);
        pkt.set_log_enabled(logEnabled);
        pkt.set_phase(phase);
        sessionControl_.send(pkt);
    }

private:
    static std::string topic(const char* key)
    {
        return Config::load()["ecal"][key].get<std::string>();
    }

    middleware::EcalProtoSubscriber<ScenePlacementsPacket> placements_;
    middleware::EcalProtoSubscriber<ComparisonPacket> comparisons_;

    middleware::EcalProtoSubscriber<JointStatePacket> joints_;

    middleware::EcalProtoSubscriber<JointStatePacket> measuredJoints_;

    middleware::EcalProtoSubscriber<JointEstimatePacket> jointEstimates_;

    middleware::EcalProtoPublisher<SessionControlPacket> sessionControl_;
};
