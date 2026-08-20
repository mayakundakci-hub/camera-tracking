#pragma once
#include <QDateTime>

#include "camera_tracking.pb.h"

#include <mutex>

using camera_tracking::ComparisonPacket;
using camera_tracking::JointEstimatePacket;
using camera_tracking::JointStatePacket;
using camera_tracking::PlacedPose;
using camera_tracking::ScenePlacementsPacket;
using camera_tracking::SessionControlPacket;

class AppState {
public:
    // --- scene placements (scene/placements) ---
    void updatePlacements(const ScenePlacementsPacket& pkt) {
        std::lock_guard<std::mutex> lk(mtx_);
        placements_ = pkt;
        placementsArrivalMs_ = QDateTime::currentMSecsSinceEpoch();
        havePlacements_ = true;
    }
    ScenePlacementsPacket placements() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return placements_;
    }
    bool havePlacements() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return havePlacements_;
    }

    // --- comparisons (scene/comparisons) ---
    void updateComparisons(const ComparisonPacket& pkt) {
        std::lock_guard<std::mutex> lk(mtx_);
        comparisons_ = pkt;
        comparisonsArrivalMs_ = QDateTime::currentMSecsSinceEpoch();
    }
    ComparisonPacket comparisons() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return comparisons_;
    }
    bool isStale(qint64 maxAgeMs = 500) const {
        std::lock_guard<std::mutex> lk(mtx_);
        const qint64 newest = std::max(placementsArrivalMs_, comparisonsArrivalMs_);
        return (QDateTime::currentMSecsSinceEpoch() - newest) > maxAgeMs;
    }

    // --- joint state (robot/joint_state) ---
    void updateJoints(const JointStatePacket& pkt) {
        std::lock_guard<std::mutex> lk(mtx_);
        joints_ = pkt;
        jointsArrivalMs_ = QDateTime::currentMSecsSinceEpoch();
        haveJoints_ = true;
    }
    JointStatePacket joints() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return joints_;
    }

    bool jointsLive(qint64 maxAgeMs = 500) const {
        std::lock_guard<std::mutex> lk(mtx_);
        return haveJoints_ &&
               (QDateTime::currentMSecsSinceEpoch() - jointsArrivalMs_) <= maxAgeMs;
    }

    // --- camera-measured angles, in JointStatePacket shape ---
    void updateMeasuredJoints(const JointStatePacket& pkt) {
        std::lock_guard<std::mutex> lk(mtx_);
        measuredJoints_ = pkt;
        measuredJointsArrivalMs_ = QDateTime::currentMSecsSinceEpoch();
        haveMeasuredJoints_ = true;
    }
    JointStatePacket measuredJoints() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return measuredJoints_;
    }
    // The hand holds its last measured pose rather than snapping home when a
    // plate blinks, so a dropout looks like a pause and not like a reset.
    bool measuredJointsLive(qint64 maxAgeMs = 1000) const {
        std::lock_guard<std::mutex> lk(mtx_);
        return haveMeasuredJoints_ &&
               (QDateTime::currentMSecsSinceEpoch() - measuredJointsArrivalMs_) <= maxAgeMs;
    }

    // --- camera-measured joint angles (scene/joint_estimates) ---
    //
    // Distinct from joints() above, which is what the CONTROLLER says. This is
    // what the cameras measured, and the pair of them is the point.
    void updateJointEstimates(const JointEstimatePacket& pkt) {
        std::lock_guard<std::mutex> lk(mtx_);
        estimates_ = pkt;
        estimatesArrivalMs_ = QDateTime::currentMSecsSinceEpoch();
        haveEstimates_ = true;
    }
    JointEstimatePacket jointEstimates() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return estimates_;
    }
    // False before the first packet, so "the backend is not publishing angles"
    // and "the angles are all zero" cannot render alike.
    bool haveJointEstimates() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return haveEstimates_;
    }

private:
    mutable std::mutex    mtx_;

    ScenePlacementsPacket placements_;
    qint64                placementsArrivalMs_ = 0;
    bool                  havePlacements_ = false;

    ComparisonPacket      comparisons_;
    qint64                comparisonsArrivalMs_ = 0;

    JointStatePacket      joints_;
    qint64                jointsArrivalMs_ = 0;
    bool                  haveJoints_ = false;

    JointStatePacket      measuredJoints_;
    qint64                measuredJointsArrivalMs_ = 0;
    bool                  haveMeasuredJoints_ = false;

    JointEstimatePacket   estimates_;
    qint64                estimatesArrivalMs_ = 0;
    bool                  haveEstimates_ = false;
};
