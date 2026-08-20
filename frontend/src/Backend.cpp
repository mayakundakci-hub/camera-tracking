#include "Backend.hpp"

#include "config.hpp"

#include <QCoreApplication>
#include <QDateTime>
#include <QVariantMap>
#include <QVector3D>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>

#ifndef M_PI
#define M_PI 3.14159265358979323846   // MSVC does not define it without _USE_MATH_DEFINES
#endif

Backend::Backend(QObject* parent) : QObject(parent), ecal_(state_)
{

    const auto& cfg = Config::load();
    if (cfg.contains("frontend") && cfg["frontend"].contains("view"))
    {
        const auto& v = cfg["frontend"]["view"];
        viewPitchDeg_ = v.value("pitch_deg", viewPitchDeg_);
        viewYawDeg_   = v.value("yaw_deg", viewYawDeg_);
        viewPadding_  = v.value("padding", viewPadding_);
    }

    if (cfg.contains("review"))
    {
        const auto& r      = cfg["review"];
        reviewWarnMm_      = r.value("warn_mm", reviewWarnMm_);
        reviewWarnDeg_     = r.value("warn_deg", reviewWarnDeg_);
        reviewCriticalMm_  = r.value("critical_mm", reviewCriticalMm_);
        reviewCriticalDeg_ = r.value("critical_deg", reviewCriticalDeg_);
    }

    reviewCriticalMm_  = std::max(reviewCriticalMm_, reviewWarnMm_ + 1e-9);
    reviewCriticalDeg_ = std::max(reviewCriticalDeg_, reviewWarnDeg_ + 1e-9);

    connect(&viewportTimer_, &QTimer::timeout, this, &Backend::viewportTick);
    viewportTimer_.start(16);   // viewport @ ~60fps

    connect(&panelTimer_, &QTimer::timeout, this, &Backend::panelTick);
    panelTimer_.start(100);   // readouts @ 10Hz (readability)
}

QString Backend::screenTitle() const
{
    switch (screen_)
    {
        case RotorPlacement:
            return QStringLiteral("Rotor Placement");
        case PositionTracking:
            return QStringLiteral("Position Tracking");
        case Home:
            break;
    }
    return QString();
}

void Backend::begin()
{
    if (screen_ != Home || !readyToBegin_) return;

    screen_ = anyGated_ ? RotorPlacement : PositionTracking;
    std::printf("[frontend/session] begin -> %s\n",
                anyGated_ ? "rotor placement review" : "position tracking (nothing to review)");
    std::fflush(stdout);
    publishSessionControl();
    emit dataChanged();
}

void Backend::setLogRequested(bool on)
{
    if (logRequested_ == on) return;
    logRequested_ = on;
    std::printf("[frontend/session] position data logging %s\n", on ? "ON" : "OFF");
    std::fflush(stdout);
    publishSessionControl();
    emit dataChanged();
}

void Backend::publishSessionControl()
{
    static const char* kPhase[] = {"home", "rotor_placement", "position_tracking"};
    ecal_.publishSessionControl(loggingActive(), kPhase[static_cast<int>(screen_)],
                                QDateTime::currentMSecsSinceEpoch() / 1000.0);
}

void Backend::viewportTick()
{
    havePlacements_ = state_.havePlacements();
    if (havePlacements_) placements_ = state_.placements();

    jointsLive_ = state_.jointsLive();
    if (jointsLive_) jointState_ = state_.joints();

    measuredJointsLive_ = state_.measuredJointsLive();
    if (measuredJointsLive_) measuredJointState_ = state_.measuredJoints();

    emit viewportChanged();
}

void Backend::panelTick()
{
    stale_ = state_.isStale();

    const ScenePlacementsPacket scene = state_.placements();
    sceneComplete_                    = scene.complete();
    anchorFrame_                      = QString::fromStdString(scene.anchor_frame());

    anchorStatus_ = scene.anchor_in_mocap().valid() ? QStringLiteral("anchored")
                                                    : QStringLiteral("NOT ANCHORED");

    int placed = 0;
    for (const auto& p : scene.placements())
        if (p.valid()) ++placed;
    placementSummary_ = QStringLiteral("%1/%2 placed").arg(placed).arg(scene.placements_size());

    std::unordered_map<std::string, const PlacedPose*> poseById;
    for (const auto& p : scene.placements())
        poseById.emplace(p.placement_id(), &p);

    const auto euler = [](const PlacedPose& p) {
        const Eigen::Quaterniond q(p.quat_w(), p.quat_x(), p.quat_y(), p.quat_z());
        const Eigen::Vector3d e = q.normalized().toRotationMatrix().eulerAngles(0, 1, 2);
        return QVector3D(static_cast<float>(e.x() * 180.0 / M_PI),
                         static_cast<float>(e.y() * 180.0 / M_PI),
                         static_cast<float>(e.z() * 180.0 / M_PI));
    };

    const auto fillSide = [&](QVariantMap& row, const QString& prefix, const std::string& id) {
        const auto it         = poseById.find(id);
        const bool ok         = it != poseById.end() && it->second->valid();
        row[prefix + "Valid"] = ok;
        if (!ok) return;
        const PlacedPose& p      = *it->second;
        row[prefix + "XMm"]      = p.pos_x() * 1000.0;
        row[prefix + "YMm"]      = p.pos_y() * 1000.0;
        row[prefix + "ZMm"]      = p.pos_z() * 1000.0;
        const QVector3D e        = euler(p);
        row[prefix + "RollDeg"]  = e.x();
        row[prefix + "PitchDeg"] = e.y();
        row[prefix + "YawDeg"]   = e.z();
    };

    comparisons_.clear();
    reviewComparisons_.clear();
    trackingRows_.clear();

    double worstSeverity = 0.0;
    double worstMm       = 0.0;
    double worstDeg      = 0.0;
    bool anyGated        = false;
    bool allGatedValid   = true;
    bool overWarnMm      = false;
    bool overWarnDeg     = false;

    const ComparisonPacket deltas = state_.comparisons();
    for (const auto& d : deltas.deltas())
    {
        QVariantMap row;
        row["objectId"]      = QString::fromStdString(d.object_id());
        row["a"]             = QString::fromStdString(d.a());
        row["b"]             = QString::fromStdString(d.b());
        row["aSource"]       = QString::fromStdString(d.a_source());
        row["bSource"]       = QString::fromStdString(d.b_source());
        row["distanceMm"]    = d.distance_mm();
        row["angleDeg"]      = d.angle_deg();
        row["dxMm"]          = d.dx_mm();
        row["dyMm"]          = d.dy_mm();
        row["dzMm"]          = d.dz_mm();
        row["timeGapS"]      = d.time_gap_s();
        row["valid"]         = d.valid();
        row["invalidReason"] = QString::fromStdString(d.invalid_reason());
        row["reviewGated"]   = d.review_gated();

        if (d.review_gated())
        {
            anyGated = true;
            if (!d.valid())
            {
                allGatedValid = false;
            }
            else
            {
                const double sMm  = d.distance_mm() / reviewCriticalMm_;
                const double sDeg = d.angle_deg() / reviewCriticalDeg_;
                worstSeverity     = std::max({worstSeverity, sMm, sDeg});
                worstMm           = std::max(worstMm, d.distance_mm());
                worstDeg          = std::max(worstDeg, d.angle_deg());

                if (d.distance_mm() >= reviewWarnMm_) overWarnMm = true;
                if (d.angle_deg() >= reviewWarnDeg_) overWarnDeg = true;
            }
            reviewComparisons_.append(row);
        }
        else
        {

            QVariantMap tracked = row;
            tracked["aId"]      = row["a"];
            tracked["bId"]      = row["b"];
            fillSide(tracked, QStringLiteral("a"), d.a());
            fillSide(tracked, QStringLiteral("b"), d.b());
            trackingRows_.append(tracked);
        }

        if (!d.review_gated() || !reviewAccepted_) comparisons_.append(row);
    }

    // --- camera-measured joint angles ---
    jointRows_.clear();
    const JointEstimatePacket est = state_.jointEstimates();
    for (const auto& j : est.joints())
    {
        QVariantMap row;
        row["jointId"]       = QString::fromStdString(j.joint_id());
        row["objectId"]      = QString::fromStdString(j.object_id());
        row["estimated"]     = j.estimated();
        row["skipReason"]    = QString::fromStdString(j.skip_reason());
        row["thetaDeg"]      = j.theta_deg();
        row["ageS"]          = j.age_s();
        row["clamped"]       = j.clamped();
        row["unconstrained"] = j.unconstrained();
        row["lowerDeg"]      = j.lower_deg();
        row["upperDeg"]      = j.upper_deg();

        row["radialErrorMm"]  = j.radial_error_mm();
        row["axialErrorMm"]   = j.axial_error_mm();
        row["residualMm"]     = j.residual_mm();
        row["residualDeg"]    = j.residual_deg();
        row["confidence"]     = j.confidence();
        row["reportedValid"]  = j.reported_valid();
        row["reportedDeg"]    = j.reported_deg();
        row["errorDeg"]       = j.error_deg();
        row["reportedStatus"] = QString::fromStdString(j.reported_status());
        jointRows_.append(row);
    }

    if (stale_)
        jointStatus_ = QStringLiteral("waiting for the backend");
    else if (!state_.haveJointEstimates())
        jointStatus_ = QStringLiteral("backend is not publishing joint angles");
    else if (jointRows_.isEmpty())
        jointStatus_ = QStringLiteral("no projected placements in the scene manifest");
    else
        jointStatus_.clear();

    reviewSeverity_          = std::clamp(worstSeverity, 0.0, 1.0);
    reviewShouldRecalibrate_ = overWarnMm || overWarnDeg;
    reviewWorstMm_           = worstMm;
    reviewWorstDeg_          = worstDeg;
    anyGated_                = anyGated;

    if (overWarnMm && overWarnDeg)
        reviewVerdict_ = QStringLiteral("position and orientation");
    else if (overWarnMm)
        reviewVerdict_ = QStringLiteral("position");
    else if (overWarnDeg)
        reviewVerdict_ = QStringLiteral("orientation");
    else
        reviewVerdict_.clear();

    if (stale_)
    {
        readyToBegin_       = false;
        beginBlockedReason_ = QStringLiteral("waiting for the backend");
    }
    else if (anyGated && !allGatedValid)
    {
        readyToBegin_       = false;
        beginBlockedReason_ = QStringLiteral("waiting for the latched placements to resolve");
    }
    else
    {
        readyToBegin_ = true;
        beginBlockedReason_.clear();
    }

    publishSessionControl();

    emit dataChanged();
}

void Backend::acceptReview()
{
    if (screen_ != RotorPlacement) return;
    reviewAccepted_ = true;
    screen_         = PositionTracking;
    std::printf("[frontend/review] latched placements ACCEPTED (worst %.2f mm / %.3f deg); "
                "position tracking, logging %s\n",
                reviewWorstMm_, reviewWorstDeg_, logRequested_ ? "ON" : "OFF");
    std::fflush(stdout);
    publishSessionControl();
    emit dataChanged();
}

void Backend::exitForRecalibration()
{
    std::printf("[frontend/review] operator rejected the latched placements "
                "(%s out of tolerance) -- exiting to recalibrate\n",
                reviewVerdict_.isEmpty() ? "nothing" : qPrintable(reviewVerdict_));
    std::fflush(stdout);
    QCoreApplication::exit(kRecalibrateExitCode);
}
