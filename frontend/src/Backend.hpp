#pragma once
#include <QObject>
#include <QTimer>
#include <QVariantList>

#include "AppState.hpp"
#include "EcalLayer.hpp"


class Backend : public QObject {
    Q_OBJECT
    
    Q_PROPERTY(QVariantList comparisons READ comparisons NOTIFY dataChanged)

    // --- workflow screens -------------
    enum Screen { Home = 0, RotorPlacement = 1, PositionTracking = 2 };
    Q_ENUM(Screen)
    Q_PROPERTY(Screen screen READ screen NOTIFY dataChanged)
    Q_PROPERTY(QString screenTitle READ screenTitle NOTIFY dataChanged)

    Q_PROPERTY(bool readyToBegin READ readyToBegin NOTIFY dataChanged)
    Q_PROPERTY(QString beginBlockedReason READ beginBlockedReason NOTIFY dataChanged)

    // --- review gate -------------------------------------------------------
    Q_PROPERTY(QVariantList reviewComparisons READ reviewComparisons NOTIFY dataChanged)
    Q_PROPERTY(double reviewSeverity READ reviewSeverity NOTIFY dataChanged)
    Q_PROPERTY(bool reviewShouldRecalibrate READ reviewShouldRecalibrate NOTIFY dataChanged)
    Q_PROPERTY(QString reviewVerdict READ reviewVerdict NOTIFY dataChanged)
    Q_PROPERTY(bool reviewAccepted READ reviewAccepted NOTIFY dataChanged)
    Q_PROPERTY(double reviewWorstMm READ reviewWorstMm NOTIFY dataChanged)
    Q_PROPERTY(double reviewWorstDeg READ reviewWorstDeg NOTIFY dataChanged)

    Q_PROPERTY(double reviewWarnMm READ reviewWarnMm CONSTANT)
    Q_PROPERTY(double reviewWarnDeg READ reviewWarnDeg CONSTANT)
    Q_PROPERTY(double reviewCriticalMm READ reviewCriticalMm CONSTANT)
    Q_PROPERTY(double reviewCriticalDeg READ reviewCriticalDeg CONSTANT)

    // --- position tracking readout -----------------------------------------
    Q_PROPERTY(QVariantList trackingRows READ trackingRows NOTIFY dataChanged)

    // --- camera-measured joint angles ---------------------------------------
    
    Q_PROPERTY(QVariantList jointRows READ jointRows NOTIFY dataChanged)
    
    Q_PROPERTY(QString jointStatus READ jointStatus NOTIFY dataChanged)

    // --- logging ------------------------------------------------------------
    
    Q_PROPERTY(bool logRequested READ logRequested WRITE setLogRequested NOTIFY dataChanged)
    Q_PROPERTY(bool loggingActive READ loggingActive NOTIFY dataChanged)

    Q_PROPERTY(bool stale READ stale NOTIFY dataChanged)
    Q_PROPERTY(bool sceneComplete READ sceneComplete NOTIFY dataChanged)
    Q_PROPERTY(QString anchorFrame READ anchorFrame NOTIFY dataChanged)
    Q_PROPERTY(QString anchorStatus READ anchorStatus NOTIFY dataChanged)
    Q_PROPERTY(QString placementSummary READ placementSummary NOTIFY dataChanged)
    
    Q_PROPERTY(bool jointsLive READ jointsLive NOTIFY dataChanged)

    Q_PROPERTY(double viewPitchDeg READ viewPitchDeg CONSTANT)
    Q_PROPERTY(double viewYawDeg READ viewYawDeg CONSTANT)
    Q_PROPERTY(double viewPadding READ viewPadding CONSTANT)

public:
    explicit Backend(QObject* parent = nullptr);

    double viewPitchDeg() const { return viewPitchDeg_; }
    double viewYawDeg() const { return viewYawDeg_; }
    double viewPadding() const { return viewPadding_; }

    QVariantList comparisons() const { return comparisons_; }

    Screen screen() const { return screen_; }
    QString screenTitle() const;
    bool readyToBegin() const { return readyToBegin_; }
    QString beginBlockedReason() const { return beginBlockedReason_; }

    QVariantList reviewComparisons() const { return reviewComparisons_; }
    QVariantList trackingRows() const { return trackingRows_; }
    QVariantList jointRows() const { return jointRows_; }
    QString      jointStatus() const { return jointStatus_; }

    bool logRequested() const { return logRequested_; }
    void setLogRequested(bool on);
    bool loggingActive() const { return logRequested_ && screen_ == PositionTracking; }
    double reviewSeverity() const { return reviewSeverity_; }
    bool reviewShouldRecalibrate() const { return reviewShouldRecalibrate_; }
    QString reviewVerdict() const { return reviewVerdict_; }
    bool reviewAccepted() const { return reviewAccepted_; }
    double reviewWorstMm() const { return reviewWorstMm_; }
    double reviewWorstDeg() const { return reviewWorstDeg_; }
    double reviewWarnMm() const { return reviewWarnMm_; }
    double reviewWarnDeg() const { return reviewWarnDeg_; }
    double reviewCriticalMm() const { return reviewCriticalMm_; }
    double reviewCriticalDeg() const { return reviewCriticalDeg_; }


    Q_INVOKABLE void begin();

    Q_INVOKABLE void acceptReview();

    Q_INVOKABLE void exitForRecalibration();

    static constexpr int kRecalibrateExitCode = 2;

    bool stale() const { return stale_; }
    bool sceneComplete() const { return sceneComplete_; }
    QString anchorFrame() const { return anchorFrame_; }
    QString anchorStatus() const { return anchorStatus_; }
    QString placementSummary() const { return placementSummary_; }

    const ScenePlacementsPacket& placements() const { return placements_; }
    bool havePlacements() const { return havePlacements_; }
    bool jointsLive() const { return jointsLive_; }
    const JointStatePacket& jointState() const { return jointState_; }

    bool measuredJointsLive() const { return measuredJointsLive_; }
    const JointStatePacket& measuredJointState() const { return measuredJointState_; }

signals:
    void dataChanged();
    void viewportChanged();

private:
    void viewportTick();
    void panelTick();
    void publishSessionControl();

    AppState  state_;
    EcalLayer ecal_;
    QTimer viewportTimer_;
    QTimer panelTimer_;

    ScenePlacementsPacket placements_;
    bool havePlacements_ = false;

    JointStatePacket jointState_;
    bool jointsLive_ = false;

    JointStatePacket measuredJointState_;
    bool measuredJointsLive_ = false;

    QVariantList comparisons_;

    QVariantList reviewComparisons_;
    QVariantList trackingRows_;
    QVariantList jointRows_;
    QString      jointStatus_;

    Screen  screen_                  = Home;
    bool    readyToBegin_            = false;
    QString beginBlockedReason_;
    bool    anyGated_                = false;

    bool    reviewAccepted_          = false;
    bool    reviewShouldRecalibrate_ = false;
    double  reviewSeverity_          = 0.0;
    double  reviewWorstMm_           = 0.0;
    double  reviewWorstDeg_          = 0.0;
    QString reviewVerdict_;

    bool    logRequested_            = true;

    double reviewWarnMm_      = 5.0;
    double reviewWarnDeg_     = 0.5;
    double reviewCriticalMm_  = 25.0;
    double reviewCriticalDeg_ = 2.0;

    bool stale_ = true;
    bool sceneComplete_ = false;
    QString anchorFrame_;
    QString anchorStatus_;
    QString placementSummary_;

    double viewPitchDeg_ = -25.0;
    double viewYawDeg_   = 220.0;
    double viewPadding_  = 1.5;
};
