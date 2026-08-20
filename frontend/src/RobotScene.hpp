#pragma once
#include <mu/robot/RobotVisualModel.hpp>

#include <ludus/RobotData.hpp>

#include "scene_config.hpp"

#include <QAbstractItemModel>
#include <QObject>
#include <QQuaternion>
#include <QString>
#include <QVector3D>

#include <map>
#include <string>
#include <vector>

// A PURE VIEWER. 
class RobotScene : public QObject {
    Q_OBJECT
    Q_PROPERTY(QAbstractItemModel* model READ model CONSTANT)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(bool loaded READ loaded NOTIFY statusChanged)
    Q_PROPERTY(int linkCount READ linkCount NOTIFY statusChanged)

public:
    explicit RobotScene(QObject* parent = nullptr);

    QAbstractItemModel* model() { return &model_; }
    QString status() const { return status_; }
    bool loaded() const { return loaded_; }
    int linkCount() const { return model_.rowCount(); }

    bool loadScene(const scene::Scene& scene);

    // Poses the whole articulated object from controller joint values.
    void applyJointState(const QString& arm, double railPositionMm,
                         const std::vector<double>& robotJointsDeg,
                         const std::vector<double>& handJointsDeg);

    void setPlacementPose(const QString& placementId, const QVector3D& positionMetres,
                          const QQuaternion& rotation, bool valid);

    void setRenderSmoothing(double alpha);

signals:
    void statusChanged();

private:
    void setStatus(QString value, bool loaded);

    void buildJointIndex();
    void setJointByName(std::vector<double>& q, const QString& armPrefix,
                        const QString& suffix, double value) const;

    bool importArticulated(const scene::Placement& placement,
                           std::vector<mu::robot::RobotVisual>& rows);
    void addMeshRow(const scene::Object& object, const scene::Placement& placement,
                    std::vector<mu::robot::RobotVisual>& rows);

    mu::robot::RobotVisualModel model_;
    QString status_;
    bool loaded_{false};

    void poseArticulated();
    ludus::RobotDescription robot_;
    bool haveRobot_{false};
    std::map<std::string, int> jointIndex_;

    std::string articulatedId_;
    QVector3D   articulatedRootMm_{0.0f, 0.0f, 0.0f};
    QQuaternion articulatedRootRot_;
    bool        articulatedRootValid_{false};
    bool        loggedArticulated_{false};

    std::vector<double> lastJoints_;

    double renderSmoothing_{1.0};   // 1.0 = raw
    bool   haveSmoothedJoints_{false};
    bool   haveSmoothedRoot_{false};

    std::map<std::string, float> meshScale_;

    struct VisualOffset {
        QVector3D   positionScene;   // already in scene units, not metres
        QQuaternion rotation;
        bool        identity{true};  // skips the composition in the common case
    };
    std::map<std::string, VisualOffset> visualOffset_;
};
