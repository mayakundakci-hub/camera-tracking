#pragma once

// ------------------------------------------------------------
// RobotScene — loads a (flattened, plain) URDF via ludus, runs forward
// kinematics at the home pose, and exposes the resulting per-link visuals as a
// mu::robot::RobotVisualModel that Mu.Material's MuMultiModelView renders.
//
// The robot files under Dimachaerus.Files/Rendering are xacro macros; urdfdom
// (and therefore ludus) cannot parse xacro. Point this at a *flattened* URDF
// produced once by the ROS `xacro` tool, e.g.
//     xacro URDFSeperatedGlobalRenderMacro.urdf -o cell_flat.urdf
//
// Static render for now: all joints sit at 0. Live joint-driven motion (from an
// eCAL joint-state topic) would call model_.updateTransform(...) per link on
// each viewport tick instead of a one-shot setVisuals().
// ------------------------------------------------------------

#include <mu/robot/RobotVisualModel.hpp>

#include <QAbstractItemModel>
#include <QObject>
#include <QString>

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

    // Imports the URDF at `path`, runs home-pose FK, and repopulates the model.
    // Returns false and sets a human-readable status() on any failure.
    Q_INVOKABLE bool loadUrdf(const QString& path);

signals:
    void statusChanged();

private:
    void setStatus(QString value, bool loaded);

    mu::robot::RobotVisualModel model_;
    QString status_;
    bool loaded_{false};
};
