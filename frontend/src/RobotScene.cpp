#include "RobotScene.hpp"

#include <ludus/RobotScene.hpp>
#include <ludus/UrdfImporter.hpp>

#include <QtMath>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iterator>


namespace {

std::array<float, 3> toF3(const std::array<double, 3>& v) {
    return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}

std::array<float, 4> toF4(const std::array<double, 4>& q) {
    return {static_cast<float>(q[0]), static_cast<float>(q[1]),
            static_cast<float>(q[2]), static_cast<float>(q[3])};
}

constexpr float kMetresToScene = 1000.0f;

std::array<float, 4> parseColor(const std::string& hex) {
    if (hex.size() != 7 || hex[0] != '#') return {0.8f, 0.8f, 0.8f, 1.0f};
    const auto channel = [&](std::size_t at) {
        return static_cast<float>(std::stoi(hex.substr(at, 2), nullptr, 16)) / 255.0f;
    };
    return {channel(1), channel(3), channel(5), 1.0f};
}

QString joinErrors(const std::vector<std::string>& errors) {
    QString out;
    for (const std::string& e : errors) {
        if (!out.isEmpty()) out += QStringLiteral("; ");
        out += QString::fromStdString(e);
    }
    return out;
}

mu::robot::RobotVisual toRobotVisual(const ludus::RobotSceneVisual& v) {
    mu::robot::RobotVisual rv;
    rv.name = v.name;
    rv.mesh_path = v.mesh_path;
    rv.object_id = v.object_id;
    rv.position = toF3(v.world.position);
    rv.rotation = toF4(v.world.rotation);
    rv.scale = toF3(v.scale);
    if (v.material) {
        rv.has_color = true;
        rv.color = {v.material->color.r, v.material->color.g,
                    v.material->color.b, v.material->color.a};
    }
    return rv;
}

} // namespace

RobotScene::RobotScene(QObject* parent)
    : QObject(parent), status_(QStringLiteral("No scene loaded")) {}

void RobotScene::setStatus(QString value, bool loaded) {
    status_ = std::move(value);
    loaded_ = loaded;
    emit statusChanged();
}

// building the scene

bool RobotScene::loadScene(const scene::Scene& sceneCfg) {
    meshScale_.clear();
    visualOffset_.clear();
    haveRobot_ = false;
    jointIndex_.clear();

    std::vector<mu::robot::RobotVisual> rows;
    int articulated = 0;

    for (const scene::Object& object : sceneCfg.objects) {
        for (const scene::Placement& placement : object.placements) {
            if (placement.source == scene::Source::JointState) {
                if (importArticulated(placement, rows)) ++articulated;
                continue;
            }
            // Frame-only objects (the anchor, typically) carry no geometry.
            if (object.visual_mesh.empty()) continue;

            if (!placement.visible) continue;

            addMeshRow(object, placement, rows);
        }
    }

    const int count = static_cast<int>(rows.size());
    model_.setVisuals(std::move(rows));

    if (count == 0) {
        setStatus(QStringLiteral("Scene has no renderable geometry"), false);
        return false;
    }

    setStatus(QStringLiteral("Loaded %1 rows (%2 articulated, %3 placed meshes)")
                  .arg(count)
                  .arg(articulated)
                  .arg(static_cast<int>(meshScale_.size())),
              true);
    return true;
}

bool RobotScene::importArticulated(const scene::Placement& placement,
                                    std::vector<mu::robot::RobotVisual>& rows) {
    const ludus::UrdfImportResult imported = ludus::import_urdf_file(placement.urdf);
    if (!imported.ok) {
        setStatus(QStringLiteral("URDF import failed for %1: %2")
                      .arg(QString::fromStdString(placement.id), joinErrors(imported.errors)),
                  false);
        return false;
    }

    ludus::RobotState home;
    home.robot_id = imported.robot.robot_id;
    home.joint_positions.assign(imported.robot.joints.size(), 0.0);

    const ludus::RobotScene built = ludus::build_robot_scene(imported.robot, home);
    if (!built.ok) {
        setStatus(QStringLiteral("Forward kinematics failed for %1: %2")
                      .arg(QString::fromStdString(placement.id), joinErrors(built.errors)),
                  false);
        return false;
    }

    rows.reserve(rows.size() + built.visuals.size());
    for (const ludus::RobotSceneVisual& v : built.visuals) rows.push_back(toRobotVisual(v));

    robot_ = imported.robot;
    haveRobot_ = true;
    articulatedId_ = placement.id;
    lastJoints_.assign(imported.robot.joints.size(), 0.0);
    buildJointIndex();
    return true;
}

void RobotScene::addMeshRow(const scene::Object& object, const scene::Placement& placement,
                             std::vector<mu::robot::RobotVisual>& rows) {
    
    mu::robot::RobotVisual rv;
    rv.name = placement.id;
    rv.mesh_path = object.visual_mesh;
    rv.has_color = true;
    rv.color = parseColor(placement.color);
    rv.position = {0.0f, 0.0f, 0.0f};

    rv.scale = {0.0f, 0.0f, 0.0f};

    meshScale_[placement.id] = static_cast<float>(object.mesh_scale);

    
    const frames::Vec3 t = object.visual_offset.translation;
    const frames::Quat q = object.visual_offset.rotation;

    VisualOffset vo;
    vo.positionScene = QVector3D(static_cast<float>(t.x()), static_cast<float>(t.y()),
                                 static_cast<float>(t.z())) * kMetresToScene;
    vo.rotation = QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                              static_cast<float>(q.y()), static_cast<float>(q.z()));
    vo.identity = t.isZero(1e-12) && std::abs(std::abs(q.w()) - 1.0) < 1e-12;
    visualOffset_[placement.id] = vo;

    rows.push_back(std::move(rv));
}

// posing

void RobotScene::buildJointIndex() {
    jointIndex_.clear();
    for (std::size_t i = 0; i < robot_.joints.size(); ++i)
        jointIndex_.emplace(robot_.joints[i].id, static_cast<int>(i));
}

void RobotScene::setJointByName(std::vector<double>& q, const QString& armPrefix,
                                 const QString& suffix, double value) const {
    auto it = jointIndex_.find((armPrefix + suffix).toStdString());


    if (it == jointIndex_.end()) it = jointIndex_.find(suffix.toStdString());

    if (it == jointIndex_.end()) return;   // joint absent in this URDF; skip quietly
    if (it->second >= 0 && static_cast<std::size_t>(it->second) < q.size())
        q[static_cast<std::size_t>(it->second)] = value;
}

void RobotScene::applyJointState(const QString& arm, double railPositionMm,
                                  const std::vector<double>& robotJointsDeg,
                                  const std::vector<double>& handJointsDeg) {
    if (!haveRobot_) return;

    QString prefix = arm.trimmed().isEmpty() ? QStringLiteral("Left") : arm.trimmed();
    if (!prefix.endsWith(QLatin1Char('_'))) prefix += QLatin1Char('_');

    std::vector<double> q(robot_.joints.size(), 0.0);

    setJointByName(q, prefix, QStringLiteral("rail_to_carriage"), railPositionMm);

    if (robotJointsDeg.size() >= 6) {
   
        const double j3Corrected = robotJointsDeg[2] + robotJointsDeg[1];

        setJointByName(q, prefix, QStringLiteral("joint_1"), qDegreesToRadians(robotJointsDeg[0]));
        setJointByName(q, prefix, QStringLiteral("joint_2"), qDegreesToRadians(robotJointsDeg[1]));
        setJointByName(q, prefix, QStringLiteral("joint_3"), qDegreesToRadians(j3Corrected));
        setJointByName(q, prefix, QStringLiteral("joint_4"), qDegreesToRadians(robotJointsDeg[3]));
        setJointByName(q, prefix, QStringLiteral("joint_5"), qDegreesToRadians(robotJointsDeg[4]));
        setJointByName(q, prefix, QStringLiteral("joint_6"), qDegreesToRadians(robotJointsDeg[5]));
    }

    static const QString kHandJoints[] = {
        QStringLiteral("hand_link_base_to_hand_link_1"),
        QStringLiteral("hand_link_1_to_hand_link_2"),
        QStringLiteral("hand_link_2_to_tool_rotate"),
    };
    const std::size_t handCount = std::min(handJointsDeg.size(), std::size(kHandJoints));
    for (std::size_t i = 0; i < handCount; ++i)
        setJointByName(q, prefix, kHandJoints[i], qDegreesToRadians(handJointsDeg[i]));

    if (haveSmoothedJoints_ && lastJoints_.size() == q.size() && renderSmoothing_ < 1.0) {
        for (std::size_t i = 0; i < q.size(); ++i)
            lastJoints_[i] += renderSmoothing_ * (q[i] - lastJoints_[i]);
    } else {
        lastJoints_ = q;
        haveSmoothedJoints_ = true;
    }
    poseArticulated();
}

void RobotScene::setRenderSmoothing(double alpha) {
    renderSmoothing_ = std::clamp(alpha, 0.01, 1.0);
}

void RobotScene::poseArticulated() {
    if (!haveRobot_) return;

    ludus::RobotState state;
    state.robot_id = robot_.robot_id;
    state.joint_positions = lastJoints_;

    const ludus::RobotScene built = ludus::build_robot_scene(robot_, state);
    if (!built.ok) return;

   
    const bool logging = articulatedRootValid_ && !loggedArticulated_;
    if (logging)
    {
        loggedArticulated_ = true;
        std::printf("[render] '%s': %zu visuals, root at [%.1f %.1f %.1f] mm\n",
                    articulatedId_.c_str(), built.visuals.size(),
                    articulatedRootMm_.x(), articulatedRootMm_.y(), articulatedRootMm_.z());
        std::printf("[render]   joints fed to FK:");
        for (double v : lastJoints_) std::printf(" %.3f", qRadiansToDegrees(v));
        std::printf(" deg\n");
    }

    // Move the existing meshes to their new world poses; no reload.
    for (const ludus::RobotSceneVisual& v : built.visuals) {
        const std::array<float, 3> p = toF3(v.world.position);
        const std::array<float, 4> r = toF4(v.world.rotation);
        const std::array<float, 3> s = toF3(v.scale);

        QVector3D   position(p[0], p[1], p[2]);
        QQuaternion rotation(r[0], r[1], r[2], r[3]);

        
        if (articulatedRootValid_) {
            position = articulatedRootMm_ + articulatedRootRot_.rotatedVector(position);
            rotation = articulatedRootRot_ * rotation;
        }

        if (logging)
            std::printf("[render]   %-24s FK [%8.1f %8.1f %8.1f] -> drawn [%8.1f %8.1f %8.1f] mm"
                        "  scale [%.3f %.3f %.3f]  %s\n",
                        v.name.c_str(), p[0], p[1], p[2],
                        position.x(), position.y(), position.z(), s[0], s[1], s[2],
                        v.mesh_path.c_str());

        model_.updateTransform(QString::fromStdString(v.name), position, rotation,
                               QVector3D(s[0], s[1], s[2]));
    }
}

void RobotScene::setPlacementPose(const QString& placementId, const QVector3D& positionMetres,
                                   const QQuaternion& rotation, bool valid) {
    
    if (!articulatedId_.empty() && placementId.toStdString() == articulatedId_) {       
        if (valid) {
            articulatedRootValid_ = true;

            const QVector3D   posMm = positionMetres * kMetresToScene;
            const QQuaternion rot   = rotation;
            if (haveSmoothedRoot_ && renderSmoothing_ < 1.0) {
                const float a = static_cast<float>(renderSmoothing_);
                articulatedRootMm_  = articulatedRootMm_ + (posMm - articulatedRootMm_) * a;
                articulatedRootRot_ =
                    QQuaternion::slerp(articulatedRootRot_, rot, a).normalized();
            } else {
                articulatedRootMm_  = posMm;
                articulatedRootRot_ = rot;
                haveSmoothedRoot_   = true;
            }
        }
        poseArticulated();
        return;
    }

    const auto it = meshScale_.find(placementId.toStdString());
    if (it == meshScale_.end())
        return;   // no geometry for this placement (frame-only)

    const float s = valid ? it->second : 0.0f;

    QVector3D   position = positionMetres * kMetresToScene;
    QQuaternion orientation = rotation;

    const auto off = visualOffset_.find(placementId.toStdString());
    if (off != visualOffset_.end() && !off->second.identity) {
        position += rotation.rotatedVector(off->second.positionScene);
        orientation = rotation * off->second.rotation;
    }

    model_.updateTransform(placementId, position, orientation, QVector3D(s, s, s));
}
