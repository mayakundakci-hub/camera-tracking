#include "RobotScene.hpp"

#include <ludus/ForwardKinematics.hpp>
#include <ludus/RobotData.hpp>
#include <ludus/UrdfImporter.hpp>

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

std::array<double, 4> quatMul(const std::array<double, 4>& a,
                              const std::array<double, 4>& b) {
    const double aw = a[0], ax = a[1], ay = a[2], az = a[3];
    const double bw = b[0], bx = b[1], by = b[2], bz = b[3];
    return {
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    };
}

std::array<double, 3> rotateVector(const std::array<double, 4>& q,
                                   const std::array<double, 3>& v) {
    const std::array<double, 3> qv{q[1], q[2], q[3]};
    const std::array<double, 3> c1{
        qv[1] * v[2] - qv[2] * v[1],
        qv[2] * v[0] - qv[0] * v[2],
        qv[0] * v[1] - qv[1] * v[0],
    };
    const std::array<double, 3> c2{
        qv[1] * c1[2] - qv[2] * c1[1],
        qv[2] * c1[0] - qv[0] * c1[2],
        qv[0] * c1[1] - qv[1] * c1[0],
    };
    return {
        v[0] + 2.0 * q[0] * c1[0] + 2.0 * c2[0],
        v[1] + 2.0 * q[0] * c1[1] + 2.0 * c2[1],
        v[2] + 2.0 * q[0] * c1[2] + 2.0 * c2[2],
    };
}

ludus::Transform compose(const ludus::Transform& parent,
                         const ludus::Transform& local) {
    ludus::Transform out;
    out.rotation = quatMul(parent.rotation, local.rotation);
    const std::array<double, 3> rotated = rotateVector(parent.rotation, local.position);
    out.position = {
        parent.position[0] + rotated[0],
        parent.position[1] + rotated[1],
        parent.position[2] + rotated[2],
    };
    return out;
}

std::array<float, 3> toF3(const std::array<double, 3>& v) {
    return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}

std::array<float, 4> toF4(const std::array<double, 4>& q) {
    return {static_cast<float>(q[0]), static_cast<float>(q[1]),
            static_cast<float>(q[2]), static_cast<float>(q[3])};
}

QString joinErrors(const std::vector<std::string>& errors) {
    QString out;
    for (const std::string& e : errors) {
        if (!out.isEmpty()) out += QStringLiteral("; ");
        out += QString::fromStdString(e);
    }
    return out;
}

} // namespace

RobotScene::RobotScene(QObject* parent)
    : QObject(parent) {
    setStatus(QStringLiteral("No robot loaded"), false);
}

void RobotScene::setStatus(QString value, bool loaded) {
    status_ = std::move(value);
    loaded_ = loaded;
    emit statusChanged();
}

bool RobotScene::loadUrdf(const QString& path) {
    if (path.trimmed().isEmpty()) {
        model_.setVisuals({});
        setStatus(QStringLiteral("No robot loaded"), false);
        return false;
    }

    const ludus::UrdfImportResult imported =
        ludus::import_urdf_file(path.toStdString());
    if (!imported.ok) {
        model_.setVisuals({});
        setStatus(QStringLiteral("URDF import failed: %1").arg(joinErrors(imported.errors)),
                  false);
        return false;
    }

    ludus::RobotState state;
    state.robot_id = imported.robot.robot_id;
    state.joint_positions.assign(imported.robot.joints.size(), 0.0);

    const ludus::FkResult fk = ludus::compute_fk(imported.robot, state);
    if (!fk.ok) {
        model_.setVisuals({});
        setStatus(QStringLiteral("Forward kinematics failed: %1").arg(joinErrors(fk.errors)),
                  false);
        return false;
    }

    std::unordered_map<std::string, ludus::Transform> linkWorld;
    linkWorld.reserve(fk.link_poses.size());
    for (const ludus::LinkPose& pose : fk.link_poses) {
        linkWorld.emplace(pose.link_id, pose.world);
    }

    std::vector<mu::robot::RobotVisual> visuals;
    for (const ludus::RobotLink& link : imported.robot.links) {
        const auto it = linkWorld.find(link.id);
        if (it == linkWorld.end()) continue;

        for (const ludus::LinkVisual& visual : link.visuals) {
            if (visual.mesh.path.empty()) continue;

            const ludus::Transform world = compose(it->second, visual.origin);

            mu::robot::RobotVisual rv;
            rv.name = visual.name;
            rv.mesh_path = visual.mesh.path;
            rv.position = toF3(world.position);
            rv.rotation = toF4(world.rotation);
            rv.scale = toF3(visual.mesh.scale);
            // Per-link URDF <material><color rgba> -> per-row color role;
            // links without a material fall back to the view's modelColor.
            // (texture_path is not carried over -- none of these robots use
            // textures, and MuMultiModelView has no textured material yet.)
            if (visual.material) {
                rv.has_color = true;
                rv.color = {visual.material->color.r, visual.material->color.g,
                            visual.material->color.b, visual.material->color.a};
            }
            visuals.push_back(std::move(rv));
        }
    }

    const int count = static_cast<int>(visuals.size());
    model_.setVisuals(std::move(visuals));

    if (count == 0) {
        setStatus(QStringLiteral("Robot \"%1\" has no renderable mesh visuals")
                      .arg(QString::fromStdString(imported.robot.robot_id)),
                  false);
        return false;
    }

    setStatus(QStringLiteral("Loaded %1: %2 link visuals")
                  .arg(QString::fromStdString(imported.robot.robot_id))
                  .arg(count),
              true);
    return true;
}
