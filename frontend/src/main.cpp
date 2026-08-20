#include <QCoreApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuaternion>
#include <QQuickStyle>
#include <QString>
#include <QVector3D>

#include <arena/transport/ecal/EcalTopic.hpp>

#include <cstdio>
#include <vector>

#include "Backend.hpp"
#include "RobotScene.hpp"
#include "config.hpp"
#include "scene_config.hpp"

int main(int argc, char** argv)
{
    arena::transport::ecal::set_loopback_enabled(true);

    QGuiApplication app(argc, argv);
    QQuickStyle::setStyle("Material");

    scene::Scene sceneCfg;
    try
    {
        sceneCfg = scene::load(Config::load()["scene"]["manifest"].get<std::string>());
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "[frontend] FATAL: %s\n", e.what());
        return 1;
    }

    int rc = 0;
    {
        Backend backend;

        RobotScene robotScene;

        robotScene.setRenderSmoothing(Config::load()["frontend"].value("render_smoothing", 0.25));
        robotScene.loadScene(sceneCfg);

        QObject::connect(&backend, &Backend::viewportChanged, &robotScene, [&] {
            if (backend.jointsLive())
            {
                const auto& js = backend.jointState();
                robotScene.applyJointState(
                    QString::fromStdString(js.arm()), js.rail_position(),
                    std::vector<double>(js.robot_joints().begin(), js.robot_joints().end()),
                    std::vector<double>(js.hand_joints().begin(), js.hand_joints().end()));
            }
            else if (backend.measuredJointsLive())
            {
                const auto& ms = backend.measuredJointState();
                robotScene.applyJointState(
                    QString::fromStdString(ms.arm()), ms.rail_position(),
                    std::vector<double>(ms.robot_joints().begin(), ms.robot_joints().end()),
                    std::vector<double>(ms.hand_joints().begin(), ms.hand_joints().end()));
            }

            if (!backend.havePlacements()) return;
            for (const auto& p : backend.placements().placements())
            {

                const QQuaternion q(p.quat_w(), p.quat_x(), p.quat_y(), p.quat_z());
                robotScene.setPlacementPose(
                    QString::fromStdString(p.placement_id()),
                    QVector3D(p.pos_x(), p.pos_y(), p.pos_z()),
                    q.lengthSquared() > 1.0e-12f ? q.normalized() : QQuaternion(), p.valid());
            }
        });

        QQmlApplicationEngine engine;
        engine.addImportPath(QStringLiteral(MULIB_QML_IMPORT_PATH));   // Mu.Material, Mu.Cad
        engine.rootContext()->setContextProperty("backend", &backend);
        engine.rootContext()->setContextProperty("robotScene", &robotScene);
        QObject::connect(
            &engine, &QQmlApplicationEngine::objectCreationFailed, &app,
            [] { QCoreApplication::exit(1); }, Qt::QueuedConnection);
        engine.loadFromModule("CameraTracking", "Main");

        rc = app.exec();
    }
    return rc;
}
