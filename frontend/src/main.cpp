// =============================================================
// frontend  (Qt Quick / QML)
//
// Layers (matches Frontend Architecture slide):
//   EcalLayer — only place that knows eCAL/protobuf exists (EcalLayer.hpp)
//   AppState  — latest packet + rolling error history + staleness (AppState.hpp)
//   Backend   — QObject bridge exposing AppState to QML (Backend.hpp)
//   RobotScene— loads a flattened URDF via ludus, home-pose FK, feeds the
//               multi-model viewport (RobotScene.hpp)
//   QML       — ViewportPanel (MuMultiModelView of the robot + tracking
//               readouts) and PanelsPanel (numeric error, legend, sparkline)
// =============================================================

#include <QByteArray>
#include <QCoreApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QString>

#include "Backend.hpp"
#include "RobotScene.hpp"

namespace {
QString resolveRobotUrdf()
{
    if (const QByteArray env = qgetenv("CAMERA_TRACKING_ROBOT_URDF"); !env.isEmpty())
        return QString::fromLocal8Bit(env);
#ifdef CAMERA_TRACKING_ROBOT_URDF
    return QStringLiteral(CAMERA_TRACKING_ROBOT_URDF);
#else
    return {};
#endif
}

} // namespace

int main(int argc, char** argv)
{
    QGuiApplication app(argc, argv);
    QQuickStyle::setStyle("Material");

    int rc = 0;
    {
        Backend backend;

        RobotScene robotScene;
        robotScene.loadUrdf(resolveRobotUrdf());

        QQmlApplicationEngine engine;
        engine.addImportPath(QStringLiteral(MULIB_QML_IMPORT_PATH));  // Mu.Material, Mu.Cad
        engine.rootContext()->setContextProperty("backend", &backend);
        engine.rootContext()->setContextProperty("robotScene", &robotScene);
        QObject::connect(
            &engine, &QQmlApplicationEngine::objectCreationFailed,
            &app, [] { QCoreApplication::exit(1); }, Qt::QueuedConnection);
        engine.loadFromModule("CameraTracking", "Main");

        rc = app.exec();
    }
    return rc;
}
