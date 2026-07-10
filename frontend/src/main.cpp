// =============================================================
// frontend  (Qt Quick / QML)
//
// Layers (matches Frontend Architecture slide):
//   EcalLayer — only place that knows eCAL/protobuf exists (EcalLayer.hpp)
//   AppState  — latest packet + rolling error history + staleness (AppState.hpp)
//   Backend   — QObject bridge exposing AppState to QML (Backend.hpp)
//   QML       — ViewportPanel (3D rotor + dual markers, rendering TODO) and
//               PanelsPanel (numeric error, legend, sparkline)
// =============================================================

#include <QCoreApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>

#include "Backend.hpp"

int main(int argc, char** argv)
{
    QGuiApplication app(argc, argv);
    QQuickStyle::setStyle("Material");

    int rc = 0;
    {
        Backend backend;

        QQmlApplicationEngine engine;
        engine.addImportPath(QStringLiteral(MULIB_QML_IMPORT_PATH));  // Mu.Material
        engine.rootContext()->setContextProperty("backend", &backend);
        QObject::connect(
            &engine, &QQmlApplicationEngine::objectCreationFailed,
            &app, [] { QCoreApplication::exit(1); }, Qt::QueuedConnection);
        engine.loadFromModule("CameraTracking", "Main");

        rc = app.exec();
    }
    return rc;
}
