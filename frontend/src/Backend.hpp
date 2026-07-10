#pragma once

// ------------------------------------------------------------
// Backend — exposes AppState to QML.
//
// Smoothness rules baked in (unchanged from the widgets version):
//   - eCAL callback ONLY writes AppState (never touches QML-facing state)
//   - viewportTick() runs on a fixed 16ms QTimer, reads latest state
//   - panelTick() refreshes slower (100ms) than the viewport (readability)
// ------------------------------------------------------------

#include <QObject>
#include <QTimer>
#include <QVariantList>
#include <QVector3D>

#include "AppState.hpp"
#include "EcalLayer.hpp"

class Backend : public QObject {
    Q_OBJECT
    Q_PROPERTY(double errorMm READ errorMm NOTIFY dataChanged)
    Q_PROPERTY(double errorXMm READ errorXMm NOTIFY dataChanged)
    Q_PROPERTY(double errorYMm READ errorYMm NOTIFY dataChanged)
    Q_PROPERTY(double errorZMm READ errorZMm NOTIFY dataChanged)
    Q_PROPERTY(bool valid READ valid NOTIFY dataChanged)
    Q_PROPERTY(bool stale READ stale NOTIFY dataChanged)
    Q_PROPERTY(bool fanucIsStub READ fanucIsStub NOTIFY dataChanged)
    Q_PROPERTY(QVariantList errorHistory READ errorHistory NOTIFY dataChanged)
    Q_PROPERTY(QVector3D fanucPos READ fanucPos NOTIFY viewportChanged)
    Q_PROPERTY(QVector3D cameraPos READ cameraPos NOTIFY viewportChanged)

public:
    explicit Backend(QObject* parent = nullptr);

    double errorMm() const { return errorMm_; }
    double errorXMm() const { return errorXMm_; }
    double errorYMm() const { return errorYMm_; }
    double errorZMm() const { return errorZMm_; }
    bool valid() const { return valid_; }
    bool stale() const { return stale_; }
    bool fanucIsStub() const { return fanucIsStub_; }
    QVariantList errorHistory() const { return errorHistory_; }
    QVector3D fanucPos() const { return fanucPos_; }
    QVector3D cameraPos() const { return cameraPos_; }

signals:
    void dataChanged();
    void viewportChanged();

private:
    void viewportTick();
    void panelTick();

    AppState  state_;
    EcalLayer ecal_;
    QTimer viewportTimer_;
    QTimer panelTimer_;

    QVector3D fanucPos_;
    QVector3D cameraPos_;
    double errorMm_ = 0.0, errorXMm_ = 0.0, errorYMm_ = 0.0, errorZMm_ = 0.0;
    bool valid_ = false, stale_ = true, fanucIsStub_ = false;
    QVariantList errorHistory_;
};
