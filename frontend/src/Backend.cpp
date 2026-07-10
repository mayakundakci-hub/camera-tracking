#include "Backend.hpp"

Backend::Backend(QObject* parent)
    : QObject(parent)
    , ecal_(state_)
{
    connect(&viewportTimer_, &QTimer::timeout, this, &Backend::viewportTick);
    viewportTimer_.start(16);   // viewport @ ~60fps

    connect(&panelTimer_, &QTimer::timeout, this, &Backend::panelTick);
    panelTimer_.start(100);     // readouts @ 10Hz (readability)
}

void Backend::viewportTick() {
    const auto pkt = state_.latest();
    // TODO: once real 3D rendering exists (Qt Quick 3D + STL/URDF assets of
    // the cell), interpolate marker motion between samples here instead of
    // snapping fanucPos_/cameraPos_ straight to the latest sample.
    fanucPos_  = QVector3D(pkt.pose_fanuc().pos_x(),  pkt.pose_fanuc().pos_y(),  pkt.pose_fanuc().pos_z());
    cameraPos_ = QVector3D(pkt.pose_camera().pos_x(), pkt.pose_camera().pos_y(), pkt.pose_camera().pos_z());
    emit viewportChanged();
}

void Backend::panelTick() {
    const auto pkt = state_.latest();
    valid_ = pkt.valid();
    stale_ = state_.isStale();
    fanucIsStub_ = pkt.fanuc_is_stub();
    errorMm_  = pkt.error_mm();
    errorXMm_ = pkt.error_x_mm();
    errorYMm_ = pkt.error_y_mm();
    errorZMm_ = pkt.error_z_mm();

    errorHistory_.clear();
    for (double v : state_.errorHistory()) errorHistory_.append(v);

    emit dataChanged();
}
