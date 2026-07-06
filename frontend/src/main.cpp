// =============================================================
// frontend  (Qt / mulib skeleton)
//
// Layers (matches Frontend Architecture slide):
//   EcalLayer      — only place that knows eCAL/protobuf exists
//   AppState       — latest packet + rolling error history + staleness
//   ViewportWidget — 3D rotor + dual markers (STL/URDF rendering TODO)
//   PanelsWidget   — numeric error, legend, plot (throttled refresh)
//
// Smoothness rules baked in:
//   - eCAL callback ONLY writes AppState (never touches widgets)
//   - render loop runs on a fixed QTimer, reads latest state
//   - panels refresh slower than viewport (readability)
// =============================================================

#include <QApplication>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include "camera_tracking.pb.h"

#include <deque>
#include <mutex>

using camera_tracking::ValidationPacket;

// ------------------------------------------------------------
// AppState — thread-safe latest-value store + error history
// ------------------------------------------------------------
class AppState {
public:
    void update(const ValidationPacket& pkt) {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_ = pkt;
        lastArrivalMs_ = QDateTime::currentMSecsSinceEpoch();
        if (pkt.valid()) {
            errorHistory_.push_back(pkt.error_mm());
            if (errorHistory_.size() > kHistoryLen) errorHistory_.pop_front();
        }
    }
    ValidationPacket latest() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return latest_;
    }
    bool isStale(qint64 maxAgeMs = 200) const {
        std::lock_guard<std::mutex> lk(mtx_);
        return (QDateTime::currentMSecsSinceEpoch() - lastArrivalMs_) > maxAgeMs;
    }
    std::deque<double> errorHistory() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return errorHistory_;
    }
private:
    static const size_t kHistoryLen = 600;  // ~1 min at 10Hz plot rate
    mutable std::mutex mtx_;
    ValidationPacket   latest_;
    qint64             lastArrivalMs_ = 0;
    std::deque<double> errorHistory_;
};

// ------------------------------------------------------------
// EcalLayer — subscribes, deserializes, writes AppState. Nothing else.
// ------------------------------------------------------------
class EcalLayer {
public:
    explicit EcalLayer(AppState& state)
        : sub_("hand/validation")
    {
        sub_.AddReceiveCallback(
            [&state](const char*, const ValidationPacket& msg, long long, long long) {
                state.update(msg);   // HOT PATH: state write only, no UI calls
            });
    }
private:
    eCAL::protobuf::CSubscriber<ValidationPacket> sub_;
};

// ------------------------------------------------------------
// ViewportWidget — 3D scene: rotor model + two markers
// ------------------------------------------------------------
class ViewportWidget : public QWidget {
public:
    explicit ViewportWidget(AppState& state) : state_(state) {
        setMinimumSize(600, 400);
        // TODO (per team feedback): load environment STLs / URDF here
        //   - publish/consume STL + URDF assets of the cell for rendering
        //   - options: Qt3D, QOpenGLWidget + custom loader, or mulib's
        //     own 3D widget once it exists
        // TODO: marker convention — blue = motor (fanuc), orange = camera
        // TODO: interpolate marker motion between samples (data rate < 60fps)
    }
    void renderTick() {
        const auto pkt   = state_.latest();
        const bool stale = state_.isStale();
        // TODO: update marker transforms from pkt.pose_fanuc / pkt.pose_camera
        // TODO: if (stale || !pkt.valid()) gray out camera marker + "no signal" badge
        update();  // schedule repaint
    }
private:
    AppState& state_;
};

// ------------------------------------------------------------
// PanelsWidget — numeric readout, legend, error plot
// ------------------------------------------------------------
class PanelsWidget : public QWidget {
public:
    explicit PanelsWidget(AppState& state) : state_(state) {
        auto* lay = new QVBoxLayout(this);
        errorLabel_ = new QLabel("error: -- mm", this);
        statusLabel_ = new QLabel("waiting for data...", this);
        lay->addWidget(errorLabel_);
        lay->addWidget(statusLabel_);
        lay->addStretch();
        // TODO: rolling error plot from state_.errorHistory()
        // TODO: per-axis breakdown, warn color above threshold
    }
    void refreshTick() {
        const auto pkt = state_.latest();
        if (state_.isStale())        statusLabel_->setText("NO DATA (stale)");
        else if (!pkt.valid())       statusLabel_->setText("tracking invalid");
        else                         statusLabel_->setText("live");
        errorLabel_->setText(QString("error: %1 mm").arg(pkt.error_mm(), 0, 'f', 2));
    }
private:
    AppState& state_;
    QLabel* errorLabel_;
    QLabel* statusLabel_;
};

// ------------------------------------------------------------
int main(int argc, char** argv)
{
    eCAL::Initialize(argc, argv, "frontend");
    QApplication app(argc, argv);

    AppState  state;
    EcalLayer ecal(state);

    QWidget window;
    window.setWindowTitle("Camera Tracking — Hand Position Validation");
    auto* outerLay = new QVBoxLayout(&window);

    // Persistent, hard-to-miss indicator: never let simulated fanuc
    // poses be mistaken for a real controller link.
    auto* stubBanner = new QLabel(&window);
    stubBanner->setAlignment(Qt::AlignCenter);
    stubBanner->setVisible(false);
    outerLay->addWidget(stubBanner);

    auto* contentLay = new QHBoxLayout();
    auto* viewport = new ViewportWidget(state);
    auto* panels   = new PanelsWidget(state);
    contentLay->addWidget(viewport, /*stretch*/ 3);
    contentLay->addWidget(panels,   /*stretch*/ 1);
    outerLay->addLayout(contentLay);
    window.show();

    // Fixed-rate loops, decoupled from data arrival
    QTimer renderTimer;   // viewport @ ~60fps
    QObject::connect(&renderTimer, &QTimer::timeout, [&] { viewport->renderTick(); });
    renderTimer.start(16);

    QTimer panelTimer;    // readouts @ 10Hz (readability)
    QObject::connect(&panelTimer, &QTimer::timeout, [&] {
        panels->refreshTick();
        const bool isStub = state.latest().fanuc_is_stub();
        stubBanner->setVisible(isStub);
        if (isStub) {
            stubBanner->setText("⚠ FANUC DATA IS SIMULATED (stub mode) — not a real controller link ⚠");
            stubBanner->setStyleSheet("background-color:#c0392b; color:white; font-weight:bold; padding:6px;");
        }
    });
    panelTimer.start(100);

    const int rc = app.exec();
    eCAL::Finalize();
    return rc;
}
