#pragma once

// ------------------------------------------------------------
// AppState — thread-safe latest-value store + error history
// ------------------------------------------------------------

#include <QDateTime>

#include "camera_tracking.pb.h"

#include <deque>
#include <mutex>

using camera_tracking::ValidationPacket;

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
