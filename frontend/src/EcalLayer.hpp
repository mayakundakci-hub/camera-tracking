#pragma once

// ------------------------------------------------------------
// EcalLayer — subscribes, deserializes, writes AppState. Nothing else.
// ------------------------------------------------------------

#include <mu/middleware/EcalProtoTopic.hpp>

#include "AppState.hpp"

class EcalLayer {
public:
    explicit EcalLayer(AppState& state)
        : sub_("frontend", "hand/validation",
               [&state](const ValidationPacket& msg) {
                   state.update(msg);   // HOT PATH: state write only, no UI calls
               })
    {
    }
private:
    mu::middleware::EcalProtoSubscriber<ValidationPacket> sub_;
};
