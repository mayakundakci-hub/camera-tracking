#pragma once

#include <middleware/EcalProtoTopic.hpp>

#include "AppState.hpp"

class EcalLayer {
public:
    explicit EcalLayer(AppState& state)
        : sub_("frontend", "hand/validation",
               [&state](const ValidationPacket& msg) {
                   state.update(msg);
               })
    {
    }
private:
    middleware::EcalProtoSubscriber<ValidationPacket> sub_;
};
