#pragma once

#include <arena/transport/ecal/ProtobufCallbackTopic.hpp>

namespace middleware {

template <typename T>
using EcalProtoPublisher = arena::transport::ecal::ProtobufCallbackPublisher<T>;

template <typename T>
using EcalProtoSubscriber = arena::transport::ecal::ProtobufCallbackSubscriber<T>;

} // namespace middleware
