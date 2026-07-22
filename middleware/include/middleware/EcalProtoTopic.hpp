#pragma once

#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace middleware {

void retain_ecal(const std::string& process_name);
void release_ecal();

template <typename T>
class EcalProtoPublisher final {
public:
  EcalProtoPublisher(std::string process_name, std::string topic_name)
      : process_name_(std::move(process_name)) {
    retain_ecal(process_name_);
    publisher_ = std::make_unique<eCAL::protobuf::CPublisher<T>>(std::move(topic_name));
  }
  ~EcalProtoPublisher() {
    publisher_.reset();
    release_ecal();
  }

  EcalProtoPublisher(const EcalProtoPublisher&) = delete;
  EcalProtoPublisher& operator=(const EcalProtoPublisher&) = delete;

  void send(const T& msg) { publisher_->Send(msg); }

private:
  std::string process_name_;
  std::unique_ptr<eCAL::protobuf::CPublisher<T>> publisher_;
};

template <typename T>
class EcalProtoSubscriber final {
public:
  using Callback = std::function<void(const T&)>;

  EcalProtoSubscriber(std::string process_name, std::string topic_name, Callback callback)
      : process_name_(std::move(process_name)), callback_(std::move(callback)) {
    retain_ecal(process_name_);
    subscriber_ = std::make_unique<eCAL::protobuf::CSubscriber<T>>(std::move(topic_name));
    subscriber_->AddReceiveCallback(
        [this](const char*, const T& msg, long long, long long, long long) { callback_(msg); });
  }
  ~EcalProtoSubscriber() {
    if (subscriber_) { subscriber_->RemReceiveCallback(); }
    subscriber_.reset();
    release_ecal();
  }

  EcalProtoSubscriber(const EcalProtoSubscriber&) = delete;
  EcalProtoSubscriber& operator=(const EcalProtoSubscriber&) = delete;

private:
  std::string process_name_;
  Callback callback_;
  std::unique_ptr<eCAL::protobuf::CSubscriber<T>> subscriber_;
};

} // namespace middleware
