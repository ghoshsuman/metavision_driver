// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef METAVISION_ROS_DRIVER__DRIVER_ROS2_H_
#define METAVISION_ROS_DRIVER__DRIVER_ROS2_H_

#include <event_array_msgs/msg/event_array.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "metavision_ros_driver/callback_handler.h"

//
// resize without initializing, taken from
// https://stackoverflow.com/questions/15219984/
// using-vectorchar-as-a-buffer-without-initializing-it-on-resize

template <typename V>
void resize_hack(V & v, size_t newSize)
{
  struct vt
  {
    typename V::value_type v;
    vt() {}
  };
  static_assert(sizeof(vt[10]) == sizeof(typename V::value_type[10]), "alignment error");
  typedef std::vector<
    vt, typename std::allocator_traits<typename V::allocator_type>::template rebind_alloc<vt>>
    V2;
  reinterpret_cast<V2 &>(v).resize(newSize);
}

namespace metavision_ros_driver
{
class MetavisionWrapper;  // forward decl

class DriverROS2 : public rclcpp::Node, public CallbackHandler
{
  using EventArrayMsg = event_array_msgs::msg::EventArray;

public:
  explicit DriverROS2(const rclcpp::NodeOptions & options);
  ~DriverROS2();

  // ---------------- inherited from CallbackHandler -----------
  void rawDataCallback(uint64_t t, const uint8_t * start, const uint8_t * end) override;
  // ---------------- end of inherited  -----------
private:
  // service call to dump biases
  void saveBiases(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // related to dynanmic config (runtime parameter update)
  rcl_interfaces::msg::SetParametersResult parameterChanged(
    const std::vector<rclcpp::Parameter> & params);
  void onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event);
  void addBiasParameter(const std::string & n, int min_v, int max_v, const std::string & desc);
  void initializeBiasParameters();
  void declareBiasParameters();

  // for primary sync
  void secondaryReadyCallback(std_msgs::msg::Header::ConstSharedPtr msg);

  // misc helper functions
  bool start();
  bool stop();
  void configureWrapper(const std::string & name);

  // for synchronization with primary
  bool sendReadyMessage();

  // ------------------------  variables ------------------------------
  std::shared_ptr<MetavisionWrapper> wrapper_;
  int width_;   // image width
  int height_;  // image height
  bool isBigEndian_;
  std::string frameId_;
  std::string encoding_;
  uint64_t seq_{0};        // sequence number
  size_t reserveSize_{0};  // recommended reserve size
  uint64_t lastMessageTime_{0};
  uint64_t messageThresholdTime_{0};  // threshold time for sending message
  size_t messageThresholdSize_{0};    // threshold size for sending message
  EventArrayMsg::UniquePtr msg_;
  rclcpp::Publisher<EventArrayMsg>::SharedPtr eventPub_;

  // ------ related to sync
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr secondaryReadyPub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr secondaryReadySub_;
  rclcpp::Duration readyIntervalTime_;  // frequency of publishing ready messages
  rclcpp::Time lastReadyTime_;          // last time ready message was published

  // ------ related to dynamic config and services
  typedef std::map<std::string, rcl_interfaces::msg::ParameterDescriptor> ParameterMap;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callbackHandle_;
  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent, std::allocator<void>>>
    parameterSubscription_;
  ParameterMap biasParameters_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr saveBiasesService_;
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__DRIVER_ROS2_H_
