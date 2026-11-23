// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_TF_MESSAGE_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_TF_MESSAGE_ROS_CONVERTER_HPP

#include <tf2_msgs/msg/tf_message.hpp>

#include "qrb/contract/tf2/v1/tf_message.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using TFMessageRosMessage = tf2_msgs::msg::TFMessage;
using TFMessageRosContract = ::qrb::contract::tf2::v1::TFMessage;

template <>
struct RosConversion<TFMessageRosMessage, TFMessageRosContract>
{
  using RosType = TFMessageRosMessage;
  using ContractType = TFMessageRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using TFMessageRosConversion = RosConversion<TFMessageRosMessage, TFMessageRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_TF_MESSAGE_ROS_CONVERTER_HPP
