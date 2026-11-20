// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_TWIST_ROS_CONVERTER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_TWIST_ROS_CONVERTER_HPP

#include <geometry_msgs/msg/twist_stamped.hpp>

#include "qrb/contract/geometry/v1/twist_stamped.pb.h"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_conversion.hpp"

namespace qrb::adapter
{

using TwistRosMessage = geometry_msgs::msg::TwistStamped;
using TwistRosContract = ::qrb::contract::geometry::v1::TwistStamped;

template <>
struct RosConversion<TwistRosMessage, TwistRosContract>
{
  using RosType = TwistRosMessage;
  using ContractType = TwistRosContract;

  static ConversionResult RosToContract(const RosType & source, ContractType & target);
  static ConversionResult ContractToRos(const ContractType & source, RosType & target);
};

using TwistRosConversion = RosConversion<TwistRosMessage, TwistRosContract>;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_TWIST_ROS_CONVERTER_HPP
