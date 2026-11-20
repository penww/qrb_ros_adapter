// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/twist_ros_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_geometry.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

ConversionResult TwistRosConversion::RosToContract(const TwistRosMessage & source,
    TwistRosContract & target)
{
  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());
  TwistToContract(source.twist, *target.mutable_twist());

  return ValidateTwistStamped(target);
}

ConversionResult TwistRosConversion::ContractToRos(const TwistRosContract & source,
    TwistRosMessage & target)
{
  ConversionResult validation = ValidateTwistStamped(source);
  if (!validation)
    return validation;

  HeaderToRos(source.header(), target.header);
  TwistToRos(source.twist(), target.twist);
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
