// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/tf_message_ros_converter.hpp"

#include <cstddef>

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_geometry.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

ConversionResult TFMessageRosConversion::RosToContract(const TFMessageRosMessage & source,
    TFMessageRosContract & target)
{
  target.Clear();
  target.mutable_transforms()->Reserve(static_cast<int>(source.transforms.size()));
  for (const auto & transform : source.transforms) {
    auto * out = target.add_transforms();
    HeaderToContract(transform.header, *out->mutable_header());
    out->set_child_frame_id(transform.child_frame_id);
    TransformToContract(transform.transform, *out->mutable_transform());
  }

  return ValidateTFMessage(target);
}

ConversionResult TFMessageRosConversion::ContractToRos(const TFMessageRosContract & source,
    TFMessageRosMessage & target)
{
  ConversionResult validation = ValidateTFMessage(source);
  if (!validation)
    return validation;

  target.transforms.resize(static_cast<std::size_t>(source.transforms_size()));
  for (int i = 0; i < source.transforms_size(); ++i) {
    const auto & transform = source.transforms(i);
    auto & out = target.transforms[static_cast<std::size_t>(i)];
    HeaderToRos(transform.header(), out.header);
    out.child_frame_id = transform.child_frame_id();
    TransformToRos(transform.transform(), out.transform);
  }
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
