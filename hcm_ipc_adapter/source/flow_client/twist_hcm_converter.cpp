// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/twist_hcm_converter.hpp"

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

template <>
ConversionResult ConvertContractToHcm<TwistHcmContract, TwistHcmDescriptor>(
    const TwistHcmContract & twist,
    const DmaBufView & /*payload*/,
    TwistHcmDescriptor & descriptor)
{
  // Everything that can fail is checked before the descriptor is written, so a
  // rejected message leaves the host's descriptor untouched.
  ConversionResult validation = ValidateTwistStamped(twist);
  if (!validation)
    return validation;
  if (twist.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN)
    return ConversionResult::Failure("frame_id is longer than the HCM Twist descriptor holds");

  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(twist.header().stamp(), timestampNs);
  if (!time)
    return time;

  if (!AssignHcmString(twist.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM Twist descriptor");
  descriptor.timestampNs = timestampNs;
  TwistToHcm(twist.twist(), descriptor.twist);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<TwistHcmContract, TwistHcmDescriptor>(
    const TwistHcmDescriptor & descriptor,
    TwistHcmContract & twist)
{
  twist.Clear();
  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *twist.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  twist.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));
  TwistToContract(descriptor.twist, *twist.mutable_twist());

  // A non-finite value produced on the HCM side must not be published to ROS.
  return ValidateTwistStamped(twist);
}

}  // namespace qrb::adapter
