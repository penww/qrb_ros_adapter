// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_TWIST_HCM_CONVERTER_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_TWIST_HCM_CONVERTER_HPP

#include "QRB/Memory/Geometry/TwistDescriptor.hpp"
#include "qrb/contract/geometry/v1/twist_stamped.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using TwistHcmContract = ::qrb::contract::geometry::v1::TwistStamped;
using TwistHcmDescriptor = QRB::Memory::TwistDescriptor_t;

template <>
ConversionResult ConvertContractToHcm<TwistHcmContract, TwistHcmDescriptor>(
    const TwistHcmContract & source,
    const DmaBufView & payload,
    TwistHcmDescriptor & target);

template <>
ConversionResult ConvertHcmToContract<TwistHcmContract, TwistHcmDescriptor>(
    const TwistHcmDescriptor & source,
    TwistHcmContract & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_TWIST_HCM_CONVERTER_HPP
