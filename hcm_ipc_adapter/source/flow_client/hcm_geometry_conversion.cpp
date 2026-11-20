// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <array>
#include <cstddef>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

namespace
{

template <typename Repeated>
void CovarianceToHcm(const Repeated & source, std::array<double, 36> & target)
{
  target.fill(0.0);
  for (int i = 0; i < source.size() && i < static_cast<int>(target.size()); ++i) {
    target[static_cast<std::size_t>(i)] = source.Get(i);
  }
}

template <typename Repeated>
void CovarianceToContract(const std::array<double, 36> & source, Repeated & target)
{
  target.Clear();
  target.Reserve(static_cast<int>(source.size()));
  for (double value : source) {
    target.Add(value);
  }
}

}  // namespace

void PoseWithCovarianceToHcm(const ::qrb::contract::geometry::v1::PoseWithCovariance & source,
    QRB::Memory::PoseWithCovariance_t & target)
{
  const auto & position = source.pose().position();
  const auto & orientation = source.pose().orientation();
  target.pose.position.x = position.x();
  target.pose.position.y = position.y();
  target.pose.position.z = position.z();
  target.pose.orientation.x = orientation.x();
  target.pose.orientation.y = orientation.y();
  target.pose.orientation.z = orientation.z();
  target.pose.orientation.w = orientation.w();
  CovarianceToHcm(source.covariance(), target.covariance);
}

void PoseWithCovarianceToContract(const QRB::Memory::PoseWithCovariance_t & source,
    ::qrb::contract::geometry::v1::PoseWithCovariance & target)
{
  auto * position = target.mutable_pose()->mutable_position();
  position->set_x(source.pose.position.x);
  position->set_y(source.pose.position.y);
  position->set_z(source.pose.position.z);
  auto * orientation = target.mutable_pose()->mutable_orientation();
  orientation->set_x(source.pose.orientation.x);
  orientation->set_y(source.pose.orientation.y);
  orientation->set_z(source.pose.orientation.z);
  orientation->set_w(source.pose.orientation.w);
  CovarianceToContract(source.covariance, *target.mutable_covariance());
}

void TwistToHcm(const ::qrb::contract::geometry::v1::Twist & source, QRB::Memory::Twist_t & target)
{
  target.linear.x = source.linear().x();
  target.linear.y = source.linear().y();
  target.linear.z = source.linear().z();
  target.angular.x = source.angular().x();
  target.angular.y = source.angular().y();
  target.angular.z = source.angular().z();
}

void TwistToContract(const QRB::Memory::Twist_t & source,
    ::qrb::contract::geometry::v1::Twist & target)
{
  auto * linear = target.mutable_linear();
  linear->set_x(source.linear.x);
  linear->set_y(source.linear.y);
  linear->set_z(source.linear.z);
  auto * angular = target.mutable_angular();
  angular->set_x(source.angular.x);
  angular->set_y(source.angular.y);
  angular->set_z(source.angular.z);
}

void TwistWithCovarianceToHcm(const ::qrb::contract::geometry::v1::TwistWithCovariance & source,
    QRB::Memory::TwistWithCovariance_t & target)
{
  TwistToHcm(source.twist(), target.twist);
  CovarianceToHcm(source.covariance(), target.covariance);
}

void TwistWithCovarianceToContract(const QRB::Memory::TwistWithCovariance_t & source,
    ::qrb::contract::geometry::v1::TwistWithCovariance & target)
{
  TwistToContract(source.twist, *target.mutable_twist());
  CovarianceToContract(source.covariance, *target.mutable_covariance());
}

}  // namespace qrb::adapter
