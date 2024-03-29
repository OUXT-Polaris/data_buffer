// Copyright (c) 2019 OUXT Polaris
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

#ifndef GEOMETRY_MSGS_DATA_BUFFER__POSE_STAMPED_DATA_BUFFER_HPP_
#define GEOMETRY_MSGS_DATA_BUFFER__POSE_STAMPED_DATA_BUFFER_HPP_

// headers in data buffer
#include <data_buffer/data_buffer_base.hpp>

// headers in ROS
#include <quaternion_operation/quaternion_operation.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace data_buffer
{
class PoseStampedDataBuffer : public DataBufferBase<geometry_msgs::msg::PoseStamped>
{
public:
  PoseStampedDataBuffer(
    rclcpp::Clock::SharedPtr clock, const std::string & key, double buffer_length);
  ~PoseStampedDataBuffer();

private:
  geometry_msgs::msg::PoseStamped interpolate(
    const geometry_msgs::msg::PoseStamped & data0, const geometry_msgs::msg::PoseStamped & data1,
    const rclcpp::Time & stamp) const override;
};
}  // namespace data_buffer

#endif  // GEOMETRY_MSGS_DATA_BUFFER__POSE_STAMPED_DATA_BUFFER_HPP_
