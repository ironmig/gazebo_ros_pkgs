// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <gazebo_ros/node.hpp>
#include <boost/variant.hpp>


#include <string>
#include <memory>


namespace gazebo_plugins
{

class GazeboRosRaySensorPrivate;

class GazeboRosRaySensor : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  GazeboRosRaySensor();

  /// \brief Destructor
  virtual ~GazeboRosRaySensor();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  std::unique_ptr<GazeboRosRaySensorPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_
