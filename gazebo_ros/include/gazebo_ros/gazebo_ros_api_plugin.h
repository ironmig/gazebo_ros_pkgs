/*
 * Copyright (C) 2012-2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef __GAZEBO_ROS_API_PLUGIN_HH__
#define __GAZEBO_ROS_API_PLUGIN_HH__

#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <gazebo_msgs/srv/delete_model.hpp>
#include <gazebo_msgs/srv/spawn_model.hpp>

#include <gazebo_ros/node.hpp>

namespace gazebo_ros
{

/// Plugin to provide ROS interfaces to gazebo
class GazeboRosApiPlugin : public gazebo::SystemPlugin
{
public:
  /// Constructor
  GazeboRosApiPlugin();

  /// Destructor
  ~GazeboRosApiPlugin();

  /// Gazebo-inherited load function
  /// \param _argc Number of command line arguments.
  /// \param _argv Array of command line arguments.
  void Load(int argc, char** argv);

private:
  /// Aliases for long service definitions
  using DeleteModelSrv = gazebo_msgs::srv::DeleteModel;
  using SpawnModelSrv = gazebo_msgs::srv::SpawnModel;
  using SetBoolSrv = std_srvs::srv::SetBool;

  /// Store world locally and create services/publishers for interacting with the world
  void WorldCreated(const std::string &_world);
  /// Publish updated states on world update events
  void OnWorldUpdateBegin();
  /// Publish the current gazebo SIM time to /clock
  void PublishSimTime();

  /// Callback for #delete_model_srv_
  void DeleteModelCB(const DeleteModelSrv::Request::SharedPtr request,
                     const DeleteModelSrv::Response::SharedPtr response);
  /// Callback for #spawn_model_srv_
  void SpawnModelCB(const SpawnModelSrv::Request::SharedPtr request,
                    const SpawnModelSrv::Response::SharedPtr response);
  /// Callback for #set_paused_srv_
  void SetPausedCB(const SetBoolSrv::Request::SharedPtr request,
                   const SetBoolSrv::Response::SharedPtr response);

  /// Currently unused
  /// TODO: lock all callbacks/calls that cannot operate in parallel
  std::mutex world_update_lock_;

  /// ROS node
  Node::SharedPtr node_;

  // Publishers
  std::shared_ptr<rclcpp::Publisher<rosgraph_msgs::msg::Clock> > clock_pub_;

  // Parameter for physics reconfigure
  rcl_interfaces::msg::SetParametersResult ParamChangeCB(const std::vector<rclcpp::Parameter>& params);

  /// Service to delete a model in the world
  rclcpp::ServiceBase::SharedPtr delete_model_srv_;
  /// Service to spawn a model into the world
  rclcpp::ServiceBase::SharedPtr spawn_model_srv_;
  /// Service to pause/resume gazebo simulation
  rclcpp::ServiceBase::SharedPtr set_paused_srv_;

  /// Event to detect when the first world is spawned
  gazebo::event::ConnectionPtr world_created_event_;
  /// Event for each world update
  gazebo::event::ConnectionPtr world_update_begin_event_;

  /// Ensures WorldCreated is only called once
  std::atomic_flag world_created_flag_;
  /// Pointer to the world used for API interactions
  gazebo::physics::WorldPtr world_;
};
}
#endif
