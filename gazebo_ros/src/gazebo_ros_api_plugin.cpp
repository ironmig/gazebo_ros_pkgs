/*
 * Copyright 2018 Open Source Robotics Foundation
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

#include <gazebo/physics/physics.hh>

#include <gazebo_ros/gazebo_ros_api_plugin.h>

namespace gazebo_ros
{

GazeboRosApiPlugin::GazeboRosApiPlugin() : world_created_flag_(false)
{
}

GazeboRosApiPlugin::~GazeboRosApiPlugin()
{
}
void GazeboRosApiPlugin::Load(int argc, char** argv)
{
  Node::InitROS(argc, argv);

  node_ = Node::Create("gazebo_ros_api");

  clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  world_created_event_ = gazebo::event::Events::ConnectWorldCreated(
    std::bind(&GazeboRosApiPlugin::WorldCreated, this, std::placeholders::_1));
}

void GazeboRosApiPlugin::WorldCreated(const std::string &_world)
{
  // Only run this for the first world
  if(world_created_flag_.test_and_set())
  {
    RCLCPP_ERROR(node_->get_logger(), 
      "A second world [%s] has been loaded. API plugin will only work for models in the first world loaded.",
      _world.c_str());
    return;
  }

  // Store a pointer to the world
  world_ = gazebo::physics::get_world(_world);
  if (!world_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not get world [%s]. API Plugin will not work",
      _world.c_str());
    return;
  }
 
  // Register parameter event
  node_->register_param_change_callback(
    std::bind(&GazeboRosApiPlugin::ParamChangeCB, this, std::placeholders::_1));

  // Connect services
  delete_model_srv_ = node_->create_service<DeleteModelSrv>("delete_model",
    std::bind(&GazeboRosApiPlugin::DeleteModelCB, this, std::placeholders::_1, std::placeholders::_2));
  spawn_model_srv_ = node_->create_service<SpawnModelSrv>("spawn_model",
    std::bind(&GazeboRosApiPlugin::SpawnModelCB, this, std::placeholders::_1, std::placeholders::_2));
  set_paused_srv_ = node_->create_service<SetBoolSrv>("set_paused",
    std::bind(&GazeboRosApiPlugin::SetPausedCB, this, std::placeholders::_1, std::placeholders::_2));

  // Connect update event
  world_update_begin_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosApiPlugin::OnWorldUpdateBegin, this));
}

void GazeboRosApiPlugin::OnWorldUpdateBegin()
{
  // TODO: lock?
  PublishSimTime();
}

rcl_interfaces::msg::SetParametersResult GazeboRosApiPlugin::ParamChangeCB(const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  // TODO: set physics parameters on world update so they are accurate if changed externally
  // TODO: lock?
  auto physics = world_->Physics();
  for(auto param : params)
  {
    std::string name = param.get_name();
    // Any parameters that don't start with physics. are no-ops
    if (name.find("physics.") != 0) continue;

    using ParameterType = rclcpp::ParameterType;
    ParameterType type = param.get_type();
    if (name == "physics.max_step_size" && type == ParameterType::PARAMETER_DOUBLE)
      physics->SetMaxStepSize(param.as_double());
    else if (name == "physics.real_time_update_rate" && type == ParameterType::PARAMETER_DOUBLE)
      physics->SetRealTimeUpdateRate(param.as_double());
    else if (name == "physics.target_real_time_factor" && type == ParameterType::PARAMETER_DOUBLE)
      physics->SetTargetRealTimeFactor(param.as_double());
    else
    {
      std::stringstream ss;
      ss << "Parameter " << name << " not recognized.";
      result.successful = false;
      result.reason = ss.str();
      break;
    }
    RCLCPP_INFO(node_->get_logger(), "Param [%s]", param.get_name().c_str());
  }
  return result;
}

void GazeboRosApiPlugin::PublishSimTime()
{
  // TODO: throttle based on parameter for publish frequency
  gazebo::common::Time sim_time = world_->SimTime();
  rosgraph_msgs::msg::Clock ros_time_;
  ros_time_.clock.sec = sim_time.sec;
  ros_time_.clock.nanosec = sim_time.nsec;
  clock_pub_->publish(ros_time_);
}

void GazeboRosApiPlugin::DeleteModelCB(const DeleteModelSrv::Request::SharedPtr request,
                                       const DeleteModelSrv::Response::SharedPtr response)
{
  // TODO: lock?
  // If model didn't exist before, fail
  world_->RemoveModel(request->model_name);
  response->success = true;
  return;
  auto model = world_->ModelByName(request->model_name);
  if (!model)
  {
    response->success = false;
    response->status_message = "model doesn't exsist";
    return;
  }

  world_->RemoveModel(model);
  if (world_->ModelByName(request->model_name))
  {
    response->success = false;
    response->status_message = "failed to delete model";
  }
  else
    response->success = true;
}

void GazeboRosApiPlugin::SpawnModelCB(const SpawnModelSrv::Request::SharedPtr request,
                                      const SpawnModelSrv::Response::SharedPtr response)
{
  // TODO: lock?
  // TODO: handle pose, model name, etc from request
  world_->InsertModelString(request->model_xml);
  response->success = true;
}

void GazeboRosApiPlugin::SetPausedCB(const SetBoolSrv::Request::SharedPtr request,
                                     const SetBoolSrv::Response::SharedPtr response)
{
  // TODO: lock?
  world_->SetPaused(request->data);
  response->success = true;
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosApiPlugin)
}
