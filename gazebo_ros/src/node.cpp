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

#include <gazebo_ros/node.hpp>

#include <memory>
#include <string>

namespace gazebo_ros
{

std::weak_ptr<Executor> Node::static_executor_;
std::mutex Node::lock_;

Node::~Node()
{
}

Node::SharedPtr Node::Create(const std::string & node_name)
{
  return Create<const std::string &>(node_name);
}

}  // namespace gazebo_ros
