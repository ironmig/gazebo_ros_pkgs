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

#include <gtest/gtest.h>

#include <gazebo_ros/test/gazebo_process.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

struct TestParams
{
  std::vector<const char *> args;
  std::vector<std::string> topics;
};

class TestPlugins : public ::testing::TestWithParam<TestParams>
{
public:
  TestPlugins() {}
  void SetUp() override;
  void TearDown() override;

protected:
  std::unique_ptr<gazebo_ros::GazeboProcess> gazebo_process_;
};

void TestPlugins::SetUp()
{
  gazebo_process_ = std::make_unique<gazebo_ros::GazeboProcess>(GetParam().args);
  ASSERT_GT(gazebo_process_->run(), 0);
}

void TestPlugins::TearDown()
{
  ASSERT_GE(gazebo_process_->terminate(), 0);
  gazebo_process_.reset();
}

TEST_P(TestPlugins, TestTopicsReceived)
{
  auto topics = GetParam().topics;
  // Create node and executor
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("my_node");
  executor.add_node(node);


  // Subscribe to topics published by plugin
  using StringPtr = std_msgs::msg::String::SharedPtr;

  // Track which topics we have received a message from
  size_t topics_received_from = 0;
  std::vector<bool> received(topics.size(), false);

  std::vector<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> subs;
  for (size_t i = 0; i < topics.size(); ++i) {
    subs.push_back(node->create_subscription<std_msgs::msg::String>(topics[i],
      [&topics_received_from, &received, i](const StringPtr msg) {
        (void) msg;
        // If this is the first message from this topic, increment the counter
        if (!received[i]) {
          received[i] = true;
          ++topics_received_from;
        }
      }));
  }

  // Wait until message is received or timeout occurs
  using namespace std::literals::chrono_literals;
  auto timeout = node->now() + rclcpp::Duration(15s);

  while (topics_received_from != topics.size() && node->now() < timeout) {
    executor.spin_once(200ms);
  }

  // Wait a little while so gazebo isn't torn down before created
  rclcpp::sleep_for(1s);

  // Assert a message was received
  EXPECT_EQ(topics_received_from, topics.size());
}

INSTANTIATE_TEST_CASE_P(Plugins, TestPlugins, ::testing::Values(
    TestParams({{"-s", "./libargs_init.so"}, {"test"}}),
    TestParams({{"-s", "./libcreate_node_without_init.so"}, {"test"}}),
    TestParams({{"-s", "./libmultiple_nodes.so"}, {"testA", "testB"}}),
    TestParams({{"-s", "libgazebo_ros_init.so", "worlds/ros_world_plugin.world",
        "ros_world_plugin:/test:=/new_test"}, {"new_test"}}),
    TestParams({{"-s", "libgazebo_ros_init.so", "worlds/sdf_node_plugin.world"}, {"/foo/my_topic"}})
  ), );

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
