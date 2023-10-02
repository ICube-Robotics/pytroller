// Copyright 2023 ICube-Robotics
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
//
// author: Maciej Bednarczyk

#include <stddef.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_@(pytroller_name).hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;

namespace
{
rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}
}  // namespace

void Test@(pytroller_class)::SetUpTestCase() { rclcpp::init(0, nullptr); }

void Test@(pytroller_class)::TearDownTestCase() { rclcpp::shutdown(); }

void Test@(pytroller_class)::SetUp()
{
  controller_ = std::make_unique<Friend@(pytroller_class)>();
}

void Test@(pytroller_class)::TearDown() { controller_.reset(nullptr); }

void Test@(pytroller_class)::SetUpController()
{
  const auto result = controller_->init("test_@(pytroller_name)");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(joint1_ci_);
  command_ifs.emplace_back(joint2_ci_);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(joint1_sip_);
  state_ifs.emplace_back(joint1_siv_);
  state_ifs.emplace_back(joint2_sip_);
  state_ifs.emplace_back(joint2_siv_);

  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(Test@(pytroller_class), ConfigureAndActivateParamsSuccess)
{
  SetUpController();

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(Test@(pytroller_class), NoCommandCheckTest)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful, no command received yet
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    controller_interface::return_type::OK);

  // // check joint commands are still the default ones
  // ASSERT_EQ(joint1_ci_.get_value(), std::numeric_limits<double>::quiet_NaN());
  // ASSERT_EQ(joint2_ci_.get_value(), std::numeric_limits<double>::quiet_NaN());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
