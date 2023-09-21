<?xml version="1.0"?>
<package format="3">
  <name>@pytroller_name</name>
  <version>0.0.1</version>
  <description>Python controller for ros2_control.</description>
  <maintainer email="@maintainer_email">@maintainer_name</maintainer>

  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>controller_interface</depend>
  <depend>generate_parameter_library</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>realtime_tools</depend>
  <depend>std_msgs</depend>
  <depend>cython</depend>

  <test_depend>ament_cmake_gmock</test_depend>
  <test_depend>controller_manager</test_depend>
  <test_depend>ros2_control_test_assets</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>