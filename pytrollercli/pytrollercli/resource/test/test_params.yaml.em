load_@(pytroller_name):
  ros__parameters:
    joints: ["joint1", "joint2"]
    interface_name: "position"
    command_topic_name: "~/commands"
    command_topic_type: "std_msgs/msg/Float64MultiArray"

test_@(pytroller_name):
  ros__parameters:
    joints: ["joint1", "joint2"]
    interface_name: "position"
    command_topic_name: "~/commands"
    command_topic_type: "std_msgs/msg/Float64MultiArray"