load_@(pytroller_name):
  ros__parameters:
    interface_full_names:
     - "joint1/position"
     - "joint2/position"
    command_topic_name: "~/commands"
    command_topic_type: "std_msgs/msg/Float64MultiArray"

test_@(pytroller_name):
  ros__parameters:
    interface_full_names:
     - "joint1/position"
     - "joint2/position"
    command_topic_name: "~/commands"
    command_topic_type: "std_msgs/msg/Float64MultiArray"
