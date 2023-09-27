@(pytroller_name):
  joints: {
    type: string_array,
    default_value: [],
    description: "Name of the joints to control",
  }
  interface_name: {
    type: string,
    default_value: "",
    description: "Name of the interface to command",
  }
  command_topic_name: {
    type: string,
    default_value: "~/commands",
    description: "Name of the subscribed command topic"
  }
  command_topic_type: {
    type: string,
    default_value: "std_msgs/msg/Float64MultiArray",
    description: "Type of the subscribed command topic"
  }
