@(pytroller_name):
  interface_full_names: {
    type: string_array,
    default_value: [],
    description: "Name (WARNING, full names, e.g., 'joint_1/effort') of the interface(s) to command",
  }
  command_topic_name: {
    type: string,
    default_value: "",
    description: "Optionnal. Name of the subscribed command topic"
  }
  command_topic_type: {
    type: string,
    default_value: "",
    description: "Optionnal. Type of the subscribed command topic"
  }

