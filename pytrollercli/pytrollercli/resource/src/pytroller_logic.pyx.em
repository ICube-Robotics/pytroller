# distutils: language = c++

from libcpp.unordered_map cimport unordered_map
from libcpp.vector cimport vector
from libcpp.string cimport string

##########################################################################################################################################################

include "../script/@(pytroller_name)_logic_impl.py"

include "@(pytroller_name)_parameters.pxd"

##########################################################################################################################################################

import importlib
from rclpy.serialization import deserialize_message

cdef public int @(pytroller_name)_logic(unordered_map[string, double] states, unordered_map[string, double] & commands, vector[int] & msg, Params param):
  try:
    mt = param.command_topic_type.decode("utf-8").split('/')
    messagetype = getattr(importlib.import_module('.'.join(mt[:2])), mt[-1])
    command_message = deserialize_message(bytes(msg), type(messagetype()))

    commands = pytroller_logic_impl(states, commands, command_message, param)
  except Exception as error:
    print("An exception occurred:", error)
    return -1
  return 0