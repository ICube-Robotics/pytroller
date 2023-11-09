# Copyright 2023 ICube-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    command_message = None
    # Read command msg if applicable (i.e., there is one...)
    if (len(msg) > 0):
      mt = param.command_topic_type.decode("utf-8").split('/')
      messagetype = getattr(importlib.import_module('.'.join(mt[:2])), mt[-1])
      command_message = deserialize_message(bytes(msg), type(messagetype()))
    # Either way, call python logic
    (&commands)[0] = pytroller_logic_impl(states, commands, command_message, param)
  except Exception as error:
    print("An exception occurred:", error)
    return -1
  return 0
