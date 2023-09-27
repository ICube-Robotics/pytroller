# distutils: language = c++

from libcpp.unordered_map cimport unordered_map
from libcpp.vector cimport vector
from libcpp.string cimport string

##########################################################################################################################################################

include "@(pytroller_name)_logic_impl.py"

##########################################################################################################################################################

cdef public int @(pytroller_name)_logic(unordered_map[string, double] states, unordered_map[string, double] & commands, vector[int] & msg):
  from @(pytroller_name)_logic import pytroller_logic_impl
  try:
    commands = pytroller_logic_impl(states, commands, msg)
  except Exception as error:
    print("An exception occurred:", error)
    return -1
  return 0