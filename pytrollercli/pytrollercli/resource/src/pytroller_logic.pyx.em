# distutils: language = c++

from libcpp.unordered_map cimport unordered_map
from libcpp.string cimport string

##########################################################################################################################################################

include "@(pytroller_name)_logic_impl.py"

##########################################################################################################################################################

cdef public int @(pytroller_name)_logic(unordered_map[string, double] states, unordered_map[string, double] references, unordered_map[string, double] & commands):
  from @(pytroller_name)_logic import pytroller_logic_impl
  try:
    commands = pytroller_logic_impl(states, references, commands)
  except Exception as error:
    print("An exception occurred:", error)
    return -1
  return 0