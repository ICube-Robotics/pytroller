# distutils: language = c++

from libcpp.vector cimport vector
from libcpp.string cimport string

cdef extern from "@(pytroller_name)_parameters.hpp" namespace "@(pytroller_name)":
  cdef cppclass Params:
    vector[string] joints
    string interface_name
    string command_topic_name
    string command_topic_type