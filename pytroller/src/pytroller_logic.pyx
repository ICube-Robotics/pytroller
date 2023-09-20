# distutils: language = c++

from libcpp.unordered_map cimport unordered_map
from libcpp.string cimport string

##########################################################################################################################################################

def pytroller_logic_impl(states, references, commands):
  from math import cos, sin

  m1 = 1.0
  l1 = 1.0
  lc1 = l1/2
  i1 = m1/12*l1*l1
  m2 = 1.0
  l2 = 1.0
  lc2 = l2/2
  i2 = m2/12*l2*l2
  g = 9.8

  kp = 10.0
  kd = 5.0

  q_pos_j1 = states['joint1/position'.encode('ascii')]
  q_pos_j2 = states['joint2/position'.encode('ascii')]
  q_vel_j1 = states['joint1/velocity'.encode('ascii')]
  q_vel_j2 = states['joint2/velocity'.encode('ascii')]

  q_ref_j1 = references['joint1/effort'.encode('ascii')]
  q_ref_j2 = references['joint2/effort'.encode('ascii')]

  g_j1 = (m1*lc1*cos(q_pos_j1) + m2*(l1*cos(q_pos_j1) + lc2*cos(q_pos_j1 + q_pos_j2)))*g
  g_j2 = m2*lc2*cos(q_pos_j1 + q_pos_j2)*g

  commands['joint1/effort'.encode('ascii')] = g_j1 + kp*(q_ref_j1 - q_pos_j1) - kd*q_vel_j1
  commands['joint2/effort'.encode('ascii')] = g_j2 + kp*(q_ref_j2 - q_pos_j2) - kd*q_vel_j2

  return commands

##########################################################################################################################################################

cdef public int pytroller_logic(unordered_map[string, double] states, unordered_map[string, double] references, unordered_map[string, double] & commands):
  from pytroller_logic import pytroller_logic_impl
  try:
    commands = pytroller_logic_impl(states, references, commands)
  except Exception as error:
    print("An exception occurred:", error)
    return -1
  return 0