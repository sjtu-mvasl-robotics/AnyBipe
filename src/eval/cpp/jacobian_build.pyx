# distutils: language = c++
from libcpp.vector cimport vector
from libcpp cimport bool
cdef extern from "jacobian.h":
    vector[double] state_vel_track(vector[double] q, vector[double] dq, vector[double] q_init, vector[double] l, bool left_leg)
    vector[double] feet_distance_track(vector[double] q, vector[double] q_init, vector[double] l)

def py_state_vel_track(q, dq, q_init, l, left_leg):
    cdef vector[double] vec_q = q
    cdef vector[double] vec_dq = dq
    cdef vector[double] vec_q_init = q_init
    cdef vector[double] vec_l = l
    return state_vel_track(vec_q, vec_dq, vec_q_init, vec_l, left_leg)

def py_feet_distance_track(q, q_init, l):
    cdef vector[double] vec_q = q
    cdef vector[double] vec_q_init = q_init
    cdef vector[double] vec_l = l
    return feet_distance_track(vec_q, vec_q_init, vec_l)
