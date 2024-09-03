#ifndef _JACOBIAN_H_
#define _JACOBIAN_H_


#pragma once

#include <iostream>
#include <cmath>
#include <vector>

std::vector<double> state_vel_track(std::vector<double> q, std::vector<double> dq, std::vector<double> q_init, std::vector<double> l, bool left_leg);
std::vector<double> feet_distance_track(std::vector<double> q, std::vector<double> q_init, std::vector<double> l);

#endif // _JACOBIAN_H_