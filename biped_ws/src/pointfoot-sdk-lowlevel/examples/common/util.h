/**
 * @file util.h
 * @brief Utility program
 * @version 1.0
 * @date 2023-12-11
 * 
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 */
#pragma once

#include <iostream>
#include <vector>

/**
 * @brief Print array
 * 
 * @param name 
 * @param arr 
 * @param size 
 * @param e 
 */
void printArray(std::string name, float *arr, uint32_t size, std::string e = "\n");
/**
 * @brief Print st::vector
 * 
 * @param name 
 * @param v 
 * @param e 
 */
void printVector(std::string name, std::vector<float> v, std::string e = "\n");
