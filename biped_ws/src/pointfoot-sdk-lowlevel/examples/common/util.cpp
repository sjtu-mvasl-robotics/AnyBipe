#include "util.h"

void printArray(std::string name, float *arr, uint32_t size, std::string e)
{
  std::cout << name << "[";
  for (uint16_t i = 0; i < size - 1; i++)
  {
    std::cout << arr[i] << ", ";
  }
  std::cout << arr[size - 1] << "]" << e;
}

void printVector(std::string name, std::vector<float> v, std::string e)
{
  std::cout << name << "[";
  for (uint16_t i = 0; i < v.size() - 1; i++)
  {
    std::cout << v[i] << ", ";
  }
  std::cout << v[v.size() - 1] << "]" << e;
}
