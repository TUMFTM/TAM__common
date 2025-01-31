// Copyright 2024 Ann-Kathrin Schwehn

#include <iostream>

#include <memory>
#include <string>
#include <cmath>

#include "track_handler_cpp/rotation.hpp"
class SoftwareModule
{
  public: 
  auto test_euler_angles(double a, double b, double c, double d)
  {
    auto test = tam::common::track::angles_to_velocity_frame(a, b, c, d); 
    return test; 
  }
  auto compare_with_original_function(double a, double b, double c, double d)
  {
    auto another_test = tam::common::track::calc_2d_heading_from_chi(a, b, c, d); 
    return another_test; 
  }

};
int main()
{
  SoftwareModule test_1; 
  SoftwareModule comparison; 

  double chi = -(M_PI / 2), theta = 0.0, mu = 0, phi = (M_PI / 2); 
  auto test = test_1.test_euler_angles(chi, theta, mu, phi); 
  auto another_test = comparison.compare_with_original_function(chi, theta, mu, phi); 
  std::cout << "yaw: " << test[0] << std::endl; 
  std::cout << "pitch: " << test[1] << std::endl; 
  std::cout << "roll: " << test[2] << std::endl; 
  std::cout << "chi: " << chi << std::endl; 
  std::cout << "theta: " << theta << std::endl; 
  std::cout << "mu: " << mu << std::endl; 
  std::cout << "phi: " << phi << std::endl;   
  std::cout << "Compare with original function: " << another_test << std::endl; 
  return 0;
}
