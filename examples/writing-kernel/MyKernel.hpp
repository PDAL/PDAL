// MyKernel.hpp

#pragma once

#include <pdal/Kernel.hpp>

#include <string>

class MyKernel : public pdal::Kernel
{
public:
  SET_KERNEL_NAME ("MyKernel", "My Awesome Kernel")
  SET_KERNEL_LINK ("http://link/to/documentation")

  MyKernel() : Kernel() {};
  int execute();

private:
  void validateSwitches();
  void addSwitches();

  std::string m_input_file;
  std::string m_output_file;
};

