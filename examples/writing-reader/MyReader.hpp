// MyReader.hpp

#pragma once

#include "Reader.hpp"

#include <string>

class MyReader : public pdal::Reader
{
public:
  SET_STAGE_NAME ("MyReader", "My Awesome Reader")
  SET_STAGE_LINK ("http://link/to/documentation")

  MyReader() : Reader() {};
  int execute();

private:
  void validateSwitches();
  void addSwitches();

  std::string m_input_file;
  std::string m_output_file;
};

