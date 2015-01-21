// MyWriter.hpp

#pragma once

#include "Writer.hpp"

#include <string>

class MyWriter : public pdal::Writer
{
public:
  SET_STAGE_NAME ("MyWriter", "My Awesome Writer")
  SET_STAGE_LINK ("http://link/to/documentation")

  MyWriter() : Writer() {};
  int execute();

private:
  void validateSwitches();
  void addSwitches();

  std::string m_input_file;
  std::string m_output_file;
};

