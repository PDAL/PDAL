// MyKernel.hpp

#pragma once

#include <pdal/Kernel.hpp>
#include <pdal/plugin.hpp>

#include <string>

namespace pdal
{

  class PDAL_DLL MyKernel : public Kernel
  {
  public:
    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;
    int execute(); // override

  private:
    MyKernel();
    void validateSwitches();
    void addSwitches();

    std::string m_input_file;
    std::string m_output_file;
  };

} // namespace pdal
