// MyKernel.hpp

#pragma once

#include <pdal/Kernel.hpp>

#include <string>

namespace pdal
{

class PDAL_DLL MyKernel : public Kernel
{
public:
    MyKernel();

    std::string getName() const;
    int execute(); // override

private:
    void addSwitches(ProgramArgs& args);

    std::string m_input_file;
    std::string m_output_file;
};

} // namespace pdal
