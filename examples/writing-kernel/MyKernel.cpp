// MyKernel.cpp

#include "MyKernel.hpp"

#include <pdal/Filter.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>

#include <memory>
#include <string>


namespace pdal {

  static PluginInfo const s_info
  {
    "kernels.mykernel",
    "MyKernel",
    "http://link/to/documentation"
  };

  CREATE_SHARED_KERNEL(MyKernel, s_info);
  std::string MyKernel::getName() const { return s_info.name; }

  MyKernel::MyKernel() : Kernel()
  {}

  void MyKernel::addSwitches(ProgramArgs& args)
  {
      args.add("input,i", "Input filename", m_input_file).setPositional();
      args.add("output,o", "Output filename", m_output_file).setPositional();
  }

  int MyKernel::execute()
  {
    PointTable table;

    Stage& reader = makeReader(m_input_file, "readers.las");

    // Options should be added in the call to makeFilter, makeReader,
    // or makeWriter so that the system can override them with those
    // provided on the command line when applicable.
    Options filterOptions;
    filterOptions.add("step", 10);
    Stage& filter = makeFilter("filters.decimation", reader, filterOptions);

    Stage& writer = makeWriter(m_output_file, filter, "writers.text");
    writer.prepare(table);
    writer.execute(table);

    return 0;
  }

} // namespace pdal
