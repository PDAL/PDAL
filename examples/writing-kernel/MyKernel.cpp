// MyKernel.cpp

#include "MyKernel.hpp"

#include <pdal/Filter.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/KernelFactory.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointTable.hpp>

#include <memory>
#include <string>


namespace pdal {

  static PluginInfo const s_info {
    "kernels.mykernel",
    "MyKernel",
    "http://link/to/documentation"
  };

  CREATE_SHARED_PLUGIN(1, 0, MyKernel, Kernel, s_info);
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
    StageFactory f;

    Stage * reader = f.createStage("readers.las");
    Options readerOptions;
    readerOptions.add("filename", m_input_file);
    reader->setOptions(readerOptions);

    Stage * filter = f.createStage("filters.decimation");
    Options filterOptions;
    filterOptions.add("step", 10);
    filter->setOptions(filterOptions);
    filter->setInput(*reader);

    Stage * writer = f.createStage("writers.text");
    Options writerOptions;
    writerOptions.add("filename", m_output_file);
    writer->setOptions(writerOptions);
    writer->setInput(*filter);
    writer->prepare(table);
    writer->execute(table);

    return 0;
  }

} // namespace pdal
