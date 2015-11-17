// MyKernel.cpp

#include "MyKernel.hpp"

#include <boost/program_options.hpp>

#include <pdal/Filter.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/KernelFactory.hpp>
#include <pdal/KernelSupport.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointTable.hpp>

#include <memory>
#include <string>

namespace po = boost::program_options;

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

  void MyKernel::validateSwitches()
  {
    if (m_input_file == "")
      throw pdal::app_usage_error("--input/-i required");
    if (m_output_file == "")
      throw pdal::app_usage_error("--output/-o required");
  }

  void MyKernel::addSwitches()
  {
    po::options_description* options = new po::options_description("file options");
    options->add_options()
    ("input,i", po::value<std::string>(&m_input_file)->default_value(""), "input file name")
    ("output,o", po::value<std::string>(&m_output_file)->default_value(""), "output file name")
    ;

    addSwitchSet(options);
    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
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
