// MyWriter.cpp

#include "MyWriter.hpp"

#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace pdal
{
  static PluginInfo const s_info = PluginInfo(
    "writers.mywriter",
    "My Awesome Writer",
    "http://path/to/documentation" );

  CREATE_SHARED_PLUGIN(1, 0, MyWriter, Writer, s_info);

  std::string MyWriter::getName() const { return s_info.name; }

  struct FileStreamDeleter
  {
    template <typename T>
    void operator()(T* ptr)
    {
      if (ptr)
      {
        ptr->flush();
        FileUtils::closeFile(ptr);
      }
    }
  };


  Options MyWriter::getDefaultOptions()
  {
    Options options;

    options.add("newline", "\n", "Newline character to use for additional lines");
    options.add("filename", "", "filename to write output to");
    options.add("datafield", "", "field to use as the User Data field");
    options.add("precision", 3, "Precision to use for data fields");

    return options;
  }


  void MyWriter::processOptions(const Options& options)
  {
    m_filename = options.getValueOrThrow<std::string>("filename");
    m_stream = FileStreamPtr(FileUtils::createFile(m_filename, true),
      FileStreamDeleter());
    if (!m_stream)
    {
      std::stringstream out;
      out << "writers.mywriter couldn't open '" << m_filename <<
        "' for output.";
      throw pdal_error(out.str());
    }

    m_newline = options.getValueOrDefault<std::string>("newline", "\n");
    m_datafield = options.getValueOrDefault<std::string>("datafield", "UserData");
    m_precision = options.getValueOrDefault<int>("precision", 3);
  }


  void MyWriter::ready(PointTableRef table)
  {
    m_stream->precision(m_precision);
    *m_stream << std::fixed;

    Dimension::Id::Enum d = table.layout()->findDim(m_datafield);
    if (d == Dimension::Id::Unknown)
    {
      std::ostringstream oss;
      oss << "Dimension not found with name '" << m_datafield << "'";
      throw pdal_error(oss.str());
    }

    m_dataDim = d;

    *m_stream << "#X:Y:Z:MyData" << m_newline;
  }


  void MyWriter::write(PointViewPtr view)
  {
      for (PointId idx = 0; idx < view->size(); ++idx)
      {
        double x = view->getFieldAs<double>(Dimension::Id::X, idx);
        double y = view->getFieldAs<double>(Dimension::Id::Y, idx);
        double z = view->getFieldAs<double>(Dimension::Id::Z, idx);
        unsigned int myData = 0;

        if (!m_datafield.empty()) {
          myData = (int)(view->getFieldAs<double>(m_dataDim, idx) + 0.5);
        }

        *m_stream << x << ":" << y << ":" << z << ":"
          << myData << m_newline;
      }
  }


  void MyWriter::done(PointTableRef)
  {
    m_stream.reset();
  }

}
