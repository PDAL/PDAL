// MyWriter.cpp

#include "MyWriter.hpp"
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{
  static PluginInfo const s_info
  {
    "writers.mywriter",
    "My Awesome Writer",
    "http://path/to/documentation"
  };

  CREATE_SHARED_STAGE(MyWriter, s_info);

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

  void MyWriter::addArgs(ProgramArgs& args)
  {
    // setPositional() Makes the argument required.
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("newline", "Line terminator", m_newline, "\n");
    args.add("datafield", "Data field", m_datafield, "UserData");
    args.add("precision", "Precision", m_precision, 3);
  }

  void MyWriter::initialize()
  {
    m_stream = FileStreamPtr(FileUtils::createFile(m_filename, true),
      FileStreamDeleter());
    if (!m_stream)
    {
      std::stringstream out;
      out << "writers.mywriter couldn't open '" << m_filename <<
        "' for output.";
      throw pdal_error(out.str());
    }
  }


  void MyWriter::ready(PointTableRef table)
  {
    m_stream->precision(m_precision);
    *m_stream << std::fixed;

    Dimension::Id d = table.layout()->findDim(m_datafield);
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
