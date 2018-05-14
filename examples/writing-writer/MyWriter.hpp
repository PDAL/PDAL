// MyWriter.hpp

#pragma once

#include <pdal/Writer.hpp>

#include <string>

namespace pdal{

  typedef std::shared_ptr<std::ostream> FileStreamPtr;

  class MyWriter : public Writer
  {
  public:
    MyWriter()
    {}

    std::string getName() const;

  private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);

    std::string m_filename;
    std::string m_newline;
    std::string m_datafield;
    int m_precision;

    FileStreamPtr m_stream;
    Dimension::Id m_dataDim;
  };

} // namespace pdal
