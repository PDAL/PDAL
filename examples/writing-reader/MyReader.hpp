// MyReader.hpp

#pragma once

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>

namespace pdal
{
  class MyReader : public Reader
  {
  public:
    MyReader() : Reader() {};
    std::string getName() const;

  private:
    std::unique_ptr<ILeStream> m_stream;
    point_count_t m_index;
    double m_scale_z;

    virtual void addDimensions(PointLayoutPtr layout);
    virtual void addArgs(ProgramArgs& args);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);
  };
}
