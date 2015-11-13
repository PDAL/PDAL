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

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();
    static Dimension::IdList getDefaultDimensions();

  private:
    std::unique_ptr<ILeStream> m_stream;
    point_count_t m_index;
    double m_scale_z;

    virtual void addDimensions(PointLayoutPtr layout);
    virtual void processOptions(const Options& options);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);
  };
}
