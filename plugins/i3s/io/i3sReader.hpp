// I3SReader.hpp

#pragma once

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>
#include "i3sReceiver.hpp"


#include <functional>
#include <queue>
#include <vector>

#include <json/json.h>

#include <arbiter/arbiter.hpp>

#include <pdal/StageFactory.hpp>


namespace pdal
{
  class I3SReader : public Reader
  {
  public:
    I3SReader() : Reader() {};
    std::string getName() const;

  private:
    std::unique_ptr<ILeStream> m_stream;
    point_count_t m_index;
    double m_scale_z;
    std::unique_ptr<arbiter::Arbiter> m_arbiter;

    I3SArgs m_args;
    Json::Value m_info;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table) override;
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);


  };
}
