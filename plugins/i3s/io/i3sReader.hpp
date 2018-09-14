// I3SReader.hpp

#pragma once

#include "../lepcc/src/include/lepcc_c_api.h"
#include "../lepcc/src/include/lepcc_types.h"
#include "i3sReceiver.hpp"
#include "pool.hpp"
#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/PointLayout.hpp>
#include <pdal/StageFactory.hpp>
#include <json/json.h>
#include <arbiter/arbiter.hpp>

#include <functional>
#include <queue>
#include <vector>
#include <algorithm>                                                            
#include <chrono>     



namespace pdal
{
  class I3SReader : public Reader
  {
  public:
    I3SReader() : Reader() {};
    std::string getName() const;
    void createView(std::string localUrl, PointViewPtr view);
    void fetchBinary(std::vector<char>& response, std::string url, int attNum, Pool& p);
    BOX3D createBounds();
    
  private:
    std::unique_ptr<ILeStream> m_stream;
    point_count_t m_index;
    double m_scale_z;
    std::unique_ptr<arbiter::Arbiter> m_arbiter;

    I3SArgs m_args;
    Json::Value m_info;
    std::mutex m_mutex;
    BOX3D m_bounds;

    struct Node 
    {
        int name;
        double minx, maxx;
        double miny, maxy;
        double minz, maxz;
        
        int childCount;
        Node parent;
        Node* children;
    };
    
    bool isRGB = false;
    bool isElevation = false;
    bool isFlags = false;
    bool isReturns = false;
    bool isClass = false;
    bool isSourceId = false;
    bool isIntensity = false;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table) override;
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);

  };
}
