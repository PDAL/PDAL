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
#include <pdal/SpatialReference.hpp>
#include <json/json.h>
#include <arbiter/arbiter.hpp>

#include <array>
#include <functional>
#include <queue>
#include <vector>
#include <algorithm>                                                            
#include <chrono>     
#include <Eigen/Geometry>
#include <gdal.h>
#include <ogr_spatialref.h>

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
    int m_nodeCap;
    int m_count = 0;

    //File System vs Curl
    bool m_file = false;

    //Spatial Reference variables
    SpatialReference m_srsIn;
    SpatialReference m_srsOut;
    bool m_inferInputSRS;
    typedef void* ReferencePtr;
    typedef void* TransformPtr;
    ReferencePtr m_in_ref_ptr;
    ReferencePtr m_out_ref_ptr;
    TransformPtr m_transform_ptr;

    //Dimension booleans
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
    BOX3D parseBox(Json::Value base);
  };
}
