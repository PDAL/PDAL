// I3SReader.hpp

#pragma once
#define ARBITER_ZLIB
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
    void fetchBinary(std::vector<char>& response, std::string url,
            std::string attNum, std::string ext);
    BOX3D createBounds();
    void buildNodeList(std::vector<int>& nodeArr, int pageIndex);
    
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
    gzip::Decompressor m_decomp;

    //Spatial Reference variables
    SpatialReference m_i3sRef;
    SpatialReference m_srsIn;
    SpatialReference m_srsOut;
    bool m_inferInputSRS;
    typedef void* ReferencePtr;
    typedef void* TransformPtr;
    ReferencePtr m_in_ref_ptr;
    ReferencePtr m_out_ref_ptr;
    TransformPtr m_transform_ptr;

    //Dimension variables
    bool isRGB = false;
    std::string idRGB;
    bool isElevation = false;
    std::string idElevation;
    bool isFlags = false;
    std::string idFlags;
    bool isReturns = false;
    std::string idReturns;
    bool isClass = false;
    std::string idClass;
    bool isSourceId = false;
    std::string idSourceId;
    bool isIntensity = false;
    std::string idIntensity;

    //methods
    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize(PointTableRef table) override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void ready(PointTableRef table) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual void done(PointTableRef table) override;
    BOX3D parseBox(Json::Value base);
  };
}
