/******************************************************************************
* Copyright (c) 2018, Kyle Mann (kyle@hobu.co)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#pragma once
#include "../lepcc/src/include/lepcc_c_api.h"
#include "../lepcc/src/include/lepcc_types.h"
#include "EsriUtil.hpp"
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
namespace
{
std::map<std::string, pdal::Dimension::Id> const m_dimensions
{
        {"INTENSITY",   pdal::Dimension::Id::Intensity},
        {"CLASS_CODE",  pdal::Dimension::Id::ClassFlags},
        {"FLAGS",       pdal::Dimension::Id::Flag},
        {"RETURNS",     pdal::Dimension::Id::NumberOfReturns},
        {"USER_DATA",   pdal::Dimension::Id::UserData},
        {"POINT_SRC_ID",pdal::Dimension::Id::PointSourceId},
        {"GPS_TIME",    pdal::Dimension::Id::GpsTime},
        {"SCAN_ANGLE",  pdal::Dimension::Id::ScanAngleRank}
};
std::map<std::string, pdal::Dimension::Type> const dimTypes
{
        {"UInt8", pdal::Dimension::Type::Unsigned8},
        {"UInt16", pdal::Dimension::Type::Unsigned16},
        {"UInt32", pdal::Dimension::Type::Unsigned32},
        {"UInt64", pdal::Dimension::Type::Unsigned64},
        {"Int8", pdal::Dimension::Type::Signed8},
        {"Int16", pdal::Dimension::Type::Signed16},
        {"Int32", pdal::Dimension::Type::Signed32},
        {"Int64", pdal::Dimension::Type::Signed64},
        {"Float64", pdal::Dimension::Type::Double},
        {"Float32", pdal::Dimension::Type::Float}
};
}

namespace pdal
{

class EsriReader : public Reader
{
public:
    BOX3D createBounds();

protected:
    virtual void initInfo() = 0;
    virtual void buildNodeList(std::vector<int>& nodes, int pageIndex) = 0;
    virtual std::vector<char> fetchBinary(std::string url, std::string attNum,
            std::string ext) const = 0;



    std::unique_ptr<ILeStream> m_stream;
    std::unique_ptr<arbiter::Arbiter> m_arbiter;

    struct I3SArgs
    {
      Bounds bounds;
      int threads = 8;
    };

    I3SArgs m_args;
    Json::Value m_info;
    std::mutex m_mutex;
    BOX3D m_bounds;
    int m_nodeCap;
    int m_maxNode = 0;


    gzip::Decompressor m_decomp;

    //Spatial Reference variables
    SpatialReference m_nativeSrs;
    SpatialReference m_ecefSrs;

    typedef void* ReferencePtr;
    typedef void* TransformPtr;
    ReferencePtr m_nativeRef;
    ReferencePtr m_ecefRef;

    TransformPtr m_toEcefTransform;
    TransformPtr m_toNativeTransform;

    struct dimData
    {
        int key;
        std::string dataType;
        Dimension::Type dimType;
        std::string name;
    };
    std::map<Dimension::Id, dimData> m_dimMap;


    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize(PointTableRef table) override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void ready(PointTableRef table) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual void done(PointTableRef table) override;
    void createView(std::string localUrl, PointViewPtr view);
    BOX3D parseBox(Json::Value base);
};

class I3SReader : public EsriReader
{
public:
    std::string getName() const override;

protected:
    virtual void initInfo() override;
    virtual void buildNodeList(std::vector<int>& nodes, int pageIndex) override;
    virtual std::vector<char> fetchBinary(std::string url, std::string attNum,
            std::string ext) const override;
};

class SlpkReader : public EsriReader
{
public:
    std::string getName() const override;

protected:
    virtual void initInfo() override;
    virtual void buildNodeList(std::vector<int>& nodes, int pageIndex) override;
    virtual std::vector<char> fetchBinary(std::string url, std::string attNum,
            std::string ext) const override;
};

} // namespace pdal

