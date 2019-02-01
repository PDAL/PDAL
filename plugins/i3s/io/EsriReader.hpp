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

#include <vector>
#include <map>
#include <math.h>

#include <json/json.h>
#include <arbiter/arbiter.hpp>

#include <pdal/Reader.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/IStream.hpp>

namespace pdal
{
namespace
{
std::map<std::string, pdal::Dimension::Id> const esriDims
{
    {"INTENSITY",   Dimension::Id::Intensity},
    {"CLASS_CODE",  Dimension::Id::ClassFlags},
    {"FLAGS",       Dimension::Id::Flag},
    {"RETURNS",     Dimension::Id::NumberOfReturns},
    {"USER_DATA",   Dimension::Id::UserData},
    {"POINT_SRC_ID",Dimension::Id::PointSourceId},
    {"GPS_TIME",    Dimension::Id::GpsTime},
    {"SCAN_ANGLE",  Dimension::Id::ScanAngleRank},
    {"RGB",         Dimension::Id::Red}
};
std::map<std::string, pdal::Dimension::Type> const dimTypes
{
    {"UInt8", Dimension::Type::Unsigned8},
    {"UInt16", Dimension::Type::Unsigned16},
    {"UInt32", Dimension::Type::Unsigned32},
    {"UInt64", Dimension::Type::Unsigned64},
    {"Int8", Dimension::Type::Signed8},
    {"Int16", Dimension::Type::Signed16},
    {"Int32", Dimension::Type::Signed32},
    {"Int64", Dimension::Type::Signed64},
    {"Float64", Dimension::Type::Double},
    {"Float32", Dimension::Type::Float}
};
}

class PDAL_DLL EsriReader : public Reader
{
public:
    BOX3D createBounds();

protected:
    virtual void initInfo() = 0;
    virtual std::vector<char> fetchBinary(std::string url, std::string attNum,
            std::string ext) const = 0;
    virtual Json::Value fetchJson(std::string) = 0;

    struct EsriArgs
    {
        Bounds bounds;
        uint16_t threads = 8;
        std::vector<std::string> dimensions;
        double min_density;
        double max_density;
    };

    struct Version
    {
        int major = 0;
        int minor = 0;
        int patch = 0;
        Version(){};
        Version(std::string vString)
        {
            std::istringstream iss(vString);
            std::string token;
            if(std::getline(iss, token, '.'))
                if(!token.empty())
                    major = std::stoi(token);
            if(std::getline(iss, token, '.'))
                if(!token.empty())
                    minor = std::stoi(token);
            if(std::getline(iss, token, '.'))
                if(!token.empty())
                    patch = std::stoi(token);
        }
        bool operator<(const Version& other)
        {
            if(this->major < other.major)
                return true;
            if(this->minor < other.minor && this->major == other.major)
                return true;
            if(this->patch < other.patch &&
                    this->major == other.major &&
                    this->minor == other.minor)
                return true;
            return false;
        }
        bool operator==(const Version& other)
        {
            return (this->patch == other.patch && this->major == other.major &&
                this->minor == other.minor);
        }
        bool operator <=(const Version& other)
            { return *this < other || *this == other; }
        bool operator >=(const Version& other)
            { return !(*this < other); }
        bool operator > (const Version& other)
            { return !(*this < other) && !(*this == other); }
    };

    std::unique_ptr<ILeStream> m_stream;
    std::unique_ptr<arbiter::Arbiter> m_arbiter;
    arbiter::gzip::Decompressor m_decomp;


    EsriArgs m_args;
    Json::Value m_info;
    std::mutex m_mutex;
    BOX3D m_bounds;
    BOX3D m_ecefBounds;
    std::map<std::string, Dimension::Id> m_dimensions;
    int m_nodeCap;
    int m_maxNode = 0;
    Version m_version;

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
    std::map<int, Json::Value> m_nodepages;

    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize(PointTableRef table) override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void ready(PointTableRef table) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual void done(PointTableRef table) override;
    void createView(std::string localUrl, int nodeIndex,  PointView& view);
    BOX3D createCube(Json::Value base);
    BOX3D parseBox(Json::Value base);
    void traverseTree(Json::Value page, int index, std::vector<int>& nodes,
        int depth, int pageIndex);
};



} // namespace pdal

