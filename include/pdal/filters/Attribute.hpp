/******************************************************************************
* Copyright (c) 2014, Howard Butler <hobu.inc@gmail.com>
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

#include <pdal/Filter.hpp>

#include <ogr_api.h>

#include <geos_c.h>



#include <map>
#include <string>

namespace pdal
{

namespace gdal
{
    class Debug;
}


namespace filters
{

typedef boost::shared_ptr<void> OGRDSPtr;
typedef boost::shared_ptr<void> OGRFeaturePtr;
typedef boost::shared_ptr<void> OGRGeometryPtr;

class AttributeInfo
{
public:
    AttributeInfo() : ds(0), lyr(0), isogr(true) {};

    std::string connection;
    std::string column;
    std::string datasource;
    OGRDSPtr ds;
    OGRLayerH lyr;
    std::string query;
    std::string layer;
    std::string value;
    bool isogr;
    Dimension::Id::Enum dim;
    AttributeInfo(const AttributeInfo& other)
        : connection(other.connection)
        , column(other.column)
        , datasource(other.datasource)
        , ds(other.ds)
        , lyr(other.lyr)
        , query(other.query)
        , layer(other.layer)
        , value(other.value)
        , isogr(other.isogr)
        , dim(other.dim) {};
    AttributeInfo& operator=(const AttributeInfo& other)
    {
        if (&other != this)
        {
            connection = other.connection;
            column = other.column;
            datasource = other.datasource;
            ds = other.ds;
            lyr = other.lyr;
            query = other.query;
            value = other.value;
            layer = other.layer;
            isogr = other.isogr;
            dim = other.dim;
        }
        return *this;
    }

};


typedef std::map< std::string, AttributeInfo> AttributeInfoMap;


class PDAL_DLL Attribute : public Filter
{
public:
    SET_STAGE_NAME("filters.attribute", "Data attribute filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.attribute.html")
    SET_STAGE_ENABLED(true)

    Attribute() : Filter(), m_geosEnvironment(0) {};
    static Options getDefaultOptions();

private:
    virtual void initialize();
    virtual void processOptions(const Options&);
    virtual void ready(PointContext ctx);
    virtual void filter(PointBuffer& buffer);
    virtual void done(PointContext ctx);

    Attribute& operator=(const Attribute&); // not implemented
    Attribute(const Attribute&); // not implemented

    typedef boost::shared_ptr<void> OGRDSPtr;

    AttributeInfoMap m_dimensions;
    GEOSContextHandle_t m_geosEnvironment;
    boost::shared_ptr<pdal::gdal::Debug> m_gdal_debug;
    void UpdateGEOSBuffer(PointBuffer& buffer, AttributeInfo& info);

};

} // namespace filters
} // namespace pdal

