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

#include <map>
#include <memory>
#include <string>

typedef struct GEOSContextHandle_HS *GEOSContextHandle_t;

typedef void *OGRLayerH;


namespace pdal
{

namespace gdal
{
    class ErrorHandler;
}

typedef std::shared_ptr<void> OGRDSPtr;
typedef std::shared_ptr<void> OGRFeaturePtr;
typedef std::shared_ptr<void> OGRGeometryPtr;

class PDAL_DLL AttributeFilter : public Filter
{
public:
    AttributeFilter() : Filter(), m_ds(0), m_lyr(0), m_geosEnvironment(0)
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const { return "filters.attribute"; }

    Options getDefaultOptions();

private:
    virtual void initialize();
    virtual void processOptions(const Options&);
    virtual void prepared(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual void filter(PointView& view);
    virtual void done(PointTableRef table);

    AttributeFilter& operator=(const AttributeFilter&); // not implemented
    AttributeFilter(const AttributeFilter&); // not implemented

    typedef std::shared_ptr<void> OGRDSPtr;

    OGRDSPtr m_ds;
    OGRLayerH m_lyr;
    std::string m_dimName;
    double m_value;
    std::string m_datasource;
    std::string m_column;
    std::string m_query;
    std::string m_layer;
    Dimension::Id::Enum m_dim;

    GEOSContextHandle_t m_geosEnvironment;
    std::unique_ptr<pdal::gdal::ErrorHandler> m_gdal_debug;
    void UpdateGEOSBuffer(PointView& view);

};

} // namespace pdal
