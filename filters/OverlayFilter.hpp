/******************************************************************************
* Copyright (c) 2017, Hobu Inc. <hobu.inc@gmail.com>
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
#include <pdal/Polygon.hpp>
#include <pdal/Streamable.hpp>

#include <map>
#include <memory>
#include <string>
#include <type_traits>

// Get GDAL's forward decls if available
// otherwise make our own
#if __has_include(<gdal_fwd.h>)
#include <gdal_fwd.h>
#else
using OGRLayerH = void *;
#endif

namespace pdal
{

namespace gdal
{
    class ErrorHandler;
}

#if __has_include(<gdal_fwd.h>)
typedef std::shared_ptr<std::remove_pointer<OGRDataSourceH>::type> OGRDSPtr;
typedef std::shared_ptr<std::remove_pointer<OGRFeatureH>::type> OGRFeaturePtr;
#else
typedef std::shared_ptr<void> OGRDSPtr;
typedef std::shared_ptr<void> OGRFeaturePtr;
#endif

class Arg;

class PDAL_EXPORT OverlayFilter : public Filter, public Streamable
{
    struct PolyVal
    {
        Polygon geom;
        int32_t val;
    };

public:
    OverlayFilter() : m_ds(0), m_lyr(0)
    {}

    std::string getName() const { return "filters.overlay"; }

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void spatialReferenceChanged(const SpatialReference& srs);
    virtual bool processOne(PointRef& point);
    virtual void initialize();
    virtual void prepared(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual void filter(PointView& view);

    OverlayFilter& operator=(const OverlayFilter&) = delete;
    OverlayFilter(const OverlayFilter&) = delete;

    OGRDSPtr m_ds;
    OGRLayerH m_lyr;
    std::string m_dimName;
    std::string m_datasource;
    std::string m_column;
    std::string m_query;
    std::string m_layer;
    Dimension::Id m_dim;
    std::vector<PolyVal> m_polygons;
    BOX2D m_bounds;
    int m_threads;

};

} // namespace pdal
