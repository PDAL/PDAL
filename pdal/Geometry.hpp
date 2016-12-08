/******************************************************************************
* Copyright (c) 2016, Howard Butler (howard@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <pdal/GDALUtils.hpp>
#include <pdal/GEOSUtils.hpp>
#include <pdal/Log.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>

#include <geos_c.h>

#include <memory>

namespace pdal
{

namespace geos {

class ErrorHandler;


struct GeometryDeleter
{
    explicit GeometryDeleter(pdal::geos::ErrorHandler& ctx)
    : m_ctx(ctx)
    {}

    GeometryDeleter() : m_ctx(geos::ErrorHandler::get()) {};

    GeometryDeleter& operator=(const GeometryDeleter& other)
    {
       if (&other!= this)
           m_ctx = other.m_ctx;
       return *this;
    }

    GeometryDeleter(const GeometryDeleter& other) : m_ctx(other.m_ctx) {};

    void operator()(GEOSGeometry* geometry) const
    {
        GEOSGeom_destroy_r(m_ctx.ctx(), geometry);
    }

    pdal::geos::ErrorHandler& m_ctx;
};

} // namespace geos


typedef std::unique_ptr<GEOSGeometry, geos::GeometryDeleter> GEOSGeomPtr;

class PDAL_DLL Geometry
{
public:
    Geometry();
    virtual ~Geometry();
    Geometry(const std::string& wkt_or_json,
           SpatialReference ref = SpatialReference());

    Geometry(GEOSGeometry* g, const SpatialReference& srs);
    Geometry(OGRGeometryH g, const SpatialReference& srs);

    Geometry(const Geometry&);
    Geometry& operator=(const Geometry&);

    OGRGeometryH getOGRHandle();


    virtual void update(const std::string& wkt_or_json,
        SpatialReference ref = SpatialReference());

    void setSpatialReference( const SpatialReference& ref)
        { m_srs = ref; }

    const SpatialReference& getSpatialReference() const
        { return m_srs; }

    Geometry transform(const SpatialReference& ref) const;

    bool equals(const Geometry& other, double tolerance=0.0001) const;
    bool operator==(const Geometry& other) const;
    bool operator!=(const Geometry& other) const;
    bool operator<(const Geometry& other) const
        { return wkt() < other.wkt(); }


    virtual  bool valid() const;
    virtual std::string validReason() const;

    std::string wkt(double precision=8, bool bOutputZ=false) const;
    std::string json(double precision=8) const;

    BOX3D bounds() const;

    operator bool () const
        { return m_geom != NULL; }

protected:

    GEOSGeomPtr m_geom;
    const GEOSPreparedGeometry *m_prepGeom;

    SpatialReference m_srs;
    geos::ErrorHandler& m_geoserr;

    void prepare();

    friend PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
        const Geometry& p);
    friend PDAL_DLL std::istream& operator>>(std::istream& istr,
        Geometry& p);
};

} // namespace pdal

