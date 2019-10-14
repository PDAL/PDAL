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

#include <pdal/Log.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/SpatialReference.hpp>

#include <memory>

class OGRGeometry;
using OGRGeometryH = void *;
using OGRSpatialReferenceH = void *;

namespace pdal
{

class BOX3D;

class PDAL_DLL Geometry
{
protected:
    Geometry();
    Geometry(const Geometry&);
    Geometry(Geometry&&);
    Geometry(const std::string& wkt_or_json,
           SpatialReference ref = SpatialReference());
    Geometry(OGRGeometryH g);
    Geometry(OGRGeometryH g, const SpatialReference& srs);

public:
    Geometry& operator=(const Geometry&);
    virtual ~Geometry();

    OGRGeometryH getOGRHandle();

    virtual void update(const std::string& wkt_or_json);
    virtual bool valid() const;
    bool srsValid() const;
    void setSpatialReference(const SpatialReference& ref);
    SpatialReference getSpatialReference() const;
    void transform(const SpatialReference& ref) const;

    std::string wkt(double precision=15, bool bOutputZ=false) const;
    std::string json(double precision=15) const;

    BOX3D bounds() const;

    operator bool () const
        { return m_geom != NULL; }
    static void throwNoGeos();

protected:
    std::unique_ptr<OGRGeometry> m_geom;

    friend PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
        const Geometry& p);
    friend PDAL_DLL std::istream& operator>>(std::istream& istr,
        Geometry& p);
};

} // namespace pdal

