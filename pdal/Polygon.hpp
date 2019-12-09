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
#include <pdal/Geometry.hpp>

namespace pdal
{

class BOX2D;
class BOX3D;

class PDAL_DLL Polygon : public Geometry
{
    using Point = std::pair<double, double>;
    using Ring = std::vector<Point>;
    struct PrivateData;

public:
    Polygon();
    virtual ~Polygon();

    Polygon(const std::string& wkt_or_json,
        SpatialReference ref = SpatialReference());
    Polygon(const BOX2D&);
    Polygon(const BOX3D&);
    Polygon(OGRGeometryH g);
    Polygon(OGRGeometryH g, const SpatialReference& srs);
    Polygon(const Polygon& poly);
    Polygon& operator=(const Polygon& src);

    OGRGeometryH getOGRHandle();

    virtual void modified() override;
    void simplify(double distance_tolerance, double area_tolerance,
        bool preserve_topology = true);
    double area() const;
    std::vector<Polygon> polygons() const;

    bool covers(const PointRef& ref) const;
    bool equal(const Polygon& p) const;
    bool overlaps(const Polygon& p) const;
    bool contains(double x, double y) const;
    bool contains(const Polygon& p) const;
    bool intersects(const Polygon& p) const;
    bool disjoint(const Polygon& p) const;
    bool touches(const Polygon& p) const;
    bool within(const Polygon& p) const;
    bool crosses(const Polygon& p) const;
    Ring exteriorRing() const;
    std::vector<Ring> interiorRings() const;

private:
    void init();

    std::unique_ptr<PrivateData> m_pd;
};

} // namespace pdal

