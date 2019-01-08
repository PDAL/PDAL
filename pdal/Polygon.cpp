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

#include <pdal/Polygon.hpp>
#include "cpl_string.h"

#include <ogr_geometry.h>

namespace pdal
{

Polygon::Polygon(OGRGeometryH g, const SpatialReference& srs) : Geometry(g, srs)
{
    // If the handle was null, we need to create an empty polygon.
    if (!m_geom)
    {
        m_geom.reset(new OGRPolygon());
        return;
    }

    OGRwkbGeometryType t = m_geom->getGeometryType();

    if (!(t == wkbPolygon ||
        t == wkbMultiPolygon ||
        t == wkbPolygon25D ||
        t == wkbMultiPolygon25D))
    {
        throw pdal::pdal_error("pdal::Polygon() cannot construct geometry "
            "because OGR geometry is not Polygon or MultiPolygon.");
    }
}

Polygon::Polygon(const BOX2D& box)
{
    OGRPolygon *poly = new OGRPolygon();
    m_geom.reset(poly);
    OGRLinearRing *lr = new OGRLinearRing();
    lr->addPoint(box.minx, box.miny);
    lr->addPoint(box.maxx, box.miny);
    lr->addPoint(box.maxx, box.maxy);
    lr->addPoint(box.minx, box.maxy);
    lr->addPoint(box.minx, box.miny);
    poly->addRingDirectly(lr);
}


Polygon::Polygon(const BOX3D& box)
{
    OGRPolygon *poly = new OGRPolygon();
    m_geom.reset(poly);
    OGRLinearRing *lr = new OGRLinearRing();
    lr->addPoint(box.minx, box.miny, box.minz);
    lr->addPoint(box.minx, box.maxy, box.minz);
    lr->addPoint(box.maxx, box.maxy, box.maxz);
    lr->addPoint(box.maxx, box.miny, box.maxz);
    lr->addPoint(box.minx, box.miny, box.minz);
    poly->addRingDirectly(lr);
}


void Polygon::simplify(double distance_tolerance, double area_tolerance)
{
    auto deleteSmallRings = [area_tolerance](OGRGeometry *geom)
    {
        OGRPolygon *poly = geom->toPolygon();

        std::vector<int> deleteRings;
        for (int i = 0; i < poly->getNumInteriorRings(); ++i)
        {
            OGRLinearRing *lr = poly->getInteriorRing(i);
            if (lr->get_Area() < area_tolerance)
                deleteRings.push_back(i + 1);
        }
        // Note that interior rings are in a list with the exterior ring,
        // which is why the ring numbers are offset by one when used in
        // this context (what a mess).
        for (auto i : deleteRings)
            poly->removeRing(i, true);
    };

    OGRGeometry *g = m_geom->SimplifyPreserveTopology(distance_tolerance);
    m_geom.reset(g);

    OGRwkbGeometryType t = m_geom->getGeometryType();
    if (t == wkbPolygon || t == wkbPolygon25D)
    {
        std::cerr << "Simplify poly!\n";
        deleteSmallRings(m_geom.get());
    }
    else if (t == wkbMultiPolygon || t == wkbMultiPolygon25D)
    {
        std::cerr << "Simplify multi poly!\n";
        OGRMultiPolygon *mpoly = m_geom->toMultiPolygon();
        for (auto it = mpoly->begin(); it != mpoly->end(); ++it)
            deleteSmallRings(*it);
    }
}


double Polygon::area() const
{
    OGRwkbGeometryType t = m_geom->getGeometryType();
    if (t == wkbPolygon || t == wkbPolygon25D)
        return m_geom->toPolygon()->get_Area();
    else if (t == wkbMultiPolygon || t == wkbMultiPolygon25D)
        return m_geom->toMultiPolygon()->get_Area();
    return 0;
}


bool Polygon::covers(const PointRef& ref) const
{
    double x = ref.getFieldAs<double>(Dimension::Id::X);
    double y = ref.getFieldAs<double>(Dimension::Id::Y);
    double z = ref.getFieldAs<double>(Dimension::Id::Z);

    OGRPoint p(x, y, z);
    return m_geom->Contains(&p) || m_geom->Touches(&p);
}


bool Polygon::overlaps(const Polygon& p) const
{
    return m_geom->Overlaps(p.m_geom.get());
}

bool Polygon::contains(const Polygon& p) const
{
    return m_geom->Contains(p.m_geom.get());
}

bool Polygon::touches(const Polygon& p) const
{
    return m_geom->Touches(p.m_geom.get());
}

bool Polygon::within(const Polygon& p) const
{
    return m_geom->Within(p.m_geom.get());
}

bool Polygon::crosses(const Polygon& p) const
{
    return m_geom->Crosses(p.m_geom.get());
}

std::vector<Polygon> Polygon::polygons() const
{
    std::vector<Polygon> polys;

    OGRwkbGeometryType t = m_geom->getGeometryType();

    if (t == wkbPolygon || t == wkbPolygon25D)
        polys.emplace_back(*this);
    else if (t == wkbMultiPolygon || t == wkbMultiPolygon25D)
    {
        OGRMultiPolygon *mPoly = m_geom->toMultiPolygon();
        for (auto it = mPoly->begin(); it != mPoly->end(); ++it)
        {
            Polygon p;
            p.m_geom.reset((*it)->clone());
            polys.push_back(p);
        }
    }
    return polys;
}


Polygon::Ring Polygon::exteriorRing() const
{
    Ring r;

    OGRwkbGeometryType t = m_geom->getGeometryType();
    if (t != wkbPolygon && t != wkbPolygon25D)
        throw pdal_error("Request for exterior ring on non-polygon.");

    OGRLinearRing *er = m_geom->toPolygon()->getExteriorRing();

    // For some reason there's no operator -> on an iterator.
    for (auto it = er->begin(); it != er->end(); ++it)
        r.push_back({(*it).getX(), (*it).getY()});
    return r;
}


std::vector<Polygon::Ring> Polygon::interiorRings() const
{
    std::vector<Ring> rings;

    OGRwkbGeometryType t = m_geom->getGeometryType();
    if (t != wkbPolygon && t != wkbPolygon25D)
        throw pdal_error("Request for exterior ring on non-polygon.");

    OGRPolygon *poly = m_geom->toPolygon();
    for (int i = 0; i < poly->getNumInteriorRings(); ++i)
    {
        OGRLinearRing *er = poly->getInteriorRing(i);

        // No operator -> on an iterator.
        Ring r;
        for (auto it = er->begin(); it != er->end(); ++it)
            r.push_back({(*it).getX(), (*it).getY()});
        rings.push_back(r);
    }
    return rings;
}

} // namespace pdal
