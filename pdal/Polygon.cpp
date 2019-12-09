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

#include <pdal/GDALUtils.hpp>
#include <pdal/Polygon.hpp>

#include "../filters/private/pnp/GridPnp.hpp"

namespace pdal
{

struct Polygon::PrivateData
{
    std::vector<GridPnp> m_grids;
};


Polygon::Polygon()
{
    init();
}


Polygon::~Polygon()
{}


Polygon::Polygon(OGRGeometryH g) : Geometry(g)
{
    init();
}


Polygon::Polygon(OGRGeometryH g, const SpatialReference& srs) : Geometry(g, srs)
{
    init();
}


void Polygon::init()
{
    m_pd.reset(new PrivateData());

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


Polygon::Polygon(const std::string& wkt_or_json, SpatialReference ref) :
    Geometry(wkt_or_json, ref), m_pd(new PrivateData)
{}


Polygon::Polygon(const BOX2D& box) : m_pd(new PrivateData)
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


Polygon::Polygon(const BOX3D& box) : m_pd(new PrivateData)
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


Polygon::Polygon(const Polygon& poly) : Geometry(poly)
{
    init();
}


Polygon& Polygon::operator=(const Polygon& src)
{
    ((Geometry *)this)->operator=((const Geometry&)src);
    m_pd.reset(new PrivateData);
    return *this;
}


void Polygon::modified()
{
    m_pd->m_grids.clear();
}


void Polygon::simplify(double distance_tolerance, double area_tolerance,
    bool preserve_topology)
{
    throwNoGeos();

    auto deleteSmallRings = [area_tolerance](OGRGeometry *geom)
    {
// Missing until GDAL 2.3.
//        OGRPolygon *poly = geom->toPolygon();
        OGRPolygon *poly = static_cast<OGRPolygon *>(geom);

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
// Missing until 2.3
//            poly->removeRing(i, true);
            OGR_G_RemoveGeometry(gdal::toHandle(poly), i, true);
    };

    OGRGeometry *g;
    if (preserve_topology)
        g = m_geom->SimplifyPreserveTopology(distance_tolerance);
    else
        g = m_geom->Simplify(distance_tolerance);

    m_geom.reset(g);

    OGRwkbGeometryType t = m_geom->getGeometryType();
    if (t == wkbPolygon || t == wkbPolygon25D)
        deleteSmallRings(m_geom.get());
    else if (t == wkbMultiPolygon || t == wkbMultiPolygon25D)
    {
// Missing until 2.3
/**
        OGRMultiPolygon *mpoly = m_geom->toMultiPolygon();
        for (auto it = mpoly->begin(); it != mpoly->end(); ++it)
            deleteSmallRings(*it);
**/
        OGRMultiPolygon *mpoly = static_cast<OGRMultiPolygon *>(m_geom.get());
        for (int i = 0; i < mpoly->getNumGeometries(); ++i)
            deleteSmallRings(mpoly->getGeometryRef(i));
    }
    modified();
}


double Polygon::area() const
{
    throwNoGeos();

    OGRwkbGeometryType t = m_geom->getGeometryType();
// Not until GDAL 2.3
/**
    if (t == wkbPolygon || t == wkbPolygon25D)
        return m_geom->toPolygon()->get_Area();
    else if (t == wkbMultiPolygon || t == wkbMultiPolygon25D)
        return m_geom->toMultiPolygon()->get_Area();
**/
    if (t == wkbPolygon || t == wkbPolygon25D)
    {
        OGRPolygon *p = static_cast<OGRPolygon *>(m_geom.get());
        return p->get_Area();
    }
    else if (t == wkbMultiPolygon || t == wkbMultiPolygon25D)
    {
        OGRMultiPolygon *p = static_cast<OGRMultiPolygon *>(m_geom.get());
        return p->get_Area();
    }
    return 0;
}


bool Polygon::covers(const PointRef& ref) const
{
    throwNoGeos();

    double x = ref.getFieldAs<double>(Dimension::Id::X);
    double y = ref.getFieldAs<double>(Dimension::Id::Y);
    double z = ref.getFieldAs<double>(Dimension::Id::Z);

    OGRPoint p(x, y, z);
    return m_geom->Contains(&p) || m_geom->Touches(&p);
}


bool Polygon::overlaps(const Polygon& p) const
{
    throwNoGeos();

    return m_geom->Overlaps(p.m_geom.get());
}

bool Polygon::contains(const Polygon& p) const
{
    throwNoGeos();

    return m_geom->Contains(p.m_geom.get());
}

bool Polygon::disjoint(const Polygon& p) const
{
    throwNoGeos();

    return m_geom->Disjoint(p.m_geom.get());
}

bool Polygon::intersects(const Polygon& p) const
{
    throwNoGeos();

    return m_geom->Intersects(p.m_geom.get());
}

/// Determine whether this polygon contains a point.
/// \param x  Point x coordinate.
/// \param y  Point y coordinate.
/// \return  Whether the polygon contains the point or not.
bool Polygon::contains(double x, double y) const
{
    if (m_pd->m_grids.empty())
        for (const Polygon& p : polygons())
            m_pd->m_grids.emplace_back(p.exteriorRing(), p.interiorRings());
    for (auto& g : m_pd->m_grids)
        if (g.inside(x, y))
            return true;
    return false;
}


bool Polygon::touches(const Polygon& p) const
{
    throwNoGeos();

    return m_geom->Touches(p.m_geom.get());
}

bool Polygon::within(const Polygon& p) const
{
    throwNoGeos();

    return m_geom->Within(p.m_geom.get());
}

bool Polygon::crosses(const Polygon& p) const
{
    throwNoGeos();

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
        // Not until GDAL 2.3
        /**
        OGRMultiPolygon *mPoly = m_geom->toMultiPolygon();
        for (auto it = mPoly->begin(); it != mPoly->end(); ++it)
        {
            Polygon p;
            p.m_geom.reset((*it)->clone());
            polys.push_back(p);
        }
        **/
        OGRMultiPolygon *mPoly = static_cast<OGRMultiPolygon *>(m_geom.get());
        for (int i = 0; i < mPoly->getNumGeometries(); ++i)
        {
            Polygon p;
            p.m_geom.reset(mPoly->getGeometryRef(i)->clone());
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

    // Not until GDAL 2.3
    /**
    OGRLinearRing *er = m_geom->toPolygon()->getExteriorRing();

    // For some reason there's no operator -> on an iterator.
    for (auto it = er->begin(); it != er->end(); ++it)
        r.push_back({(*it).getX(), (*it).getY()});
    **/
    OGRLinearRing *er =
        static_cast<OGRPolygon *>(m_geom.get())->getExteriorRing();
    for (int i = 0; i < er->getNumPoints(); ++i)
        r.push_back({er->getX(i), er->getY(i)});

    return r;
}


std::vector<Polygon::Ring> Polygon::interiorRings() const
{
    std::vector<Ring> rings;

    OGRwkbGeometryType t = m_geom->getGeometryType();
    if (t != wkbPolygon && t != wkbPolygon25D)
        throw pdal_error("Request for exterior ring on non-polygon.");

//    OGRPolygon *poly = m_geom->toPolygon();
     OGRPolygon *poly = static_cast<OGRPolygon *>(m_geom.get());
    for (int i = 0; i < poly->getNumInteriorRings(); ++i)
    {
        OGRLinearRing *er = poly->getInteriorRing(i);

        Ring r;
        for (int j = 0; j < er->getNumPoints(); ++j)
            r.push_back({er->getX(j), er->getY(j)});
        rings.push_back(r);
    }
    return rings;
}

} // namespace pdal
