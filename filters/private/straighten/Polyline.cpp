/******************************************************************************
 * Copyright (c) 2023, Guilhem Villemin (guilhem.villemin@altametris.com)
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

// Avoid conflicting declaration of min/max macros in windows headers
#if !defined(NOMINMAX) &&                                                      \
    (defined(_WIN32) || defined(_WIN32_) || defined(WIN32) || defined(_WIN64))
#define NOMINMAX
#ifdef max
#undef max
#undef min
#endif
#endif

#include <limits>
#include <numeric>
#pragma warning(push)
#pragma warning(disable : 4251)
#include <ogr_api.h>
#include <ogr_geometry.h>
#pragma warning(pop)

#include "Polyline.hpp"
#include "Utils.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>

namespace pdal
{
namespace straighten
{

Polyline::Polyline()
{
    init();
}

Polyline::~Polyline() {}

Polyline::Polyline(OGRGeometryH g) : Geometry(g)
{
    init();
}

Polyline::Polyline(OGRGeometryH g, const SpatialReference& srs)
    : Geometry(g, srs)
{
    init();
}

void Polyline::init()
{

    // If the handle was null, we need to create an empty Polyline.
    if (!m_geom)
    {
        OGRGeometry* newGeom;
        OGRGeometryFactory::createFromWkt("LINESTRING ZM EMPTY", nullptr,
                                          &newGeom);
        m_geom.reset(newGeom);
        return;
    }

    OGRwkbGeometryType t = m_geom->getGeometryType();

    if (t != wkbLineStringZM)
    {
        throw pdal::pdal_error("pdal::Polyline() cannot construct geometry "
                               "because OGR geometry is not LineStringZM.");
    }

    m_table.reset(new PointTable);
    m_table->layout()->registerDim(Dimension::Id::X);
    m_table->layout()->registerDim(Dimension::Id::Y);
    m_table->layout()->registerDim(Dimension::Id::Z);
    m_table->layout()->registerDim(Dimension::Id::W);
    m_table->layout()->registerDim(Dimension::Id::Roll);
    m_table->layout()->registerDim(Dimension::Id::Azimuth);
    m_view.reset(new PointView(*m_table));
    OGRLineString* p = static_cast<OGRLineString*>(m_geom.get());
    int size = p->getNumPoints();
    double cumDist = 0.0;
    for (int i = 0; i < size; ++i)
    {
        m_view->setField(Dimension::Id::X, i, p->getX(i));
        m_view->setField(Dimension::Id::Y, i, p->getY(i));
        m_view->setField(Dimension::Id::Z, i, p->getZ(i));
        m_view->setField(Dimension::Id::Roll, i, p->getM(i));

        if (i != 0)
        {
            const double dx = p->getX(i) - p->getX(i - 1);
            const double dy = p->getY(i) - p->getY(i - 1);
            cumDist += std::sqrt(dx * dx + dy * dy);
        }
        m_view->setField(Dimension::Id::W, i, cumDist);

        if (i + 1 != size)
            m_view->setField(Dimension::Id::Azimuth, i,
                             Utils::azimuth(p->getX(i), p->getY(i),
                                            p->getX(i + 1), p->getY(i + 1)));
        else
            m_view->setField(Dimension::Id::Azimuth, i,
                             Utils::azimuth(p->getX(i - 1), p->getY(i - 1),
                                            p->getX(i), p->getY(i)));
    }

    m_index.reset(new KD2Index(*m_view));
    m_index->build();
}

Polyline::Polyline(const std::string& wkt_or_json, SpatialReference ref)
    : Geometry(wkt_or_json, ref)
{
    std::cout << "init from string" << std::endl;
    init();
}

Polyline::Polyline(const Polyline& poly) : Geometry(poly)
{
    init();
}

Polyline& Polyline::operator=(const Polyline& src)
{
    ((Geometry*)this)->operator=((const Geometry&)src);
    return *this;
}

void Polyline::modified()
{
    init();
}

void Polyline::clear()
{
    // Geometry::clear();
    m_geom.reset(new OGRLineString());
    modified();
}

void Polyline::interpolate(const PointRef& point, double& x, double& y,
                           double& z, double& m, double& azimuth,
                           double& offset)
{
    OGRLineString* p = static_cast<OGRLineString*>(m_geom.get());
    const double pk = point.getFieldAs<double>(Dimension::Id::X);
    int size = m_view->size();
    if (size == 0 || size == 1)
        throw pdal::pdal_error("pdal::Polyline() has not enough points (" +
                               std::to_string(size) + ").");

    // well get the first point where W is higher than pk
    PointViewIter currentPointIter =
        std::find_if(m_view->begin(), m_view->end(),
                     [&](const PointRef pt)
                     { return pk < pt.getFieldAs<double>(Dimension::Id::W); });
    // If even last point is not a candidate, then we just get last point
    if (currentPointIter == m_view->end())
        --currentPointIter;

    // if it is first one (pk is negative) we taking second point
    if (currentPointIter == m_view->begin())
        ++currentPointIter;

    const PointRef previousPoint = *(currentPointIter - 1);
    const PointRef currentPoint = *currentPointIter;
    const double prevX = previousPoint.getFieldAs<double>(Dimension::Id::X);
    const double prevY = previousPoint.getFieldAs<double>(Dimension::Id::Y);
    const double currentX = currentPoint.getFieldAs<double>(Dimension::Id::X);
    const double currentY = currentPoint.getFieldAs<double>(Dimension::Id::Y);
    const double tx = currentX - prevX;
    const double ty = currentY - prevY;
    const double segmentLength = std::sqrt(tx * tx + ty * ty);

    offset = previousPoint.getFieldAs<double>(Dimension::Id::W);

    const double ratio = (pk - offset) / segmentLength;

    const double prevZ = previousPoint.getFieldAs<double>(Dimension::Id::Z);
    const double currentZ = currentPoint.getFieldAs<double>(Dimension::Id::Z);
    const double prevM = previousPoint.getFieldAs<double>(Dimension::Id::Roll);
    const double currentM =
        currentPoint.getFieldAs<double>(Dimension::Id::Roll);
    const double prevAzimuth =
        previousPoint.getFieldAs<double>(Dimension::Id::Azimuth);
    const double currentAzimuth =
        currentPoint.getFieldAs<double>(Dimension::Id::Azimuth);

    x = prevX;
    y = prevY;
    m = prevM;
    z = prevZ;
    azimuth = prevAzimuth;
    if (ratio > 1)
    {
        x = currentX;
        y = currentY;
        m = currentM;
        z = currentZ;
        azimuth = currentAzimuth;
    }
    else if (ratio > 0)
    {
        x = ratio * currentX + (1 - ratio) * prevX;
        y = ratio * currentY + (1 - ratio) * prevY;
        z = ratio * currentZ + (1 - ratio) * prevZ;
        azimuth = Utils::angularRatio(prevAzimuth, currentAzimuth, ratio);
        m = Utils::angularRatio(prevM, currentM, ratio);
    }
}

double Polyline::closestSegment(const PointRef& point, double& x, double& y,
                                double& z, double& m, double& azimuth,
                                double& offset)
{
    double sqrDist = std::numeric_limits<double>::max();
    double testDist = 0;
    double segmentPtX, segmentPtY;

    const double candX = point.getFieldAs<double>(Dimension::Id::X);
    const double candY = point.getFieldAs<double>(Dimension::Id::Y);

    // getting the 3 closest points to build the 2 closest segments
    PointIdList list = m_index->neighbors(candX, candY, 3);
    // sorting the candidates by id, to get the right segments
    std::sort(list.begin(), list.end());

    // if a segment is very long,
    // closest candidates might all be on the same side
    // that's why we add one point before
    if (list.front() != 0)
        list.insert(list.begin(), list.front() - 1);
    // and one after
    if (list.back() != m_view->size() - 1)
        list.insert(list.end(), list.back() + 1);
    // we want to make sure we have all the points
    // if we get indices like 6 8 10, we want
    // vertices 6, 7, 8, 9, 10 in our list
    if (list.size() != list.back() - list.front() + 1)
    {
        PointIdList final(list.back() - list.front() + 1);
        std::iota(final.begin(), final.end(), list.front());
        list = final;
    }

    int size = list.size();
    if (size == 0 || size == 1)
        return -1;

    for (int i = 1; i < size; ++i)
    {
        const double prevX =
            m_view->getFieldAs<double>(Dimension::Id::X, list[i - 1]);
        const double prevY =
            m_view->getFieldAs<double>(Dimension::Id::Y, list[i - 1]);
        const double currentX =
            m_view->getFieldAs<double>(Dimension::Id::X, list[i]);
        const double currentY =
            m_view->getFieldAs<double>(Dimension::Id::Y, list[i]);
        testDist = Utils::sqrDistToLine(candX, candY, prevX, prevY, currentX,
                                        currentY, segmentPtX, segmentPtY);
        if (testDist < sqrDist)
        {

            offset = 0.0;
            // list[i-1] if the first point of the current segment in the
            // polyline list[i] = list[i-1]+1 this is where we stop accumulating
            // distances
            for (PointId j = 1; j < list[i]; ++j)
            {
                const double dx =
                    m_view->getFieldAs<double>(Dimension::Id::X, j) -
                    m_view->getFieldAs<double>(Dimension::Id::X, j - 1);
                const double dy =
                    m_view->getFieldAs<double>(Dimension::Id::Y, j) -
                    m_view->getFieldAs<double>(Dimension::Id::Y, j - 1);
                offset += std::sqrt(dx * dx + dy * dy);
            }
            sqrDist = testDist;
            x = segmentPtX;
            y = segmentPtY;

            const double dx = x - prevX;
            const double dy = y - prevY;
            const double tx = currentX - prevX;
            const double ty = currentY - prevY;
            const double ratio =
                std::sqrt((dx * dx + dy * dy) / (tx * tx + ty * ty));
            offset += std::sqrt((dx * dx + dy * dy));

            z = m_view->getFieldAs<double>(Dimension::Id::Z, list[i]) * ratio +
                m_view->getFieldAs<double>(Dimension::Id::Z, list[i - 1]) *
                    (1 - ratio);

            // angular mean for m, which is supposed to be in radians
            azimuth = Utils::angularRatio(
                m_view->getFieldAs<double>(Dimension::Id::Azimuth, list[i - 1]),
                m_view->getFieldAs<double>(Dimension::Id::Azimuth, list[i]),
                ratio);
            m = Utils::angularRatio(
                m_view->getFieldAs<double>(Dimension::Id::Roll, list[i - 1]),
                m_view->getFieldAs<double>(Dimension::Id::Roll, list[i]),
                ratio);
        }
    }
    return sqrDist;
}
} // namespace straighten
} // namespace pdal