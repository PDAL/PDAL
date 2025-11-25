/******************************************************************************
* Copyright (c) 2025, Hobu Inc.
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

#include "M3C2Filter.hpp"

#include <pdal/KDIndex.hpp>

namespace pdal
{

struct M3C2Filter::Args
{
};

static StaticPluginInfo const s_info
{
    "filters.m3c2",
    "Fill me in",
    "http://pdal.io/stages/filters.m3c2.html"
};

CREATE_STATIC_STAGE(M3C2Filter, s_info)

std::string M3C2Filter::getName() const
{
    return s_info.name;
}


M3C2Filter::M3C2Filter()
{}


void M3C2Filter::addArgs(ProgramArgs& args)
{
/**
    args.add("count", "The number of points to fetch to determine the "
        "ground point [Default: 1].", m_count, point_count_t(1));
    args.add("max_distance", "Ground points beyond this distance will not "
        "influence nearest neighbor interpolation of height above ground."
        "[Default: None]", m_maxDistance);
    args.add("allow_extrapolation", "If true and count > 1, allow "
        "extrapolation [Default: true].", m_allowExtrapolation, true);
**/
}


void M3C2Filter::addDimensions(PointLayoutPtr layout)
{
/**
    layout->registerDim(Dimension::Id::HeightAboveGround);
**/
}


void M3C2Filter::filter(PointView& view)
{
    // Compute the radius of a ball that fits around the cylinder.
    m_ballRadius = std::sqrt(m_cylRadius2 + m_cylHalfLen * m_cylHalfLen);
}

void M3C2Filter::calcStats(PointView& v1, PointView& v2, PointView& cores)
{
    for (PointRef core : *cores)
    {
        Vector3d pos(core.getFieldAs<double>(Dimension::Id::X),
            core.getFieldAs<double>(Dimension::Id::Y),
            core.getFieldAs<double>(Dimension::Id::Z));
        Vector3d normal(core.getFieldAs<double>(Dimension::Id::NormalX),
            core.getFieldAs<double>(Dimension::Id::NormalY),
            core.getFieldAs<double>(Dimension::Id::NormalZ));
        normal *= m_cylHalfLength;
        calcStats(pos, normal, v1, v2);
    }
}

void M3C2Filter::calcStats(Eigen3d cylCenter, Eigen3d cylNormal,
    const PointViewPtr& v1, const PointViewPtr& v2)
{
    PointIdList pts1 = v1->build3dIndex().radius(cylCenter(0), cylCenter(1), cylCenter(2),
        m_ballRadius);
    pts1 = filterPoints(cylCenter, cylNormal, pts1, v1);

    if (pts1.size() < MinPoints)
        return;

    PointIdList pts2 = v2->build3dIndex().radius(cylCenter(0), cylCenter(1), cylCenter(2),
        m_ballRadius);
    pts2 = filterPoints(cylCenter, cylNormal, pts2, v2);

    if (pts2.size < MinPoints)
        return;
}

PointIdList M3C2Filter::filterPoints(Vector3d cylCenter, Vector3d cylNormal,
    const PointIdList& ids, const PointViewPtr& view)
{
    PointIdList validated;
    for (PointId id : idx)
    {
        Vector3d point(view->getFieldAs(id, Dimension::Id::X),
            view->getFieldAs(id, Dimension::Id::Y),
            view->getFieldAs(id, Dimension::Id::Z));
        if (pointPasses(point, cylCenter, cylNormal))
            validated.push_back(id);
    }
    return validated;
}

bool M3C2Filter::pointPasses(Eigen::Vector3d point, Vector3d cylCenter, Vector3d cylNormal)
{
    // Calculate the distance from the point to the line that goes through the cylinder normal
    // vector. Make sure it's less than the cylinder radius.
    Eigen::ParameterizedLine line(cylCenter, cylNormal);
    if (line.squaredDistance(point) > m_cylRadius2)
        return false;

    // Calculate the distance from the point to the plane that's parallel to the cylinder's end
    // and goes through the cylinder "center" (our core point). Make sure that's less than
    // the half length of the cylinder.
    Eigen::Hyperplane plane(cylNormal, cylCenter);
    if (plane.absDistance(point) > m_cylHalfLen)
        return false;

    return true;
}

} // namespace pdal
