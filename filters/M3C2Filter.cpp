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

#include <Eigen/Geometry>

namespace pdal
{

struct M3C2Filter::Args
{
    double normRadius;
    double cylRadius;
    double cylHalfLen;
};

struct M3C2Filter::Private
{
    double cylRadius2;
    double cylBallRadius;
    double minPoints;
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


M3C2Filter::M3C2Filter() : m_args(new M3C2Filter::Args), m_p(new M3C2Filter::Private)
{}


void M3C2Filter::addArgs(ProgramArgs& args)
{
    args.add("normal_radius", "The radius to use for finding neighbors in the "
        "calculation of normals [Default: 2].", m_args->normRadius, 2.0);
    args.add("cyl_radius", "The radius of the cylinder of neighbors used for calculating change "
        "[Default: 2].", m_args->cylRadius, 2.0);
    args.add("cyl_halflen", "The half-length of the cylinder of neighbors used used for "
        "calculating change [Default: 5].", m_args->cylHalfLen, 5.0);
}


void M3C2Filter::addDimensions(PointLayoutPtr layout)
{
}


void M3C2Filter::filter(PointView& view)
{
    // Compute the radius of a ball that fits around the cylinder.
    m_p->cylBallRadius = std::sqrt(m_p->cylRadius2 + m_args->cylHalfLen * m_args->cylHalfLen);
}

void M3C2Filter::calcStats(PointView& v1, PointView& v2, PointView& cores)
{
    for (PointRef core : cores)
    {
        Eigen::Vector3d pos(core.getFieldAs<double>(Dimension::Id::X),
            core.getFieldAs<double>(Dimension::Id::Y),
            core.getFieldAs<double>(Dimension::Id::Z));
        Eigen::Vector3d normal(core.getFieldAs<double>(Dimension::Id::NormalX),
            core.getFieldAs<double>(Dimension::Id::NormalY),
            core.getFieldAs<double>(Dimension::Id::NormalZ));
        calcStats(pos, normal, v1, v2);
    }
}

void M3C2Filter::calcStats(Eigen::Vector3d cylCenter, Eigen::Vector3d cylNormal,
    PointView& v1, PointView& v2)
{
    KD3Index::RadiusResults pts1;
    v1.build3dIndex().radius(cylCenter(0), cylCenter(1), cylCenter(2),
        m_p->cylBallRadius, pts1);
    pts1 = filterPoints(cylCenter, cylNormal, pts1, v1);

    if (pts1.size() < m_p->minPoints)
        return;

    KD3Index::RadiusResults pts2;
    v2.build3dIndex().radius(cylCenter(0), cylCenter(1), cylCenter(2),
        m_p->cylBallRadius, pts2);
    pts2 = filterPoints(cylCenter, cylNormal, pts2, v2);

    if (pts2.size() < m_p->minPoints)
        return;
}

KD3Index::RadiusResults M3C2Filter::filterPoints(Eigen::Vector3d cylCenter,
    Eigen::Vector3d cylNormal, const KD3Index::RadiusResults& pts, const PointView& view)
{
    KD3Index::RadiusResults validated;
    for (KD3Index::RadiusResult res : pts)
    {
        PointId id = res.first;

        Eigen::Vector3d point(view.getFieldAs<double>(Dimension::Id::X, id),
            view.getFieldAs<double>(Dimension::Id::Y, id),
            view.getFieldAs<double>(Dimension::Id::Z, id));
        if (pointPasses(point, cylCenter, cylNormal))
            validated.push_back(res);
    }
    return validated;
}

bool M3C2Filter::pointPasses(Eigen::Vector3d point, Eigen::Vector3d cylCenter,
    Eigen::Vector3d cylNormal)
{
    // Calculate the distance from the point to the line that goes through the cylinder normal
    // vector. Make sure it's less than the cylinder radius.
    Eigen::ParametrizedLine<double, 3> line(cylCenter, cylNormal);
    if (line.squaredDistance(point) > m_p->cylRadius2)
        return false;

    // Calculate the distance from the point to the plane that's parallel to the cylinder's end
    // and goes through the cylinder "center" (our core point). Make sure that's less than
    // the half length of the cylinder.
    Eigen::Hyperplane<double, 3> plane(cylNormal, cylCenter);
    if (plane.absDistance(point) > m_args->cylHalfLen)
        return false;

    return true;
}

} // namespace pdal
