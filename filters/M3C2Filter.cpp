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
#include <algorithm>
#include <numeric>

#include "private/Comparison.hpp"
#include <pdal/private/MathUtils.hpp>

namespace pdal
{

struct M3C2Filter::Args
{
    double normalRadius;
    double cylRadius;
    double cylHalfLen;
    int samplePct;
    double regError;
};

struct M3C2Filter::Private
{
    PointViewPtr v1;
    PointViewPtr v2;
    PointViewPtr cores;
    double cylRadius2;
    double cylBallRadius;
    double minPoints;
    Dimension::Id distanceDim;
    Dimension::Id uncertaintyDim;
    Dimension::Id significantDim;
    Dimension::Id stdDev1Dim;
    Dimension::Id stdDev2Dim;
    Dimension::Id n1Dim;
    Dimension::Id n2Dim;
};

static StaticPluginInfo const s_info
{
    "filters.m3c2",
    "Compute the 3D distance between two sets of points based on the M3C2 algorithm",
    "http://pdal.io/stages/filters.m3c2.html"
};

CREATE_STATIC_STAGE(M3C2Filter, s_info)

std::string M3C2Filter::getName() const
{
    return s_info.name;
}


M3C2Filter::M3C2Filter() : m_args(new M3C2Filter::Args), m_p(new M3C2Filter::Private)
{}

M3C2Filter::~M3C2Filter() {}


void M3C2Filter::addArgs(ProgramArgs& args)
{
    args.add("normal_radius", "The radius to use for finding neighbors in the "
        "calculation of normals [Default: 2].", m_args->normalRadius, 2.0);
    args.add("cyl_radius", "The radius of the cylinder of neighbors used for calculating change "
        "[Default: 2].", m_args->cylRadius, 2.0);
    args.add("cyl_halflen", "The half-length of the cylinder of neighbors used used for "
        "calculating change [Default: 5].", m_args->cylHalfLen, 5.0);
    args.add("sample_pct", "Sampling percentage for first point view. Ignored if a core view "
        "is provided.", m_args->samplePct, 10);
    args.add("reg_error", "Registration error [Default: 0].", m_args->regError, 0.0);
}


void M3C2Filter::addDimensions(PointLayoutPtr layout)
{
    m_p->distanceDim = layout->assignDim("m3c2_distance", Dimension::Type::Double);
    m_p->uncertaintyDim = layout->assignDim("m3c2_uncertainty", Dimension::Type::Double);
    m_p->significantDim = layout->assignDim("m3c2_significant", Dimension::Type::Unsigned8);
    m_p->stdDev1Dim = layout->assignDim("m3c2_std_dev1", Dimension::Type::Double);
    m_p->stdDev2Dim = layout->assignDim("m3c2_std_dev2", Dimension::Type::Double);
    m_p->n1Dim = layout->assignDim("m3c2_count1", Dimension::Type::Unsigned16);
    m_p->n2Dim = layout->assignDim("m3c2_count2", Dimension::Type::Unsigned16);
}


void M3C2Filter::initialize()
{
    m_p->cylRadius2 = std::pow(m_args->cylRadius, 2.0);

    // Compute the radius of a ball that fits around the cylinder.
    m_p->cylBallRadius = std::sqrt(m_p->cylRadius2 + m_args->cylHalfLen * m_args->cylHalfLen);
}

PointViewSet M3C2Filter::run(PointViewPtr view)
{
    if (!m_p->v1)
        m_p->v1 = view;
    else if (!m_p->v2)
        m_p->v2 = view;
    else if (!m_p->cores)
        m_p->cores = view;
    else
        throwError("Too many views provided. Only two or three views are supported.");

    PointViewSet set;
    set.insert(view);
    return set;
}

void M3C2Filter::done(PointTableRef _)
{
    if (!m_p->v1)
        throwError("Missing first view.");
    if (!m_p->v2)
        throwError("Missing second view.");
    if (!m_p->cores)
    {
        m_p->cores = m_p->v1->makeNew();
        createSample(*m_p->v1, *m_p->cores);
    }

    BOX3D v1Bounds;
    m_p->v1->calculateBounds(v1Bounds);

    PointGrid g1(v1Bounds.to2d(), *m_p->v1);
    for (PointRef pt : *m_p->v1)
        g1.add(pt.getFieldAs<double>(Dimension::Id::X),
            pt.getFieldAs<double>(Dimension::Id::Y),
            pt.getFieldAs<double>(Dimension::Id::Z));

    BOX3D v2Bounds;
    m_p->v2->calculateBounds(v2Bounds);

    PointGrid g2(v2Bounds.to2d(), *m_p->v2);
    for (PointRef pt : *m_p->v2)
        g2.add(pt.getFieldAs<double>(Dimension::Id::X),
            pt.getFieldAs<double>(Dimension::Id::Y),
            pt.getFieldAs<double>(Dimension::Id::Z));

    for (PointRef ref : *m_p->cores)
    {
        PointIdList pts1;
        PointIdList pts2;

        Eigen::Vector3d core(ref.getFieldAs<double>(Dimension::Id::X),
            ref.getFieldAs<double>(Dimension::Id::Y),
            ref.getFieldAs<double>(Dimension::Id::Z));

        Eigen::Vector3d normal = findNormal(core, g1);

        // Find search box around cylinder
        Eigen::Vector3d radius(m_args->cylRadius, m_args->cylRadius, m_args->cylRadius);
        Eigen::Vector3d end1 = core + (normal * m_args->cylHalfLen);
        Eigen::Vector3d end2 = core - (normal * m_args->cylHalfLen);
        Eigen::Vector3d c1 = end1 + radius;
        Eigen::Vector3d c2 = end1 - radius;
        Eigen::Vector3d c3 = end2 + radius;
        Eigen::Vector3d c4 = end2 - radius;
        BOX2D box;
        box.grow(c1(0), c1(1));
        box.grow(c2(0), c2(1));
        box.grow(c3(0), c3(1));
        box.grow(c4(0), c4(1));

        PointIdList pts = g1.findNeighbors(box);
        std::vector<double> dists1 = filterPoints(core, normal, g1.view(), pts);

        pts = g2.findNeighbors(box);
        std::vector<double> dists2 = filterPoints(core, normal, g2.view(), pts);

        Stats stats;
        if (calcStats(dists1, dists2, stats))
        {
            ref.setField(m_p->distanceDim, stats.distance);
            ref.setField(m_p->uncertaintyDim, stats.uncertainty);
            ref.setField(m_p->significantDim, stats.significant);
            ref.setField(m_p->stdDev1Dim, stats.stdDev1);
            ref.setField(m_p->stdDev2Dim, stats.stdDev2);
            ref.setField(m_p->n1Dim, stats.n1);
            ref.setField(m_p->n2Dim, stats.n2);
        }
    }
}

Eigen::Vector3d M3C2Filter::findNormal(Eigen::Vector3d pos, const PointGrid& grid)
{
    PointIdList neighbors = grid.findNeighbors3d(pos, m_args->normalRadius);
    math::NormalResult res = math::findNormal(grid.view(), neighbors);
    return res.normal;
}

// Super simple sampling.  We just take random points up to the percentage requested.
void M3C2Filter::createSample(PointView& source, PointView& dest)
{
    PointIdList ids(source.size());
    std::iota(ids.begin(), ids.end(), 0);
    std::random_device gen;
    std::shuffle(ids.begin(), ids.end(), std::mt19937(gen()));

    // Get the sample from the front of the shuffled list.
    ids.resize(static_cast<point_count_t>(source.size() * (m_args->samplePct / 100.0)));
    for (PointId id : ids)
        dest.appendPoint(source, id);
}

bool M3C2Filter::calcStats(const std::vector<double>& pts1, const std::vector<double>& pts2,
    Stats& stats)
{
    if (pts1.size() < m_p->minPoints || pts2.size() < m_p->minPoints)
        return false;

    // Set square distances to distances
    double sum = 0;
    double sum2 = 0;
    for (double val : pts1)
    {
        sum += val;
        sum2 += val * val;
    }
    double mean1 = sum / pts1.size();
    // This is a bad variance calcuation from a computational standpoint but it's simple.
    double var1 = sum2 / pts1.size() - mean1 * mean1;

    sum = 0;
    sum2 = 0;
    for (double val : pts2)
    {
        sum += val;
        sum2 += val * val;
    }
    double mean2 = sum / pts2.size();
    double var2 = sum2 / pts2.size() - mean2 * mean2;

    double lodVar = var1 / pts1.size() + var2 / pts2.size();
    double lod = 1.96 * (std::sqrt(lodVar) + m_args->regError);

    stats.distance = mean2 - mean1;
    stats.uncertainty = lod;
    stats.significant = std::abs(stats.distance) > lod;
    stats.stdDev1 = std::sqrt(var1);
    stats.stdDev2 = std::sqrt(var2);
    stats.n1 = pts1.size();
    stats.n2 = pts2.size();

    return true;
}

std::vector<double> M3C2Filter::filterPoints(Eigen::Vector3d cylCenter, Eigen::Vector3d cylNormal,
    const PointView& view, const PointIdList& neighbors)
{
    std::vector<double> dists;

    if (!neighbors.size())
        return dists;

    dists.reserve(neighbors.size());
    size_t start = 0;
    for (PointId id : neighbors)
    {
        Eigen::Vector3d point(view.getFieldAs<double>(Dimension::Id::X, id),
            view.getFieldAs<double>(Dimension::Id::Y, id),
            view.getFieldAs<double>(Dimension::Id::Z, id));

        double dist = pointPasses(point, cylCenter, cylNormal);
        if (!std::isnan(dist))
            dists.push_back(dist);
    }
    return dists;
}

double M3C2Filter::pointPasses(Eigen::Vector3d point, Eigen::Vector3d cylCenter,
    Eigen::Vector3d cylNormal)
{
    // Calculate the distance from the point to the line that goes through the cylinder normal
    // vector. Make sure it's less than the cylinder radius.
    Eigen::ParametrizedLine<double, 3> line(cylCenter, cylNormal);
    if (line.squaredDistance(point) > m_p->cylRadius2)
        return std::numeric_limits<double>::quiet_NaN();

    // Calculate the distance from the point to the plane that's parallel to the cylinder's end
    // and goes through the cylinder "center" (our core point). Make sure that's less than
    // the half length of the cylinder.
    Eigen::Hyperplane<double, 3> plane(cylNormal, cylCenter);
    double dist = plane.signedDistance(point);
    if (std::abs(dist) > m_args->cylHalfLen)
        return std::numeric_limits<double>::quiet_NaN();

    return dist;
}

} // namespace pdal
