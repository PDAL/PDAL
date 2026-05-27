/******************************************************************************
* Copyright (c) 2020, Hobu Inc.
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

#include "KDIndex.hpp"
#include "private/KDImpl.hpp"
#include "private/PointGrid.hpp"

namespace pdal
{

//
// KD2Index
//

KD2Index::KD2Index(const PointView& buf) 
    : m_buf(buf), m_impl(new PointGrid(m_buf, {Dimension::Id::X, Dimension::Id::Y}))
{}

KD2Index::~KD2Index()
{}

void KD2Index::build()
{
    m_impl->build();
}

PointId KD2Index::neighbor(double x, double y) const
{
    PointIdList ids = neighbors(x, y, 1);
    return (ids.size() ? ids[0] : 0);
}


PointId KD2Index::neighbor(PointId idx) const
{
    PointIdList ids = neighbors(idx, 1);
    return (ids.size() ? ids[0] : 0);
}

PointId KD2Index::neighbor(PointRef &point) const
{
    PointIdList ids = neighbors(point, 1);
    return (ids.size() ? ids[0] : 0);
}

PointIdList KD2Index::neighbors(double x, double y, point_count_t k) const
{
    PointGrid::DistanceResults results = m_impl->knnSearch(x, y, k);

    PointIdList ids(results.size());
    for (std::size_t i = 0; i < results.size(); ++i)
        ids[i] = results[i].index;
    return ids;
}

PointIdList KD2Index::neighbors(PointId idx, point_count_t k) const
{
    double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);

    return neighbors(x, y, k);
}

PointIdList KD2Index::neighbors(PointRef &point, point_count_t k) const
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);

    return neighbors(x, y, k);
}

void KD2Index::knnSearch(double x, double y, point_count_t k,
    PointIdList *i, std::vector<double> *sq_d) const
{
    PointGrid::DistanceResults results = m_impl->knnSearch(x, y, k);

    // For some reason we pass args as pointers, so grab some references to
    // make things cleaner below.
    PointIdList& indices = *i;
    std::vector<double>& sqr_dists = *sq_d;

    // Initialize for the case where we have fewer results than expected.
    if (k != results.size())
    {
        std::fill(indices.begin(), indices.begin() + k, 0);
        std::fill(sqr_dists.begin(), sqr_dists.begin() + k,
            (std::numeric_limits<double>::max)());
        k = results.size();
    }

    for (size_t i = 0; i < k; ++i)
    {
        indices[i] = results[i].index;
        sqr_dists[i] = results[i].sqr_dist;
    }
}

void KD2Index::knnSearch(PointId idx, point_count_t k, PointIdList *indices,
    std::vector<double> *sqr_dists) const
{
    double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
    knnSearch(x, y, k, indices, sqr_dists);
}

void KD2Index::knnSearch(PointRef& point, point_count_t k, PointIdList *indices,
    std::vector<double> *sqr_dists) const
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    knnSearch(x, y, k, indices, sqr_dists);
}

PointIdList KD2Index::radius(const double& x, const double& y, const double& r) const
{
    return m_impl->radius(x, y, r);
}

PointIdList KD2Index::radius(PointId idx, const double& r) const
{
    double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);

    return radius(x, y, r);
}

PointIdList KD2Index::radius(PointRef &point, const double& r) const
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);

    return radius(x, y, r);
}

PointIdList KD2Index::boxNeighborhood(const BOX2D& extent) const
{
    return m_impl->boxEncloses(extent);
}

//
// KD3Index
//

KD3Index::KD3Index(const PointView& buf) 
    : m_buf(buf), m_impl(new PointGrid(m_buf, {Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z}))
{}

KD3Index::~KD3Index()
{}

void KD3Index::build()
{
    m_impl->build();
}

PointId KD3Index::neighbor(double x, double y, double z) const
{
    PointIdList ids = neighbors(x, y, z, 1);
    return (ids.size() ? ids[0] : 0);
}

PointId KD3Index::neighbor(PointId idx) const
{
    PointIdList ids = neighbors(idx, 1);
    return (ids.size() ? ids[0] : 0);
}

PointId KD3Index::neighbor(PointRef &point) const
{
    PointIdList ids = neighbors(point, 1);
    return (ids.size() ? ids[0] : 0);
}

PointIdList KD3Index::neighbors(double x, double y, double z,
    point_count_t k, size_t stride) const
{
    return m_impl->neighbors(x, y, z, k, stride);
}

PointIdList KD3Index::neighbors(PointId idx, point_count_t k,
    size_t stride) const
{
    double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
    double z = m_buf.getFieldAs<double>(Dimension::Id::Z, idx);

    return neighbors(x, y, z, k, stride);
}

PointIdList KD3Index::neighbors(PointRef &point, point_count_t k,
    size_t stride) const
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);

    return neighbors(x, y, z, k, stride);
}

void KD3Index::knnSearch(double x, double y, double z, point_count_t k,
    PointIdList *i, std::vector<double> *sq_d) const
{
    PointGrid::DistanceResults results = m_impl->knnSearch(x, y, z, k);

    // For some reason we pass args as pointers, so grab some references to
    // make things cleaner below.
    PointIdList& indices = *i;
    std::vector<double>& sqr_dists = *sq_d;

    // Initialize for the case where we have fewer results than expected.
    if (k != results.size())
    {
        std::fill(indices.begin(), indices.begin() + k, 0);
        std::fill(sqr_dists.begin(), sqr_dists.begin() + k,
            (std::numeric_limits<double>::max)());
        k = results.size();
    }

    for (size_t i = 0; i < k; ++i)
    {
        indices[i] = results[i].index;
        sqr_dists[i] = results[i].sqr_dist;
    }
}

void KD3Index::knnSearch(PointId idx, point_count_t k, PointIdList *indices,
    std::vector<double> *sqr_dists) const
{
    double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
    double z = m_buf.getFieldAs<double>(Dimension::Id::Z, idx);

    knnSearch(x, y, z, k, indices, sqr_dists);
}

void KD3Index::knnSearch(PointRef &point, point_count_t k,
    PointIdList *indices, std::vector<double> *sqr_dists) const
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);

    knnSearch(x, y, z, k, indices, sqr_dists);
}

PointIdList KD3Index::radius(double x, double y, double z, double r) const
{
    return m_impl->radius(x, y, z, r);
}

PointIdList KD3Index::radius(PointId idx, double r) const
{
    double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
    double z = m_buf.getFieldAs<double>(Dimension::Id::Z, idx);

    return radius(x, y, z, r);
}

PointIdList KD3Index::radius(PointRef &point, double r) const
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);

    return radius(x, y, z, r);
}

//
// KDFlexIndex
//

KDFlexIndex::KDFlexIndex(const PointView& buf, const Dimension::IdList& dims) :
    m_buf(buf), m_dims(dims)
{
    BOX2D bounds;
    for (PointId idx = 0; idx < m_buf.size(); idx++)
    {
        double q = m_buf.getFieldAs<double>(m_dims[0], idx);
        double r = m_buf.getFieldAs<double>(m_dims[1], idx);
        bounds.grow(q, r);
    }
    m_impl.reset(new PointGrid(bounds, buf, dims));
}

KDFlexIndex::~KDFlexIndex()
{}

void KDFlexIndex::build()
{
    m_impl->build();
}

PointId KDFlexIndex::neighbor(PointRef &point) const
{
    PointIdList ids = neighbors(point, 1);
    return (ids.size() ? ids[0] : 0);
}

PointIdList KDFlexIndex::neighbors(PointRef &point, point_count_t k, size_t stride) const
{
    return m_impl->neighbors(point, k, stride);
}

PointIdList KDFlexIndex::radius(PointRef &point, double r) const
{
    PointGrid::DistanceResults results = m_impl->radiusSearch(point, r);
    PointIdList neighbors(results.size());
    for (size_t i = 0; i < results.size(); ++i)
        neighbors.push_back(results[i].index);
    return neighbors;
}

} // namespace pdal
