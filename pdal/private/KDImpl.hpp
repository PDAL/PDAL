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

#pragma once

#include <nanoflann/nanoflann.hpp>

namespace pdal
{

class KD2Impl
{
public:
    KD2Impl(const PointView& buf) : m_buf(buf),
        m_index(2, *this, nanoflann::KDTreeSingleIndexAdaptorParams(100))
    {}

    std::size_t kdtree_get_point_count() const
    {
        return m_buf.size();
    }

    double kdtree_get_pt(const PointId idx, int dim) const
    {
        using namespace Dimension;
        std::array<Id, 2> ids { Id::X, Id::Y };
        return m_buf.getFieldAs<double>(ids[dim], idx);
    }
    
    double kdtree_distance(const double *p1, const PointId p2_idx,
        size_t /*numDims*/) const
    {
        double d0 = p1[0] - m_buf.getFieldAs<double>(Dimension::Id::X, p2_idx);
        double d1 = p1[1] - m_buf.getFieldAs<double>(Dimension::Id::Y, p2_idx);

        return (d0 * d0 + d1 * d1);
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const
    {
        if (m_buf.empty())
            bb = {};
        else
        {
            BOX2D bounds;
            m_buf.calculateBounds(bounds);

            bb = { {bounds.minx, bounds.maxx}, {bounds.miny, bounds.maxy} };
        }
        return true;
    }

    void build()
    {
        m_index.buildIndex();
    }

    PointIdList neighbors(double x, double y, point_count_t k) const
    {
        k = (std::min)(m_buf.size(), k);
        PointIdList output(k);
        std::vector<double> out_dist_sqr(k);
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k);

        resultSet.init(&output[0], &out_dist_sqr[0]);

        std::array<double, 2> pt { x, y };
        m_index.findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
        return output;
    }

    void knnSearch(double x, double y, point_count_t k,
        PointIdList *indices, std::vector<double> *sqr_dists) const
    {
        k = (std::min)(m_buf.size(), k);
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k);

        resultSet.init(&indices->front(), &sqr_dists->front());

        std::array<double, 2> pt { x, y };
        m_index.findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
    }

    PointIdList radius(double const& x, double const& y, double const& r) const
    {
        PointIdList output;
        std::vector<std::pair<std::size_t, double>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        std::array<double, 2> pt { x, y };

        // Our distance metric is square distance, so we use the square of
        // the radius.
        const std::size_t count =
            m_index.radiusSearch(&pt[0], r * r, ret_matches, params);

        for (std::size_t i = 0; i < count; ++i)
            output.push_back(ret_matches[i].first);
        return output;
    }

private:
    const PointView& m_buf;

    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<
        double, KD2Impl, double>, KD2Impl, -1, std::size_t> KDTree;

    KDTree m_index;
};

class KD3Impl
{
public:
    KD3Impl(const PointView& buf) : m_buf(buf),
        m_index(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(100))
    {}

    std::size_t kdtree_get_point_count() const
    {
        return m_buf.size();
    }

    double kdtree_get_pt(const PointId idx, int dim) const
    {
        if (idx >= m_buf.size())
            return 0.0;

        using namespace Dimension;
        std::array<Id, 3> ids { Id::X, Id::Y, Id::Z };
        if ((size_t)dim >= ids.size())
            throw pdal_error("kdtree_get_pt: Request for invalid dimension "
                "from nanoflann");

        return m_buf.getFieldAs<double>(ids[dim], idx);
    }

    double kdtree_distance(const double *p1, const PointId p2_idx,
        size_t /*numDims*/) const
    {
        double d0 = p1[0] - m_buf.getFieldAs<double>(Dimension::Id::X, p2_idx);
        double d1 = p1[1] - m_buf.getFieldAs<double>(Dimension::Id::Y, p2_idx);
        double d2 = p1[2] - m_buf.getFieldAs<double>(Dimension::Id::Z, p2_idx);

        return (d0 * d0 + d1 * d1 + d2 * d2);
    }
        
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const
    {
        if (m_buf.empty())
            bb = {};
        else
        {
            BOX3D bounds;
            m_buf.calculateBounds(bounds);
            bb = { {bounds.minx, bounds.maxx}, {bounds.miny, bounds.maxy},
                {bounds.minz, bounds.maxz} };
        }
        return true;
    }

    void build()
    {
        m_index.buildIndex();
    }

    PointIdList neighbors(double x, double y, double z, point_count_t k,
        size_t stride) const
    {
        // Account for input buffer size smaller than requested number of
        // neighbors, then determine the number of neighbors to extract based
        // on the desired stride.
        k = (std::min)(m_buf.size(), k);
        point_count_t k2 = stride * k;

        // Prepare output indices and squared distances.
        PointIdList output(k2);
        std::vector<double> out_dist_sqr(k2);

        // Set the query point.
        std::vector<double> pt { x, y, z };

        // Extract k*stride neighbors, then return only k, selecting every nth
        // neighbor at the given stride.
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k2);
        resultSet.init(&output[0], &out_dist_sqr[0]);
        m_index.findNeighbors(resultSet, &pt[0], nanoflann::SearchParams());

        // Perform the downsampling if a stride is provided.
        if (stride > 1)
        {
            for (size_t i = 1; i < k; ++i)
                output[i] = output[i * stride];
            output.resize(k);
        }
        return output;
    }

    void knnSearch(double x, double y, double z, point_count_t k,
        PointIdList *indices, std::vector<double> *sqr_dists) const
    {
        k = (std::min)(m_buf.size(), k);
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k);

        resultSet.init(&indices->front(), &sqr_dists->front());

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        pt.push_back(z);
        m_index.findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
    }

    PointIdList radius(double x, double y, double z, double r) const
    {
        PointIdList output;
        std::vector<std::pair<std::size_t, double>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        std::vector<double> pt { x, y, z };

        // Our distance metric is square distance, so we use the square of
        // the radius.
        const std::size_t count =
            m_index.radiusSearch(&pt[0], r * r, ret_matches, params);

        for (std::size_t i = 0; i < count; ++i)
            output.push_back(ret_matches[i].first);
        return output;
    }

private:
    const PointView& m_buf;

    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<
        double, KD3Impl, double>, KD3Impl, -1, std::size_t> KDTree;

    KDTree m_index;
};

class KDFlexImpl
{
public:
    KDFlexImpl(const PointView& buf, const Dimension::IdList& dims) :
        m_buf(buf), m_dims(dims),
        m_index(m_dims.size(), *this,
            nanoflann::KDTreeSingleIndexAdaptorParams(100))
    {}

    std::size_t kdtree_get_point_count() const
    {
        return m_buf.size();
    }

    void build()
    {
        m_index.buildIndex();
    }

    PointIdList neighbors(PointRef &point, point_count_t k, size_t stride) const
    {
        // Account for input buffer size smaller than requested number of
        // neighbors, then determine the number of neighbors to extract based
        // on the desired stride.
        k = (std::min)(m_buf.size(), k);
        point_count_t k2 = stride * k;

        // Prepare output indices and squared distances.
        PointIdList output(k2);
        std::vector<double> out_dist_sqr(k2);

        // Set the query point.
        std::vector<double> pt;
        for (auto const& dim : m_dims)
        {
            double val = point.getFieldAs<double>(dim);
            pt.push_back(val);
        }

        // Extract k*stride neighbors, then return only k, selecting every nth
        // neighbor at the given stride.
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k2);
        resultSet.init(&output[0], &out_dist_sqr[0]);
        m_index.findNeighbors(resultSet, &pt[0], nanoflann::SearchParams());

        // Perform the downsampling if a stride is provided.
        if (stride > 1)
        {
            for (size_t i = 1; i < k; ++i)
                output[i] = output[i * stride];
            output.resize(k);
        }
        return output;
    }

    PointIdList radius(PointId idx, double r) const
    {
        PointIdList output;
        std::vector<std::pair<std::size_t, double>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        std::vector<double> pt;
        for (auto const& dim : m_dims)
        {
            double val = m_buf.getFieldAs<double>(dim, idx);
            pt.push_back(val);
        }

        // Our distance metric is square distance, so we use the square of
        // the radius.
        const std::size_t count =
            m_index.radiusSearch(pt.data(), r * r, ret_matches, params);

        for (std::size_t i = 0; i < count; ++i)
            output.push_back(ret_matches[i].first);
        return output;
    }

    inline double kdtree_get_pt(const PointId idx, int dim) const
    {
        if (idx >= m_buf.size())
            return 0.0;

        return m_buf.getFieldAs<double>(m_dims[dim], idx);
    }

    inline double kdtree_distance(const double* p1, const PointId idx,
                                  size_t /*numDims*/) const
    {
        double result(0.0);
        for (size_t i = 0; i < m_dims.size(); ++i)
        {
            double d = p1[i] - m_buf.getFieldAs<double>(m_dims[i], idx);
            result += d * d;
        }

        return result;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const
    {
        if (m_buf.empty())
            bb = {};
        else
        {
            for (size_t j = 0; j < m_dims.size(); ++j)
            {
                double val = m_buf.getFieldAs<double>(m_dims[j], 0);
                bb[j].low = val;
                bb[j].high = val;
            }

            for (PointId i = 1; i < m_buf.size(); ++i)
            {
                for (size_t j = 0; j < m_dims.size(); ++j)
                {
                    double val = m_buf.getFieldAs<double>(m_dims[j], i);
                    if (val < bb[j].low)
                        bb[j].low = val;
                    if (val > bb[j].high)
                        bb[j].high = val;
                }
            }
        }
        return true;
    }

private:
    const PointView& m_buf;
    const Dimension::IdList& m_dims;

    typedef nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor<
        double, KDFlexImpl, double>, KDFlexImpl, -1, std::size_t> KDTree;

    KDTree m_index;
};

} // namespace pdal
