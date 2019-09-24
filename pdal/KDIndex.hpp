/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <memory>

#include <nanoflann/nanoflann.hpp>

#include <pdal/EigenUtils.hpp>
#include <pdal/PointView.hpp>

namespace nanoflann
{
    template<typename Distance, class DatasetAdaptor, int DIM,
        typename IndexType> class KDTreeSingleIndexAdaptor;

    template<class T, class DataSource, typename _DistanceType>
    struct L2_Adaptor;
}

namespace pdal
{

template<int DIM>
class PDAL_DLL KDIndex
{
protected:
    KDIndex(const PointView& buf) : m_buf(buf)
    {}

    ~KDIndex()
    {}

public:
    std::size_t kdtree_get_point_count() const
        { return m_buf.size(); }

    double kdtree_get_pt(const PointId idx, int dim) const;
    double kdtree_distance(const double *p1, const PointId p2_idx,
        size_t /*numDims*/) const;
    template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const;
    void build()
    {
        m_index.reset(new my_kd_tree_t(DIM, *this,
            nanoflann::KDTreeSingleIndexAdaptorParams(100)));
        m_index->buildIndex();
    }

protected:
    const PointView& m_buf;

    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<
        double, KDIndex, double>, KDIndex, -1, std::size_t> my_kd_tree_t;

    std::unique_ptr<my_kd_tree_t> m_index;

private:
    KDIndex(const KDIndex&);
    KDIndex& operator=(KDIndex&);
};

class PDAL_DLL KD2Index : public KDIndex<2>
{
public:
    KD2Index(const PointView& buf) : KDIndex<2>(buf)
    {
        if (!buf.hasDim(Dimension::Id::X))
            throw pdal_error("KD2Index: point view missing 'X' dimension.");
        if (!buf.hasDim(Dimension::Id::Y))
            throw pdal_error("KD2Index: point view missing 'Y' dimension.");
    }

    PointId neighbor(double x, double y) const
    {
        PointIdList ids = neighbors(x, y, 1);
        return (ids.size() ? ids[0] : 0);
    }

    PointId neighbor(PointId idx) const
    {
        PointIdList ids = neighbors(idx, 1);
        return (ids.size() ? ids[0] : 0);
    }

    PointId neighbor(PointRef &point) const
    {
        PointIdList ids = neighbors(point, 1);
        return (ids.size() ? ids[0] : 0);
    }

    PointIdList neighbors(double x, double y, point_count_t k) const
    {
        k = (std::min)(m_buf.size(), k);
        PointIdList output(k);
        std::vector<double> out_dist_sqr(k);
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k);

        resultSet.init(&output[0], &out_dist_sqr[0]);

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
        return output;
    }

    PointIdList neighbors(PointId idx, point_count_t k) const
    {
        double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
        double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);

        return neighbors(x, y, k);
    }

    PointIdList neighbors(PointRef &point, point_count_t k) const
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);

        return neighbors(x, y, k);
    }

    void knnSearch(double x, double y, point_count_t k,
        PointIdList *indices, std::vector<double> *sqr_dists) const
    {
        k = (std::min)(m_buf.size(), k);
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k);

        resultSet.init(&indices->front(), &sqr_dists->front());

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
    }

    void knnSearch(PointId idx, point_count_t k, PointIdList *indices,
        std::vector<double> *sqr_dists) const
    {
        double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
        double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);

        knnSearch(x, y, k, indices, sqr_dists);
    }

    void knnSearch(PointRef& point, point_count_t k, PointIdList *indices,
        std::vector<double> *sqr_dists) const
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);

        knnSearch(x, y, k, indices, sqr_dists);
    }

    PointIdList radius(double const& x, double const& y,
        double const& r) const
    {
        PointIdList output;
        std::vector<std::pair<std::size_t, double>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);

        // Our distance metric is square distance, so we use the square of
        // the radius.
        const std::size_t count =
            m_index->radiusSearch(&pt[0], r * r, ret_matches, params);

        for (std::size_t i = 0; i < count; ++i)
            output.push_back(ret_matches[i].first);
        return output;
    }

    PointIdList radius(PointId idx, double const& r) const
    {
        double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
        double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);

        return radius(x, y, r);
    }

    PointIdList radius(PointRef &point, double const& r) const
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);

        return radius(x, y, r);
    }
};

class PDAL_DLL KD3Index : public KDIndex<3>
{
public:
    KD3Index(const PointView& buf) : KDIndex<3>(buf)
    {
        if (!buf.hasDim(Dimension::Id::X))
            throw pdal_error("KD3Index: point view missing 'X' dimension.");
        if (!buf.hasDim(Dimension::Id::Y))
            throw pdal_error("KD3Index: point view missing 'Y' dimension.");
        if (!buf.hasDim(Dimension::Id::Z))
            throw pdal_error("KD3Index: point view missing 'Z' dimension.");
    }

    PointId neighbor(double x, double y, double z) const
    {
        PointIdList ids = neighbors(x, y, z, 1);
        return (ids.size() ? ids[0] : 0);
    }

    PointId neighbor(PointId idx) const
    {
        PointIdList ids = neighbors(idx, 1);
        return (ids.size() ? ids[0] : 0);
    }

    PointId neighbor(PointRef &point) const
    {
        PointIdList ids = neighbors(point, 1);
        return (ids.size() ? ids[0] : 0);
    }

    PointIdList neighbors(double x, double y, double z,
        point_count_t k, size_t stride=1) const
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
        pt.push_back(x);
        pt.push_back(y);
        pt.push_back(z);

        // Extract k*stride neighbors, then return only k, selecting every nth
        // neighbor at the given stride.
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k2);
        resultSet.init(&output[0], &out_dist_sqr[0]);
        m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams());

        // Perform the downsampling if a stride is provided.
        if (stride > 1)
        {
            for (size_t i = 1; i < k; ++i)
                output[i] = output[i * stride];
            output.resize(k);
        }
        return output;
    }

    PointIdList neighbors(PointId idx, point_count_t k, size_t stride=1) const
    {
        double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
        double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
        double z = m_buf.getFieldAs<double>(Dimension::Id::Z, idx);

        return neighbors(x, y, z, k, stride);
    }

    PointIdList neighbors(PointRef &point, point_count_t k,
        size_t stride=1) const
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);
        double z = point.getFieldAs<double>(Dimension::Id::Z);

        return neighbors(x, y, z, k, stride);
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
        m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
    }

    void knnSearch(PointId idx, point_count_t k, PointIdList *indices,
        std::vector<double> *sqr_dists) const
    {
        double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
        double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
        double z = m_buf.getFieldAs<double>(Dimension::Id::Z, idx);

        knnSearch(x, y, z, k, indices, sqr_dists);
    }

    void knnSearch(PointRef &point, point_count_t k,
        PointIdList *indices, std::vector<double> *sqr_dists) const
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);
        double z = point.getFieldAs<double>(Dimension::Id::Z);

        knnSearch(x, y, z, k, indices, sqr_dists);
    }

    PointIdList radius(double x, double y, double z, double r) const
    {
        PointIdList output;
        std::vector<std::pair<std::size_t, double>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        pt.push_back(z);

        // Our distance metric is square distance, so we use the square of
        // the radius.
        const std::size_t count =
            m_index->radiusSearch(&pt[0], r * r, ret_matches, params);

        for (std::size_t i = 0; i < count; ++i)
            output.push_back(ret_matches[i].first);
        return output;
    }

    PointIdList radius(PointId idx, double r) const
    {
        double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
        double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
        double z = m_buf.getFieldAs<double>(Dimension::Id::Z, idx);

        return radius(x, y, z, r);
    }

    PointIdList radius(PointRef &point, double r) const
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);
        double z = point.getFieldAs<double>(Dimension::Id::Z);

        return radius(x, y, z, r);
    }
};

template<>
inline
double KDIndex<2>::kdtree_get_pt(const PointId idx, int dim) const
{
    if (idx >= m_buf.size())
        return 0.0;

    Dimension::Id id = Dimension::Id::Unknown;
    switch (dim)
    {
    case 0:
        id = Dimension::Id::X;
        break;
    case 1:
        id = Dimension::Id::Y;
        break;
    default:
        throw pdal_error("kdtree_get_pt: Request for invalid dimension "
            "from nanoflann");
    }
    return m_buf.getFieldAs<double>(id, idx);
}

template<>
inline
double KDIndex<3>::kdtree_get_pt(const PointId idx, int dim) const
{
    if (idx >= m_buf.size())
        return 0.0;

    Dimension::Id id = Dimension::Id::Unknown;
    switch (dim)
    {
    case 0:
        id = Dimension::Id::X;
        break;
    case 1:
        id = Dimension::Id::Y;
        break;
    case 2:
        id = Dimension::Id::Z;
        break;
    default:
        throw pdal_error("kdtree_get_pt: Request for invalid dimension "
            "from nanoflann");
    }
    return m_buf.getFieldAs<double>(id, idx);
}

// nanoflann hands us a vector that represents the position of p1.  We fetch
// the position of p2 and and compute the square distance.
template<>
inline double KDIndex<2>::kdtree_distance(const double *p1, const PointId idx,
    size_t /*numDims*/) const
{
    double d0 = p1[0] - m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    double d1 = p1[1] - m_buf.getFieldAs<double>(Dimension::Id::Y, idx);

    return (d0 * d0 + d1 * d1);
}

template<>
inline double KDIndex<3>::kdtree_distance(const double *p1, const PointId idx,
    size_t /*numDims*/) const
{
    double d0 = p1[0] - m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    double d1 = p1[1] - m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
    double d2 = p1[2] - m_buf.getFieldAs<double>(Dimension::Id::Z, idx);

    return (d0 * d0 + d1 * d1 + d2 * d2);
}

template<>
template <class BBOX>
bool KDIndex<2>::kdtree_get_bbox(BBOX& bb) const
{
    if (m_buf.empty())
    {
        bb[0].low = 0.0;
        bb[0].high = 0.0;
        bb[1].low = 0.0;
        bb[1].high = 0.0;
    }
    else
    {
        BOX2D bounds;
        calculateBounds(m_buf, bounds);

        bb[0].low = bounds.minx;
        bb[0].high = bounds.maxx;
        bb[1].low = bounds.miny;
        bb[1].high = bounds.maxy;
    }
    return true;
}

template<>
template <class BBOX>
bool KDIndex<3>::kdtree_get_bbox(BBOX& bb) const
{
    if (m_buf.empty())
    {
        bb[0].low = 0.0;
        bb[0].high = 0.0;
        bb[1].low = 0.0;
        bb[1].high = 0.0;
        bb[2].low = 0.0;
        bb[2].high = 0.0;
    }
    else
    {
        BOX3D bounds;
        calculateBounds(m_buf, bounds);

        bb[0].low = bounds.minx;
        bb[0].high = bounds.maxx;
        bb[1].low = bounds.miny;
        bb[1].high = bounds.maxy;
        bb[2].low = bounds.minz;
        bb[2].high = bounds.maxz;
    }
    return true;
}

} // namespace pdal
