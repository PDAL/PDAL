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

#include "nanoflann.hpp"

#include <memory>
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
    double kdtree_distance(const PointId idx, point_count_t size) const;
    template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const;
    void build()
    {
        m_index.reset(new my_kd_tree_t(DIM, *this,
            nanoflann::KDTreeSingleIndexAdaptorParams(10, DIM)));
        m_index->buildIndex();
    }

    std::vector<PointId> radius(double const& x, double const& y,
        double const& z, double const& r) const;

    std::vector<PointId> neighbors(double const& x, double const& y,
        double const& z, point_count_t count) const;

protected:
    const PointView& m_buf;

    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Adaptor<
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
    {}

    PointId neighbor(double x, double y)
    {
        std::vector<PointId> ids = neighbors(x, y, 1);
        return (ids.size() ? ids[0] : 0);
    }

    std::vector<PointId> neighbors(double x, double y, point_count_t k)
    {
        k = std::min(m_buf.size(), k);
        std::vector<PointId> output(k);
        std::vector<double> out_dist_sqr(k);
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k);

        resultSet.init(&output[0], &out_dist_sqr[0]);

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
        return output;
    }

    std::vector<PointId> radius(double const& x, double const& y,
        double const& r) const
    {
        std::vector<PointId> output;
        std::vector<std::pair<std::size_t, double>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        const std::size_t count =
            m_index->radiusSearch(&pt[0], r, ret_matches, params);

        for (std::size_t i = 0; i < count; ++i)
            output.push_back(ret_matches[i].first);
        return output;
    }
};

class PDAL_DLL KD3Index : public KDIndex<3>
{
public:
    KD3Index(const PointView& buf) : KDIndex<3>(buf)
    {}

    PointId neighbor(double x, double y, double z)
    {
        std::vector<PointId> ids = neighbors(x, y, z, 1);
        return (ids.size() ? ids[0] : 0);
    }

    std::vector<PointId> neighbors(double x, double y, double z,
        point_count_t k)
    {
        k = std::min(m_buf.size(), k);
        std::vector<PointId> output(k);
        std::vector<double> out_dist_sqr(k);
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k);

        resultSet.init(&output[0], &out_dist_sqr[0]);

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        pt.push_back(z);
        m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
        return output;
    }

    std::vector<PointId> radius(double x, double y, double z, double r) const
    {
        std::vector<PointId> output;
        std::vector<std::pair<std::size_t, double>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        pt.push_back(z);
        const std::size_t count =
            m_index->radiusSearch(&pt[0], r, ret_matches, params);

        for (std::size_t i = 0; i < count; ++i)
            output.push_back(ret_matches[i].first);
        return output;
    }
};

template<>
inline
double KDIndex<2>::kdtree_get_pt(const PointId idx, int dim) const
{
    if (idx >= m_buf.size())
        return 0.0;

    Dimension::Id::Enum id = Dimension::Id::Unknown;
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

    Dimension::Id::Enum id = Dimension::Id::Unknown;
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

template<>
inline
double KDIndex<2>::kdtree_distance(const PointId idx, point_count_t size) const
{
    double d0 = m_buf.getFieldAs<double>(Dimension::Id::X, idx) -
        m_buf.getFieldAs<double>(Dimension::Id::X, size - 1);
    double d1 = m_buf.getFieldAs<double>(Dimension::Id::Y, idx) -
        m_buf.getFieldAs<double>(Dimension::Id::Y, size - 1);

    return (d0 * d0 + d1 * d1);
}

template<>
inline
double KDIndex<3>::kdtree_distance(const PointId idx, point_count_t size) const
{
    double d0 = m_buf.getFieldAs<double>(Dimension::Id::X, idx) -
        m_buf.getFieldAs<double>(Dimension::Id::X, size - 1);
    double d1 = m_buf.getFieldAs<double>(Dimension::Id::Y, idx) -
        m_buf.getFieldAs<double>(Dimension::Id::Y, size - 1);
    double d2 = m_buf.getFieldAs<double>(Dimension::Id::Z, idx) -
        m_buf.getFieldAs<double>(Dimension::Id::Z, size - 1);

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
        BOX3D bounds = m_buf.calculateBounds();

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
        BOX3D bounds = m_buf.calculateBounds();

        bb[0].low = bounds.minx;
        bb[0].high = bounds.maxx;
        bb[1].low = bounds.miny;
        bb[1].high = bounds.maxy;
        bb[2].low = bounds.minz;
        bb[3].high = bounds.maxz;
    }
    return true;
}

//ABELL - Specialize in subclass
template<int DIM>
std::vector<PointId> KDIndex<DIM>::neighbors(double const& x, double const& y,
    double const& z, point_count_t k) const
{
    k = std::min(m_buf.size(), k);
    std::vector<PointId> output(k);
    std::vector<double> out_dist_sqr(k);
    nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k);

    resultSet.init(&output[0], &out_dist_sqr[0]);

    std::vector<double> pt;
    pt.push_back(x);
    pt.push_back(y);
    pt.push_back(z);
    m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
    return output;
}

template<int DIM>
std::vector<PointId> KDIndex<DIM>::radius(double const& x, double const& y,
    double const& z, double const& r) const
{
    std::vector<PointId> output;
    std::vector<std::pair<std::size_t, double>> ret_matches;
    nanoflann::SearchParams params;
    params.sorted = true;

    std::vector<double> pt;
    pt.push_back(x);
    pt.push_back(y);
    pt.push_back(z);
    const std::size_t count =
        m_index->radiusSearch(&pt[0], r, ret_matches, params);

    for (std::size_t i = 0; i < count; ++i)
        output.push_back(ret_matches[i].first);
    return output;
}

} // namespace pdal

