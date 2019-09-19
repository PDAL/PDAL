/******************************************************************************
 * Copyright (c) 2019, Bradley J Chambers (brad.chambers@gmail.com)
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

// Adapted from MIT-licensed implemenation provided by
// https://github.com/intel-isl/Open3D/pull/1038.

#include "DBSCANFilter.hpp"

#include <pdal/KDIndex.hpp>

#include <string>
#include <unordered_set>

namespace pdal
{

template<int DIM>
class KDFlexIndex
{
protected:
    const PointView& m_buf;

    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<
        double, KDFlexIndex, double>, KDFlexIndex, -1, std::size_t> my_kd_tree_t;

    std::unique_ptr<my_kd_tree_t> m_index;

public:
    std::size_t kdtree_get_point_count() const
        { return m_buf.size(); }

    void build()
    {
        m_index.reset(new my_kd_tree_t(DIM, *this,
            nanoflann::KDTreeSingleIndexAdaptorParams(100)));
        m_index->buildIndex();
    }

    KDFlexIndex(const PointView& buf) : m_buf(buf)
    {
        if (!m_buf.hasDim(Dimension::Id::X))
            throw pdal_error("KDFlexIndex: point view missing 'X' dimension.");
        if (!m_buf.hasDim(Dimension::Id::Y))
            throw pdal_error("KDFlexIndex: point view missing 'Y' dimension.");
        if (!m_buf.hasDim(Dimension::Id::Z))
            throw pdal_error("KDFlexIndex: point view missing 'Z' dimension.");
        if (!m_buf.hasDim(Dimension::Id::NormalX))
            throw pdal_error("KDFlexIndex: point view missing 'NormalX' dimension.");
        if (!m_buf.hasDim(Dimension::Id::NormalY))
            throw pdal_error("KDFlexIndex: point view missing 'NormalY' dimension.");
        if (!m_buf.hasDim(Dimension::Id::NormalZ))
            throw pdal_error("KDFlexIndex: point view missing 'NormalZ' dimension.");
    }

    //KDFlexIndex(const PointView& buf) : KDFlexIndex<DIM>(buf)
    //{
    //    if (!buf.hasDim(Dimension::Id::X))
    //        throw pdal_error("KDFlexIndex: point view missing 'X' dimension.");
    //    if (!buf.hasDim(Dimension::Id::Y))
    //        throw pdal_error("KDFlexIndex: point view missing 'Y' dimension.");
    //    if (!buf.hasDim(Dimension::Id::Z))
    //        throw pdal_error("KDFlexIndex: point view missing 'Z' dimension.");
    //}

    ~KDFlexIndex()
    {}

    //PointId neighbor(double x, double y, double z) const
    //{
    //    PointIdList ids = neighbors(x, y, z, 1);
    //    return (ids.size() ? ids[0] : 0);
    //}

    //PointId neighbor(PointId idx) const
    //{
    //    PointIdList ids = neighbors(idx, 1);
    //    return (ids.size() ? ids[0] : 0);
    //}

    //PointId neighbor(PointRef &point) const
    //{
    //    PointIdList ids = neighbors(point, 1);
    //    return (ids.size() ? ids[0] : 0);
    //}

    PointIdList neighbors(double x, double y, double z, double nx, double ny, double nz,
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
        pt.push_back(nx);
        pt.push_back(ny);
        pt.push_back(nz);

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

    //PointIdList neighbors(PointId idx, point_count_t k, size_t stride=1) const
    //{
    //    double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    //    double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
    //    double z = m_buf.getFieldAs<double>(Dimension::Id::Z, idx);
    //
    //    return neighbors(x, y, z, k, stride);
    //}

    //PointIdList neighbors(PointRef &point, point_count_t k, size_t stride=1) const
    //{
    //    double x = point.getFieldAs<double>(Dimension::Id::X);
    //    double y = point.getFieldAs<double>(Dimension::Id::Y);
    //    double z = point.getFieldAs<double>(Dimension::Id::Z);
    //
    //    return neighbors(x, y, z, k, stride);
    //}

    void knnSearch(double x, double y, double z, double nx, double ny, double nz, point_count_t k,
        PointIdList *indices, std::vector<double> *sqr_dists)
    {
        k = (std::min)(m_buf.size(), k);
        nanoflann::KNNResultSet<double, PointId, point_count_t> resultSet(k);

        resultSet.init(&indices->front(), &sqr_dists->front());

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        pt.push_back(z);
        pt.push_back(nx);
        pt.push_back(ny);
        pt.push_back(nz);
        m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
    }

    //void knnSearch(PointId idx, point_count_t k, PointIdList *indices,
    //    std::vector<double> *sqr_dists)
    //{
    //    double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    //    double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
    //    double z = m_buf.getFieldAs<double>(Dimension::Id::Z, idx);
    //
    //    knnSearch(x, y, z, k, indices, sqr_dists);
    //}

    //void knnSearch(PointRef &point, point_count_t k,
    //    PointIdList *indices, std::vector<double> *sqr_dists)
    //{
    //    double x = point.getFieldAs<double>(Dimension::Id::X);
    //    double y = point.getFieldAs<double>(Dimension::Id::Y);
    //    double z = point.getFieldAs<double>(Dimension::Id::Z);
    //
    //    knnSearch(x, y, z, k, indices, sqr_dists);
    //}

    PointIdList radius(double x, double y, double z, double nx, double ny, double nz, double r) const
    {
        PointIdList output;
        std::vector<std::pair<std::size_t, double>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        pt.push_back(z);
        pt.push_back(nx);
        pt.push_back(ny);
        pt.push_back(nz);

        // Our distance metric is square distance, so we use the square of
        // the radius.
        const std::size_t count =
            m_index->radiusSearch(&pt[0], r * r, ret_matches, params);

        for (std::size_t i = 0; i < count; ++i)
            output.push_back(ret_matches[i].first);
        return output;
    }

    //PointIdList radius(PointId idx, double r) const
    //{
    //    double x = m_buf.getFieldAs<double>(Dimension::Id::X, idx);
    //    double y = m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
    //    double z = m_buf.getFieldAs<double>(Dimension::Id::Z, idx);
    //
    //    return radius(x, y, z, r);
    //}

    //PointIdList radius(PointRef &point, double r) const
    //{
    //    double x = point.getFieldAs<double>(Dimension::Id::X);
    //    double y = point.getFieldAs<double>(Dimension::Id::Y);
    //    double z = point.getFieldAs<double>(Dimension::Id::Z);
    //
    //    return radius(x, y, z, r);
    //}

    inline double kdtree_get_pt(const PointId idx, int dim) const
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
        case 3:
            id = Dimension::Id::NormalX;
            break;
        case 4:
            id = Dimension::Id::NormalY;
            break;
        case 5:
            id = Dimension::Id::NormalZ;
            break;
        default:
            throw pdal_error("kdtree_get_pt: Request for invalid dimension "
                "from nanoflann");
        }
        return m_buf.getFieldAs<double>(id, idx);
    }

    inline double kdtree_distance(const double *p1, const PointId idx,
        size_t /*numDims*/) const
    {
        double d0 = p1[0] - m_buf.getFieldAs<double>(Dimension::Id::X, idx);
        double d1 = p1[1] - m_buf.getFieldAs<double>(Dimension::Id::Y, idx);
        double d2 = p1[2] - m_buf.getFieldAs<double>(Dimension::Id::Z, idx);
        double d3 = p1[3] - m_buf.getFieldAs<double>(Dimension::Id::NormalX, idx);
        double d4 = p1[4] - m_buf.getFieldAs<double>(Dimension::Id::NormalY, idx);
        double d5 = p1[5] - m_buf.getFieldAs<double>(Dimension::Id::NormalZ, idx);

        return (d0 * d0 + d1 * d1 + d2 * d2 + d3 * d3 + d4 * d4 + d5 * d5);
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const
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
            double minx, miny, minz;
            minx = miny = minz = std::numeric_limits<double>::max();
            double maxx, maxy, maxz;
            maxx = maxy = maxz = std::numeric_limits<double>::lowest();

            for (PointId i = 0; i < m_buf.size(); ++i)
            {
                double nx = m_buf.getFieldAs<double>(Dimension::Id::NormalX, i);
                double ny = m_buf.getFieldAs<double>(Dimension::Id::NormalY, i);
                double nz = m_buf.getFieldAs<double>(Dimension::Id::NormalZ, i);
                if (nx < minx)
                    minx = nx;
                if (nx > maxx)
                    maxx = nx;
                if (ny < miny)
                    miny = ny;
                if (ny > maxy)
                    maxy = ny;
                if (nz < minz)
                    minz = nz;
                if (nz > maxz)
                    maxz = nz;
            }

            bb[0].low = minx;
            bb[0].high = maxx;
            bb[1].low = miny;
            bb[1].high = maxy;
            bb[2].low = minz;
            bb[2].high = maxz;
        }
        return true;
    }

private:
    KDFlexIndex(const KDFlexIndex&);
    KDFlexIndex& operator=(KDFlexIndex&);

};

static StaticPluginInfo const s_info{
    "filters.dbscan", "DBSCAN Clustering.",
    "http://pdal.io/stages/filters.dbscan.html"};

CREATE_STATIC_STAGE(DBSCANFilter, s_info)

std::string DBSCANFilter::getName() const
{
    return s_info.name;
}

void DBSCANFilter::addArgs(ProgramArgs& args)
{
    args.add("min_points", "Min points per cluster", m_minPoints,
             static_cast<uint64_t>(6));
    args.add("eps", "Epsilon", m_eps, 1.0);
}

void DBSCANFilter::addDimensions(PointLayoutPtr layout)
{
    m_cluster =
        layout->registerOrAssignDim("ClusterID", Dimension::Type::Signed64);
}

void DBSCANFilter::filter(PointView& view)
{
    // Construct 3D KDIndex for radius search.
    KD3Index& kdi = view.build3dIndex();

    KDNormalIndex kdni(view);
    kdni.build();

    KDFlexIndex<6> kdfi(view);
    kdfi.build();

    // First pass through point cloud precomputes neighbor indices and
    // initializes ClusterID to -2.
    std::vector<PointIdList> neighbors(view.size());
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        neighbors[idx] = kdi.radius(idx, m_eps);
        view.setField(m_cluster, idx, -2);
    }

    // Second pass through point cloud performs DBSCAN clustering.
    int64_t cluster_label = 0;
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        // Point has already been labeled, so move on to next.
        if (view.getFieldAs<int64_t>(m_cluster, idx) != -2)
            continue;

        // Density of the neighborhood does not meet minimum number of points
        // constraint, label as noise.
        if (neighbors[idx].size() < m_minPoints)
        {
            view.setField(m_cluster, idx, -1);
            continue;
        }

        // Initialize some bookkeeping to track which neighbors have been
        // considered for addition to the current cluster.
        std::unordered_set<PointId> neighbors_next(neighbors[idx].begin(),
                                                   neighbors[idx].end());
        std::unordered_set<PointId> neighbors_visited;
        neighbors_visited.insert(idx);

        // Unlabeled point encountered; assign cluster label.
        view.setField(m_cluster, idx, cluster_label);

        // Consider all neighbors.
        while (!neighbors_next.empty())
        {
            // Select first neighbor and move it to the visited set.
            PointId p = *neighbors_next.begin();
            neighbors_next.erase(neighbors_next.begin());
            neighbors_visited.insert(p);

            // Reassign cluster label to neighbor previously marked as noise.
            if (view.getFieldAs<int64_t>(m_cluster, p) == -1)
                view.setField(m_cluster, p, cluster_label);

            // Neighbor has already been labeled, so move on to next.
            if (view.getFieldAs<int64_t>(m_cluster, p) != -2)
                continue;

            // Assign cluster label to neighbor.
            view.setField(m_cluster, p, cluster_label);

            // If density of neighbor's neighborhood is sufficient, add it's
            // neighbors to the set of neighbors to consider if they are not
            // already.
            if (neighbors[p].size() >= m_minPoints)
            {
                for (PointId q : neighbors[p])
                {
                    if (neighbors_visited.count(q) == 0)
                        neighbors_next.insert(q);
                }
            }
        }

        cluster_label++;
    }
}

} // namespace pdal
