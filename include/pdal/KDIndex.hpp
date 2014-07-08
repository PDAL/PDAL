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

#include <pdal/third/nanoflann.hpp>

#include <pdal/PointBuffer.hpp>

namespace pdal
{

class PDAL_DLL KDIndex
{
public:
    KDIndex(PointBuffer& buf) : m_buf(buf), m_index(0)
    {
        const Schema& s = m_buf.getSchema();
        m_dim[0] = s.getDimensionPtr("X");
        m_dim[1] = s.getDimensionPtr("Y");
        m_dim[2] = s.getDimensionPtr("Z");
        m_3d = (bool)m_dim[2];
    }

    ~KDIndex()
        { delete m_index; }

    size_t kdtree_get_point_count() const
        { return m_buf.size(); }

    double kdtree_get_pt(const size_t idx, int dim) const
    {
        return m_buf.getFieldAs<double>(*m_dim[dim], idx);
    }

    double kdtree_distance(const double *p1, const size_t idx_p2,
        size_t size) const
    {
        double d0 = m_buf.getFieldAs<double>(*m_dim[0], idx_p2) -
            m_buf.getFieldAs<double>(*m_dim[0], size - 1);
        double d1 = m_buf.getFieldAs<double>(*m_dim[1], idx_p2) -
            m_buf.getFieldAs<double>(*m_dim[1], size - 1);

        double output(d0 * d0 + d1 * d1);
        if (m_3d && m_dim[2])
        {
            double d2 = m_buf.getFieldAs<double>(*m_dim[2], idx_p2) -
                m_buf.getFieldAs<double>(*m_dim[2], size - 1);
            output += d2 * d2;
        }
        return output;
    }
    
    template <class BBOX> bool kdtree_get_bbox(BBOX &bb) const
    {
        pdal::Bounds<double> const& bounds = m_buf.getSpatialBounds();
        if (bounds.empty())
            return false;

        size_t nDims = m_3d && m_dim[2] ? 3 : 2;
        for (size_t i = 0; i < nDims; ++i)
        {
            bb[i].low = bounds.getMinimum(i);
            bb[i].high = bounds.getMaximum(i);
        }
        return true;
    }

    std::vector<size_t>
    neighbors(double const& x, double const& y, double const& z,
        double distance, boost::uint32_t count = 1);
    std::vector<size_t> radius(double const& x, double const& y,
        double const& z, double const& r);
    void build(bool b3d = true);
    
private:
    PointBuffer& m_buf;
    const Dimension* m_dim[3];
    bool m_3d;

    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Adaptor<double, KDIndex>, KDIndex> my_kd_tree_t;
    my_kd_tree_t* m_index;
};

} // namespace pdal

