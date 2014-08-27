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

#include <pdal/KDIndex.hpp>

namespace pdal
{

void KDIndex::build(bool b3D)
{
    m_3d = b3D;
    size_t nDims = m_3d && m_buf.hasDim(Dimension::Id::Z) ? 3 : 2;
    delete m_index;
    m_index = new my_kd_tree_t(nDims, *this,
        nanoflann::KDTreeSingleIndexAdaptorParams(10, nDims));
    m_index->buildIndex();
}

std::vector<size_t> KDIndex::radius(double const& x, double const& y,
    double const& z, double const& r) const
{
    std::vector<size_t> output;
    std::vector<std::pair<size_t, double>> ret_matches;
    nanoflann::SearchParams params;
    params.sorted = true;

    std::vector<double> pt;
    pt.push_back(x);
    pt.push_back(y);
    pt.push_back(z);
    const size_t count = m_index->radiusSearch(&pt[0], r, ret_matches, params);

    for (size_t i = 0; i < count; ++i)
        output.push_back(ret_matches[i].first);
    return output;
}

std::vector<size_t> KDIndex::neighbors(double const& x, double const& y,
    double const& z, double distance, boost::uint32_t k) const
{
    std::vector<size_t> output(k);
    std::vector<double> out_dist_sqr(k);
    nanoflann::KNNResultSet<double> resultSet(k);

    resultSet.init(&output[0], &out_dist_sqr[0]);

    std::vector<double> pt;
    pt.push_back(x);
    pt.push_back(y);
    if (m_3d)
        pt.push_back(z);
    m_index->findNeighbors(resultSet, &pt[0], nanoflann::SearchParams(10));
    return output;
}

} // namespace pdal

