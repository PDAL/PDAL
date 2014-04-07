/******************************************************************************
 * Copyright (c) 2013, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/filters/Splitter.hpp>

#include <boost/scoped_ptr.hpp>

#include <iostream>
#include <limits>

namespace pdal
{
namespace filters
{

Splitter::Splitter(Stage& prevStage, Options const& options)
    : pdal::Filter(prevStage, options)
    , m_leaf_size(0)
    , m_inverse_leaf_size(0)
    , m_min_b_x(0)
    , m_min_b_y(0)
    , m_max_b_x(0)
    , m_max_b_y(0)
    , m_div_b_x(0)
    , m_div_b_y(0)
    , m_divb_mul_x(0)
    , m_divb_mul_y(0)
{
    m_length = options.getValueOrDefault<boost::uint32_t>("length", 1000);
}

void Splitter::initialize()
{
    Filter::initialize();

    if (m_length == 0)
    {
        throw pdal_error("splitter length cannot be 0!");
    }

    setLeafSize(m_length);

    return;
}

Options Splitter::getDefaultOptions()
{
    Options options;
    Option length("length", 1000, "Splitter length");
    options.add(length);

    return options;
}

struct tile_point_idx
{
    boost::uint32_t tile_idx;
    boost::uint32_t point_idx;

    tile_point_idx(boost::uint32_t tile_idx_,
                   boost::uint32_t point_idx_)
        : tile_idx(tile_idx_)
        , point_idx(point_idx_) {};
    bool operator < (const tile_point_idx &p) const
    {
        return (tile_idx < p.tile_idx);
    }
};

void Splitter::generateTiles(PointBuffer& buffer, PointBuffer& tiled_buffer)
{
    boost::uint64_t count = getPrevStage().getNumPoints();
    if (count > std::numeric_limits<boost::uint32_t>::max())
        throw pdal_error("numPoints too large for Splitter");

    Schema const& schema = buffer.getSchema();
    Dimension const& dimX = schema.getDimension("X");
    Dimension const& dimY = schema.getDimension("Y");

    // get bounds of the entire dataset
    Bounds<double> const& b = getPrevStage().getBounds();
    double minx = b.getMinimum(0);
    double maxx = b.getMaximum(0);
    double miny = b.getMinimum(1);
    double maxy = b.getMaximum(1);

    // check that the leaf size is not too small
    boost::int64_t dx = static_cast<boost::int64_t>((maxx-minx) * m_inverse_leaf_size)+1;
    boost::int64_t dy = static_cast<boost::int64_t>((maxy-miny) * m_inverse_leaf_size)+1;
    if ((dx*dy) > static_cast<boost::int64_t>(std::numeric_limits<boost::int32_t>::max()))
        throw pdal_error("Leaf size too small for the input dataset. Integer indices would overflow.");

    // compute the minimum and maximum bounding box values
    m_min_b_x = static_cast<boost::uint32_t>(std::floor(minx * m_inverse_leaf_size));
    m_max_b_x = static_cast<boost::uint32_t>(std::floor(maxx * m_inverse_leaf_size));
    m_min_b_y = static_cast<boost::uint32_t>(std::floor(miny * m_inverse_leaf_size));
    m_max_b_y = static_cast<boost::uint32_t>(std::floor(maxy * m_inverse_leaf_size));

    // compute the number of divisions in x and y
    m_div_b_x = m_max_b_x - m_min_b_x + 1;
    m_div_b_y = m_max_b_y - m_min_b_y + 1;

    // setup the division multiplier
    m_divb_mul_x = 1;
    m_divb_mul_y = m_div_b_x;

    // create index_vector to collect indices of points falling within each tile
    std::vector<tile_point_idx> index_vector;
    index_vector.reserve(buffer.getNumPoints());

    // we want to use the incoming buffer because we probably
    // have a cache filter on here, so we want to only read
    // all the data one time.
    boost::scoped_ptr<StageSequentialIterator> iter(getPrevStage().createSequentialIterator(buffer));

    // go over all points and insert them into the index_vector with calculated idx
    // points with the same idx lie within the same tile
    boost::uint32_t counter(0);
    while (!iter->atEnd())
    {
        boost::uint32_t numRead =  iter->read(buffer);

        double x(0.0);
        double y(0.0);
        for (boost::uint32_t j = 0; j < numRead; j++)
        {
            x = buffer.applyScaling(dimX, j);
            y = buffer.applyScaling(dimY, j);

            boost::uint32_t ij0 = static_cast<boost::uint32_t>(std::floor(x * m_inverse_leaf_size) - static_cast<float>(m_min_b_x));
            boost::uint32_t ij1 = static_cast<boost::uint32_t>(std::floor(y * m_inverse_leaf_size) - static_cast<float>(m_min_b_y));

            boost::uint32_t tile_idx = ij0 * m_divb_mul_x + ij1 * m_divb_mul_y;
            index_vector.push_back(tile_point_idx(static_cast<boost::uint32_t>(tile_idx), counter));

            counter++;
        }

        if (iter->atEnd())
        {
            break;
        }
    }

    // sort index_vector by target cell, all points belonging to the same cell will be adjacent
    std::sort(index_vector.begin(), index_vector.end(), std::less<tile_point_idx>());

    // count output cells
    boost::uint32_t index = 0;
    std::vector<std::pair<boost::uint32_t, boost::uint32_t> > first_and_last_indices_vector;
    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size())
    {
        boost::uint32_t i = index + 1;
        while (i < index_vector.size() && index_vector[i].tile_idx == index_vector[index].tile_idx)
            ++i;
        first_and_last_indices_vector.push_back(std::pair<boost::uint32_t, boost::uint32_t>(index, i-1));
        index = i;
    }

    PointBuffer outputData(buffer.getSchema(), buffer.getCapacity());
    outputData.setNumPoints(0);

    boost::uint32_t fidx = 0;
    boost::uint32_t lidx = 0;

    for (boost::uint32_t cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
    {
        boost::uint32_t first_index = first_and_last_indices_vector[cp].first;
        boost::uint32_t last_index = first_and_last_indices_vector[cp].second;

        boost::uint32_t tile_size = last_index - first_index + 1;

        PointBuffer tmpData(buffer.getSchema(), tile_size);
        tmpData.setNumPoints(0);

        boost::uint32_t ii = 0;
        for (boost::uint32_t j = first_index; j <= last_index; ++j)
        {
            tmpData.copyPointFast(ii, index_vector[j].point_idx, buffer);
            ii++;
        }
        tmpData.setNumPoints(tile_size);

        outputData.copyPointsFast(outputData.getNumPoints(), 0, tmpData, tmpData.getNumPoints());
        outputData.setNumPoints(outputData.getNumPoints() + tmpData.getNumPoints());

        lidx = fidx + tile_size-1;

        pdal::filters::splitter::Tile t;
        t.m_first = fidx;
        t.m_last = lidx;
        m_tiles.push_back(t);

        fidx = lidx+1;
    }

    tiled_buffer.resize(outputData.getNumPoints());
    tiled_buffer.setNumPoints(0);
    tiled_buffer.copyPointsFast(0, 0, outputData, outputData.getNumPoints());
    tiled_buffer.setNumPoints(outputData.getNumPoints());

    return;
}

pdal::StageRandomIterator* Splitter::createRandomIterator(PointBuffer&) const
{
    throw iterator_not_found("Splitter random iterator not implemented");
}

pdal::StageSequentialIterator* Splitter::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Splitter(*this, buffer);
}

namespace iterators
{
namespace sequential
{

Splitter::Splitter(pdal::filters::Splitter const& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_splitter(filter)
    , m_currentTileId(0)
    , m_tiled_buffer(0)
{

    m_tiled_buffer = new PointBuffer(buffer.getSchema(), buffer.getNumPoints());
    const_cast<pdal::filters::Splitter&>(m_splitter).generateTiles(buffer, *m_tiled_buffer);

    m_splitter.log()->get(logDEBUG3) << "generated temporary tiled buffer with " << m_splitter.getTileCount()
                                     << " tiles\n";

    return;
}

boost::uint64_t Splitter::skipImpl(boost::uint64_t count)
{
    return naiveSkipImpl(count);
}

boost::uint32_t Splitter::fillUserBuffer(PointBuffer& buffer,
        filters::splitter::Tile const& tile)
{
    boost::uint32_t numPointsThisTile = tile.getNumPoints();

    buffer.resize(numPointsThisTile);
    buffer.setNumPoints(0);
    buffer.copyPointsFast(0, tile.first(), *m_tiled_buffer, numPointsThisTile);
    buffer.setNumPoints(numPointsThisTile);

    return numPointsThisTile;
}

boost::uint32_t Splitter::readBufferImpl(PointBuffer& buffer)
{
    if (m_currentTileId == m_splitter.getTileCount())
        return 0; // we're done

    filters::splitter::Tile const& tile = m_splitter.getTile(m_currentTileId);
    boost::uint32_t numPointsThisTile = tile.getNumPoints();

    boost::uint32_t numRead = fillUserBuffer(buffer, tile);

    //buffer.setSpatialBounds(tile.getBounds());
    buffer.setNumPoints(numRead);

    m_splitter.log()->get(logDEBUG3) << "filled tile " << m_currentTileId
                                     << " of " << m_splitter.getTileCount()
                                     << " with " << numRead << " points\n";

    m_currentTileId++;

    return numRead;
}

bool Splitter::atEndImpl() const
{
    if (m_currentTileId == m_splitter.getTileCount())
        return true;
    else
        return false;
}

Splitter::~Splitter()
{
    delete m_tiled_buffer;
}

} // sequential
} // iterators

} // filters
} // pdal

