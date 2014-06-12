/******************************************************************************
 * Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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

#ifndef PDAL_SPLITTER_H
#define PDAL_SPLITTER_H

#include <vector>

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/PointBuffer.hpp>

#include <boost/scoped_ptr.hpp>

namespace pdal
{
class Stage;

namespace filters
{

// namespace iterators
// {
// namespace sequential
// {
// class Splitter;
// }
// }

class PDAL_DLL Splitter;

namespace splitter
{

class PDAL_DLL Tile
{
public:
    friend class pdal::filters::Splitter;

    inline boost::uint32_t getNumPoints() const
    {
        boost::int32_t size = m_last - m_first + 1;
        if (size < 0)
            throw pdal_error("m_last - m_first + 1 was less than 0 in Tile::getNumPoints()!");
        return boost::uint32_t(size);
    }

    inline boost::uint32_t first() const
    {
        return m_first;
    }
    inline boost::uint32_t last() const
    {
        return m_last;
    }

private:
    boost::uint32_t m_first;
    boost::uint32_t m_last;
};

} // namespace splitter

class PDAL_DLL Splitter : public pdal::Filter
{
public:
    SET_STAGE_NAME("filters.splitter", "Splitter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.splitter.html")
    SET_STAGE_ENABLED(true)

    Splitter(const Options&);

    static Options getDefaultOptions();

    inline void setLeafSize(const float &leaf_size)
    {
        m_leaf_size = leaf_size;
        m_inverse_leaf_size = 1 / m_leaf_size;
    }

    inline float getLeafSize() const
    {
        return m_leaf_size;
    }

    inline boost::uint32_t getNrDivisionsX() const
    {
        return m_div_b_x;
    }

    inline boost::uint32_t getNrDivisionsY() const
    {
        return m_div_b_y;
    }

    inline boost::uint32_t getDivisionMultiplierX() const
    {
        return m_divb_mul_x;
    }

    inline boost::uint32_t getDivisionMultiplierY() const
    {
        return m_divb_mul_y;
    }

    inline virtual boost::uint64_t getNumPoints() const
    {
        return getPrevStage().getNumPoints();
    }

    void generateTiles(PointBuffer& buffer, PointBuffer& tiled_buffer);

    std::vector<splitter::Tile>::size_type getTileCount() const
    {
        return m_tiles.size();
    }

    const splitter::Tile& getTile(std::vector<splitter::Tile>::size_type i) const
    {
        return m_tiles[i];
    }

    inline boost::uint32_t getLength() const
    {
        return m_length;
    }

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer& buffer) const;

protected:
    float m_leaf_size, m_inverse_leaf_size;
    boost::uint32_t m_min_b_x, m_min_b_y, m_max_b_x, m_max_b_y, m_div_b_x, m_div_b_y, m_divb_mul_x, m_divb_mul_y;

private:
    boost::uint32_t m_length;
    std::vector<splitter::Tile> m_tiles;

    Splitter& operator=(const Splitter&); // not implemented
    Splitter(const Splitter&); // not implemented
    virtual void initialize();
};

namespace iterators
{
namespace sequential
{

class PDAL_DLL Splitter : public pdal::FilterSequentialIterator
{
public:
    Splitter(pdal::filters::Splitter const& filter, PointBuffer& buffer);
    ~Splitter();

protected:
    virtual boost::uint32_t readBufferImpl(PointBuffer&);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    bool atEndImpl() const;
    boost::uint32_t fillUserBuffer(PointBuffer& buffer,
                                   filters::splitter::Tile const& tile);

    pdal::filters::Splitter const& m_splitter;
    std::size_t m_currentTileId;
    PointBuffer* m_tiled_buffer;
};

} // sequential
} // iterators

} // namespace filters
} // namespace pdal

#endif

