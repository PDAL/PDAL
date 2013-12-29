/******************************************************************************
 * $Id$
 *
 * Project:  PDAL - http://pdal.org - A BSD library for point cloud data.
 * Purpose:  PDAL chipper class
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2011, Howard Butler
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#ifndef PDAL_CHIPPER_H
#define PDAL_CHIPPER_H

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


namespace iterators { namespace sequential { class Chipper; }}

class PDAL_DLL Chipper;

namespace chipper
{

enum Direction
{
    DIR_X,
    DIR_Y,
    DIR_NONE
};

class PDAL_DLL PtRef
{
public:
    double m_pos;
    boost::uint32_t m_ptindex;
    boost::uint32_t m_oindex;

    bool operator < (const PtRef& pt) const
    {
        return m_pos < pt.m_pos;
    }
};

struct PDAL_DLL RefList
{
public:
    std::vector<PtRef> m_vec;
    Direction m_dir;

    RefList(Direction dir = DIR_NONE) : m_dir(dir)
    {}
    std::vector<PtRef>::size_type size() const
    {
        return m_vec.size();
    }
    void reserve(std::vector<PtRef>::size_type n)
    {
        m_vec.reserve(n);
    }
    void resize(std::vector<PtRef>::size_type n)
    {
        m_vec.resize(n);
    }
    void push_back(const PtRef& ref)
    {
        m_vec.push_back(ref);
    }
    std::vector<PtRef>::iterator begin()
    {
        return m_vec.begin();
    }
    std::vector<PtRef>::iterator end()
    {
        return m_vec.end();
    }
    PtRef& operator[](boost::uint32_t pos)
    {
        return m_vec[pos];
    }
    std::string Dir()
    {
        if (m_dir == DIR_X)
            return "X";
        else if (m_dir == DIR_Y)
            return "Y";
        else
            return "NONE";
    }
};



class PDAL_DLL Block
{
public:
    friend class pdal::filters::Chipper;

    inline boost::uint32_t GetID(boost::uint32_t const& index) const
    {
        return (*m_list_p)[index].m_ptindex;
    }
    
    inline boost::uint32_t GetSize() const
    {
        boost::int32_t size = m_right - m_left + 1;
        if (size < 0)
            throw pdal_error("m_right - m_left + 1 was less than 0 in Block::size()!");
        return boost::uint32_t(size);
    }
    
    inline boost::uint32_t left() const { return m_left; }
    inline boost::uint32_t right() const { return m_right; }

private:
    RefList *m_list_p;
    boost::uint32_t m_left;
    boost::uint32_t m_right;
    pdal::Bounds<double> m_bounds;

    // double m_xmin;
    // double m_ymin;
    // double m_xmax;
    // double m_ymax;

public:
    std::vector<boost::uint32_t> GetIDs() const;
    pdal::Bounds<double> const& GetBounds() const
    {
        return m_bounds;
    }
    void SetBounds(pdal::Bounds<double> const& bounds)
    {
        m_bounds = bounds;
    }
};

} // namespace chipper

class PDAL_DLL Chipper : public pdal::Filter
{
public:
    SET_STAGE_NAME("filters.chipper", "Chipper")

    Chipper(Stage& prevStage, const Options&);

    virtual void initialize();
    static Options getDefaultOptions();
    static std::vector<Dimension> getDefaultDimensions();

    void Chip(PointBuffer& buffer);
    std::vector<chipper::Block>::size_type GetBlockCount() const
    {
        return m_blocks.size();
    }
    const chipper::Block& GetBlock(std::vector<chipper::Block>::size_type i) const
    {
        return m_blocks[i];
    }

    bool supportsIterator(StageIteratorType t) const
    {
        if (t == StageIterator_Sequential) return true;
        return false;
    }
    
    inline boost::uint32_t getThreshold() const
    {
        return m_threshold;
    }

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer& buffer) const;

private:
    void Load(PointBuffer& buffer, chipper::RefList& xvec, chipper::RefList& yvec, chipper::RefList& spare);
    void Partition(boost::uint32_t size);
    void Split(chipper::RefList& xvec, chipper::RefList& yvec, chipper::RefList& spare);
    void DecideSplit(chipper::RefList& v1, chipper::RefList& v2, chipper::RefList& spare,
                     boost::uint32_t left, boost::uint32_t right);
    void Split(chipper::RefList& wide, chipper::RefList& narrow,chipper::RefList& spare,
               boost::uint32_t left, boost::uint32_t right);
    void FinalSplit(chipper::RefList& wide, chipper::RefList& narrow,
                    boost::uint32_t pleft, boost::uint32_t pcenter);
    void Emit(chipper::RefList& wide, boost::uint32_t widemin, boost::uint32_t widemax,
              chipper::RefList& narrow, boost::uint32_t narrowmin, boost::uint32_t narrowmax);

    Schema alterSchema(Schema const& schema);

    boost::uint32_t m_threshold;
    std::vector<chipper::Block> m_blocks;
    std::vector<boost::uint32_t> m_partitions;
    chipper::RefList m_xvec;
    chipper::RefList m_yvec;
    chipper::RefList m_spare;

    Chipper& operator=(const Chipper&); // not implemented
    Chipper(const Chipper&); // not implemented
};

namespace iterators
{
namespace sequential
{

class PDAL_DLL Chipper : public pdal::FilterSequentialIterator
{
public:
    Chipper(pdal::filters::Chipper const& filter, PointBuffer& buffer);
    ~Chipper();

protected:
    virtual void readBufferBeginImpl(PointBuffer& buffer);
    virtual boost::uint32_t readBufferImpl(PointBuffer&);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    bool atEndImpl() const;
    boost::uint32_t fillUserBuffer( PointBuffer& buffer,
                                    filters::chipper::Block const& block);

    pdal::filters::Chipper const& m_chipper;
    std::size_t m_currentBlockId;
    boost::uint64_t m_currentPointCount;
    PointBuffer* m_one_point;
    Schema const* m_current_read_schema;
    StageRandomIterator * m_random_iterator;
    schema::DimensionMap* m_one_point_dimension_map;
    pdal::Dimension const* m_dimPoint;
    pdal::Dimension const* m_dimBlock;

};

}
} // iterators::sequential



} // namespace filters

} // namespace liblas

#endif
