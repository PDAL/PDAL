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

#ifndef INCLUDED_FILTERS_MOSAICFILTER_HPP
#define INCLUDED_FILTERS_MOSAICFILTER_HPP

#include <vector>
#include <map>

#include <pdal/MultiFilter.hpp>
#include <pdal/MultiFilterIterator.hpp>
#include <pdal/StageIterator.hpp>


namespace pdal
{
namespace filters
{


// this doesn't derive from Stage since it takes more than one stage as input
class PDAL_DLL Mosaic : public MultiFilter
{
public:
    SET_STAGE_NAME("filters.mosaic", "Mosaic Filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.mosaic.html")
    SET_STAGE_ENABLED(true)
    
    Mosaic(const Options& options) : MultiFilter(options)
        {}

    virtual void initialize();
    static Options getDefaultOptions();

    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const
        { return NULL; }

private:
    Mosaic& operator=(const Mosaic&); // not implemented
    Mosaic(const Mosaic&); // not implemented
};



namespace iterators
{
    typedef boost::shared_ptr<schema::DimensionMap> DimensionMapPtr;
    typedef boost::shared_ptr<PointBuffer> BufferPtr;
    typedef std::map<int, DimensionMapPtr> DimensionMaps;
    typedef std::map<int, BufferPtr> BufferMap;
    
namespace sequential
{

class PDAL_DLL Mosaic : public pdal::MultiFilterSequentialIterator
{
public:
    Mosaic(const pdal::filters::Mosaic& filter, PointBuffer& buffer,
        LogPtr log, const Options& options);
    ~Mosaic();

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;
    DimensionMapPtr fetchDimensionMap(PointBuffer const& user_buffer,
        BufferPtr stage_buffer);
    BufferPtr fetchPointBuffer(PointBuffer const& user_buffer);
    DimensionMapPtr m_active_dimension;
    DimensionMaps m_dimensions;

    BufferMap m_buffers;
    LogPtr m_log;
    Options m_options;
};


}
} // iterators::sequential

}
} // namespaces

#endif
