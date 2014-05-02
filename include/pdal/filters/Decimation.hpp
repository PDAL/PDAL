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

#ifndef INCLUDED_FILTERS_DECIMATIONFILTER_HPP
#define INCLUDED_FILTERS_DECIMATIONFILTER_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>

namespace pdal
{
class PointBuffer;
}

namespace pdal
{
namespace filters
{


// we keep only 1 out of every step points; if step=100, we get 1% of the file
class PDAL_DLL Decimation : public Filter
{
public:
    SET_STAGE_NAME("filters.decimation", "Decimation Filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.decimation.html")        
    SET_STAGE_ENABLED(true)

    Decimation(const Options&);
    Decimation(boost::uint32_t step) : m_step(step)
        {}

    virtual void initialize();
    static Options getDefaultOptions();

    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const;

    boost::uint32_t getStep() const;

    boost::uint32_t processBuffer(PointBuffer& dstData,
        const PointBuffer& srcData, boost::uint64_t srcStartIndex) const;

private:
    boost::uint32_t m_step;

    Decimation& operator=(const Decimation&); // not implemented
    Decimation(const Decimation&); // not implemented
};


namespace iterators
{
    
namespace decimation
{

class PDAL_DLL IteratorBase
{

public:
    IteratorBase(pdal::filters::Decimation const& filter, PointBuffer& buffer);

protected:
    const pdal::filters::Decimation& m_decimationFilter;
    boost::int64_t m_startingIndex;

    boost::uint32_t decimateData(PointBuffer& buffer, StageSequentialIterator& iterator);        

private:
    IteratorBase& operator=(IteratorBase const&);
};
} // decimation
namespace sequential
{


class PDAL_DLL Decimation : public pdal::FilterSequentialIterator, public decimation::IteratorBase
{
public:
    Decimation(const pdal::filters::Decimation& filter, PointBuffer& buffer);
    virtual ~Decimation(){};
    
private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;
};


} //sequential

namespace random
{
    
class PDAL_DLL Decimation : public pdal::FilterRandomIterator, public decimation::IteratorBase
{
public:
    Decimation(const pdal::filters::Decimation& filter, PointBuffer& buffer,
        boost::uint32_t offset);
    virtual ~Decimation() {};

protected:
    virtual boost::uint32_t readBufferImpl(PointBuffer& buffer);
    
    virtual boost::uint64_t seekImpl(boost::uint64_t);

    boost::uint32_t m_offset;
};    
} // random

} // iterators

} // filters
} // pdal

#endif
