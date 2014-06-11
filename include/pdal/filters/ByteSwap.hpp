/******************************************************************************
* Copyright (c) 2011, Howard Butler <hobu.inc@gmail.com>
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

#ifndef INCLUDED_FILTERS_BYTESWAPFILTER_HPP
#define INCLUDED_FILTERS_BYTESWAPFILTER_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>
#include <pdal/Bounds.hpp>

namespace pdal
{
class PointBuffer;
}

namespace pdal
{
namespace filters
{

class ByteSwapFilterSequentialIterator;

// removes any points outside of the given range
// updates the header accordingly
class PDAL_DLL ByteSwap : public Filter
{
public:
    SET_STAGE_NAME("filters.byteswap", "Byte-swap Filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.byteswap.html")
    SET_STAGE_ENABLED(true)
    
    ByteSwap(const Options& options) : Filter(options)
        {}
    ByteSwap()
        {}

    static Options getDefaultOptions();

    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const
        { return NULL; }

    // returns number of points accepted into the data buffer (which may
    // be less than data.getNumPoints(), if we're calling this routine
    // multiple times with the same buffer
    boost::uint32_t processBuffer(PointBuffer& dstData,
        const PointBuffer& srcData) const;

private:
    ByteSwap& operator=(const ByteSwap&); // not implemented
    ByteSwap(const ByteSwap&); // not implemented
    virtual void initialize(PointContext);
};


namespace iterators
{
namespace sequential
{

class PDAL_DLL ByteSwap : public pdal::FilterSequentialIterator
{
public:
    ByteSwap(const pdal::filters::ByteSwap& filter, PointBuffer& buffer);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    const pdal::filters::ByteSwap& m_swapFilter;
};

}
} // iterators::sequential

}
} // namespaces

#endif
