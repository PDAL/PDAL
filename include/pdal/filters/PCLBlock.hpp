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

#ifndef INCLUDED_FILTERS_PCLBLOCK_HPP
#define INCLUDED_FILTERS_PCLBLOCK_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>

#include <boost/shared_ptr.hpp>

#ifdef PDAL_HAVE_PCL
// pcl includes
#endif

namespace pdal
{
class PointBuffer;

namespace filters
{


class PDAL_DLL PCLBlock : public Filter
{
public:
    SET_STAGE_NAME("filters.pclblock", "PCL Block implementation")
    SET_STAGE_LINK("http://www.pdal.io/stages/filters.pclblock.html")  
#ifdef PDAL_HAVE_PCL
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif
        
    PCLBlock(const Options& options) : Filter(options)
        {}
    ~PCLBlock()
        {}

    pdal::StageSequentialIterator*
    createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const
    {
        throw pdal::not_yet_implemented(
            "PCLBlock random iterator not implemented");
    }

    boost::uint32_t processBuffer(PointBuffer& srcData, std::string& filename,
                                  PointBuffer& dstData) const;

private:
    PCLBlock& operator=(const PCLBlock&); // not implemented
    PCLBlock(const PCLBlock&); // not implemented
};

namespace iterators
{
namespace sequential
{


class PDAL_DLL PCLBlock : public pdal::FilterSequentialIterator
{
public:
    PCLBlock(const pdal::filters::PCLBlock& filter, PointBuffer& buffer);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    const pdal::filters::PCLBlock& m_pclblockFilter;
};



} // sequential
} // iterators

} // filters
} // pdal

#endif

