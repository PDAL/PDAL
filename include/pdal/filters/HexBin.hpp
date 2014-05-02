/******************************************************************************
* Copyright (c) 2013, Andrew Bell (andrew.bell.ia@gmail.com)
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

#ifndef INCLUDED_FILTERS_HEXBINFILTER_HPP
#define INCLUDED_FILTERS_HEXBINFILTER_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>

#include <boost/shared_ptr.hpp>

#ifdef PDAL_HAVE_HEXER

#include <hexer/Mathpair.hpp>
#include <hexer/HexGrid.hpp>
#include <hexer/Processor.hpp>

#endif

namespace pdal
{
namespace filters
{


class PDAL_DLL HexBin : public Filter
{
public:
    SET_STAGE_NAME("filters.hexbin", "Hexbin implementation")
    SET_STAGE_LINK("http://pdal.io/stages/filters.hexbin.html")       
#ifdef PDAL_HAVE_HEXER
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif

    HexBin(const Options& options) : Filter(options)
        {}

    static Options getDefaultOptions();

    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const
    {
        throw pdal::not_yet_implemented(
            "HexBin random iterator not implemented");
    }

private:
    HexBin& operator=(const HexBin&); // not implemented
    HexBin(const HexBin&); // not implemented
    
};

namespace iterators
{

namespace hexbin
{

class PDAL_DLL IteratorBase
{
public:
    IteratorBase(pdal::filters::HexBin const& filter, 
                 PointBuffer& buffer,
                 boost::uint32_t numPoints,
                 std::string const& name,
                 LogPtr log,
                 Options const& options);
    ~IteratorBase();
protected:
    pdal::filters::HexBin const& m_filter;
    Dimension const* m_dim_x;
    Dimension const* m_dim_y;
    boost::uint32_t m_numPoints;
    std::string m_name;
    LogPtr m_log;
    Options m_options;
    
#ifdef PDAL_HAVE_HEXER
    hexer::HexGrid* m_grid;
	std::vector<hexer::Point> m_samples;
#endif
    boost::uint32_t m_sample_size;
    boost::uint32_t m_sample_number;
    boost::int32_t m_density;
    double m_edge_size;

private:
    IteratorBase& operator=(IteratorBase const&);

    
};

} // hexer
    
namespace sequential
{


class PDAL_DLL HexBin : public pdal::FilterSequentialIterator, public hexbin::IteratorBase
{
public:
    HexBin(pdal::filters::HexBin const& filter, 
                 PointBuffer& buffer,
                 boost::uint32_t numPoints,
                 std::string const& name,
                 LogPtr log,
                 Options options);
    ~HexBin();

protected:
    // virtual void readBufferBeginImpl(PointBuffer&);
    // virtual void readBufferEndImpl(PointBuffer&);
    // virtual void readEndImpl();    

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;    

};



} // sequential

// namespace random
// {
// 
// class PDAL_DLL HexBin : public pdal::FilterRandomIterator, public inplacereprojection::IteratorBase
// {
// public:
//     InPlaceReprojection(const pdal::filters::InPlaceReprojection& filter, PointBuffer& buffer);
//     virtual ~InPlaceReprojection(){};
// 
// protected:
//     virtual boost::uint32_t readBufferImpl(PointBuffer& buffer);
// 
//     
//     virtual boost::uint64_t seekImpl(boost::uint64_t);
// 
// 
// };    
//     
//     
// } // random

} // iterators


}
} // namespaces

#endif
