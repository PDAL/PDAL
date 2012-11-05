/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#ifndef INCLUDED_FILTERS_INDEXFILTER_HPP
#define INCLUDED_FILTERS_INDEXFILTER_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>

#include <boost/scoped_array.hpp>

#ifdef PDAL_HAVE_FLANN
#include <flann/flann.hpp>
#endif


namespace pdal
{
namespace filters
{

class PDAL_DLL Index : public Filter
{
public:
    SET_STAGE_NAME("filters.index", "Generates a spatial index to support range and spatial queries")

    Index(Stage& prevStage, const Options&);
    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    bool supportsIterator(StageIteratorType t) const
    {
        if (t == StageIterator_Sequential) return true;

        return false;
    }

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const
    {
        return NULL;
    }

    void processBuffer(PointBuffer& data) const;

    void setNumDimensions(boost::uint32_t dimensions) 
    {
        if (!(dimensions == 2) && !(dimensions == 3))
            throw pdal_error("Dimension count must be 2 or 3 for index queries");
        m_dimensions = dimensions; 
    }
    
    boost::uint32_t const& getNumDimensions() const;


private:

    Index& operator=(const Index&); // not implemented
    Index(const Index&); // not implemented
    boost::uint32_t m_dimensions;    
};

namespace iterators
{
namespace sequential
{


class PDAL_DLL Index : public pdal::FilterSequentialIterator
{
public:
    Index(const pdal::filters::Index& filter, PointBuffer& buffer);
    virtual ~Index();
    
    std::vector<boost::uint32_t> query(double const& x, double const& y, double const& z, double distance, boost::uint32_t count=1);
    

    virtual void readBufferBeginImpl(PointBuffer&);
    void build();

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    double getScaledValue(PointBuffer& data,
                          Dimension const& d,
                          std::size_t pointIndex) const;    
    const pdal::filters::Index& m_stage;
    
    std::vector<float> m_data;


#ifdef PDAL_HAVE_FLANN
    flann::KDTreeSingleIndex<flann::L2_Simple<float> >* m_index;
    flann::Matrix<float>* m_dataset;    
#endif      
    pdal::Dimension const* m_xDim;
    pdal::Dimension const* m_yDim;
    pdal::Dimension const* m_zDim;
    
};


}
} // iterators::sequential


}
} // namespaces

#endif
