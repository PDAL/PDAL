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

#ifndef INCLUDED_FILTERS_CROPFILTER_HPP
#define INCLUDED_FILTERS_CROPFILTER_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>
#include <pdal/Bounds.hpp>

#ifdef PDAL_HAVE_GEOS
#include <geos_c.h>
#endif

namespace pdal
{

class PointBuffer;

namespace filters
{

// removes any points outside of the given range
// updates the header accordingly
class PDAL_DLL Crop : public Filter
{
public:
    SET_STAGE_NAME("filters.crop", "Crop Filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.crop.html")
    
    Crop(Stage& prevStage, const Options&);
    Crop(Stage& prevStage, Bounds<double> const& bounds);
    ~Crop();
    
    virtual void initialize();
    static Options getDefaultOptions();

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

    // returns number of points accepted into the data buffer (which may be less than data.getNumPoints(),
    // if we're calling this routine multiple times with the same buffer
    boost::uint32_t processBuffer(PointBuffer const& srcData, PointBuffer& dstData) const;

    const Bounds<double>& getBounds() const;
    inline boost::uint32_t getDimensions() const { return m_dimensions; }
    inline void setDimensions(boost::uint32_t const& dimensions) { m_dimensions = dimensions; }

private:
    Bounds<double> m_bounds;
    bool bCropOutside;

#ifdef PDAL_HAVE_GEOS
	GEOSContextHandle_t m_geosEnvironment;
    GEOSGeometry* m_geosGeometry; 
    GEOSPreparedGeometry const* m_geosPreparedGeometry;
#else   
    void* m_geosEnvironment;
    void* m_geosGeometry;
    void* m_geosPreparedGeometry;
    typedef struct GEOSGeometry* GEOSGeometryHS;
#endif

    boost::uint32_t m_dimensions;

    Bounds <double> computeBounds(GEOSGeometry const* geometry);
    
    Crop& operator=(const Crop&); // not implemented
    Crop(const Crop&); // not implemented
};


namespace iterators
{
namespace sequential
{


class PDAL_DLL Crop : public pdal::FilterSequentialIterator
{
public:
    Crop(const pdal::filters::Crop& filter, PointBuffer& buffer);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    const pdal::filters::Crop& m_cropFilter;
};


} // sequential
} // iterators

} // filters
} // pdal

#endif
