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

#pragma once

#include <pdal/Filter.hpp>
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
    SET_STAGE_ENABLED(true)
    
    Crop(const Options&);
    
    static Options getDefaultOptions();

    const Bounds<double>& getBounds() const;

private:
    Bounds<double> m_bounds;
    bool m_cropOutside;
    std::string m_poly;

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

    virtual void processOptions(const Options& options);
    virtual void ready(PointContext ctx);
    virtual PointBufferSet run(PointBufferPtr buffer);
    virtual void done(PointContext ctx);
    void crop(PointBuffer& input, PointBuffer& output);
    Bounds <double> computeBounds(GEOSGeometry const *geometry);
    
    Crop& operator=(const Crop&); // not implemented
    Crop(const Crop&); // not implemented
};

} // namespace filters
} // namespace pdal

