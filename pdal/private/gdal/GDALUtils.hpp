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

#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>

class OGRGeometry;

// Get GDAL's forward decls if available
// otherwise make our own
#if __has_include(<gdal_fwd.h>)
#include <gdal_fwd.h>
#else
using OGRGeometryH = void *;
using OGRSpatialReferenceH = void *;
#endif


#include <vector>

namespace pdal
{
class Polygon;
struct OGRSpecOptions;

namespace gdal
{

PDAL_EXPORT void registerDrivers();
PDAL_EXPORT void unregisterDrivers();
PDAL_EXPORT bool reprojectBounds(Bounds& box, const SpatialReference& srcSrs,
    const SpatialReference& dstSrs);
PDAL_EXPORT bool reprojectBounds(BOX3D& box, const SpatialReference& srcSrs,
    const SpatialReference& dstSrs);
PDAL_EXPORT bool reprojectBounds(BOX2D& box, const SpatialReference& srcSrs,
    const SpatialReference& dstSrs);
PDAL_EXPORT bool reproject(double& x, double& y, double& z,
    const SpatialReference& srcSrs, const SpatialReference& dstSrs);
PDAL_EXPORT std::string lastError();

// Exported for test support. Not sure why the above are exported.
PDAL_EXPORT std::vector<Polygon> getPolygons(const OGRSpecOptions& ogr);

// New signatures to support extraction of SRS from the end of geometry
// specifications.
OGRGeometry *createFromWkt(const std::string& s, std::string& srs);
OGRGeometry *createFromWkb(const std::string& s, std::string& srs);
OGRGeometry *createFromGeoJson(const std::string& s, std::string& srs);

inline OGRGeometry *fromHandle(OGRGeometryH geom)
{ return reinterpret_cast<OGRGeometry *>(geom); }

inline OGRGeometryH toHandle(OGRGeometry *h)
{ return reinterpret_cast<OGRGeometryH>(h); }

} // namespace gdal
} // namespace pdal
