/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <pdal/pdal_types.hpp>

#include <cstdarg>

#ifdef PDAL_HAVE_GEOS
#include <geos_c.h>
#endif

#pragma once

namespace pdal
{

#ifdef PDAL_HAVE_GEOS
namespace
{

static GEOSContextHandle_t s_environment(NULL);
static int s_contextCnt(0);

static void GEOSErrorHandler(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    char buf[1024];

    vsnprintf(buf, sizeof(buf), fmt, args);
    std::cout << "GEOS error: " << buf << std::endl;

    va_end(args);
}

static void GEOSWarningHandler(const char *fmt, ...)
{
    va_list args;

    char buf[1024];
    vsnprintf(buf, sizeof(buf), fmt, args);
    std::cout << "GEOS warning: " << buf << std::endl;

    va_end(args);
}

static GEOSContextHandle_t init()
{
    if (s_contextCnt == 0)
        s_environment = initGEOS_r(GEOSWarningHandler, GEOSErrorHandler);
    s_contextCnt++;
    return s_environment;
}

static void finish()
{
    s_contextCnt--;
    if (s_contextCnt == 0)
        finishGEOS_r(s_environment);
}

} // Unnamed namespace

namespace Geometry
{

static std::string smoothPolygon(const std::string& wkt, double tolerance, uint32_t precision)
{
    GEOSContextHandle_t env = init();

    GEOSGeometry *geom = GEOSGeomFromWKT_r(env, wkt.c_str());
    if (!geom)
        return "";

    GEOSGeometry *smoothed = GEOSTopologyPreserveSimplify_r(env, geom,
        tolerance);
    if (!smoothed)
        return "";

    std::vector<GEOSGeometry*> geometries;

    int numGeom = GEOSGetNumGeometries_r(env, smoothed);
    for (int n = 0; n < numGeom; ++n)
    {
        const GEOSGeometry* m = GEOSGetGeometryN_r(env, smoothed, n);
        GEOSGeometry* exterior = GEOSGeom_clone_r(env, GEOSGetExteriorRing_r(env, m));

        std::vector<GEOSGeometry*> keep_rings;
        int numRings = GEOSGetNumInteriorRings_r(env, smoothed);
        double area_threshold = 6 * tolerance * tolerance;
        for (int i = 0; i < numRings; ++i)
        {
            double area(0.0);
            GEOSGeometry* cring = GEOSGeom_clone_r(env, GEOSGetInteriorRingN_r(env, smoothed, i));
            GEOSGeometry* aring = GEOSGeom_createPolygon_r(env, cring, NULL, 0);

            int errored = GEOSArea_r(env, aring, &area);
//             std::cout << "ring area: " << area << " threshold: " << area_threshold << std::endl;
            if (errored == 0)
                throw pdal::pdal_error("Unable to get area of ring!");
            if (area > area_threshold)
            {
                keep_rings.push_back(cring);
            }
        }

        GEOSGeometry* p = GEOSGeom_createPolygon_r(env,exterior, keep_rings.data(), keep_rings.size());
        if (p == NULL) throw pdal::pdal_error("smooth polygon could not be created!" );
        geometries.push_back(p);
    }

    GEOSGeometry* o = GEOSGeom_createCollection_r(env, GEOS_MULTIPOLYGON, geometries.data(), geometries.size());

    GEOSWKTWriter *writer = GEOSWKTWriter_create_r(env);
    GEOSWKTWriter_setRoundingPrecision_r(env, writer, precision);

    char *smoothWkt = GEOSWKTWriter_write_r(env, writer, o);

    std::string output(smoothWkt);
    GEOSFree_r(env, smoothWkt);
    GEOSWKTWriter_destroy_r(env, writer);
    GEOSGeom_destroy_r(env, geom);
    GEOSGeom_destroy_r(env, smoothed);
    finish();
    return output;
}

static double computeArea(const std::string& wkt)
{
    GEOSContextHandle_t env = init();

    GEOSGeometry *geom = GEOSGeomFromWKT_r(env, wkt.c_str());
    if (!geom)
        return 0.0;

    double output(0.0);
    int er = GEOSArea_r(env, geom, &output);
    GEOSGeom_destroy_r(env, geom);
    finish();
    return output;
}
#else

static std::string smoothPolygon(const std::string& wkt, double tolerance)
{
    throw pdal_error("Can't call smoothPolygon.  PDAL not built with GEOS.");
}

#endif

} // namespace Geometry

} // namespace pdal

