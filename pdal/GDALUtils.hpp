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

#include <array>
#include <functional>
#include <mutex>
#include <sstream>
#include <vector>

#include <pdal/pdal_internal.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Log.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/JsonFwd.hpp>

#include <cpl_conv.h>
#include <gdal_priv.h>
#include <ogr_api.h>
#include <ogr_geometry.h>
#include <ogr_srs_api.h>

class OGRSpatialReference;

namespace pdal
{
class Polygon;

namespace gdal
{

PDAL_DLL void registerDrivers();
PDAL_DLL void unregisterDrivers();
PDAL_DLL bool reprojectBounds(Bounds& box, const SpatialReference& srcSrs,
    const SpatialReference& dstSrs);
PDAL_DLL bool reprojectBounds(BOX3D& box, const SpatialReference& srcSrs,
    const SpatialReference& dstSrs);
PDAL_DLL bool reprojectBounds(BOX2D& box, const SpatialReference& srcSrs,
    const SpatialReference& dstSrs);
PDAL_DLL bool reproject(double& x, double& y, double& z,
    const SpatialReference& srcSrs, const SpatialReference& dstSrs);
PDAL_DLL std::string lastError();

typedef std::shared_ptr<void> RefPtr;

class SpatialRef
{
public:
    SpatialRef()
        { newRef(OSRNewSpatialReference("")); }
    SpatialRef(const std::string& srs)
    {
        newRef(OSRNewSpatialReference(""));
        if (OSRSetFromUserInput(get(), srs.data()) != OGRERR_NONE)
            m_ref.reset();
    }

    void setFromLayer(OGRLayerH layer)
    {
        if (layer)
        {
            OGRSpatialReferenceH s = OGR_L_GetSpatialRef(layer);
            if (s)
            {
                OGRSpatialReferenceH clone = OSRClone(s);
                newRef(clone);
            }
        }
    }
    operator bool () const
        { return m_ref.get() != NULL; }
    OGRSpatialReferenceH get() const
        { return m_ref.get(); }
    std::string wkt() const
    {
        std::string output;

        if (m_ref.get())
        {
            char *pszWKT = NULL;
            OSRExportToWkt(m_ref.get(), &pszWKT);
            bool valid = (bool)*pszWKT;
            output = pszWKT;
            CPLFree(pszWKT);
        }
        return output;
    }

    bool empty() const
    {
        return wkt().empty();
    }

private:
    void newRef(void *v)
    {
        m_ref = RefPtr(v, [](void* t){ OSRDestroySpatialReference(t); } );
    }

    RefPtr m_ref;
};


// This is a little confusing because we have a singleton error handler with
// a single log pointer, but we set the log pointer/debug state as if we
// were taking advantage of GDAL's thread-specific error handing.
//
// We lock the log/debug so that it doesn't
// get changed while another thread is using or setting.
class PDAL_DLL ErrorHandler
{
public:
    ErrorHandler();
    ~ErrorHandler();

    /**
      Get the singleton error handler.

      \return  Reference to the error handler.
    */
    static ErrorHandler& getGlobalErrorHandler();

    /**
      Set the log and debug state of the error handler.  This is
      a convenience and is equivalent to calling setLog() and setDebug().

      \param log  Log to write to.
      \param doDebug  Debug state of the error handler.
    */
    void set(LogPtr log, bool doDebug);

    /**
      Set the log to which error/debug messages should be written.

      \param log  Log to write to.
    */
    void setLog(LogPtr log);

    /**
      Set the debug state of the error handler.  Setting to true will also
      set the environment variable CPL_DEBUG to "ON".  This will force GDAL
      to emit debug error messages which will be logged by this handler.

      \param doDebug  Whether we're setting or clearing the debug state.
    */
    void setDebug(bool doDebug);

    /**
      Get the last error and clear the error last error value.

      \return  The last error number.
    */
    int errorNum();

    static void CPL_STDCALL trampoline(::CPLErr code, int num, char const* msg)
    {
        ErrorHandler::getGlobalErrorHandler().handle(code, num, msg);
    }

private:
    void handle(::CPLErr level, int num, const char *msg);

private:
    std::mutex m_mutex;
    bool m_debug;
    pdal::LogPtr m_log;
    mutable int m_errorNum;
    bool m_cplSet;
};

class ErrorHandlerSuspender
{
public:
    ErrorHandlerSuspender()
        { CPLPushErrorHandler(CPLQuietErrorHandler); }
    ~ErrorHandlerSuspender()
        { (void)CPLPopErrorHandler(); }
};

OGRGeometry *createFromWkt(const char *s);
OGRGeometry *createFromGeoJson(const char *s);

// New signatures to support extraction of SRS from the end of geometry
// specifications..
OGRGeometry *createFromWkt(const std::string& s, std::string& srs);
OGRGeometry *createFromGeoJson(const std::string& s, std::string& srs);

std::vector<Polygon> getPolygons(const NL::json& ogr);

inline OGRGeometry *fromHandle(OGRGeometryH geom)
{ return reinterpret_cast<OGRGeometry *>(geom); }

inline OGRGeometryH toHandle(OGRGeometry *h)
{ return reinterpret_cast<OGRGeometryH>(h); }

} // namespace gdal

} // namespace pdal
