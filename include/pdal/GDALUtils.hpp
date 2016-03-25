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

#include <pdal/pdal_internal.hpp>
#include <pdal/Dimension.hpp>

#include <pdal/Log.hpp>

#include <array>
#include <functional>
#include <sstream>
#include <vector>

#include <cpl_port.h>
#include <gdal.h>
#include <cpl_vsi.h>
#include <cpl_conv.h>
#include <ogr_api.h>
#include <ogr_srs_api.h>

namespace pdal
{

class SpatialReference;

namespace gdal
{

typedef std::shared_ptr<void> RefPtr;

class SpatialRef
{
public:
    SpatialRef()
        { newRef(OSRNewSpatialReference("")); }
    SpatialRef(const std::string& srs)
    {
        newRef(OSRNewSpatialReference(""));
        OSRSetFromUserInput(get(), srs.data());
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
        char *pszWKT = NULL;
        OSRExportToWkt(m_ref.get(), &pszWKT);
        bool valid = (bool)*pszWKT;
        std::string output(pszWKT);
        CPLFree(pszWKT);
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

class Geometry
{
public:
    Geometry()
        {}
    Geometry(const std::string& wkt, const SpatialRef& srs)
    {
        OGRGeometryH geom;

        char *p_wkt = const_cast<char *>(wkt.data());
        OGRSpatialReferenceH ref = srs.get();
        if (srs.empty())
        {
            ref = NULL;
        }
        OGRErr err = OGR_G_CreateFromWkt(&p_wkt, ref, &geom);
        if (err != OGRERR_NONE)
            throw pdal::pdal_error("unable to construct OGR geometry from wkt!");
        newRef(geom);
    }

    operator bool () const
        { return get() != NULL; }
    OGRGeometryH get() const
        { return m_ref.get(); }

    void transform(const SpatialRef& out_srs)
    {
        OGR_G_TransformTo(m_ref.get(), out_srs.get());
    }

    std::string wkt() const
    {
        char* p_wkt = 0;
        OGRErr err = OGR_G_ExportToWkt(m_ref.get(), &p_wkt);
        return std::string(p_wkt);
    }

    void setFromGeometry(OGRGeometryH geom)
        {
            if (geom)
                newRef(OGR_G_Clone(geom));
        }

private:
    void newRef(void *v)
    {
        m_ref = RefPtr(v, [](void* t){ OGR_G_DestroyGeometry(t); } );
    }
    RefPtr m_ref;
};


class PDAL_DLL ErrorHandler
{
public:
    class ExceptionSuspender
    {
    public:
        ExceptionSuspender()
        {
            doThrow = get().willThrow();
            get().setThrow(false);
        }
        ~ExceptionSuspender()
        {
            get().setThrow(doThrow);
        }

    private:
        bool doThrow;
    };

    /**
      Get the singleton error handler.

      \return  Reference to the error handler.
    */
    static ErrorHandler& get();

    /**
      Set the log and debug state of the error handler.  This is
      a convenience and is equivalent to calling setLog() and setDebug().

      \param log  Log to write to.
      \param doDebug  Debug state of the error handler.
    */
    void set(LogPtr log, bool doDebug);

    /**
      Set the log, debug and throw states of the error handler.  This is
      a convenience and is equivalent to calling setLog(), setDebug() and
      setThrow().

      \param log  Log to write to.
      \param doDebug  Debug state of the error handler.
      \param doThrow  Whether failures/fatals should cause an exception.
    */
    void set(LogPtr log, bool doDebug, bool doThrow);

    /**
      Set whether failures and fatal errors should be logged or cause an
      exception.

      \param doThrow  Whether failures/fatals should cause exceptions.
    */
    void setThrow(bool doThrow);

    /**
      Determine if the handler will throw exceptions on failures and fatal
      errors.

      \return  Whether failures/fatals will cause exceptions.
    */
    bool willThrow() const;

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

private:
    ErrorHandler();
    ~ErrorHandler();

    void handle(::CPLErr level, int num, const char *msg);

private:
    bool m_debug;
    pdal::LogPtr m_log;
    bool m_throw;
    int m_errorNum;
    bool m_cplSet;

    static ErrorHandler m_instance;
};


namespace GDALError
{

enum Enum
{
    None,
    NotOpen,
    CantOpen,
    NoData,
    InvalidBand,
    NoTransform,
    NotInvertible,
    CantReadBlock
};

} // namespace GDALError

class PDAL_DLL Raster
{

public:
    Raster(const std::string& filename);
    ~Raster();
    GDALError::Enum open();
    void close();

    GDALError::Enum read(double x, double y, std::vector<double>& data);
    std::vector<pdal::Dimension::Type::Enum> getPDALDimensionTypes() const
       { return m_types; }
    /**
      Read a raster band (layer) into a vector.

      \param band  Vector into which data will be read.  The vector will
        be resized appropriately to hold the data.
      \param nBand  Band number to read.  Band numbers start at 1.
    */
    GDALError::Enum readBand(std::vector<uint8_t>& band, int nBand);

    void pixelToCoord(int column, int row, std::array<double, 2>& output) const;
    SpatialReference getSpatialRef() const;
    std::string errorMsg() const
        { return m_errorMsg; }

    std::string m_filename;

    std::array<double, 6> m_forward_transform;
    std::array<double, 6> m_inverse_transform;

    int m_raster_x_size;
    int m_raster_y_size;

    int m_band_count;
    mutable std::vector<pdal::Dimension::Type::Enum> m_types;
    std::vector<std::array<double, 2>> m_block_sizes;

    GDALDatasetH m_ds;
    std::string m_errorMsg;

private:
    bool getPixelAndLinePosition(double x, double y,
        int32_t& pixel, int32_t& line);
    GDALError::Enum computePDALDimensionTypes();
};

} // namespace gdal


PDAL_DLL std::string transformWkt(std::string wkt, const SpatialReference& from,
    const SpatialReference& to);


} // namespace pdal

