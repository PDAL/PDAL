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
#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>

#include <pdal/Log.hpp>

#include <array>
#include <functional>
#include <mutex>
#include <sstream>
#include <vector>

#include <cpl_port.h>
#include <gdal_priv.h>
#include <cpl_vsi.h>
#include <cpl_conv.h>
#include <ogr_api.h>
#include <ogr_srs_api.h>

namespace pdal
{

class SpatialReference;

namespace gdal
{

PDAL_DLL void registerDrivers();
PDAL_DLL void unregisterDrivers();
PDAL_DLL bool reprojectBounds(BOX3D& box, const std::string& srcSrs,
    const std::string& dstSrs);
PDAL_DLL bool reprojectBounds(BOX2D& box, const std::string& srcSrs,
    const std::string& dstSrs);
PDAL_DLL bool reprojectPoint(double& x, double& y, double& z,
    const std::string& srcSrs, const std::string& dstSrs);
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
        bool isJson = wkt.find("{") != wkt.npos ||
                      wkt.find("}") != wkt.npos;

        if (!isJson)
        {
            OGRErr err = OGR_G_CreateFromWkt(&p_wkt, ref, &geom);
            if (err != OGRERR_NONE)
            {
                std::cout << "wkt: " << wkt << std::endl;
                std::ostringstream oss;
                oss << "unable to construct OGR Geometry";
                oss << " '" << CPLGetLastErrorMsg() << "'";
                throw pdal::pdal_error(oss.str());
            }
        }
        else
        {
            // Assume it is GeoJSON and try constructing from that
            geom = OGR_G_CreateGeometryFromJson(p_wkt);

            if (!geom)
                throw pdal_error("Unable to create geometry from "
                    "input GeoJSON");

            OGR_G_AssignSpatialReference(geom, ref);
        }

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


// This is a little confusing because we have a singleton error handler with
// a single log pointer, but we set the log pointer/debug state as if we
// were taking advantage of GDAL's thread-specific error handing.
//
// We lock the log/debug so that it doesn't
// get changed while another thread is using or setting.
class PDAL_DLL ErrorHandler
{
public:

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

    ErrorHandler();

private:
    void handle(::CPLErr level, int num, const char *msg);

private:
    std::mutex m_mutex;
    bool m_debug;
    pdal::LogPtr m_log;
    int m_errorNum;
    bool m_cplSet;
};


enum class GDALError
{
    None,
    NotOpen,
    CantOpen,
    NoData,
    InvalidBand,
    NoTransform,
    NotInvertible,
    CantReadBlock,
    InvalidDriver,
    DriverNotFound,
    CantCreate,
    InvalidOption,
    CantWriteBlock
};

class PDAL_DLL Raster
{

public:
    /**
      Constructor.

      \param filename  Filename of raster file.
      \param drivername  Optional name of driver to use to open raster file.
    */
    Raster(const std::string& filename, const std::string& drivername = "");

    /**
      Constructor.

      \param filename  Filename of raster file.
      \param drivername  Optional name of driver to use to open raster file.
      \param srs  SpatialReference of the raster.
      \param pixelToPos  Transformation matrix to convert raster positions to
        geolocations.
    */
    Raster(const std::string& filename, const std::string& drivername,
        const SpatialReference& srs, const std::array<double, 6> pixelToPos);


    /**
      Destructor.  Closes an open raster.
    */
    ~Raster();

    /**
      Open raster file for reading.
    */
    GDALError open();

    /**
      Open a raster for writing.

      \param width  Width of the raster in cells (X direction)
      \param height  Height of the raster in cells (Y direction)
      \param numBands  Number of bands in the raster.
      \param type  Datatype (int, float, etc.) of the raster data.
      \param noData  Value that indiciates no data in a raster cell.
      \param options  GDAL driver options.
    */
    GDALError open(int width, int height, int numBands, Dimension::Type type,
        double noData, StringList options = StringList());

    /**
      Close the raster and deallocate the underlying dataset.
    */
    void close();

    /**
      Read an entire raster band (layer) into a vector.

      \param band  Vector into which data will be read.  The vector will
        be resized appropriately to hold the data.
      \param nBand  Band number to read.  Band numbers start at 1.
      \return Error code or GDALError::None.
    */
    GDALError readBand(std::vector<uint8_t>& band, int nBand);

    /**
      Write an entire raster band (layer) into raster to be written with GDAL.

      \param data  Linearized raster data to be written.
      \param nBand  Band number to write.
      \param name  Name of the raster band.
    */
    GDALError writeBand(const uint8_t *data, int nBand,
        const std::string& name = "");

    /**
      Read the data for each band at x/y into a vector of doubles.  x and y
      are transformed to the basis of the raster before the data is fetched.

      \param x  X position to read
      \param y  Y position to read
      \param data  Vector in which to store data.
    */
    GDALError read(double x, double y, std::vector<double>& data);

    /**
      Get a vector of dimensions that map to the bands of a raster.
    */
    std::vector<pdal::Dimension::Type> getPDALDimensionTypes() const
       { return m_types; }

    /**
      Convert an X/Y raster position into geo-located position using the
      raster's transformation matrix.

      \param column  raster column whose position should be calculated
      \param row  raster row whose position should be calculated
      \param[out]  Array containing the geo-located position of the pixel.
    */
    void pixelToCoord(int column, int row, std::array<double, 2>& output) const;

    /**
      Get the spatial reference associated with the raster.

      \return  The associated spatial reference.
    */
    SpatialReference getSpatialRef() const;

    /**
      Get the most recent error message.
    */
    std::string errorMsg() const
        { return m_errorMsg; }

    /**
      Get the number of bands in the raster.

      \return  The number of bands in the raster.
    */
    int bandCount() const
        { return m_numBands; }

    /**
      Get the width of the raster (X direction)
    */
    int width() const
        { return m_width; }

    /**
      Get the height of the raster (Y direction)
    */
    int height() const
        { return m_height; }

    std::string const& filename() { return m_filename; }

private:
    std::string m_filename;

    int m_width;
    int m_height;
    int m_numBands;
    std::string m_drivername;
    std::array<double, 6> m_forwardTransform;
    std::array<double, 6> m_inverseTransform;
    SpatialReference m_srs;
    GDALDataset *m_ds;

    std::string m_errorMsg;
    mutable std::vector<pdal::Dimension::Type> m_types;
    std::vector<std::array<double, 2>> m_block_sizes;

    bool getPixelAndLinePosition(double x, double y,
        int32_t& pixel, int32_t& line);
    GDALError computePDALDimensionTypes();
};

} // namespace gdal


PDAL_DLL std::string transformWkt(std::string wkt, const SpatialReference& from,
    const SpatialReference& to);

} // namespace pdal

