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

#include <sstream>
#include <vector>
#include <array>

#include <boost/function.hpp>
#include <boost/bind.hpp>

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

    ErrorHandler(bool isDebug, pdal::LogPtr log);
    ~ErrorHandler();

    static void CPL_STDCALL trampoline(::CPLErr code, int num, char const* msg)
    {
        ErrorHandler* debug =
            static_cast<ErrorHandler*>(CPLGetErrorHandlerUserData());
        if (!debug)
            return;

        // if (!debug->m_log->get()) return;
        debug->m_gdal_callback(code, num, msg);
    }

    void log(::CPLErr code, int num, char const* msg);
    void error(::CPLErr code, int num, char const* msg);

    inline LogPtr getLogger() const { return m_log; }
    inline void setLogger(LogPtr logger) { m_log = logger; }

private:
    boost::function<void(CPLErr, int, char const*)> m_gdal_callback;
    bool m_isDebug;
    pdal::LogPtr m_log;
};

class PDAL_DLL Raster

{

public:
    Raster(const std::string& filename);
    ~Raster();
    bool open();
    void close();

    bool read(double x, double y, std::vector<double>& data);
    std::vector<pdal::Dimension::Type::Enum> getPDALDimensionTypes() const
    {
        return m_types;
    }
    bool readBand(std::vector<uint8_t>& band, int nBand);

    void pixelToCoord(int column, int row, std::array<double, 2>& output) const;

    SpatialReference getSpatialRef() const;

    std::string m_filename;

    std::array<double, 6> m_forward_transform;
    std::array<double, 6> m_inverse_transform;

    int m_raster_x_size;
    int m_raster_y_size;

    int m_block_x;
    int m_block_y;

    size_t m_size;
    int m_band_count;
    std::vector<pdal::Dimension::Type::Enum> m_types;
    std::vector<std::array<double, 2>> m_block_sizes;

    GDALDatasetH m_ds;

private:
    bool getPixelAndLinePosition(double x, double y,
                                 std::array<double, 6> const& inverse,
                                 int32_t& pixel, int32_t& line);
    std::vector<pdal::Dimension::Type::Enum> computePDALDimensionTypes() const;
    std::vector<std::array<int, 2>> fetchGDALBlockSizes() const;

};

} // namespace gdal


PDAL_DLL std::string transformWkt(std::string wkt, const SpatialReference& from,
    const SpatialReference& to);


} // namespace pdal

