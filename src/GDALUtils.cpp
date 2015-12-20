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

#include <pdal/GDALUtils.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/util/Utils.hpp>

#include <functional>
#include <map>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{
namespace gdal
{

ErrorHandler::ErrorHandler(bool isDebug, pdal::LogPtr log)
    : m_isDebug(isDebug)
    , m_log(log)
{
    if (m_isDebug)
    {
        const char* gdal_debug = ::pdal::Utils::getenv("CPL_DEBUG");
        if (gdal_debug == 0)
        {
            pdal::Utils::putenv("CPL_DEBUG=ON");
        }
        m_gdal_callback = std::bind(&ErrorHandler::log, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3);
    }
    else
    {
        m_gdal_callback = std::bind(&ErrorHandler::error, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3);
    }

    CPLPushErrorHandlerEx(&ErrorHandler::trampoline, this);
}

void ErrorHandler::log(::CPLErr code, int num, char const* msg)
{
    std::ostringstream oss;

    if (code == CE_Failure || code == CE_Fatal)
        error(code, num, msg);
    else if (code == CE_Debug)
    {
        oss << "GDAL debug: " << msg;
        if (m_log)
            m_log->get(LogLevel::Debug) << oss.str() << std::endl;
    }
}


void ErrorHandler::error(::CPLErr code, int num, char const* msg)
{
    std::ostringstream oss;
    if (code == CE_Failure || code == CE_Fatal)
    {
        oss << "GDAL Failure number = " << num << ": " << msg;
        throw pdal_error(oss.str());
    }
}


ErrorHandler::~ErrorHandler()
{
    CPLPopErrorHandler();
}

Raster::Raster(const std::string& filename)
    : m_filename(filename)
    , m_forward_transform{ { 0, 1, 0, 0, 0, 1 } }
    , m_inverse_transform{ { 0, 1, 0, 0, 0, 1 } }
    , m_raster_x_size(0)
    , m_raster_y_size(0)
    , m_block_x(0)
    , m_block_y(0)
    , m_band_count(0)
    , m_ds(0)
{}

GDALError::Enum Raster::open()
{
    GDALError::Enum error = GDALError::None;
    if (m_ds)
        return error;

    m_ds = GDALOpen(m_filename.c_str(), GA_ReadOnly);
    if (m_ds == NULL)
    {
        m_errorMsg = "Unable to open GDAL datasource '" + m_filename + "'.";
        return GDALError::CantOpen;
    }

    // GDAL docs state that we should return an identity transform, even
    // on error, which should let things work.
    if (GDALGetGeoTransform(m_ds, &(m_forward_transform.front())) != CE_None)
    {
        m_errorMsg = "Unable to get geotransform for raster '" +
            m_filename + "'.";
        error = GDALError::NoTransform;
    }

    if (!GDALInvGeoTransform(&(m_forward_transform.front()),
        &(m_inverse_transform.front())))
    {
        m_errorMsg = "Geotransform for raster '" + m_filename + "' not "
            "intertible";
        error = GDALError::NotInvertible;
    }

    m_raster_x_size = GDALGetRasterXSize(m_ds);
    m_raster_y_size = GDALGetRasterYSize(m_ds);
    m_band_count = GDALGetRasterCount(m_ds);

    if (computePDALDimensionTypes() == GDALError::InvalidBand)
        error = GDALError::InvalidBand;
    return error;
}

void Raster::pixelToCoord(int col, int row, std::array<double, 2>& output) const
{
    // from http://gis.stackexchange.com/questions/53617/how-to-find-lat-lon-values-for-every-pixel-in-a-geotiff-file
    double c = m_forward_transform[0];
    double a = m_forward_transform[1];
    double b = m_forward_transform[2];
    double f = m_forward_transform[3];
    double d = m_forward_transform[4];
    double e = m_forward_transform[5];

    //ABELL - Not sure why this is right.  You can think of this like:
    //   output[0] = a * (col + .5) + b * (row + .5) + c;
    //   output[1] = d * (col + .5) + e * (row + .5) + f;
    //   Is there some reason why you want to "move" the points in the raster
    //   to a location between the rows/columns?  Seems that you would just
    //   use 'c' and 'f' to shift everything a half-row and half-column if
    //   that's what you wanted.
    //   Also, this isn't what GDALApplyGeoTransform does.  And why aren't
    //   we just calling GDALApplyGeoTransform?
    output[0] = a*col + b*row + a*0.5 + b*0.5 + c;
    output[1] = d*col + e*row + d*0.5 + e*0.5 + f;
}

// Determines the pixel/line position given an x/y.
// No reprojection is done at this time.
bool Raster::getPixelAndLinePosition(double x, double y,
    int32_t& pixel, int32_t& line)
{
    pixel = (int32_t)std::floor(m_inverse_transform[0] +
        (m_inverse_transform[1] * x) + (m_inverse_transform[2] * y));
    line = (int32_t) std::floor(m_inverse_transform[3] +
        (m_inverse_transform[4] * x) + (m_inverse_transform[5] * y));

    // Return false if we're out of bounds.
    return (pixel >= 0 && pixel < m_raster_x_size &&
        line >= 0 && line < m_raster_y_size);
}

Dimension::Type::Enum convertGDALtoPDAL(GDALDataType t)
{
    using namespace Dimension::Type;
    switch (t)
    {
        case GDT_Byte:
            return Unsigned8;
        case GDT_UInt16:
            return Unsigned16;
        case GDT_Int16:
            return Signed16;
        case GDT_UInt32:
            return Unsigned32;
        case GDT_Int32:
            return Signed32;
        case GDT_CFloat32:
            return Float;
        case GDT_CFloat64:
            return Double;
        default:
            return None;
    }
    return None;
}

GDALError::Enum Raster::readBand(std::vector<uint8_t>& data, int nBand)
{
    GDALError::Enum error = GDALError::None;

    data.resize(m_raster_x_size * m_raster_y_size);
    GDALRasterBandH band = GDALGetRasterBand(m_ds, nBand);
    if (!band)
    {
        std::ostringstream oss;
        oss <<  "Unable to get band " << nBand <<  " from raster data "
            "source '" << m_filename << "'.";
        m_errorMsg = oss.str();
        return GDALError::InvalidBand;
    }

    int nXBlockSize(0);
    int nYBlockSize(0);

    GDALGetBlockSize(band, &nXBlockSize, &nYBlockSize);

    int nXBlocks = (GDALGetRasterBandXSize(band) + nXBlockSize - 1) /
        nXBlockSize;
    int nYBlocks = (GDALGetRasterBandYSize(band) + nYBlockSize - 1) /
        nYBlockSize;

    for (int iYBlock = 0; iYBlock < nYBlocks; iYBlock++)
    {
        int nXValid(0); int nYValid(0);
        for (int iXBlock = 0; iXBlock < nXBlocks; iXBlock++)
        {

             if ((iXBlock+1) * nXBlockSize > GDALGetRasterBandXSize(band))
                 nXValid = GDALGetRasterBandXSize(band) - iXBlock * nXBlockSize;
             else
                 nXValid = nXBlockSize;
             if ((iYBlock+1) * nYBlockSize > GDALGetRasterBandYSize(band))
                 nYValid = GDALGetRasterBandYSize(band) - iYBlock * nYBlockSize;
             else
                 nYValid = nYBlockSize;

            int offset = iXBlock * (nXValid * nYValid) +
                iYBlock * (nXValid * nYValid);

            if (GDALReadBlock(band, iXBlock, iYBlock, data.data() + offset) !=
                CPLE_None)
            {
                error = GDALError::CantReadBlock;
                std::ostringstream oss;
                oss << "Unable to read block for (" << iXBlock << ", " <<
                    iYBlock << ") for raster '" << m_filename << "'.";
                m_errorMsg = oss.str();
            }
        }
    }
    return error;
}


GDALError::Enum Raster::computePDALDimensionTypes()
{
    if (!m_ds)
        return GDALError::NotOpen;

    m_types.clear();
    for (int i=0; i < m_band_count; ++i)
    {
        GDALRasterBandH band = GDALGetRasterBand(m_ds, i+1);
        if (!band)
        {
            std::ostringstream oss;

            oss << "Unable to get band " << (i + 1) <<
                " from raster data source '" << m_filename << "'.";
            m_errorMsg = oss.str();
            return GDALError::InvalidBand;
        }

        GDALDataType t = GDALGetRasterDataType(band);
        int x(0), y(0);
        GDALGetBlockSize(band, &x, &y);
        m_types.push_back(convertGDALtoPDAL(t));
    }
    return GDALError::None;
}


GDALError::Enum Raster::read(double x, double y, std::vector<double>& data)
{
    if (!m_ds)
        return GDALError::NotOpen;

    int32_t pixel(0);
    int32_t line(0);
    data.resize(m_band_count);

    std::array<double, 2> pix = { {0.0, 0.0} };

    // No data at this x,y if we can't compute a pixel/line location
    // for it.
    if (!getPixelAndLinePosition(x, y, pixel, line))
        return GDALError::NoData;

    for (int i=0; i < m_band_count; ++i)
    {
        GDALRasterBandH b = GDALGetRasterBand(m_ds, i+1);
        if (GDALRasterIO(b, GF_Read, pixel, line, 1, 1,
            &pix[0], 1, 1, GDT_CFloat64, 0, 0) == CE_None)
        {
            // we read a pixel put its values in our vector
            data[i] = pix[0];
        }
    }

    return GDALError::None;
}


SpatialReference Raster::getSpatialRef() const
{
    SpatialReference srs;

    if (m_ds)
        srs = SpatialReference(GDALGetProjectionRef(m_ds));
    return srs;
}


Raster::~Raster()
{
    close();
}


void Raster::close()
{
    if (m_ds != 0)
    {
        GDALClose(m_ds);
        m_ds = 0;
    }
    m_types.clear();
}

} // namespace gdal


std::string transformWkt(std::string wkt, const SpatialReference& from,
    const SpatialReference& to)
{
    //ABELL - Should this throw?  Return empty string?
    if (from.empty() || to.empty())
        return wkt;

    gdal::SpatialRef fromRef(from.getWKT());
    gdal::SpatialRef toRef(to.getWKT());
    gdal::Geometry geom(wkt, fromRef);
    geom.transform(toRef);
    return geom.wkt();
}

} // namespace pdal

