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
        m_gdal_callback = std::bind(&ErrorHandler::log, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    }
    else
    {
        m_gdal_callback = std::bind(&ErrorHandler::error, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
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
    , m_raster_x_size(0)
    , m_raster_y_size(0)
    , m_block_x(0)
    , m_block_y(0)
    , m_size(0)
    , m_band_count(0)
    , m_ds(0)

{
    m_forward_transform.fill(0.0);
    m_inverse_transform.fill(0.0);
}

bool Raster::open()
{
    if (m_ds)
        return true; // already open

    m_ds = GDALOpen(m_filename.c_str(), GA_ReadOnly);
    if (m_ds == NULL)
        throw pdal_error("Unable to open GDAL datasource!");

    if (GDALGetGeoTransform(m_ds, &(m_forward_transform.front())) != CE_None)
        throw pdal_error("unable to fetch forward geotransform for raster!");

    if (!GDALInvGeoTransform(&(m_forward_transform.front()),
        &(m_inverse_transform.front())))
        throw pdal_error("unable to fetch inverse geotransform for raster!");

    m_raster_x_size = GDALGetRasterXSize(m_ds);
    m_raster_y_size = GDALGetRasterYSize(m_ds);
    m_band_count = GDALGetRasterCount(m_ds);

    m_types = computePDALDimensionTypes();
    m_size = 0;
    for(auto t: m_types)
    {
        m_size += pdal::Dimension::size(t);
    }
    return true;
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

    output[0] = a*col + b*row + a*0.5 + b*0.5 + c;
    output[1] = d*col + e*row + d*0.5 + e*0.5 + f;
}

// Determines the pixel/line position given an x/y.
// No reprojection is done at this time.
bool Raster::getPixelAndLinePosition(double x, double y,
                                     std::array<double, 6> const& inverse,
                                    int32_t& pixel, int32_t& line)
{
    pixel = (int32_t)std::floor(inverse[0] + (inverse[1] * x) +
        (inverse[2] * y));
    line = (int32_t) std::floor(inverse[3] + (inverse[4] * x) +
        (inverse[5] * y));

    int xs = m_raster_x_size;
    int ys = m_raster_y_size;

    if (!xs || !ys)
        throw pdal_error("Unable to get X or Y size from raster!");

    if (pixel < 0 || line < 0 || pixel >= xs || line  >= ys)
    {
        // The x, y is not coincident with this raster
        return false;
    }

    return true;
}

pdal::Dimension::Type::Enum convertGDALtoPDAL(GDALDataType t)
{

    using namespace pdal::Dimension::Type;
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

bool Raster::readBand(std::vector<uint8_t>& data, int nBand)
{
    data.resize(m_raster_x_size * m_raster_y_size);

    GDALRasterBandH band = GDALGetRasterBand(m_ds, nBand);
    if (!band)
    {
        std::ostringstream oss;
        oss << "Unable to get band " << nBand <<
            " from data source!";
        throw pdal_error(oss.str());
    }

    int nXBlockSize(0);
    int nYBlockSize(0);

    GDALGetBlockSize(band, &nXBlockSize, &nYBlockSize);

    int nXBlocks = (GDALGetRasterBandXSize(band) + nXBlockSize - 1) / nXBlockSize;
    int nYBlocks = (GDALGetRasterBandYSize(band) + nYBlockSize - 1) / nYBlockSize;

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

            int offset = iXBlock * (nXValid * nYValid) + iYBlock * (nXValid * nYValid);

            CPLErr err = GDALReadBlock(band, iXBlock, iYBlock, data.data() + offset);
            if (err != CPLE_None)
            {
                std::ostringstream oss;
                oss << "unable to read block for ("<<iXBlock <<","<< iYBlock <<")";
                throw pdal::pdal_error(oss.str());
            }

        }

    }
    return true;
}

std::vector<std::array<int, 2>> Raster::fetchGDALBlockSizes() const
{
    std::vector<std::array<int, 2>> output;
    for (int i=1; i < m_band_count; ++i)
    {
        GDALRasterBandH band = GDALGetRasterBand(m_ds, i);
        if (!band)
        {
            std::ostringstream oss;
            oss << "Unable to get band " << i <<
                " from data source!";
            throw pdal_error(oss.str());
        }
        int x(0), y(0);
        GDALGetBlockSize(band, &x, &y);
        std::array<int, 2> a;
        a[0] = x;
        a[1] = y;
        output.push_back(a);
    }


    return output;
}

std::vector<pdal::Dimension::Type::Enum> Raster::computePDALDimensionTypes() const
{

    if (!m_ds) throw pdal::pdal_error("raster is not open!");

    std::vector<pdal::Dimension::Type::Enum> output;
    for (int i=0; i < m_band_count; ++i)
    {
        GDALRasterBandH band = GDALGetRasterBand(m_ds, i+1);
        if (!band)
        {
            std::ostringstream oss;
            oss << "Unable to get band " << i+1 <<
                " from data source!";
            throw pdal_error(oss.str());
        }

        GDALDataType t = GDALGetRasterDataType(band);
        int x(0), y(0);
        GDALGetBlockSize(band, &x, &y);
        pdal::Dimension::Type::Enum ptype = convertGDALtoPDAL(t);

        output.push_back(ptype);
    }
    return output;
}


bool Raster::read(double x, double y, std::vector<double>& data)
{

    if (!m_ds)
        throw pdal::pdal_error("Unable to read() because raster data source is not open");

    int32_t pixel(0);
    int32_t line(0);
    data.resize(m_band_count);

    std::array<double, 2> pix = { {0.0, 0.0} };

    // No data at this x,y if we can't compute a pixel/line location
    // for it.
    if (!getPixelAndLinePosition(x, y, m_inverse_transform, pixel, line))
        return false;

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

    return true;
}

SpatialReference Raster::getSpatialRef() const
{
    if (!m_ds)
        throw pdal::pdal_error("Unable to getSpatialRef() because raster data source is not open");

    const char* wkt = GDALGetProjectionRef(m_ds);

    SpatialReference r(wkt);
    return r;

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
    m_size = 0;
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

