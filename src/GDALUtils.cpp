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

ErrorHandler ErrorHandler::m_instance;

ErrorHandler& ErrorHandler::get()
{
    return m_instance;
}


ErrorHandler::ErrorHandler() : m_throw(true), m_errorNum(0)
{
    std::string value;

    auto cb = [](::CPLErr level, int num, const char *msg)
    {
        ErrorHandler::get().handle(level, num, msg);
    };
    m_cplSet = (Utils::getenv("CPL_DEBUG", value) == 0);
    m_debug = m_cplSet;
    CPLPushErrorHandler(cb);
}


ErrorHandler::~ErrorHandler()
{
    CPLPopErrorHandler();
}


void ErrorHandler::set(LogPtr log, bool debug, bool doThrow)
{
    setLog(log);
    setDebug(debug);
    setThrow(doThrow);
}


void ErrorHandler::set(LogPtr log, bool debug)
{
    setLog(log);
    setDebug(debug);
}


void ErrorHandler::setLog(LogPtr log)
{
    m_log = log;
}


void ErrorHandler::setDebug(bool debug)
{
    m_debug = debug;
    if (!m_cplSet)
    {
        if (debug)
            Utils::setenv("CPL_DEBUG", "ON");
        else
            Utils::unsetenv("CPL_DEBUG");
    }
}


void ErrorHandler::setThrow(bool doThrow)
{
    m_throw = doThrow;
}


bool ErrorHandler::willThrow() const
{
    return m_throw;
}


int ErrorHandler::errorNum()
{
    int errorNum = m_errorNum;
    m_errorNum = 0;
    return errorNum;
}


void ErrorHandler::handle(::CPLErr level, int num, char const* msg)
{
    std::ostringstream oss;

    m_errorNum = num;
    if (level == CE_Failure || level == CE_Fatal)
    {
        oss << "GDAL failure (" << num << ") " << msg;
        if (m_throw)
            throw pdal_error(oss.str());
        else if (m_log)
            m_log->get(LogLevel::Error) << oss.str() << std::endl;
    }
    else if (m_debug && level == CE_Debug)
    {
        oss << "GDAL debug: " << msg;
        if (m_log)
            m_log->get(LogLevel::Debug) << oss.str() << std::endl;
    }
}


struct InvalidBand {};
struct CantReadBlock {};

/*
  Reads a GDAL band into a vector.
*/
class BandReader
{
public:
    /*
      Constructor that populates various band information.

      \param ds  GDAL dataset handle.
      \param bandNum  Band number.  Band numbers start at 1.
    */
    BandReader(GDALDatasetH ds, int bandNum);

    /*
      Read the band into the vector.  Reads a block at a time.  Each
      block is either fully populated with data or a partial block.
      Partial blocks appear at the X and Y margins when the total size in
      the doesn't divide evenly by the block size for both the X and Y
      dimensions.

      \ptData Vector into which the data should be read.  The vector is
        resized as necessary.
    */
    void read(std::vector<uint8_t>& ptData);
private:
    GDALDatasetH m_ds;  /// Dataset handle
    int m_bandNum;  /// Band number.  Band numbers start at 1.
    GDALRasterBandH m_band;  /// Band handle
    int m_xTotalSize, m_yTotalSize;  /// Total size (x and y) of the raster
    int m_xBlockSize, m_yBlockSize;  /// Size (x and y) of blocks
    int m_xBlockCnt, m_yBlockCnt;    /// Number of blocks in each direction
    size_t m_eltSize;                /// Size in bytes of each band element.
    std::vector<uint8_t> m_buf;      /// Block read buffer.

    /*
      Read a block's worth of data.

      \param x  X coordinate of block to read.
      \param y  Y coordinate of block to read.
      \param data  Pointer to vector in which to store data.  Vector must
        be sufficiently sized to hold all data.
    */
    void readBlock(int x, int y, uint8_t *data);
};


/*
  Create a band reader for a single bad of a GDAL dataset.

  \param ds  GDAL dataset handle.
  \param bandNum  Band number (1-indexed).
*/
BandReader::BandReader(GDALDatasetH ds, int bandNum) : m_ds(ds),
    m_bandNum(bandNum), m_xBlockSize(0), m_yBlockSize(0)
{
    m_band = GDALGetRasterBand(m_ds, m_bandNum);
    if (!m_band)
        throw InvalidBand();

    // Perhaps raster bands can have different sizes than the raster itself?
    m_xTotalSize = GDALGetRasterBandXSize(m_band);
    m_yTotalSize = GDALGetRasterBandYSize(m_band);

    GDALGetBlockSize(m_band, &m_xBlockSize, &m_yBlockSize);

    m_xBlockCnt = ((m_xTotalSize - 1) / m_xBlockSize) + 1;
    m_yBlockCnt = ((m_yTotalSize - 1) / m_yBlockSize) + 1;
}


/*
  Read a raster band into an array.

  \param ptData  Vector to contain raster (Y major - X varies fastest).
    ptData is resized to fit data in the band.
*/
void BandReader::read(std::vector<uint8_t>& ptData)
{
    GDALDataType t = GDALGetRasterDataType(m_band);
    m_eltSize = GDALGetDataTypeSize(t) / CHAR_BIT;
    m_buf.resize(m_xBlockSize * m_yBlockSize * m_eltSize);
    ptData.resize(m_xTotalSize * m_yTotalSize * m_eltSize);

    uint8_t *data = ptData.data();
    for (int y = 0; y < m_yBlockCnt; ++y)
        for (int x = 0; x < m_xBlockCnt; ++x)
            readBlock(x, y, data);
}


/*
  Read a block's worth of data.

  Read data into a block-sized buffer.  Then copy data from the block buffer
  into the destination array at the proper location to build a complete
  raster.

  \param x  X coordinate of the block to read.
  \param y  Y coordinate of the block to read.
  \param data  Pointer to the data vector that contains the raster information.
*/
void BandReader::readBlock(int x, int y, uint8_t *data)
{
    if (GDALReadBlock(m_band, x, y, m_buf.data()) != CPLE_None)
        throw CantReadBlock();

    int xWidth = 0;
    if (x == m_xBlockCnt - 1)
        xWidth = m_xTotalSize % m_xBlockSize;
    if (xWidth == 0)
        xWidth = m_xBlockSize;

    int yHeight = 0;
    if (y == m_yBlockCnt - 1)
        yHeight = m_yTotalSize % m_yBlockSize;
    if (yHeight == 0)
        yHeight = m_yBlockSize;

    uint8_t *bp = m_buf.data();
    // Go through rows copying data.  Increment the buffer pointer by the
    // width of the row.
    for (int row = 0; row < yHeight; ++row)
    {
        int wholeRows = m_xTotalSize * ((y * m_yBlockSize) + row);
        int partialRows = m_xBlockSize * x;
        uint8_t *dp = data + ((wholeRows + partialRows) * m_eltSize);
        std::copy(bp, bp + (xWidth * m_eltSize), dp);

        // Blocks are always full-sized, even if only some of the data is valid,
        // so we use m_xBlockSize instead of xWidth.
        bp += (m_xBlockSize * m_eltSize);
    }
}


Raster::Raster(const std::string& filename)
    : m_filename(filename)
    , m_raster_x_size(0)
    , m_raster_y_size(0)
    , m_band_count(0)
    , m_ds(0)
{
    m_forward_transform.fill(0);
    m_forward_transform[1] = 1;
    m_forward_transform[5] = 1;
    m_inverse_transform.fill(0);
    m_inverse_transform[1] = 1;
    m_inverse_transform[5] = 1;
}


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

    for (int i = 0; i < m_band_count; ++i)
    {
        int fail = 0;
        GDALRasterBandH band = GDALGetRasterBand(m_ds, i + 1);
        double v = GDALGetRasterNoDataValue(band, &fail);
    }
    if (computePDALDimensionTypes() == GDALError::InvalidBand)
        error = GDALError::InvalidBand;
    return error;
}


void Raster::pixelToCoord(int col, int row, std::array<double, 2>& output) const
{
    /**
    double *xform = const_cast<double *>(m_forward_transform.data());
    GDALApplyGeoTransform(xform, col, row, &output[0], &output[1]);
    **/

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
        case GDT_Float32:
            return Float;
        case GDT_Float64:
            return Double;
        case GDT_CInt16:
        case GDT_CInt32:
        case GDT_CFloat32:
        case GDT_CFloat64:
            throw pdal_error("GDAL complex float type unsupported.");
        case GDT_Unknown:
            throw pdal_error("GDAL unknown type unsupported.");
        case GDT_TypeCount:
            throw pdal_error("Detected bad GDAL data type.");
    }
    return None;
}


GDALError::Enum Raster::readBand(std::vector<uint8_t>& points, int nBand)
{
    try 
    {
        BandReader(m_ds, nBand).read(points);
    }
    catch (InvalidBand)
    {
        std::stringstream oss;
        oss << "Unable to get band " << nBand << " from raster '" <<
            m_filename << "'.";
        m_errorMsg = oss.str();
        return GDALError::InvalidBand;
    }
    catch (CantReadBlock)
    {
        std::ostringstream oss;
        oss << "Unable to read block for for raster '" << m_filename << "'.";
        m_errorMsg = oss.str();
        return GDALError::CantReadBlock;
    }
    return GDALError::None;
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

