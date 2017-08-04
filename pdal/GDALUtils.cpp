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

#include <ogr_spatialref.h>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{
namespace gdal
{

namespace
{

Dimension::Type toPdalType(GDALDataType t)
{
    switch (t)
    {
        case GDT_Byte:
            return Dimension::Type::Unsigned8;
        case GDT_UInt16:
            return Dimension::Type::Unsigned16;
        case GDT_Int16:
            return Dimension::Type::Signed16;
        case GDT_UInt32:
            return Dimension::Type::Unsigned32;
        case GDT_Int32:
            return Dimension::Type::Signed32;
        case GDT_Float32:
            return Dimension::Type::Float;
        case GDT_Float64:
            return Dimension::Type::Double;
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
    return Dimension::Type::None;
}

GDALDataType toGdalType(Dimension::Type t)
{
    switch (t)
    {
    case Dimension::Type::Unsigned8:
    case Dimension::Type::Signed8:
        return GDT_Byte;
    case Dimension::Type::Unsigned16:
        return GDT_UInt16;
    case Dimension::Type::Signed16:
        return GDT_Int16;
    case Dimension::Type::Unsigned32:
        return GDT_UInt32;
    case Dimension::Type::Signed32:
        return GDT_Int32;
    case Dimension::Type::Float:
        return GDT_Float32;
    case Dimension::Type::Double:
        return GDT_Float64;
    case Dimension::Type::Unsigned64:
    case Dimension::Type::Signed64:
        throw pdal_error("PDAL 64-bit integer type unsupported.");
    case Dimension::Type::None:
        throw pdal_error("PDAL 'none' type unsupported.");
	default:
        throw pdal_error("Unrecognized PDAL dimension type.");

    }
}

} //unnamed namespace

/**
  Reproject a bounds box from a source projection to a destination.
  \param box  Bounds box to be reprojected in-place.
  \param srcSrs  String in WKT or other suitable format of box coordinates.
  \param dstSrs  String in WKT or other suitable format to which
    coordinates should be projected.
  \return  Whether the reprojection was successful or not.
*/
bool reprojectBounds(BOX3D& box, const std::string& srcSrs,
    const std::string& dstSrs)
{
    OGRSpatialReference src;
    OGRSpatialReference dst;

    OGRErr srcOk = OSRSetFromUserInput(&src, srcSrs.c_str());
    OGRErr dstOk = OSRSetFromUserInput(&dst, dstSrs.c_str());
    if (srcOk != OGRERR_NONE || dstOk != OGRERR_NONE)
        return false;

    OGRCoordinateTransformationH transform =
        OCTNewCoordinateTransformation(&src, &dst);

    bool ok = (OCTTransform(transform, 1, &box.minx, &box.miny, &box.minz) &&
        OCTTransform(transform, 1, &box.maxx, &box.maxy, &box.maxz));
    OCTDestroyCoordinateTransformation(transform);
    return ok;
}


/**
  Reproject a bounds box from a source projection to a destination.
  \param box  2D Bounds box to be reprojected in-place.
  \param srcSrs  String in WKT or other suitable format of box coordinates.
  \param dstSrs  String in WKT or other suitable format to which
    coordinates should be projected.
  \return  Whether the reprojection was successful or not.
*/
bool reprojectBounds(BOX2D& box, const std::string& srcSrs,
    const std::string& dstSrs)
{
    BOX3D b(box);
    bool res = reprojectBounds(b, srcSrs, dstSrs);
    box = b.to2d();
    return res;
}


/**
  Reproject a point from a source projection to a destination.
  \param x  X coordinate of point to be reprojected.
  \param y  Y coordinate of point to be reprojected.
  \param z  Z coordinate of point to be reprojected.
  \param srcSrs  String in WKT or other suitable format of box coordinates.
  \param dstSrs  String in WKT or other suitable format to which
    coordinates should be projected.
  \return  Whether the reprojection was successful or not.
*/
bool reprojectPoint(double& x, double& y, double& z, const std::string& srcSrs,
    const std::string& dstSrs)
{
    OGRSpatialReference src;
    OGRSpatialReference dst;

    OGRErr srcOk = OSRSetFromUserInput(&src, srcSrs.c_str());
    OGRErr dstOk = OSRSetFromUserInput(&dst, dstSrs.c_str());
    if (srcOk != OGRERR_NONE || dstOk != OGRERR_NONE)
        return false;

    OGRCoordinateTransformationH transform =
        OCTNewCoordinateTransformation(&src, &dst);

    bool ok = (OCTTransform(transform, 1, &x, &y, &z));
    OCTDestroyCoordinateTransformation(transform);
    return ok;
}


std::string lastError()
{
    return CPLGetLastErrorMsg();
}


static ErrorHandler* s_gdalErrorHandler= 0;

void registerDrivers()
{
    static std::once_flag flag;

    auto init = []() -> void
    {
        GDALAllRegister();
        OGRRegisterAll();
    };

    std::call_once(flag, init);
}


void unregisterDrivers()
{
    GDALDestroyDriverManager();
}


ErrorHandler& ErrorHandler::getGlobalErrorHandler()
{
    static std::once_flag flag;

    auto init = []()
    {
       s_gdalErrorHandler = new ErrorHandler();
    };

    std::call_once(flag, init);
    return *s_gdalErrorHandler;
}

ErrorHandler::ErrorHandler() : m_errorNum(0)
{
    std::string value;

    // Will return thread-local setting
    const char* set = CPLGetConfigOption("CPL_DEBUG", "");
    m_cplSet = (bool)set ;
    m_debug = m_cplSet;

    // Push on a thread-local error handler
    CPLSetErrorHandler(&ErrorHandler::trampoline);
}


void ErrorHandler::set(LogPtr log, bool debug)
{
    setLog(log);
    setDebug(debug);
}


void ErrorHandler::setLog(LogPtr log)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_log = log;
}


void ErrorHandler::setDebug(bool debug)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_debug = debug;

    if (debug)
        CPLSetThreadLocalConfigOption("CPL_DEBUG", "ON");
    else
        CPLSetThreadLocalConfigOption("CPL_DEBUG", NULL);
}

int ErrorHandler::errorNum()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_errorNum;
}

void ErrorHandler::handle(::CPLErr level, int num, char const* msg)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::ostringstream oss;

    m_errorNum = num;
    if (level == CE_Failure || level == CE_Fatal)
    {
        oss << "GDAL failure (" << num << ") " << msg;
        if (m_log)
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
struct CantWriteBlock {};

/*
  Slight abstraction of a GDAL raster band.
*/
class Band
{
public:
    /**
      Create an object for reading a band of a GDAL dataset.

      \param ds  GDAL dataset handle.
      \param bandNum  Band number (1-indexed).
    */
    Band(GDALDataset *ds, int bandNum, const std::string& name = "");

    /**
      Create an object for writing a band of data to a GDAL dataset.

      \param ds  GDAL dataset handle.
      \param width  Raster band width.
      \param height  Raster band height.
      \param bandNum  Band number in raster (1-indexed).
      \param name  Band name
    */
    /**
    Band(GDALDataset *ds, int width, int height, int bandNum,
        const std::string& name = "");
**/

    /*
      Read the band into the vector.  Reads a block at a time.  Each
      block is either fully populated with data or a partial block.
      Partial blocks appear at the X and Y margins when the total size in
      the doesn't divide evenly by the block size for both the X and Y
      dimensions.

      \param  Data Vector into which the data should be read.  The vector is
        resized as necessary.
    */
    void read(std::vector<uint8_t>& ptData);

    /*
      Write linearized data pointed to by \c data into the band.

      \param data  Pointer to beginning of band
    */
    void write(const uint8_t *data);
private:
    GDALDataset *m_ds;  /// Dataset handle
    int m_bandNum;  /// Band number.  Band numbers start at 1.
    GDALRasterBand *m_band;  /// Band handle
    int m_xTotalSize, m_yTotalSize;  /// Total size (x and y) of the raster
    int m_xBlockSize, m_yBlockSize;  /// Size (x and y) of blocks
    int m_xBlockCnt, m_yBlockCnt;    /// Number of blocks in each direction
    size_t m_eltSize;                /// Size in bytes of each band element.
    std::vector<uint8_t> m_buf;      /// Block read buffer.
    std::string m_name;              /// Band name.

    /*
      Read a block's worth of data.

      \param x  X coordinate of block to read.
      \param y  Y coordinate of block to read.
      \param data  Pointer to vector in which to store data.  Vector must
        be sufficiently sized to hold all data.
    */
    void readBlock(int x, int y, uint8_t *data);

    /**
      Write a block of a band to a raster.

      \param x  X coordinate of block.
      \param y  Y coordinate of block.
      \param data  Pointer to beginning of raster data (not block data).
    */
    void writeBlock(int x, int y, const uint8_t *data);
};


Band::Band(GDALDataset *ds, int bandNum, const std::string& name) : m_ds(ds),
    m_bandNum(bandNum), m_xBlockSize(0), m_yBlockSize(0)
{
    m_band = m_ds->GetRasterBand(m_bandNum);
    if (!m_band)
        throw InvalidBand();

    if (name.size())
    {
        m_band->SetDescription(name.data());
        // We don't care about offset, but this sets the flag to indicate
        // that the metadata has changed.
        m_band->SetOffset(m_band->GetOffset(NULL) + .00001);
        m_band->SetOffset(m_band->GetOffset(NULL) - .00001);
    }

    m_xTotalSize = m_band->GetXSize();
    m_yTotalSize = m_band->GetYSize();

    m_band->GetBlockSize(&m_xBlockSize, &m_yBlockSize);
    GDALDataType t = m_band->GetRasterDataType();
    m_eltSize = GDALGetDataTypeSize(t) / CHAR_BIT;
    m_buf.resize(m_xBlockSize * m_yBlockSize * m_eltSize);

    m_xBlockCnt = ((m_xTotalSize - 1) / m_xBlockSize) + 1;
    m_yBlockCnt = ((m_yTotalSize - 1) / m_yBlockSize) + 1;
}


void Band::read(std::vector<uint8_t>& ptData)
{
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
void Band::readBlock(int x, int y, uint8_t *data)
{
    if (m_band->ReadBlock(x, y, m_buf.data()) != CPLE_None)
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


void Band::write(const uint8_t *data)
{
    for (int y = 0; y < m_yBlockCnt; ++y)
        for (int x = 0; x < m_xBlockCnt; ++x)
            writeBlock(x, y, data);
}


void Band::writeBlock(int x, int y, const uint8_t *data)
{
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

    // Go through rows copying data.
    uint8_t *dstStart = m_buf.data();
    // Go through rows copying data.  Increment the buffer pointer by the
    // width of the row.
    for (int row = 0; row < yHeight; ++row)
    {
        // Find the offset location in the source container.
        int wholeRowElts = m_xTotalSize * ((y * m_yBlockSize) + row);
        int partialRowElts = m_xBlockSize * x;

        const uint8_t *srcStart = data +
            ((wholeRowElts + partialRowElts) * m_eltSize);
        const uint8_t *srcEnd = srcStart + (m_xBlockSize * m_eltSize);
        std::copy(srcStart, srcEnd, dstStart);

        // Blocks are always full-sized, even if only some of the data is valid,
        // so we use m_xBlockSize instead of xWidth.
        dstStart += (m_xBlockSize * m_eltSize);
    }
    if (m_band->WriteBlock(x, y, m_buf.data()) != CPLE_None)
        throw CantWriteBlock();
}

Raster::Raster(const std::string& filename, const std::string& drivername)
    : m_filename(filename)
    , m_width(0)
    , m_height(0)
    , m_numBands(0)
    , m_drivername(drivername)
    , m_ds(0)
{
    m_forwardTransform.fill(0);
    m_forwardTransform[1] = 1;
    m_forwardTransform[5] = 1;
    m_inverseTransform.fill(0);
    m_inverseTransform[1] = 1;
    m_inverseTransform[5] = 1;
}


Raster::Raster(const std::string& filename, const std::string& drivername,
    const SpatialReference& srs, const std::array<double, 6> pixelToPos)
    : m_filename(filename)
    , m_width(0)
    , m_height(0)
    , m_numBands(0)
    , m_drivername(drivername)
    , m_forwardTransform(pixelToPos)
    , m_srs(srs)
    , m_ds(0)
{}


GDALError Raster::open(int width, int height, int numBands,
    Dimension::Type type, double noData, StringList options)
{
    if (m_drivername.empty())
        m_drivername = "GTiff";

    m_width = width;
    m_height = height;
    m_numBands = numBands;

    if (!GDALInvGeoTransform(m_forwardTransform.data(),
        m_inverseTransform.data()))
    {
        m_errorMsg = "Geotransform for raster '" + m_filename + "' not "
           "invertible";
        return GDALError::NotInvertible;
    }

    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName(
        m_drivername.data());
    if (!driver)
    {
        m_errorMsg = "Driver '" + m_drivername + "' not found.";
        return GDALError::DriverNotFound;
    }

    std::string item;
    const char *itemp = driver->GetMetadataItem(GDAL_DCAP_CREATE);
    if (itemp)
        item = itemp;
    if (item != "YES")
    {
        m_errorMsg = "Requested driver '" + m_drivername + "' does not "
            "support file creation.";
        return GDALError::InvalidDriver;
    }

    std::vector<const char *> opts;

    for (size_t i = 0; i < options.size(); ++i)
    {
        if (options[i].find("INTERLEAVE") == 0)
        {
            m_errorMsg = "INTERLEAVE GDAL driver option not supported.";
            return GDALError::InvalidOption;
        }
        opts.push_back(options[i].data());
    }
    opts.push_back("INTERLEAVE=BAND");
    opts.push_back(NULL);

    m_ds = driver->Create(m_filename.data(), m_width, m_height, m_numBands,
        toGdalType(type), const_cast<char **>(opts.data()));
    if (m_ds == NULL)
    {
        m_errorMsg = "Unable to open GDAL datasource '" + m_filename + "'.";
        return GDALError::CantCreate;
    }

    if (m_srs.valid())
        m_ds->SetProjection(m_srs.getWKT().data());

    m_ds->SetGeoTransform(m_forwardTransform.data());
    for (int i = 0; i < m_numBands; ++i)
    {
        GDALRasterBand *band = m_ds->GetRasterBand(i + 1);
        band->SetNoDataValue(noData);
    }

    return GDALError::None;
}


GDALError Raster::open()
{
    GDALError error = GDALError::None;
    if (m_ds)
        return error;

#if (GDAL_VERSION_MAJOR > 1)
    const char ** driverP = NULL;
    const char *drivers[2] = {0};
    if (!m_drivername.empty())
    {
        drivers[0] = m_drivername.c_str();
        driverP = drivers;
    }

    m_ds = (GDALDataset *)GDALOpenEx(m_filename.c_str(), GA_ReadOnly, driverP,
        nullptr, nullptr);
#else
    m_ds = (GDALDataset *)GDALOpen(m_filename.c_str(), GA_ReadOnly);
#endif
    if (m_ds == NULL)
    {
        m_errorMsg = "Unable to open GDAL datasource '" + m_filename + "'.";
        return GDALError::CantOpen;
    }

    // An identity transform is returned on error.
    if (m_ds->GetGeoTransform(m_forwardTransform.data()) != CE_None)
    {
        m_errorMsg = "Unable to get geotransform for raster '" +
            m_filename + "'.";
        error = GDALError::NoTransform;
    }

    if (!GDALInvGeoTransform(m_forwardTransform.data(),
        m_inverseTransform.data()))
    {
        m_errorMsg = "Geotransform for raster '" + m_filename + "' not "
           "invertible";
        error = GDALError::NotInvertible;
    }

    m_width = m_ds->GetRasterXSize();
    m_height = m_ds->GetRasterYSize();
    m_numBands = m_ds->GetRasterCount();

    if (computePDALDimensionTypes() == GDALError::InvalidBand)
        error = GDALError::InvalidBand;
    return error;
}


GDALError Raster::readBand(std::vector<uint8_t>& points, int nBand)
{
    try
    {
        Band(m_ds, nBand).read(points);
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


GDALError Raster::writeBand(const uint8_t *data, int nBand,
    const std::string& name)
{
    try
    {
        Band(m_ds, nBand, name).write(data);
    }
    catch (CantWriteBlock)
    {
        std::ostringstream oss;
        oss << "Unable to write block for for raster '" << m_filename << "'.";
        m_errorMsg = oss.str();
        return GDALError::CantWriteBlock;
    }
    return GDALError::None;
}


void Raster::pixelToCoord(int col, int row, std::array<double, 2>& output) const
{
    /**
    double *xform = const_cast<double *>(m_forwardTransform.data());
    GDALApplyGeoTransform(xform, col, row, &output[0], &output[1]);
    **/

    // from http://gis.stackexchange.com/questions/53617/how-to-find-lat-lon-values-for-every-pixel-in-a-geotiff-file
    double c = m_forwardTransform[0];
    double a = m_forwardTransform[1];
    double b = m_forwardTransform[2];
    double f = m_forwardTransform[3];
    double d = m_forwardTransform[4];
    double e = m_forwardTransform[5];

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
    pixel = (int32_t)std::floor(m_inverseTransform[0] +
        (m_inverseTransform[1] * x) + (m_inverseTransform[2] * y));
    line = (int32_t) std::floor(m_inverseTransform[3] +
        (m_inverseTransform[4] * x) + (m_inverseTransform[5] * y));

    // Return false if we're out of bounds.
    return (pixel >= 0 && pixel < m_width &&
        line >= 0 && line < m_height);
}


/*
  Compute a vector of the PDAL datatypes that are stored in the raster
  bands of a dataset.
*/
GDALError Raster::computePDALDimensionTypes()
{
    if (!m_ds)
    {
        m_errorMsg = "Raster not open.";
        return GDALError::NotOpen;
    }

    m_types.clear();
    for (int i = 0; i < m_numBands; ++i)
    {
        // Raster bands are numbered from 1.
        GDALRasterBand *band = m_ds->GetRasterBand(i + 1);
        if (!band)
        {
            std::ostringstream oss;

            oss << "Unable to get band " << (i + 1) <<
                " from raster data source '" << m_filename << "'.";
            m_errorMsg = oss.str();
            return GDALError::InvalidBand;
        }
        m_types.push_back(toPdalType(band->GetRasterDataType()));
    }
    return GDALError::None;
}


GDALError Raster::read(double x, double y, std::vector<double>& data)
{
    if (!m_ds)
    {
        m_errorMsg = "Raster not open.";
        return GDALError::NotOpen;
    }

    int32_t pixel(0);
    int32_t line(0);
    data.resize(m_numBands);

    std::array<double, 2> pix = { {0.0, 0.0} };

    // No data at this x,y if we can't compute a pixel/line location
    // for it.
    if (!getPixelAndLinePosition(x, y, pixel, line))
    {
        m_errorMsg = "Requested location is not in the raster.";
        return GDALError::NoData;
    }

    for (int i=0; i < m_numBands; ++i)
    {
        GDALRasterBandH b = GDALGetRasterBand(m_ds, i + 1);
        if (GDALRasterIO(b, GF_Read, pixel, line, 1, 1,
            &pix[0], 1, 1, GDT_Float64, 0, 0) == CE_None)
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
        srs = SpatialReference(m_ds->GetProjectionRef());
    return srs;
}


Raster::~Raster()
{
    close();
}


void Raster::close()
{
    delete m_ds;
    m_ds = nullptr;
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

