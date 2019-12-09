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

#include <pdal/Polygon.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/private/SrsTransform.hpp>
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/Utils.hpp>

#include <functional>
#include <map>

#include "GDALUtils.hpp"

#include <ogr_spatialref.h>
#include <ogr_p.h>
#include <ogr_api.h>
#include <ogrsf_frmts.h>
#include <nlohmann/json.hpp>

#pragma warning(disable: 4127)  // conditional expression is constant


namespace pdal
{

namespace oldgdalsupport
{
	OGRErr createFromWkt(char **s, OGRGeometry **newGeom);
	OGRGeometry* createFromGeoJson(char **s);
} // namespace oldgdalsupport


namespace gdal
{

namespace
{

/**
  Convert a GDAL type string to a PDAL dimension type.

  \param gdalType  String representing the GDAL type.
  \return  PDAL type associated with \gdalType.
*/
Dimension::Type toPdalType(const std::string& gdalType)
{
    if (gdalType == "Byte")
        return Dimension::Type::Unsigned8;
    else if (gdalType == "UInt16")
        return Dimension::Type::Unsigned16;
    else if (gdalType == "Int16")
        return Dimension::Type::Signed16;
    else if (gdalType == "UInt32")
        return Dimension::Type::Unsigned32;
    else if (gdalType == "Int32")
        return Dimension::Type::Signed32;
    else if (gdalType == "Float32")
        return Dimension::Type::Float;
    else if (gdalType == "Float64")
        return Dimension::Type::Double;
    return Dimension::Type::None;
}


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
  Reproject a point from a source projection to a destination.
  \param x  X coordinate of point to be reprojected in-place.
  \param y  Y coordinate of point to be reprojected in-place.
  \param z  Z coordinate of point to be reprojected in-place.
  \param srcSrs  String in WKT or other suitable format of box coordinates.
  \param dstSrs  String in WKT or other suitable format to which
    coordinates should be projected.
  \return  Whether the reprojection was successful or not.
*/
bool reproject(double& x, double& y, double& z, const std::string& srcSrs,
    const std::string& dstSrs)
{
    return SrsTransform(srcSrs, dstSrs).transform(x, y, z);
}


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
    SrsTransform transform(srcSrs, dstSrs);

    bool ok = transform.transform(box.minx, box.miny, box.minz);
    if (ok)
        ok = transform.transform(box.maxx, box.maxy, box.maxz);
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


std::string lastError()
{
    return CPLGetLastErrorMsg();
}


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
    static ErrorHandler s_gdalErrorHandler;

    return s_gdalErrorHandler;
}

ErrorHandler::ErrorHandler() : m_errorNum(0)
{
    std::string value;

    // Will return thread-local setting
    const char* set = CPLGetConfigOption("CPL_DEBUG", "");
    m_cplSet = (bool)set;
    m_debug = m_cplSet;

    // Push on a thread-local error handler
    CPLSetErrorHandler(&ErrorHandler::trampoline);
}


ErrorHandler::~ErrorHandler()
{
    CPLSetErrorHandler(nullptr);
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

Raster* Raster::memoryCopy() const
{

    GDALDriver *mem= GetGDALDriverManager()->GetDriverByName( "MEM");
    if (!mem)
    {
        return nullptr;
    }

    if (!m_ds)
        throw pdal::pdal_error("driver is not open!");

    GDALDataset* mem_ds = mem->CreateCopy("", m_ds, FALSE, nullptr, nullptr, nullptr);

    Raster* r = new Raster(mem_ds);
    r->wake();
    return r;
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
    m_bandType = type;
    m_dstNoData = noData;

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

    GDALError error = validateType(type, driver);
    if (error != GDALError::None)
        return error;

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
    // If the nodata value is NaN, set a default based on type.
    if (std::isnan(m_dstNoData))
    {
        switch (type)
        {
        case Dimension::Type::Unsigned8:
            m_dstNoData = 255;
            break;
        case Dimension::Type::Signed8:
            m_dstNoData = -127;
            break;
        case Dimension::Type::Unsigned16:
        case Dimension::Type::Unsigned32:
            m_dstNoData = 9999;
            break;
        default:
            m_dstNoData = -9999;
            break;
        }
    }

    for (int i = 0; i < m_numBands; ++i)
    {
        GDALRasterBand *band = m_ds->GetRasterBand(i + 1);
        band->SetNoDataValue(m_dstNoData);
    }

    return GDALError::None;
}




GDALError Raster::open()
{
    GDALError error = GDALError::None;
    if (m_ds)
        return error;

    const char ** driverP = NULL;
    const char *drivers[2] = {0};
    if (!m_drivername.empty())
    {
        drivers[0] = m_drivername.c_str();
        driverP = drivers;
    }

    m_ds = (GDALDataset *)GDALOpenEx(m_filename.c_str(), GA_ReadOnly, driverP,
        nullptr, nullptr);
    error = wake();
    return error;
}

GDALError Raster::wake()
{
    GDALError error = GDALError::None;
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


/**
  \param type    Reqested type of the raster.
  \param driver  Pointer to the GDAL driver being used to write the raster.
  \return  Requested type, or if not supported, the preferred type to use
      for the raster.
*/
GDALError Raster::validateType(Dimension::Type& type,
    GDALDriver *driver)
{
    // Convert the string of supported GDAL types to a vector of PDAL types,
    // ignoring types that aren't supported by PDAL (mostly complex values).
    std::vector<Dimension::Type> types;
    const char *itemp = driver->GetMetadataItem(GDAL_DMD_CREATIONDATATYPES);
    if (itemp)
    {
        StringList items = Utils::split2(std::string(itemp), ' ');
        for (auto& i : items)
        {
            Dimension::Type t = toPdalType(i);
            if (t != Dimension::Type::None)
                types.push_back(t);
        }
    }

    // If requested type is not supported, return an error.
    if (type != Dimension::Type::None && !Utils::contains(types, type))
    {
        m_errorMsg = "Requested type '" + Dimension::interpretationName(type) +
            "' not supported by GDAL driver '" + m_drivername + "'.";
        return GDALError::InvalidType;
    }

    // If no type is requested, take the "largest" one.
    if (type == Dimension::Type::None)
    {
        std::sort(types.begin(), types.end());
        type = types.back();
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
    GDALClose(m_ds);
    m_ds = nullptr;
    m_types.clear();
}


OGRGeometry *createFromWkt(const char *s)
{
    OGRGeometry *newGeom;
#if ((GDAL_VERSION_MAJOR == 2) && GDAL_VERSION_MINOR < 3)
    char *cs = const_cast<char *>(s);
    oldgdalsupport::createFromWkt(&cs, &newGeom);
#else
    OGRGeometryFactory::createFromWkt(s, nullptr, &newGeom);
#endif
    return newGeom;
}


OGRGeometry *createFromWkt(const std::string& s, std::string& srs)
{
    OGRGeometry *newGeom;
	char *buf = const_cast<char *>(s.data());
#if ((GDAL_VERSION_MAJOR == 2) && GDAL_VERSION_MINOR < 3)
    oldgdalsupport::createFromWkt(&buf, &newGeom);
#else
    OGRGeometryFactory::createFromWkt(&buf, nullptr, &newGeom);
    if (!newGeom)
        throw pdal_error("Couldn't convert WKT string to geometry.");
    srs = buf;
#endif

	std::string::size_type pos = 0;
	pos = Utils::extractSpaces(srs, pos);
	if (pos == srs.size())
		srs.clear();
    else
    {
        if (srs[pos++] != '/')
            throw pdal_error("Invalid character following valid geometry.");
        pos += Utils::extractSpaces(srs, pos);
        srs = srs.substr(pos);
    }

    return newGeom;
}

OGRGeometry *createFromGeoJson(const char *s)
{
#if ((GDAL_VERSION_MAJOR == 2) && GDAL_VERSION_MINOR < 3)
    char* p = const_cast<char*>(s);
    return oldgdalsupport::createFromGeoJson((char**)&p);
#else
    return OGRGeometryFactory::createFromGeoJson(s);
#endif
}

OGRGeometry *createFromGeoJson(const std::string& s, std::string& srs)
{
// Call this function instead after we've past supporting GDAL 2.2
//    return OGRGeometryFactory::createFromGeoJson(s);

    char *cs = const_cast<char *>(s.data());
    OGRGeometry *newGeom = oldgdalsupport::createFromGeoJson(&cs);
    if (!newGeom)
        throw pdal_error("Couldn't convert GeoJSON to geometry.");
    srs = cs;

	std::string::size_type pos = 0;
	pos = Utils::extractSpaces(srs, pos);
	if (pos == srs.size())
		srs.clear();
    else
    {
        if (srs[pos++] != '/')
            throw pdal_error("Invalid character following valid geometry.");
        pos += Utils::extractSpaces(srs, pos);
        srs = srs.substr(pos);
    }
    return newGeom;
}


std::vector<Polygon> getPolygons(const NL::json& ogr)
{
    registerDrivers();
    const NL::json& datasource = ogr.at("datasource");

    char** papszDriverOptions = nullptr;
    if (ogr.count("drivers"))
    {

        const NL::json& dops = ogr.at("drivers");
        std::vector<std::string> driverOptions = dops.get<std::vector<std::string>>();
        for(const auto& s: driverOptions)
            papszDriverOptions = CSLAddString(papszDriverOptions, s.c_str());
    }
    std::vector<const char*> openoptions{};

    char** papszOpenOptions = nullptr;
    if (ogr.count("openoptions"))
    {

        const NL::json& oops = ogr.at("openoptions");
        std::vector<std::string> openOptions = oops.get<std::vector<std::string>>();
        for(const auto& s: openOptions)
            papszOpenOptions = CSLAddString(papszOpenOptions, s.c_str());
    }

    std::string dsString = datasource.get<std::string>();
    unsigned int openFlags = GDAL_OF_READONLY | GDAL_OF_VECTOR | GDAL_OF_VERBOSE_ERROR;
    GDALDataset* ds = (GDALDataset*) GDALOpenEx( dsString.c_str(),
                                                 openFlags,
                                                 papszDriverOptions, papszOpenOptions, NULL );
    CSLDestroy(papszDriverOptions);
    CSLDestroy(papszOpenOptions);
    if (!ds)
        throw pdal_error("Unable to read datasource in fetchOGRGeometries " + datasource.dump() );

    OGRLayer* poLayer(nullptr);

    if (ogr.count("layer"))
    {
        const NL::json& layer = ogr.at("layer");
        std::string lyrString = layer.get<std::string>();
        poLayer = ds->GetLayerByName( lyrString.c_str() );

        if (!poLayer)
            throw pdal_error("Unable to read layer in fetchOGRGeometries " + layer.dump() );
    }

    OGRFeature *poFeature (nullptr);
    OGRGeometry* filterGeometry(nullptr);
    std::string dialect("OGRSQL");

    if (ogr.count("sql"))
    {

        const NL::json sql = ogr.at("sql");

        if (ogr.count("options"))
        {
            const NL::json options = ogr.at("options");
            if (options.count("dialect"))
                dialect = options.at("dialect").get<std::string>();

            if (options.count("geometry"))
            {
                std::string wkt_or_json = options.at("geometry").get<std::string>();
                bool isJson = (wkt_or_json.find("{") != wkt_or_json.npos) ||
                              (wkt_or_json.find("}") != wkt_or_json.npos);

                std::string srs;
                if (isJson)
                {
                    filterGeometry = gdal::createFromGeoJson(wkt_or_json, srs);
                    if (!filterGeometry)
                        throw pdal_error("Unable to create filter geometry from input GeoJSON");
                }
                else
                {
                    filterGeometry = gdal::createFromWkt(wkt_or_json, srs);
                    if (!filterGeometry)
                        throw pdal_error("Unable to create filter geometry from input WKT");
                }
                filterGeometry->assignSpatialReference(
                    new OGRSpatialReference(SpatialReference(srs).getWKT().data()));

            }
        }

        std::string query = sql.get<std::string>();

        // execute the query to get the SRS without
        // the filterGeometry, assign it to the filterGeometry,
        // and then query again.
        if (filterGeometry)
        {
            poLayer = ds->ExecuteSQL( query.c_str(),
                                      NULL,
                                      dialect.c_str());
            if (!poLayer)
                throw pdal_error("unable to execute sql query!");

            filterGeometry->transformTo(poLayer->GetSpatialRef());
        }

        poLayer = ds->ExecuteSQL( query.c_str(),
                                  filterGeometry,
                                  dialect.c_str());
        if (!poLayer)
            throw pdal_error("unable to execute sql query!");
    }

    std::vector<Polygon> polys;
    while ((poFeature = poLayer->GetNextFeature()) != NULL)
    {
        polys.emplace_back(poFeature->GetGeometryRef());
        OGRFeature::DestroyFeature( poFeature );
    }
    return polys;
}

} // namespace gdal

namespace oldgdalsupport
{

#if (GDAL_VERSION_MAJOR == 2) && (GDAL_VERSION_MINOR < 3)
OGRErr createFromWkt(char **s, OGRGeometry **ppoReturn )
{
    const char *pszInput = *s;
    *ppoReturn = nullptr;

/* -------------------------------------------------------------------- */
/*      Get the first token, which should be the geometry type.         */
/* -------------------------------------------------------------------- */
    char szToken[1000] = {};
    if( OGRWktReadToken( pszInput, szToken ) == nullptr )
        return OGRERR_CORRUPT_DATA;

/* -------------------------------------------------------------------- */
/*      Instantiate a geometry of the appropriate type.                 */
/* -------------------------------------------------------------------- */
    OGRGeometry *poGeom = nullptr;
    if( STARTS_WITH_CI(szToken, "POINT") )
    {
        poGeom = new OGRPoint();
    }
    else if( STARTS_WITH_CI(szToken, "LINESTRING") )
    {
        poGeom = new OGRLineString();
    }
    else if( STARTS_WITH_CI(szToken, "POLYGON") )
    {
        poGeom = new OGRPolygon();
    }
    else if( STARTS_WITH_CI(szToken,"TRIANGLE") )
    {
        poGeom = new OGRTriangle();
    }
    else if( STARTS_WITH_CI(szToken, "GEOMETRYCOLLECTION") )
    {
        poGeom = new OGRGeometryCollection();
    }
    else if( STARTS_WITH_CI(szToken, "MULTIPOLYGON") )
    {
        poGeom = new OGRMultiPolygon();
    }
    else if( STARTS_WITH_CI(szToken, "MULTIPOINT") )
    {
        poGeom = new OGRMultiPoint();
    }
    else if( STARTS_WITH_CI(szToken, "MULTILINESTRING") )
    {
        poGeom = new OGRMultiLineString();
    }
    else if( STARTS_WITH_CI(szToken, "CIRCULARSTRING") )
    {
        poGeom = new OGRCircularString();
    }
    else if( STARTS_WITH_CI(szToken, "COMPOUNDCURVE") )
    {
        poGeom = new OGRCompoundCurve();
    }
    else if( STARTS_WITH_CI(szToken, "CURVEPOLYGON") )
    {
        poGeom = new OGRCurvePolygon();
    }
    else if( STARTS_WITH_CI(szToken, "MULTICURVE") )
    {
        poGeom = new OGRMultiCurve();
    }
    else if( STARTS_WITH_CI(szToken, "MULTISURFACE") )
    {
        poGeom = new OGRMultiSurface();
    }

    else if( STARTS_WITH_CI(szToken,"POLYHEDRALSURFACE") )
    {
        poGeom = new OGRPolyhedralSurface();
    }

    else if( STARTS_WITH_CI(szToken,"TIN") )
    {
        poGeom = new OGRTriangulatedSurface();
    }

    else
    {
        return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
    }

/* -------------------------------------------------------------------- */
/*      Do the import.                                                  */
/* -------------------------------------------------------------------- */
    const OGRErr eErr = poGeom->importFromWkt(s);

/* -------------------------------------------------------------------- */
/*      Assign spatial reference system.                                */
/* -------------------------------------------------------------------- */
    if( eErr == OGRERR_NONE )
    {
        if( poGeom->hasCurveGeometry() &&
            CPLTestBool(CPLGetConfigOption("OGR_STROKE_CURVE", "FALSE")) )
        {
            OGRGeometry* poNewGeom = poGeom->getLinearGeometry();
            delete poGeom;
            poGeom = poNewGeom;
        }
        *ppoReturn = poGeom;
    }
    else
    {
        delete poGeom;
    }

    return eErr;
}
#endif // GDAL version limit

OGRGeometry* createFromGeoJson(char **s)
{
    // Go through a supposed JSON object string, looking for the
    // closing brace.  Return just past its position.
    auto findEnd = [](std::string s, std::string::size_type pos)
    {
        bool inString(false);
        std::string check("{}\"");
        std::string::size_type startPos(pos);
        pos = Utils::extractSpaces(s, pos);
        if (s[pos++] != '{')
            return std::string::npos;
        int cnt = 1;
        while (cnt && pos != std::string::npos)
        {
            pos = s.find_first_of(check, pos);
            if (pos == std::string::npos)
                return pos;
            if (s[pos] == '"')
            {
                // We're guaranteed that the beginning seq. of chars is such
                // we won't check an invalid ref.
                if (!inString || s[pos - 1] != '\\' || s[pos - 2] == '\\')
                    inString = !inString;
            }
            else if (!inString && s[pos] == '{')
                cnt++;
            else if (!inString && s[pos] == '}')
                cnt--;
            pos++;
        }
        if (cnt != 0)
            return std::string::npos;
        return pos;
    };

    std::string ss(*s);
    // Search the string for the end of the JSON.
    std::string::size_type pos = findEnd(ss, 0);
    if (pos == std::string::npos)
        return nullptr;

    // Just send the JSON stuff to the OGR function.
    ss = ss.substr(0, pos);
    OGRGeometryH h = OGR_G_CreateGeometryFromJson(ss.c_str());

    // Increment the initial string pointer to just past the JSON.
    *s += pos;
    return (reinterpret_cast<OGRGeometry *>(h));
}

} // namespace oldgdalsupport

} // namespace pdal
