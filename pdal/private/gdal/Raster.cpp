/******************************************************************************
* Copyright (c) 2020, Hobu Inc.
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

#pragma warning(push)
#pragma warning(disable: 4251)
#include <gdal.h>
#include <gdal_priv.h>
#pragma warning(pop)

#include <pdal/util/Algorithm.hpp>

#include "Raster.hpp"
#include "GDALUtils.hpp"

namespace pdal
{
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

} // unnamed namespace

/**
  Create a copy of the raster in memory.
  \return  Pointer to the new raster.
*/
Raster* Raster::memoryCopy() const
{

    GDALDriver *mem = GetGDALDriverManager()->GetDriverByName("MEM");
    if (!mem)
    {
        return nullptr;
    }

    if (!m_ds)
        throw pdal::pdal_error("driver is not open!");

    GDALDataset* mem_ds = mem->CreateCopy("", m_ds, FALSE, nullptr, nullptr,
        nullptr);

    Raster* r = new Raster(mem_ds);
    r->wake();
    return r;
}


/**
  Constructor for reading a raster.
  \param filename  Raster filename.
  \param drivername  Driver to use when opening raster.
*/
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


/**
  Constructor to use when creating a raster.
  \param filename  Filename of raster.
  \param drivername  Driver to use when opening the raster.

*/
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


/**
  Open a raster destined for output.
  \param width  Raster width.
  \param height  Raster height.
  \param numBands  Number of bands in the raster.
  \param type  Datatype of the raster.
  \param noData  Nodata value for "empty" cells.
  \param options  A list of option strings that are passed to the GDAL driver.
*/
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

    registerDrivers();
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


/**
  Open a raster for output.
  \return  Error code, or GDALError::None.
*/
GDALError Raster::open()
{
    if (m_ds)
        return GDALError::None;

    const char ** driverP = NULL;
    const char *drivers[2] = {0};
    if (!m_drivername.empty())
    {
        drivers[0] = m_drivername.c_str();
        driverP = drivers;
    }

    registerDrivers();
    m_ds = (GDALDataset *)GDALOpenEx(m_filename.c_str(),
        GDAL_OF_READONLY | GDAL_OF_RASTER, driverP, nullptr, nullptr);
    return wake();
}


/**
  Initialize a raster after opening.
*/
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


/**
  Convert a pixel X/Y position to a position in a coord. reference system.
  \param col  Pixel column.
  \param row  Pixel row.
  \param[out] output  Array of 2 doubles with of the position in a CRS.
*/
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


/**
  Determines the pixel/line position given a coordinate position.
  \param x  X coordinate of point.
  \param y  Y coordinate of point.
  \param[out] pixel  Raster pixel (column) position.
  \param[out] line  Raster line (row) position.
*/
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


/**
  Compute a vector of the PDAL datatypes that are stored in the raster
  bands of a dataset.
  \return  Error code or GDALError::None.
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


/**
  Fetch the raster data associated with the point at a position.
  \param x  X position of point to fetch raster data for.
  \param y  Y position of point to fetch raster data for.
  \param[out] data  Vector of raster data associated with the provided point.
  \return  Error code or GDALError::None.
*/
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


/**
  Get the spatial reference associated with a raster.
  \return  Associated spatial reference.
*/
SpatialReference Raster::getSpatialRef() const
{
    SpatialReference srs;

    if (m_ds)
        srs = SpatialReference(m_ds->GetProjectionRef());
    return srs;
}


/**
  Destructor.  Closes the raster if open.
*/
Raster::~Raster()
{
    close();
}


/**
  Close an open raster.
*/
void Raster::close()
{
    GDALClose(m_ds);
    m_ds = nullptr;
    m_types.clear();
}


BOX2D Raster::bounds() const
{
    std::array<double, 2> coords;

    pixelToCoord(height(), width(), coords);
    double maxx = coords[0];
    double maxy = coords[1];

    pixelToCoord(0, 0, coords);
    double minx = coords[0];
    double miny = coords[1];
    return BOX2D(minx, miny, maxx, maxy);
}


BOX3D Raster::bounds(int nBand) const
{
    BOX2D box2 = bounds();

    double minimum; double maximum;
    double mean; double stddev;
    if (statistics(nBand, &minimum, &maximum, &mean, &stddev) !=
            GDALError::None)
        return BOX3D();

    return BOX3D(box2.minx, box2.miny, minimum,
            box2.maxx, box2.maxy, maximum);
}


// Get statistics for a raster band.
GDALError Raster::statistics(int nBand, double *minimum, double *maximum,
    double *mean, double *stddev, bool approx, bool force) const
{
    try
    {
        Band<double>(m_ds, nBand).statistics(minimum, maximum, mean, stddev,
            approx, force);
    }
    catch (InvalidBand)
    {
        m_errorMsg = "Unable to get band " + std::to_string(nBand) +
            " from raster '" + m_filename + "'.";
        return GDALError::InvalidBand;
    }
    catch (BadBand)
    {
        m_errorMsg = "Unable to read band/block information from "
            "raster '" + m_filename + "'.";
        return GDALError::BadBand;
    }
    return GDALError::None;
}

} // namespace gdal
} // namespace pdal
