/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/filters/Colorization.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <algorithm>

#include <pdal/PointBuffer.hpp>

#ifdef PDAL_HAVE_GDAL
#include <gdal.h>
#include <ogr_spatialref.h>
#include <pdal/GDALUtils.hpp>
#endif

namespace pdal
{
namespace filters
{


#ifdef PDAL_HAVE_GDAL
struct GDALSourceDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        ::GDALClose(ptr);
    }
};
#endif

Colorization::Colorization(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_ds(0)
    , m_gdal_debug(0)
{
    return;
}

Colorization::~Colorization()
{
#ifdef PDAL_HAVE_GDAL
    if (m_ds != 0)
    {
        GDALClose(m_ds);
        m_ds = 0;
    }
        

    if (m_gdal_debug)
        delete m_gdal_debug;
#endif
}

void Colorization::initialize()
{
    Filter::initialize();

    collectOptions();

#ifdef PDAL_HAVE_GDAL

    m_gdal_debug = new pdal::gdal::Debug(isDebug(), log());
    m_forward_transform.assign(0.0);
    m_inverse_transform.assign(0.0);

    std::string filename = getOptions().getValueOrThrow<std::string>("raster");

    log()->get(logDEBUG) << "Using " << filename << " for raster" << std::endl;
    m_ds = GDALOpen(filename.c_str(), GA_ReadOnly);
    if (m_ds == NULL)
        throw pdal_error("Unable to open GDAL datasource!");

    if (GDALGetGeoTransform(m_ds, &(m_forward_transform.front())) != CE_None)
    {
        throw pdal_error("unable to fetch forward geotransform for raster!");
    }

    GDALInvGeoTransform(&(m_forward_transform.front()), &(m_inverse_transform.front()));

#endif
    return;
}


const Options Colorization::getDefaultOptions() const
{
    Options options;

    Option x("x_dim", std::string("X"), "Dimension name to use for 'X' data");
    Option y("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");

    pdal::Option red("dimension", "Red", "");
    pdal::Option b0("band",1, "");
    pdal::Option s0("scale", 1.0f, "scale factor for this dimension");
    pdal::Options redO;
    redO.add(b0);
    redO.add(s0);
    red.setOptions(redO);

    pdal::Option green("dimension", "Green", "");
    pdal::Option b1("band",2, "");
    pdal::Option s1("scale", 1.0f, "scale factor for this dimension");
    pdal::Options greenO;
    greenO.add(b1);
    greenO.add(s1);
    green.setOptions(greenO);

    pdal::Option blue("dimension", "Blue", "");
    pdal::Option b2("band",3, "");
    pdal::Option s2("scale", 1.0f, "scale factor for this dimension");
    pdal::Options blueO;
    blueO.add(b2);
    blueO.add(s2);
    blue.setOptions(blueO);

    pdal::Option reproject("reproject", false, "Reproject the input data into the same coordinate system as the raster?");

    options.add(x);
    options.add(y);
    options.add(red);
    options.add(green);
    options.add(blue);
    options.add(reproject);


    return options;
}

void Colorization::collectOptions()
{

    Options options = getOptions();

    std::vector<Option> dimensions = options.getOptions("dimension");
    std::vector<Option>::const_iterator i;

    if (dimensions.size() == 0)
    {
        // No dimension mappings were given. We'll just assume
        // Red => 1
        // Green => 2
        // Blue => 3

        m_band_map.insert(std::pair<std::string, boost::uint32_t>("Red", 1));
        m_scale_map.insert(std::pair<std::string, double>("Red", 1.0));
        m_band_map.insert(std::pair<std::string, boost::uint32_t>("Green", 2));
        m_scale_map.insert(std::pair<std::string, double>("Green", 1.0));
        m_band_map.insert(std::pair<std::string, boost::uint32_t>("Blue", 3));
        m_scale_map.insert(std::pair<std::string, double>("Blue", 1.0));
        log()->get(logDEBUG) << "No dimension mappings were given. Using default mappings." << std::endl;

        return;
    }
    for (i = dimensions.begin(); i != dimensions.end(); ++i)
    {
        boost::optional<Options const&> dimensionOptions = i->getOptions();
        boost::uint32_t band_id(65536);
        double scale(1.0);
        std::string name = i->getValue<std::string>();
        if (dimensionOptions)
        {
            band_id = dimensionOptions->getValueOrThrow<boost::uint32_t>("band");
            scale = dimensionOptions->getValueOrDefault<double>("scale", 1.0);
        }
        else
        {
            std::ostringstream oss;
            oss << "No band and scaling information given for dimension '" << name<< "'";
            throw pdal_error(oss.str());
        }

        m_band_map.insert(std::pair<std::string, boost::uint32_t>(name, band_id));
        m_scale_map.insert(std::pair<std::string, double>(name, scale));
    }

    return;
}



void Colorization::processBuffer(PointBuffer& /* data */) const
{
    return;
}


pdal::StageSequentialIterator* Colorization::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Colorization(*this, buffer);
}


namespace iterators
{
namespace sequential
{


Colorization::Colorization(const pdal::filters::Colorization& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_stage(filter)
{
    return;
}

void Colorization::readBufferBeginImpl(PointBuffer& buffer)
{
    // Cache dimension positions

    pdal::Schema const& schema = buffer.getSchema();
    m_dimX = &(schema.getDimension(m_stage.getOptions().getValueOrDefault<std::string>("x_dim", "X")));
    m_dimY = &(schema.getDimension(m_stage.getOptions().getValueOrDefault<std::string>("y_dim", "Y")));

    std::map<std::string, boost::uint32_t> band_map = m_stage.getBandMap();
    std::map<std::string, double> scale_map = m_stage.getScaleMap();

    for (std::map<std::string, boost::uint32_t>::const_iterator i = band_map.begin();
            i != band_map.end();
            ++i)
    {
        pdal::Dimension const* dim = &(schema.getDimension(i->first));
        m_dimensions.push_back(dim);
        m_bands.push_back(static_cast<boost::uint32_t>(i->second));
        std::map<std::string, double>::const_iterator t = scale_map.find(i->first);
        double scale(1.0);
        if (t != scale_map.end())
            scale = t->second;
        m_scales.push_back(scale);
    }


}

// Determines the pixel/line position given an x/y.
// No reprojection is done at this time.
bool Colorization::getPixelAndLinePosition(double x,
        double y,
        boost::array<double, 6> const& inverse,
        boost::int32_t& pixel,
        boost::int32_t& line,
        void* ds)
{
#ifdef PDAL_HAVE_GDAL
    pixel = (boost::int32_t) std::floor(
                inverse[0]
                + inverse[1] * x
                + inverse[2] * y);
    line = (boost::int32_t) std::floor(
               inverse[3]
               + inverse[4] * x
               + inverse[5] * y);

    if (pixel < 0 || line < 0
            || pixel >= GDALGetRasterXSize(ds)
            || line  >= GDALGetRasterYSize(ds)
       )
    {
        // The x, y is not coincident with this raster
        return false;
    }
#endif

    return true;
}


boost::uint32_t Colorization::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);

#ifdef PDAL_HAVE_GDAL

    boost::array<double, 6> inverse = m_stage.getInverseTransform();

    void* ds = m_stage.getDataSource();

    boost::int32_t pixel(0);
    boost::int32_t line(0);
    double x(0.0);
    double y(0.0);
    bool fetched(false);

    boost::array<double, 2> pix;
    pix.assign(0.0);

    for (boost::uint32_t pointIndex=0; pointIndex<numRead; pointIndex++)
    {
        x = getScaledValue(data, *m_dimX, pointIndex);
        y = getScaledValue(data, *m_dimY, pointIndex);

        fetched = getPixelAndLinePosition(x, y, inverse, pixel, line, ds);
        if (!fetched)
            continue;

        for (std::vector<boost::int32_t>::size_type i = 0;
                i < m_bands.size(); i++)
        {
            GDALRasterBandH hBand = GDALGetRasterBand(ds, m_bands[i]);
            if (hBand == NULL)
            {
                std::ostringstream oss;
                oss << "Unable to get band " << m_bands[i] << " from data source!";
                throw pdal_error(oss.str());
            }
            if (GDALRasterIO(hBand, GF_Read, pixel, line, 1, 1,
                             &pix[0], 1, 1, GDT_CFloat64, 0, 0) == CE_None)
            {

                double output = pix[0];
                output = output * m_scales[i];
                setScaledValue(data, output, *m_dimensions[i], pointIndex);
            }
        }

    }

#endif
    return numRead;
}


boost::uint64_t Colorization::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Colorization::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

double Colorization::getScaledValue(PointBuffer& data,
                                    Dimension const& d,
                                    std::size_t pointIndex) const
{
    double output(0.0);

    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);

    boost::uint32_t size = d.getByteSize();
    switch (d.getInterpretation())
    {
        case dimension::Float:
            if (size == 4)
            {
                flt = data.getField<float>(d, pointIndex);
                output = static_cast<double>(flt);
            }
            if (size == 8)
            {
                output = data.getField<double>(d, pointIndex);
            }
            break;

        case dimension::SignedInteger:
        case dimension::SignedByte:
            if (size == 1)
            {
                i8 = data.getField<boost::int8_t>(d, pointIndex);
                output = d.applyScaling<boost::int8_t>(i8);
            }
            if (size == 2)
            {
                i16 = data.getField<boost::int16_t>(d, pointIndex);
                output = d.applyScaling<boost::int16_t>(i16);
            }
            if (size == 4)
            {
                i32 = data.getField<boost::int32_t>(d, pointIndex);
                output = d.applyScaling<boost::int32_t>(i32);
            }
            if (size == 8)
            {
                i64 = data.getField<boost::int64_t>(d, pointIndex);
                output = d.applyScaling<boost::int64_t>(i64);
            }
            break;

        case dimension::UnsignedInteger:
        case dimension::UnsignedByte:
            if (size == 1)
            {
                u8 = data.getField<boost::uint8_t>(d, pointIndex);
                output = d.applyScaling<boost::uint8_t>(u8);
            }
            if (size == 2)
            {
                u16 = data.getField<boost::uint16_t>(d, pointIndex);
                output = d.applyScaling<boost::uint16_t>(u16);
            }
            if (size == 4)
            {
                u32 = data.getField<boost::uint32_t>(d, pointIndex);
                output = d.applyScaling<boost::uint32_t>(u32);
            }
            if (size == 8)
            {
                u64 = data.getField<boost::uint64_t>(d, pointIndex);
                output = d.applyScaling<boost::uint64_t>(u64);
            }
            break;

        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be reprojected");
    }

    return output;
}


void Colorization::setScaledValue(PointBuffer& data,
                                  double value,
                                  Dimension const& d,
                                  std::size_t pointIndex) const
{

    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);


    boost::uint32_t size = d.getByteSize();
    switch (d.getInterpretation())
    {
        case dimension::Float:
            if (size == 4)
            {
                flt = static_cast<float>(value);
                data.setField<float>(d, pointIndex, flt);
            }
            if (size == 8)
            {
                data.setField<double>(d, pointIndex, value);
            }
            break;

        case dimension::SignedInteger:
        case dimension::SignedByte:
            if (size == 1)
            {
                i8 = d.removeScaling<boost::int8_t>(value);
                data.setField<boost::int8_t>(d, pointIndex, i8);
            }
            if (size == 2)
            {
                i16 = d.removeScaling<boost::int16_t>(value);
                data.setField<boost::int16_t>(d, pointIndex, i16);
            }
            if (size == 4)
            {
                i32 = d.removeScaling<boost::int32_t>(value);
                data.setField<boost::int32_t>(d, pointIndex, i32);
            }
            if (size == 8)
            {
                i64 = d.removeScaling<boost::int64_t>(value);
                data.setField<boost::int64_t>(d, pointIndex, i64);
            }
            break;

        case dimension::UnsignedInteger:
        case dimension::UnsignedByte:
            if (size == 1)
            {
                u8 = d.removeScaling<boost::uint8_t>(value);
                data.setField<boost::uint8_t>(d, pointIndex, u8);
            }
            if (size == 2)
            {
                u16 = d.removeScaling<boost::uint16_t>(value);
                data.setField<boost::uint16_t>(d, pointIndex, u16);
            }
            if (size == 4)
            {
                u32 = d.removeScaling<boost::uint32_t>(value);
                data.setField<boost::uint32_t>(d, pointIndex, u32);
            }
            if (size == 8)
            {
                u64 = d.removeScaling<boost::uint64_t>(value);
                data.setField<boost::uint64_t>(d, pointIndex, u64);
            }
            break;

        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be reprojected");

    }
}

}
} // iterators::sequential


}
} // namespaces
