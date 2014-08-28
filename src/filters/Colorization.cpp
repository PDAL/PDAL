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

#include <array>

#include <pdal/filters/Colorization.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/GlobalEnvironment.hpp>

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


void Colorization::initialize()
{
#ifdef PDAL_HAVE_GDAL
    GlobalEnvironment::get().getGDALEnvironment();
    GlobalEnvironment::get().getGDALDebug()->addLog(log());
#endif
}


Options Colorization::getDefaultOptions()
{
    Options options;

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

    pdal::Option reproject("reproject", false,
        "Reproject the input data into the same coordinate system as "
        "the raster?");

    options.add(red);
    options.add(green);
    options.add(blue);
    options.add(reproject);

    return options;
}


void Colorization::processOptions(const Options& options)
{
    m_rasterFilename = options.getValueOrThrow<std::string>("raster");
    std::vector<Option> dimensions = options.getOptions("dimension");

    if (dimensions.size() == 0)
    {
        m_bands.emplace_back("Red", Dimension::Id::Red, 1, 1.0);
        m_bands.emplace_back("Green", Dimension::Id::Green, 2, 1.0);
        m_bands.emplace_back("Blue", Dimension::Id::Blue, 3, 1.0);
        log()->get(LogLevel::Debug) << "No dimension mappings were given. "
            "Using default mappings." << std::endl;
    }
    for (auto i = dimensions.begin(); i != dimensions.end(); ++i)
    {
        std::string name = i->getValue<std::string>();
        boost::optional<Options const&> dimensionOptions = i->getOptions();
        if (!dimensionOptions)
        {
            std::ostringstream oss;
            oss << "No band and scaling information given for dimension '" <<
                name << "'";
            throw pdal_error(oss.str());
        }
        uint32_t bandId =
            dimensionOptions->getValueOrThrow<uint32_t>("band");
        double scale =
            dimensionOptions->getValueOrDefault<double>("scale", 1.0);
        m_bands.emplace_back(name, Dimension::Id::Unknown, bandId, scale);
    }
}


void Colorization::ready(PointContext ctx)
{
    m_forward_transform.assign(0.0);
    m_inverse_transform.assign(0.0);

#ifdef PDAL_HAVE_GDAL

    log()->get(LogLevel::Debug) << "Using " << m_rasterFilename <<
        " for raster" << std::endl;
    m_ds = GDALOpen(m_rasterFilename.c_str(), GA_ReadOnly);
    if (m_ds == NULL)
        throw pdal_error("Unable to open GDAL datasource!");

    if (GDALGetGeoTransform(m_ds, &(m_forward_transform.front())) != CE_None)
        throw pdal_error("unable to fetch forward geotransform for raster!");

    if (!GDALInvGeoTransform(&(m_forward_transform.front()),
        &(m_inverse_transform.front())))
        throw pdal_error("unable to fetch inverse geotransform for raster!");
#endif

    for (auto bi = m_bands.begin(); bi != m_bands.end(); ++bi)
    {
        if (bi->m_dim == Dimension::Id::Unknown)
            bi->m_dim = ctx.findDim(bi->m_name);
        if (bi->m_dim == Dimension::Id::Unknown)
            throw pdal_error((std::string)"Can't colorize - no dimension " +
                bi->m_name);
    }
}


void Colorization::filter(PointBuffer& buffer)
{
#ifdef PDAL_HAVE_GDAL
    int32_t pixel(0);
    int32_t line(0);

    std::array<double, 2> pix = { {0.0, 0.0} };
    for (PointId idx = 0; idx < buffer.size(); ++idx)
    {
        double x = buffer.getFieldAs<double>(Dimension::Id::X, idx);
        double y = buffer.getFieldAs<double>(Dimension::Id::Y, idx);

        if (!getPixelAndLinePosition(x, y, m_inverse_transform, pixel,
                line, m_ds))
            continue;

        for (auto bi = m_bands.begin(); bi != m_bands.end(); ++bi)
        {
            BandInfo& b = *bi;
            GDALRasterBandH hBand = GDALGetRasterBand(m_ds, b.m_band);
            if (hBand == NULL)
            {
                std::ostringstream oss;
                oss << "Unable to get band " << b.m_band <<
                    " from data source!";
                throw pdal_error(oss.str());
            }
            if (GDALRasterIO(hBand, GF_Read, pixel, line, 1, 1,
                &pix[0], 1, 1, GDT_CFloat64, 0, 0) == CE_None)
                buffer.setField(b.m_dim, idx, pix[0] * b.m_scale);
        }
    }
#endif
}


// Determines the pixel/line position given an x/y.
// No reprojection is done at this time.
bool Colorization::getPixelAndLinePosition(double x, double y,
    boost::array<double, 6> const& inverse, int32_t& pixel,
    int32_t& line, void *ds)
{
#ifdef PDAL_HAVE_GDAL
    pixel = (int32_t)std::floor(inverse[0] + (inverse[1] * x) +
        (inverse[2] * y));
    line = (int32_t) std::floor(inverse[3] + (inverse[4] * x) +
        (inverse[5] * y));

    int xs = GDALGetRasterXSize(ds);
    int ys = GDALGetRasterYSize(ds);

    if (!xs || !ys)
        throw pdal_error("Unable to get X or Y size from raster!");

    if (pixel < 0 || line < 0 || pixel >= xs || line  >= ys)
    {
        // The x, y is not coincident with this raster
        return false;
    }
#endif
    return true;
}


void Colorization::done(PointContext ctx)
{
#ifdef PDAL_HAVE_GDAL
    if (m_ds != 0)
    {
        GDALClose(m_ds);
        m_ds = 0;
    }
#endif
}

} // namespace filters
} // namespace pdal

