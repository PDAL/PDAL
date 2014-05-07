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

#include <pdal/filters/Crop.hpp>

#include <pdal/PointBuffer.hpp>
#include <sstream>
#include <cstdarg>

namespace pdal
{

#ifdef PDAL_HAVE_GEOS
namespace geos
{
static void _GEOSErrorHandler(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    char buf[1024];

    vsnprintf(buf, sizeof(buf), fmt, args);
    std::cerr << "GEOS Error: " << buf << std::endl;

    va_end(args);
}

static void _GEOSWarningHandler(const char *fmt, ...)
{
    va_list args;

    char buf[1024];
    vsnprintf(buf, sizeof(buf), fmt, args);
    std::cout << "GEOS warning: " << buf << std::endl;

    va_end(args);
}

} // geos
#endif

namespace filters
{


Crop::Crop(const Options& options) : 
    pdal::Filter(options)
    , m_geosEnvironment(0)
    , m_geosGeometry(0)
    , m_geosPreparedGeometry(0)
    , m_dimensions(3)
{
    m_bounds =
        options.getValueOrDefault<Bounds<double>>("bounds", Bounds<double>());
    bCropOutside = options.getValueOrDefault<bool>("outside", false);
}


Crop::Crop(Bounds<double> const& bounds) : Crop{}
{
    m_bounds = bounds;
}

Crop::Crop() :
    bCropOutside(false)
    , m_geosEnvironment(0)
    , m_geosGeometry(0)
    , m_geosPreparedGeometry(0)
    , m_dimensions(3)
{}

Crop::~Crop()
{
#ifdef PDAL_HAVE_GEOS
    if (m_geosPreparedGeometry)
        GEOSPreparedGeom_destroy_r(m_geosEnvironment, m_geosPreparedGeometry);

    if (m_geosGeometry)
        GEOSGeom_destroy_r(m_geosEnvironment, m_geosGeometry);

    if (m_geosEnvironment)
        finishGEOS_r(m_geosEnvironment);
#endif
}


void Crop::initialize()
{
#ifdef PDAL_HAVE_GEOS
    std::string wkt =
        getOptions().getValueOrDefault<std::string>("polygon", "");
    if (!wkt.empty())
    {
        m_geosEnvironment = initGEOS_r(pdal::geos::_GEOSWarningHandler,
            pdal::geos::_GEOSErrorHandler);
        m_geosGeometry = GEOSGeomFromWKT_r(m_geosEnvironment, wkt.c_str());
        if (!m_geosGeometry)
            throw pdal_error("unable to import polygon WKT");

        int gtype = GEOSGeomTypeId_r(m_geosEnvironment, m_geosGeometry);
        if (!(gtype == GEOS_POLYGON || gtype == GEOS_MULTIPOLYGON))
            throw pdal_error("input WKT was not a POLYGON or MULTIPOLYGON");

        char* out_wkt = GEOSGeomToWKT_r(m_geosEnvironment, m_geosGeometry);
        log()->get(logDEBUG2) << "Ingested WKT for filters.crop: " <<
            std::string(out_wkt) <<std::endl;
        GEOSFree_r(m_geosEnvironment, out_wkt);

        bool bValid(false);
        bValid = (bool)(GEOSisValid_r(m_geosEnvironment, m_geosGeometry));
        if (!bValid)
        {
            char* reason =
                GEOSisValidReason_r(m_geosEnvironment, m_geosGeometry);
            std::ostringstream oss;
            oss << "WKT is invalid: " << std::string(reason) << std::endl;
            GEOSFree_r(m_geosEnvironment, reason);
            throw pdal_error(oss.str());
        }

        m_geosPreparedGeometry =
            GEOSPrepare_r(m_geosEnvironment, m_geosGeometry);
        if (!m_geosPreparedGeometry)
            throw pdal_error("unable to prepare geometry for "
                "index-accellerated intersection");
        m_bounds = computeBounds(m_geosGeometry);
        log()->get(logDEBUG) << "Computed bounds from given WKT: " <<
            m_bounds <<std::endl;
    }
    else
    {
        log()->get(logDEBUG) << "Using simple bounds for filters.crop: " <<
            m_bounds << std::endl;
    }

#endif

    setBounds(m_bounds);
    setNumPoints(0);
}


Options Crop::getDefaultOptions()
{
    Options options;
    Option bounds("bounds",Bounds<double>(),"bounds to crop to");
    Option polygon("polygon", std::string(""),
        "WKT POLYGON() string to use to filter points");

    Option inside("inside", true, "keep points that are inside or outside "
        "the given polygon");

    options.add(inside);
    options.add(polygon);
    options.add(bounds);
    return options;
}


const Bounds<double>& Crop::getBounds() const
{
    return m_bounds;
}


Bounds<double> Crop::computeBounds(GEOSGeometry const* geometry)
{
    Bounds<double> output;

#ifdef PDAL_HAVE_GEOS
    bool bFirst(true);

    GEOSGeometry const* ring = GEOSGetExteriorRing_r(m_geosEnvironment,
        geometry);
    GEOSCoordSequence const* coords = GEOSGeom_getCoordSeq_r(m_geosEnvironment,
        ring);

    GEOSCoordSeq_getDimensions_r(m_geosEnvironment, coords, &m_dimensions);
    log()->get(logDEBUG) << "Inputted WKT had " << m_dimensions <<
        " dimensions" <<std::endl;

    uint32_t count(0);
    GEOSCoordSeq_getSize_r(m_geosEnvironment, coords, &count);
    pdal::Vector<double> p(0.0, 0.0, 0.0);

    double x(0.0);
    double y(0.0);
    double z(0.0);
    for (unsigned i = 0; i < count; ++i)
    {
        GEOSCoordSeq_getOrdinate_r(m_geosEnvironment, coords, i, 0, &x);
        GEOSCoordSeq_getOrdinate_r(m_geosEnvironment, coords, i, 1, &y);
        if (m_dimensions > 2)
            GEOSCoordSeq_getOrdinate_r(m_geosEnvironment, coords, i, 2, &z);
        p.set(0, x);
        p.set(1, y);
        if (m_dimensions > 2)
            p.set(2, z);
        if (bFirst)
        {
            output = Bounds<double>(p, p);
            bFirst = false;
        }
        output.grow(p);
    }
#else
    boost::ignore_unused_variable_warning(geometry);
#endif
    return output;
}


// append all points from src buffer to end of dst buffer, based on the our bounds
uint32_t Crop::processBuffer(PointBuffer const& srcData,
    PointBuffer& dstData) const
{
    Bounds<double> const& filter_bounds = this->getBounds();
    Bounds<double> const& buffer_bounds = srcData.getSpatialBounds();

    dstData.setNumPoints(0);

    if (buffer_bounds.empty())
        log()->get(logDEBUG2) <<
            "Buffer bounds was empty, reader did not set!" << std::endl;

    Schema const& schema = srcData.getSchema();
    uint32_t count = srcData.getNumPoints();
    bool logOutput = log()->getLevel() > logDEBUG4;
    if (logOutput)
        log()->floatPrecision(8);

    std::string x_name =
        getOptions().getValueOrDefault<std::string>("x_dim", "X");
    std::string y_name =
        getOptions().getValueOrDefault<std::string>("y_dim", "Y");
    std::string z_name =
        getOptions().getValueOrDefault<std::string>("z_dim", "Z");

    log()->get(logDEBUG2) << "x_dim '" << x_name <<"' requested" << std::endl;
    log()->get(logDEBUG2) << "y_dim '" << y_name <<"' requested" << std::endl;
    log()->get(logDEBUG2) << "z_dim '" << z_name <<"' requested" << std::endl;

    Dimension const& dimX = schema.getDimension(x_name);
    Dimension const& dimY = schema.getDimension(y_name);
    Dimension const& dimZ = schema.getDimension(z_name);

    log()->get(logDEBUG2) << "x_dim '" << x_name <<"' fetched: " <<
        dimX << std::endl;
    log()->get(logDEBUG2) << "y_dim '" << y_name <<"' fetched: " <<
        dimY << std::endl;
    log()->get(logDEBUG2) << "z_dim '" << z_name <<"' fetched: " <<
        dimZ << std::endl;

    std::string wkt =
        getOptions().getValueOrDefault<std::string>("polygon", "");

    boost::uint32_t copy_index(0);
    for (boost::uint32_t index = 0; index<count; index++)
    {
        // need to scale the values
        double x = srcData.applyScaling(dimX, index);
        double y = srcData.applyScaling(dimY, index);
        double z = srcData.applyScaling(dimZ, index);

        if (logOutput)
        {
            log()->floatPrecision(10);
            log()->get(logDEBUG5) << "input: " << x << " y: " << y <<
                " z: " << z << std::endl;
        }

        if (wkt.empty())
        {
            // We don't have a polygon, just a bounds. Filter on that
            // by itself.
            Vector<double> p(x,y,z);

            if (!bCropOutside && filter_bounds.contains(p))
            {
                dstData.copyPointFast(copy_index, index, srcData);
                dstData.setNumPoints(copy_index+1);
                ++copy_index;
                if (logOutput)
                    log()->get(logDEBUG5) << "point is inside polygon!" << 
                        std::endl;
            }
        }
#ifdef PDAL_HAVE_GEOS
        else
        {
            int ret(0);

            // precise filtering based on the geometry
            GEOSCoordSequence* coords =
                GEOSCoordSeq_create_r(m_geosEnvironment, 1, 3);
            if (!coords)
                throw pdal_error("unable to allocate coordinate sequence");
            ret = GEOSCoordSeq_setX_r(m_geosEnvironment, coords, 0, x);
            if (!ret)
                throw pdal_error("unable to set x for coordinate sequence");
            ret = GEOSCoordSeq_setY_r(m_geosEnvironment, coords, 0, y);
            if (!ret)
                throw pdal_error("unable to set y for coordinate sequence");
            ret = GEOSCoordSeq_setZ_r(m_geosEnvironment, coords, 0, z);
            if (!ret)
                throw pdal_error("unable to set z for coordinate sequence");

            GEOSGeometry* p = GEOSGeom_createPoint_r(m_geosEnvironment, coords);
            if (!p)
                throw pdal_error("unable to allocate candidate test point");

            if (static_cast<bool>(GEOSPreparedContains_r(m_geosEnvironment,
                m_geosPreparedGeometry, p)) == !bCropOutside)
            {
                dstData.copyPointFast(copy_index, index, srcData);
                dstData.setNumPoints(copy_index + 1);
                ++copy_index;
                if (logOutput)
                    log()->get(logDEBUG5) << "point is inside polygon!" <<
                        std::endl;
            }
            GEOSGeom_destroy_r(m_geosEnvironment, p);
        }
#endif
    }
    return srcData.getNumPoints();
}


pdal::StageSequentialIterator*
Crop::createSequentialIterator(PointBuffer& buffer) const
{
    return new iterators::sequential::Crop(*this, buffer);
}


namespace iterators
{
namespace sequential
{


Crop::Crop(const pdal::filters::Crop& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_cropFilter(filter)
{}


boost::uint64_t Crop::skipImpl(boost::uint64_t count)
{
    return getPrevIterator().skip(count);
}


boost::uint32_t Crop::readBufferImpl(PointBuffer& data)
{
    // The client has asked us for dstData.getCapacity() points.
    // We will read from our previous stage until we get that amount (or
    // until the previous stage runs out of points).

    uint32_t originalCapacity = data.getCapacity();
    int64_t numPointsNeeded = static_cast<int64_t>(data.getCapacity());

    PointBuffer outputData(data.getSchema(), originalCapacity);
    PointBuffer tmpData(data.getSchema(), originalCapacity);
    // PointBuffer tmpData(outputData);

    m_cropFilter.log()->get(logDEBUG2) << "Fetching for block of size: " <<
        numPointsNeeded << std::endl;

    while (numPointsNeeded > 0)
    {
        // we got no data, and there is no more to get -- exit the loop
        if (getPrevIterator().atEnd())
        {
            m_cropFilter.log()->get(logDEBUG2) <<
                "previous iterator is .atEnd, stopping" << std::endl;
            break;
        }

        data.resize(static_cast<boost::uint32_t>(numPointsNeeded));

        // read from prev stage
        const boost::uint32_t numSrcPointsRead = getPrevIterator().read(data);
        m_cropFilter.log()->get(logDEBUG3) << "Fetched " << numSrcPointsRead <<
            " from previous iterator. " << std::endl;
        assert(numSrcPointsRead <= numPointsNeeded);


        // copy points from src (prev stage) into dst (our stage)
        const uint32_t numPointsProcessed =
            m_cropFilter.processBuffer(data, tmpData);
        m_cropFilter.log()->get(logDEBUG3) << "Processed " <<
            numPointsProcessed << " in intersection filter" << std::endl;

        m_cropFilter.log()->get(logDEBUG3) << tmpData.getNumPoints() <<
            " passed intersection filter" << std::endl;

        if (tmpData.getNumPoints() > 0)
        {
            outputData.copyPointsFast(outputData.getNumPoints(), 0, tmpData,
                tmpData.getNumPoints());
            outputData.setNumPoints(outputData.getNumPoints() +
                tmpData.getNumPoints());
        }

        numPointsNeeded -= tmpData.getNumPoints() ;
        m_cropFilter.log()->get(logDEBUG3) << numPointsNeeded <<
            " left to read this block" << std::endl;
    }

    uint32_t numPointsAchieved = outputData.getNumPoints();
    data.resize(originalCapacity);
    data.setNumPoints(0);
    data.copyPointsFast(0, 0, outputData, outputData.getNumPoints());
    data.setNumPoints(outputData.getNumPoints());
    m_cropFilter.log()->get(logDEBUG2) << "Copying " <<
        outputData.getNumPoints() << " at end of readBufferImpl" << std::endl;
    return numPointsAchieved;
}


bool Crop::atEndImpl() const
{
    // we don't have a fixed point count --
    // we are at the end only when our source is at the end
    const StageSequentialIterator& iter = getPrevIterator();
    return iter.atEnd();
}


}
} // iterators::sequential

}
} // namespaces
