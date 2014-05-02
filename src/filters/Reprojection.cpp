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

#include <pdal/filters/Reprojection.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <pdal/PointBuffer.hpp>


#ifdef PDAL_HAVE_GDAL
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include <gdal.h>
#include <ogr_spatialref.h>
#include <pdal/GDALUtils.hpp>
#endif

namespace pdal
{
namespace filters
{


#ifdef PDAL_HAVE_GDAL
struct OGRSpatialReferenceDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        ::OSRDestroySpatialReference(ptr);
    }
};

struct OSRTransformDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        ::OCTDestroyCoordinateTransformation(ptr);
    }
};


struct GDALSourceDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        ::GDALClose(ptr);
    }
};
#endif



Reprojection::Reprojection(const Options& options)
    : pdal::Filter(options)
    , m_outSRS(options.getValueOrThrow<pdal::SpatialReference>("out_srs"))
    , m_inferInputSRS(false)
{
    if (options.hasOption("in_srs"))
    {
        m_inSRS = options.getValueOrThrow<pdal::SpatialReference>("in_srs");
        m_inferInputSRS = false;
    }
    else
    {
        m_inferInputSRS = true;
    }
}


Reprojection::Reprojection(const SpatialReference& outSRS)
    : m_outSRS(outSRS)
    , m_inferInputSRS(true)
{}


Reprojection::Reprojection(const SpatialReference& inSRS,
                           const SpatialReference& outSRS)
    : m_inSRS(inSRS)
    , m_outSRS(outSRS)
    , m_inferInputSRS(false)
{}


void Reprojection::initialize()
{
    Filter::initialize();

    checkImpedance();

    if (m_inferInputSRS)
    {
        m_inSRS = getPrevStage().getSpatialReference();
    }

#ifdef PDAL_HAVE_GDAL

    m_gdal_debug = boost::shared_ptr<pdal::gdal::Debug>(new pdal::gdal::Debug(isDebug(), log()));

    m_in_ref_ptr = ReferencePtr(OSRNewSpatialReference(0), OGRSpatialReferenceDeleter());
    m_out_ref_ptr = ReferencePtr(OSRNewSpatialReference(0), OGRSpatialReferenceDeleter());

    int result = OSRSetFromUserInput(m_in_ref_ptr.get(), m_inSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE)
    {
        std::ostringstream msg;
        msg << "Could not import input spatial reference for ReprojectionFilter:: "
            << CPLGetLastErrorMsg() << " code: " << result
            << " wkt: '" << m_inSRS.getWKT() << "'";
        throw std::runtime_error(msg.str());
    }

    result = OSRSetFromUserInput(m_out_ref_ptr.get(), m_outSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE)
    {
        std::ostringstream msg;
        msg << "Could not import output spatial reference for ReprojectionFilter:: "
            << CPLGetLastErrorMsg() << " code: " << result
            << " wkt: '" << m_outSRS.getWKT() << "'";
        std::string message(msg.str());
        throw std::runtime_error(message);
    }
    m_transform_ptr = TransformPtr(OCTNewCoordinateTransformation(m_in_ref_ptr.get(), m_out_ref_ptr.get()), OSRTransformDeleter());

    if (!m_transform_ptr.get())
    {
        std::ostringstream msg;
        msg << "Could not construct CoordinateTransformation in ReprojectionFilter:: ";
        std::string message(msg.str());
        throw std::runtime_error(message);
    }

#endif

    setSpatialReference(m_outSRS);

    updateBounds();

    return;
}


Options Reprojection::getDefaultOptions()
{
    Options options;
    return options;
}


void Reprojection::updateBounds()
{
    const Bounds<double>& oldBounds = getBounds();

    double minx = oldBounds.getMinimum(0);
    double miny = oldBounds.getMinimum(1);
    double minz = oldBounds.getMinimum(2);
    double maxx = oldBounds.getMaximum(0);
    double maxy = oldBounds.getMaximum(1);
    double maxz = oldBounds.getMaximum(2);

    try
    {

        transform(minx, miny, minz);
        transform(maxx, maxy, maxz);

    }
    catch (pdal::pdal_error&)
    {
        return;
    }

    Bounds<double> newBounds(minx, miny, minz, maxx, maxy, maxz);

    setBounds(newBounds);

    return;
}


void Reprojection::checkImpedance()
{
    const Schema& schema = this->getSchema();

    boost::optional<Dimension const&> x = schema.getDimension("X");
    boost::optional<Dimension const&> y = schema.getDimension("Y");
    boost::optional<Dimension const&> z = schema.getDimension("Z");

    if (!(x->getByteSize() != 8) ||
            !(y->getByteSize() != 8) ||
            !(z->getByteSize() != 8))
    {
        throw impedance_invalid("Reprojection filter requires X,Y,Z dimensions as doubles");
    }

    return;
}


void Reprojection::transform(double& x, double& y, double& z) const
{

#ifdef PDAL_HAVE_GDAL
    int ret = 0;

    ret = OCTTransform(m_transform_ptr.get(), 1, &x, &y, &z);
    if (!ret)
    {
        std::ostringstream msg;
        msg << "Could not project point for ReprojectionTransform::" << CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }
#else
    boost::ignore_unused_variable_warning(x);
    boost::ignore_unused_variable_warning(y);
    boost::ignore_unused_variable_warning(z);
#endif

    return;
}


void Reprojection::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    const Schema& schema = data.getSchema();

    Dimension const& dimX = schema.getDimension("X");
    Dimension const& dimY = schema.getDimension("Y");
    Dimension const& dimZ = schema.getDimension("Z");


    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        double x = data.getField<double>(dimX, pointIndex);
        double y = data.getField<double>(dimY, pointIndex);
        double z = data.getField<double>(dimZ, pointIndex);

        this->transform(x,y,z);

        data.setField<double>(dimX, pointIndex, x);
        data.setField<double>(dimY, pointIndex, y);
        data.setField<double>(dimZ, pointIndex, z);

        data.setNumPoints(pointIndex+1);
    }

    return;
}


pdal::StageSequentialIterator* Reprojection::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Reprojection(*this, buffer);
}


namespace iterators
{
namespace sequential
{


Reprojection::Reprojection(const pdal::filters::Reprojection& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_reprojectionFilter(filter)
{
    return;
}


boost::uint32_t Reprojection::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);

    m_reprojectionFilter.processBuffer(data);

    return numRead;
}


boost::uint64_t Reprojection::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Reprojection::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

}
} // namespaces

}
} // namespaces
