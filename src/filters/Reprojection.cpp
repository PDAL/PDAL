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
    : pdal::Filter(options), m_inferInputSRS(true)
{}


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


void Reprojection::processOptions(const Options& options)
{
    m_outSRS = options.getValueOrThrow<pdal::SpatialReference>("out_srs");
    if (options.hasOption("in_srs"))
    {
        m_inSRS = options.getValueOrThrow<pdal::SpatialReference>("in_srs");
        m_inferInputSRS = false;
    }
}


void Reprojection::ready(PointContext ctx)
{
    if (m_inferInputSRS)
        m_inSRS = ctx.spatialRef();

#ifdef PDAL_HAVE_GDAL
    m_gdal_debug = boost::shared_ptr<pdal::gdal::Debug>(
        new pdal::gdal::Debug(isDebug(), log()));

    m_in_ref_ptr = ReferencePtr(OSRNewSpatialReference(0),
        OGRSpatialReferenceDeleter());
    m_out_ref_ptr = ReferencePtr(OSRNewSpatialReference(0),
        OGRSpatialReferenceDeleter());

    int result =
        OSRSetFromUserInput(m_in_ref_ptr.get(),
            m_inSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE)
    {
        std::ostringstream msg;
        msg << "Could not import input spatial reference for "
            "ReprojectionFilter:: " << CPLGetLastErrorMsg() << " code: " <<
            result << " wkt: '" << m_inSRS.getWKT() << "'";
        throw std::runtime_error(msg.str());
    }

    result = OSRSetFromUserInput(m_out_ref_ptr.get(),
        m_outSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE)
    {
        std::ostringstream msg;
        msg << "Could not import output spatial reference for "
            "ReprojectionFilter:: " << CPLGetLastErrorMsg() << " code: " <<
            result << " wkt: '" << m_outSRS.getWKT() << "'";
        std::string message(msg.str());
        throw std::runtime_error(message);
    }
    m_transform_ptr = TransformPtr(
        OCTNewCoordinateTransformation(m_in_ref_ptr.get(),
            m_out_ref_ptr.get()), OSRTransformDeleter());

    if (!m_transform_ptr.get())
    {
        std::string msg = "Could not construct CoordinateTransformation in "
            "ReprojectionFilter:: ";
        throw std::runtime_error(msg);
    }

#endif
    setSpatialReference(m_outSRS);
    m_bounds.transform([this](double& x, double& y, double& z) mutable
        { this->transform(x, y, z); } );
}


void Reprojection::transform(double& x, double& y, double& z)
{
#ifdef PDAL_HAVE_GDAL
    int ret = OCTTransform(m_transform_ptr.get(), 1, &x, &y, &z);
    if (ret == 0)
    {
        std::ostringstream msg;
        msg << "Could not project point for ReprojectionTransform::" <<
            CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }
#else
    boost::ignore_unused_variable_warning(x);
    boost::ignore_unused_variable_warning(y);
    boost::ignore_unused_variable_warning(z);
#endif
}


void Reprojection::filter(PointBuffer& data)
{
    const Schema& schema = data.getSchema();

    Dimension const& dimX = schema.getDimension("X");
    Dimension const& dimY = schema.getDimension("Y");
    Dimension const& dimZ = schema.getDimension("Z");

    for (PointId id = 0; id < data.size(); ++id)
    {
        double x = data.getFieldAs<double>(dimX, id);
        double y = data.getFieldAs<double>(dimY, id);
        double z = data.getFieldAs<double>(dimZ, id);

        transform(x, y, z);

        // If you get an exception out of this, you're probably reading from
        // a scaled int and writing to a scaled int and you've overflowed
        // the integer.
        // ABELL - Need auto-scaling to deal with this, or always use X,Y,Z
        //   as doubles, or something more sophisticated..
        data.setFieldUnscaled(dimX, id, x);
        data.setFieldUnscaled(dimY, id, y);
        data.setFieldUnscaled(dimZ, id, z);
    }
}

} // namespace filter

} // namespace pdal
