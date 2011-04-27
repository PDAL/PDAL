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

#include <libpc/filters/ReprojectionFilter.hpp>

#include <libpc/Dimension.hpp>
#include <libpc/Schema.hpp>
#include <libpc/exceptions.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/filters/ReprojectionFilterIterator.hpp>

#ifdef LIBPC_HAVE_GDAL
#include <gdal.h>
#include <ogr_spatialref.h>
#endif

namespace libpc { namespace filters {


#ifdef LIBPC_HAVE_GDAL
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



ReprojectionFilter::ReprojectionFilter(const Stage& prevStage,
                                       const SpatialReference& inSRS,
                                       const SpatialReference& outSRS)
    : Filter(prevStage)
    , m_inSRS(inSRS)
    , m_outSRS(outSRS)
{
    checkImpedance();

    initialize();

    return;
}


void ReprojectionFilter::checkImpedance()
{
    const Schema& schema = this->getSchema();

    if (!schema.hasDimension(Dimension::Field_X, Dimension::Double) ||
        !schema.hasDimension(Dimension::Field_Y, Dimension::Double) ||
        !schema.hasDimension(Dimension::Field_Z, Dimension::Double))
    {
        throw impedance_invalid("Reprojection filter requires X,Y,Z dimensions as doubles");
    }

    return;
}


void ReprojectionFilter::initialize()
{
#ifdef LIBPC_HAVE_GDAL
    
    m_in_ref_ptr = ReferencePtr(OSRNewSpatialReference(0), OGRSpatialReferenceDeleter());
    m_out_ref_ptr = ReferencePtr(OSRNewSpatialReference(0), OGRSpatialReferenceDeleter());
    
    int result = OSRSetFromUserInput(m_in_ref_ptr.get(), m_inSRS.getWKT(libpc::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE) 
    {
        std::ostringstream msg; 
        msg << "Could not import input spatial reference for ReprojectionTransform:: " 
            << CPLGetLastErrorMsg() << " code: " << result 
            << "wkt: '" << m_inSRS.getWKT() << "'";
        throw std::runtime_error(msg.str());
    }
    
    result = OSRSetFromUserInput(m_out_ref_ptr.get(), m_outSRS.getWKT(libpc::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE) 
    {
        std::ostringstream msg; 
        msg << "Could not import output spatial reference for ReprojectionTransform:: " 
            << CPLGetLastErrorMsg() << " code: " << result 
            << "wkt: '" << m_outSRS.getWKT() << "'";
        std::string message(msg.str());
        throw std::runtime_error(message);
    }
    m_transform_ptr = TransformPtr(OCTNewCoordinateTransformation( m_in_ref_ptr.get(), m_out_ref_ptr.get()), OSRTransformDeleter());
#endif

    return;
}


void ReprojectionFilter::transform(double& x, double& y, double& z) const
{
    int ret = 0;

    ret = OCTTransform(m_transform_ptr.get(), 1, &x, &y, &z);    
    if (!ret)
    {
        std::ostringstream msg; 
        msg << "Could not project point for ReprojectionTransform::" << CPLGetLastErrorMsg() << ret;
        throw std::runtime_error(msg.str());
    }
    
    return;
}


const std::string& ReprojectionFilter::getDescription() const
{
    static std::string name("Reprojection Filter");
    return name;
}


const std::string& ReprojectionFilter::getName() const
{
    static std::string name("filters.reprojection");
    return name;
}


void ReprojectionFilter::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    const SchemaLayout& schemaLayout = data.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();

    const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Double);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Double);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Double);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        double x = data.getField<boost::int32_t>(pointIndex, indexX);
        double y = data.getField<boost::int32_t>(pointIndex, indexY);
        double z = data.getField<boost::int32_t>(pointIndex, indexZ);

        this->transform(x,y,z);

        data.setField<double>(pointIndex, indexX, x);
        data.setField<double>(pointIndex, indexY, y);
        data.setField<double>(pointIndex, indexZ, z);

        data.setNumPoints(pointIndex+1);
    }

    return;
}


libpc::SequentialIterator* ReprojectionFilter::createSequentialIterator() const
{
    return new ReprojectionFilterSequentialIterator(*this);
}

} } // namespaces
