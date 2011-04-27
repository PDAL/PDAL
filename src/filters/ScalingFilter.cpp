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

#include <libpc/filters/ScalingFilter.hpp>

#include <libpc/Dimension.hpp>
#include <libpc/Schema.hpp>
#include <libpc/exceptions.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/filters/ScalingFilterIterator.hpp>


namespace libpc { namespace filters {



ScalingFilter::ScalingFilter(const Stage& prevStage)
    : Filter(prevStage)
{
    checkImpedance();

    initialize();

    return;
}


void ScalingFilter::checkImpedance()
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


void ScalingFilter::initialize()
{
    return;
}


//void ScalingFilter::transform(double& x, double& y, double& z) const
//{
//#ifdef LIBPC_HAVE_GDAL
//    
//    int ret = 0;
//
//    ret = OCTTransform(m_transform_ptr.get(), 1, &x, &y, &z);    
//    if (!ret)
//    {
//        std::ostringstream msg; 
//        msg << "Could not project point for ReprojectionTransform::" << CPLGetLastErrorMsg() << ret;
//        throw std::runtime_error(msg.str());
//    }
//    
//    //if (m_new_header.get()) 
//    //{
//    //    point.SetHeaderPtr(m_new_header);
//    //}
//
//    //point.SetX(x);
//    //point.SetY(y);
//    //point.SetZ(z);
//    //
//    //if (detail::compare_distance(point.GetRawX(), (std::numeric_limits<boost::int32_t>::max)()) ||
//    //    detail::compare_distance(point.GetRawX(), (std::numeric_limits<boost::int32_t>::min)())) {
//    //    throw std::domain_error("X scale and offset combination is insufficient to represent the data");
//    //}
//
//    //if (detail::compare_distance(point.GetRawY(), (std::numeric_limits<boost::int32_t>::max)()) ||
//    //    detail::compare_distance(point.GetRawY(), (std::numeric_limits<boost::int32_t>::min)())) {
//    //    throw std::domain_error("Y scale and offset combination is insufficient to represent the data");
//    //}    
//
//    //if (detail::compare_distance(point.GetRawZ(), (std::numeric_limits<boost::int32_t>::max)()) ||
//    //    detail::compare_distance(point.GetRawZ(), (std::numeric_limits<boost::int32_t>::min)())) {
//    //    throw std::domain_error("Z scale and offset combination is insufficient to represent the data");
//    //}        
//
//#else
//    boost::ignore_unused_variable_warning(x);
//    boost::ignore_unused_variable_warning(y);
//    boost::ignore_unused_variable_warning(z);
//#endif
//
//    return;
//}
//

const std::string& ScalingFilter::getDescription() const
{
    static std::string name("Reprojection Filter");
    return name;
}


const std::string& ScalingFilter::getName() const
{
    static std::string name("filters.reprojection");
    return name;
}


void ScalingFilter::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    const SchemaLayout& schemaLayout = data.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();

    const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    const Dimension& xDim = schema.getDimension(indexX);
    const Dimension& yDim = schema.getDimension(indexY);
    const Dimension& zDim = schema.getDimension(indexZ);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        boost::int32_t xraw = data.getField<boost::int32_t>(pointIndex, indexX);
        boost::int32_t yraw = data.getField<boost::int32_t>(pointIndex, indexY);
        boost::int32_t zraw = data.getField<boost::int32_t>(pointIndex, indexZ);

        double x = xDim.applyScaling(xraw);
        double y = yDim.applyScaling(yraw);
        double z = zDim.applyScaling(zraw);

        //this->transform(x,y,z);

        xraw = xDim.removeScaling<boost::int32_t>(x);
        yraw = yDim.removeScaling<boost::int32_t>(y);
        zraw = zDim.removeScaling<boost::int32_t>(z);

        data.setField<boost::int32_t>(pointIndex, indexX, xraw);
        data.setField<boost::int32_t>(pointIndex, indexY, yraw);
        data.setField<boost::int32_t>(pointIndex, indexZ, zraw);

        data.setNumPoints(pointIndex+1);
    }

    return;
}


libpc::SequentialIterator* ScalingFilter::createSequentialIterator() const
{
    return new ScalingFilterSequentialIterator(*this);
}

} } // namespaces
