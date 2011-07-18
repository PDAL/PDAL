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

#include <pdal/filters/ScalingFilter.hpp>

#include <pdal/Dimension.hpp>
#include <pdal/Schema.hpp>
#include <pdal/exceptions.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/filters/ScalingFilterIterator.hpp>


namespace pdal { namespace filters {


ScalingFilter::ScalingFilter(const DataStagePtr& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
     throw not_yet_implemented("scaling filter options support"); 
}


ScalingFilter::ScalingFilter(const DataStagePtr& prevStage, bool forward)
    : Filter(prevStage, Options::none())
    , m_customScaleOffset(false)
    , m_scaleX(0.0)
    , m_scaleY(0.0)
    , m_scaleZ(0.0)
    , m_offsetX(0.0)
    , m_offsetY(0.0)
    , m_offsetZ(0.0)
    , m_forward(forward)
{
    checkImpedance();

    initialize();

    return;
}


ScalingFilter::ScalingFilter(const DataStagePtr& prevStage, double scaleX, double offsetX, double scaleY, double offsetY, double scaleZ, double offsetZ, bool forward)
    : Filter(prevStage, Options::none())
    , m_customScaleOffset(true)
    , m_scaleX(scaleX)
    , m_scaleY(scaleY)
    , m_scaleZ(scaleZ)
    , m_offsetX(offsetX)
    , m_offsetY(offsetY)
    , m_offsetZ(offsetZ)
    , m_forward(forward)
{
    checkImpedance();

    initialize();

    return;
}


void ScalingFilter::checkImpedance()
{
    Schema& schema = this->getSchemaRef();

    if (m_forward)
    {
        // doubles --> ints

        const int indexXd = schema.getDimensionIndex(Dimension::Field_X, Dimension::Double);
        const int indexYd = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Double);
        const int indexZd = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Double);

        const Dimension& dimXd = schema.getDimension(indexXd);
        const Dimension& dimYd = schema.getDimension(indexYd);
        const Dimension& dimZd = schema.getDimension(indexZd);

        Dimension dimXi(Dimension::Field_X, Dimension::Int32);
        Dimension dimYi(Dimension::Field_Y, Dimension::Int32);
        Dimension dimZi(Dimension::Field_Z, Dimension::Int32);

        if (!schema.hasDimension(dimXd) || !schema.hasDimension(dimYd) || !schema.hasDimension(dimZd))
        {
            throw impedance_invalid("Scaling filter requires X,Y,Z dimensions as doubles (forward direction)");
        }
        if (schema.hasDimension(dimXi) || schema.hasDimension(dimYi) || schema.hasDimension(dimZi))
        {
            throw impedance_invalid("Scaling filter requires X,Y,Z dimensions as ints not be initially present (forward direction)");
        }

        schema.removeDimension(dimXd);
        schema.removeDimension(dimYd);
        schema.removeDimension(dimZd);

        if (m_customScaleOffset)
        {
            dimXi.setNumericScale(m_scaleX);
            dimXi.setNumericOffset(m_offsetX);
            dimYi.setNumericScale(m_scaleY);
            dimYi.setNumericOffset(m_offsetY);
            dimZi.setNumericScale(m_scaleZ);
            dimZi.setNumericOffset(m_offsetZ);
        }
        else
        {
            dimXi.setNumericScale(dimXd.getNumericScale());
            dimXi.setNumericOffset(dimXd.getNumericOffset());
            dimYi.setNumericScale(dimYd.getNumericScale());
            dimYi.setNumericOffset(dimYd.getNumericOffset());
            dimZi.setNumericScale(dimZd.getNumericScale());
            dimZi.setNumericOffset(dimZd.getNumericOffset());
        }

        schema.addDimension(dimXi);
        schema.addDimension(dimYi);
        schema.addDimension(dimZi);
    }
    else
    {
        const int indexXi = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
        const int indexYi = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
        const int indexZi = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
        
        const Dimension& dimXi = schema.getDimension(indexXi);
        const Dimension& dimYi = schema.getDimension(indexYi);
        const Dimension& dimZi = schema.getDimension(indexZi);

        Dimension dimXd(Dimension::Field_X, Dimension::Double);
        Dimension dimYd(Dimension::Field_Y, Dimension::Double);
        Dimension dimZd(Dimension::Field_Z, Dimension::Double);

        // ints --> doubles
        if (!schema.hasDimension(dimXi) || !schema.hasDimension(dimYi) || !schema.hasDimension(dimZi))
        {
            throw impedance_invalid("Scaling filter requires X,Y,Z dimensions as int32s (reverse direction)");
        }
        if (schema.hasDimension(dimXd) || schema.hasDimension(dimYd) || schema.hasDimension(dimZd))
        {
            throw impedance_invalid("Scaling filter requires X,Y,Z dimensions as int32s not be initially present (reverse direction)");
        }

        if (m_customScaleOffset)
        {
            dimXd.setNumericScale(m_scaleX);
            dimXd.setNumericOffset(m_offsetX);
            dimYd.setNumericScale(m_scaleY);
            dimYd.setNumericOffset(m_offsetY);
            dimZd.setNumericScale(m_scaleZ);
            dimZd.setNumericOffset(m_offsetZ);
        }
        else
        {
            dimXd.setNumericScale(dimXi.getNumericScale());
            dimXd.setNumericOffset(dimXi.getNumericOffset());
            dimYd.setNumericScale(dimYi.getNumericScale());
            dimYd.setNumericOffset(dimYi.getNumericOffset());
            dimZd.setNumericScale(dimZi.getNumericScale());
            dimZd.setNumericOffset(dimZi.getNumericOffset());
        }

        schema.removeDimension(dimXi);
        schema.removeDimension(dimYi);
        schema.removeDimension(dimZi);

        schema.addDimension(dimXd);
        schema.addDimension(dimYd);
        schema.addDimension(dimZd);
    }

    return;
}


void ScalingFilter::initialize()
{
    return;
}


const std::string& ScalingFilter::getDescription() const
{
    static std::string name("Scaling Filter");
    return name;
}


const std::string& ScalingFilter::getName() const
{
    static std::string name("filters.scaling");
    return name;
}


void ScalingFilter::processBuffer(const PointBuffer& srcData, PointBuffer& dstData) const
{
    const boost::uint32_t numPoints = srcData.getNumPoints();

    const SchemaLayout& srcSchemaLayout = srcData.getSchemaLayout();
    const Schema& srcSchema = srcSchemaLayout.getSchema();

    const SchemaLayout& dstSchemaLayout = dstData.getSchemaLayout();
    const Schema& dstSchema = dstSchemaLayout.getSchema();

    // rather than think about "src/dst", we will think in terms of "doubles" and "ints"
    const Schema& schemaD = (m_forward ? srcSchema : dstSchema);
    const Schema& schemaI = (m_forward ? dstSchema : srcSchema);

    assert(schemaD.hasDimension(Dimension::Field_X, Dimension::Double));
    assert(schemaI.hasDimension(Dimension::Field_X, Dimension::Int32));

    const int indexXd = schemaD.getDimensionIndex(Dimension::Field_X, Dimension::Double);
    const int indexYd = schemaD.getDimensionIndex(Dimension::Field_Y, Dimension::Double);
    const int indexZd = schemaD.getDimensionIndex(Dimension::Field_Z, Dimension::Double);
    const int indexXi = schemaI.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    const int indexYi = schemaI.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    const int indexZi = schemaI.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);

    const Dimension& dimXd = schemaD.getDimension(indexXd);
    const Dimension& dimYd = schemaD.getDimension(indexYd);
    const Dimension& dimZd = schemaD.getDimension(indexZd);
    const Dimension& dimXi = schemaI.getDimension(indexXi);
    const Dimension& dimYi = schemaI.getDimension(indexYi);
    const Dimension& dimZi = schemaI.getDimension(indexZi);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        if (m_forward)
        {
            // doubles --> ints  (removeScaling)
            const double xd = srcData.getField<double>(pointIndex, indexXd);
            const double yd = srcData.getField<double>(pointIndex, indexYd);
            const double zd = srcData.getField<double>(pointIndex, indexZd);

            const boost::int32_t xi = dimXi.removeScaling<boost::int32_t>(xd);
            const boost::int32_t yi = dimYi.removeScaling<boost::int32_t>(yd);
            const boost::int32_t zi = dimZi.removeScaling<boost::int32_t>(zd);

            dstData.setField<boost::int32_t>(pointIndex, indexXi, xi);
            dstData.setField<boost::int32_t>(pointIndex, indexYi, yi);
            dstData.setField<boost::int32_t>(pointIndex, indexZi, zi);
        }
        else
        {
            // ints --> doubles  (applyScaling)
            const boost::int32_t xi = srcData.getField<boost::int32_t>(pointIndex, indexXi);
            const boost::int32_t yi = srcData.getField<boost::int32_t>(pointIndex, indexYi);
            const boost::int32_t zi = srcData.getField<boost::int32_t>(pointIndex, indexZi);

            const double xd = dimXd.applyScaling(xi);
            const double yd = dimYd.applyScaling(yi);
            const double zd = dimZd.applyScaling(zi);

            dstData.setField<double>(pointIndex, indexXd, xd);
            dstData.setField<double>(pointIndex, indexYd, yd);
            dstData.setField<double>(pointIndex, indexZd, zd);
        }

        dstData.setNumPoints(pointIndex+1);
    }

    return;
}


pdal::StageSequentialIterator* ScalingFilter::createSequentialIterator() const
{
    return new ScalingFilterSequentialIterator(*this);
}

} } // namespaces
