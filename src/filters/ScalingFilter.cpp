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
#include <pdal/SchemaLayout.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/filters/ScalingFilterIterator.hpp>

#include <iostream>
#include <map>
namespace pdal { namespace filters {


// ------------------------------------------------------------------------

ScalingFilterBase::ScalingFilterBase(Stage& prevStage, bool isDescaling, const Options& options)
    : Filter(prevStage, options)
    , m_customScaleOffset(false)
    , m_scaleX(1.0)
    , m_scaleY(1.0)
    , m_scaleZ(1.0)
    , m_offsetX(0.0)
    , m_offsetY(0.0)
    , m_offsetZ(0.0)
    , m_isDescaling(isDescaling)
{
    int sum = 0;
    if (options.hasOption("scale_x")) ++sum;
    if (options.hasOption("scale_y")) ++sum;
    if (options.hasOption("scale_z")) ++sum;
    if (options.hasOption("offset_x")) ++sum;
    if (options.hasOption("offset_y")) ++sum;
    if (options.hasOption("offset_z")) ++sum;
    if (sum == 6)
    {
        m_customScaleOffset = true;
    }
    else if (sum == 0)
    {
        m_customScaleOffset = false;
    }
    else
    {
        throw pdal_error("not all 6 scaling factor options specified");
    }

    m_scaleX = options.getValueOrDefault<double>("scale_x", 1.0);
    m_scaleY = options.getValueOrDefault<double>("scale_y", 1.0);
    m_scaleZ = options.getValueOrDefault<double>("scale_z", 1.0);
    m_offsetX = options.getValueOrDefault<double>("offset_x", 0.0);
    m_offsetY = options.getValueOrDefault<double>("offset_y", 0.0);
    m_offsetZ = options.getValueOrDefault<double>("offset_z", 0.0);

    return;
}


ScalingFilterBase::ScalingFilterBase(Stage& prevStage, bool isDescaling)
    : Filter(prevStage, Options::none())
    , m_customScaleOffset(false)
    , m_scaleX(0.0)
    , m_scaleY(0.0)
    , m_scaleZ(0.0)
    , m_offsetX(0.0)
    , m_offsetY(0.0)
    , m_offsetZ(0.0)
    , m_isDescaling(isDescaling)
{
    return;
}


ScalingFilterBase::ScalingFilterBase(Stage& prevStage, bool isDescaling, double scaleX, double offsetX, double scaleY, double offsetY, double scaleZ, double offsetZ)
    : Filter(prevStage, Options::none())
    , m_customScaleOffset(true)
    , m_scaleX(scaleX)
    , m_scaleY(scaleY)
    , m_scaleZ(scaleZ)
    , m_offsetX(offsetX)
    , m_offsetY(offsetY)
    , m_offsetZ(offsetZ)
    , m_isDescaling(isDescaling)
{
    return;
}


void ScalingFilterBase::initialize()
{
    Filter::initialize();

    checkImpedance();

    return;
}


void ScalingFilterBase::checkImpedance()
{
    Schema& schema = this->getSchemaRef();

    if (m_isDescaling)
    {
        // doubles --> ints

        // verify we have the doubles we need, and add the ints if we have to
        if (!schema.hasDimension(DimensionId::X_f64) || !schema.hasDimension(DimensionId::Y_f64) || !schema.hasDimension(DimensionId::X_f64))
        {
            throw impedance_invalid("Descaling filter requires X,Y,Z dimensions as doubles");
        }
        if (!schema.hasDimension(DimensionId::X_i32))
        {
            schema.appendDimension(DimensionId::X_i32);
        }
        if (!schema.hasDimension(DimensionId::Y_i32))
        {
            schema.appendDimension(DimensionId::Y_i32);
        }
        if (!schema.hasDimension(DimensionId::Z_i32))
        {
            schema.appendDimension(DimensionId::Z_i32);
        }

        const Dimension& dimXd = schema.getDimension(DimensionId::X_f64);
        const Dimension& dimYd = schema.getDimension(DimensionId::Y_f64);
        const Dimension& dimZd = schema.getDimension(DimensionId::Z_f64);

        Dimension& dimXi = schema.getDimension(DimensionId::X_i32);
        Dimension& dimYi = schema.getDimension(DimensionId::Y_i32);
        Dimension& dimZi = schema.getDimension(DimensionId::Z_i32);

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

    }
    else
    {
        // ints --> doubles

        // verify we have the ints we need, and add the doubles if we have to
        if (!schema.hasDimension(DimensionId::X_i32) || !schema.hasDimension(DimensionId::X_i32) || !schema.hasDimension(DimensionId::X_i32))
        {
            throw impedance_invalid("Scaling filter requires X,Y,Z dimensions as int32s");
        }
        if (!schema.hasDimension(DimensionId::X_f64))
        {
            schema.appendDimension(DimensionId::X_f64);
        }
        if (!schema.hasDimension(DimensionId::Y_f64))
        {
            schema.appendDimension(DimensionId::Y_f64);
        }
        if (!schema.hasDimension(DimensionId::Z_f64))
        {
            schema.appendDimension(DimensionId::Z_f64);
        }
        
        const Dimension& dimXi = schema.getDimension(DimensionId::X_i32);
        const Dimension& dimYi = schema.getDimension(DimensionId::Y_i32);
        const Dimension& dimZi = schema.getDimension(DimensionId::Z_i32);

        Dimension& dimXd = schema.getDimension(DimensionId::X_f64);
        Dimension& dimYd = schema.getDimension(DimensionId::Y_f64);
        Dimension& dimZd = schema.getDimension(DimensionId::Z_f64);

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
    }

    return;
}


void ScalingFilterBase::processBuffer(const PointBuffer& srcData, PointBuffer& dstData) const
{
    const boost::uint32_t numPoints = srcData.getNumPoints();

    const SchemaLayout& srcSchemaLayout = srcData.getSchemaLayout();
    const Schema& srcSchema = srcSchemaLayout.getSchema();

    const SchemaLayout& dstSchemaLayout = dstData.getSchemaLayout();
    const Schema& dstSchema = dstSchemaLayout.getSchema();

    // rather than think about "src/dst", we will think in terms of "doubles" and "ints"
    const Schema& schemaD = (m_isDescaling ? srcSchema : dstSchema);
    const SchemaLayout& schemaLayoutD = (m_isDescaling ? srcSchemaLayout : dstSchemaLayout);
    const Schema& schemaI = (m_isDescaling ? dstSchema : srcSchema);
    const SchemaLayout& schemaLayoutI = (m_isDescaling ? dstSchemaLayout : srcSchemaLayout);

    assert(schemaD.hasDimension(DimensionId::X_f64));
    assert(schemaI.hasDimension(DimensionId::X_i32));

    const int indexXd = schemaLayoutD.getDimensionIndex(DimensionId::X_f64);
    const int indexYd = schemaLayoutD.getDimensionIndex(DimensionId::Y_f64);
    const int indexZd = schemaLayoutD.getDimensionIndex(DimensionId::Z_f64);
    const int indexXi = schemaLayoutI.getDimensionIndex(DimensionId::X_i32);
    const int indexYi = schemaLayoutI.getDimensionIndex(DimensionId::Y_i32);
    const int indexZi = schemaLayoutI.getDimensionIndex(DimensionId::Z_i32);

    const Dimension& dimXd = schemaD.getDimension(DimensionId::X_f64);
    const Dimension& dimYd = schemaD.getDimension(DimensionId::Y_f64);
    const Dimension& dimZd = schemaD.getDimension(DimensionId::Z_f64);
    const Dimension& dimXi = schemaI.getDimension(DimensionId::X_i32);
    const Dimension& dimYi = schemaI.getDimension(DimensionId::Y_i32);
    const Dimension& dimZi = schemaI.getDimension(DimensionId::Z_i32);
    
    // For each dimension in the source layout, find its corresponding dimension 
    // in the destination layout, and put its byte offset in the map for it.  
    std::vector<DimensionLayout> const& src_layouts = srcSchemaLayout.getDimensionLayouts();
    std::map<boost::uint32_t, boost::uint32_t> dimensions_lookup;
    boost::uint32_t dstFieldIndex = 0;
    for (std::vector<DimensionLayout>::const_iterator i = src_layouts.begin(); i != src_layouts.end(); ++i)
    {
        Dimension const& d = i->getDimension();
        std::size_t src_offset = i->getByteOffset();
        dstFieldIndex = dstSchemaLayout.getDimensionIndex(d);
        dimensions_lookup[dstFieldIndex] = src_offset;
    }
    
    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {

        if (m_isDescaling)
        {
            // doubles --> ints  (removeScaling)
            const double xd = srcData.getField<double>(pointIndex, indexXd);
            const double yd = srcData.getField<double>(pointIndex, indexYd);
            const double zd = srcData.getField<double>(pointIndex, indexZd);

            const boost::int32_t xi = dimXi.removeScaling<boost::int32_t>(xd);
            const boost::int32_t yi = dimYi.removeScaling<boost::int32_t>(yd);
            const boost::int32_t zi = dimZi.removeScaling<boost::int32_t>(zd);

            const boost::uint8_t* src_raw_data = srcData.getData(pointIndex);

            for (std::map<boost::uint32_t, boost::uint32_t>::const_iterator i = dimensions_lookup.begin();
            i != dimensions_lookup.end(); ++i)
            {
                boost::uint32_t dstFieldIndex = i->first;
                boost::uint32_t src_offset = i ->second;
                dstData.setFieldData(pointIndex, dstFieldIndex, src_raw_data+src_offset);
            }

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
            
            const boost::uint8_t* src_raw_data = srcData.getData(pointIndex);
            
            for (std::map<boost::uint32_t, boost::uint32_t>::const_iterator i = dimensions_lookup.begin();
            i != dimensions_lookup.end(); ++i)
            {
                boost::uint32_t dstFieldIndex = i->first;
                boost::uint32_t src_offset = i ->second;
                dstData.setFieldData(pointIndex, dstFieldIndex, src_raw_data+src_offset);
            }

            dstData.setField<double>(pointIndex, indexXd, xd);
            dstData.setField<double>(pointIndex, indexYd, yd);
            dstData.setField<double>(pointIndex, indexZd, zd);
        }

        dstData.setNumPoints(pointIndex+1);
    }

    return;
}


pdal::StageSequentialIterator* ScalingFilterBase::createSequentialIterator() const
{
    return new ScalingFilterSequentialIterator(*this);
}


// ------------------------------------------------------------------------


ScalingFilter::ScalingFilter(Stage& prevStage, const Options& options)
    : ScalingFilterBase(prevStage, false, options)
{
}


ScalingFilter::ScalingFilter(Stage& prevStage)
    : ScalingFilterBase(prevStage, false)
{
}


ScalingFilter::ScalingFilter(Stage& prevStage, double scaleX, double offsetX, double scaleY, double offsetY, double scaleZ, double offsetZ)
    : ScalingFilterBase(prevStage, false, scaleX, offsetX, scaleY, offsetY, scaleZ, offsetZ)
{
    return;
}


const Options ScalingFilter::getDefaultOptions() const
{
    Options options;
    return options;
}

// ------------------------------------------------------------------------


DescalingFilter::DescalingFilter(Stage& prevStage, const Options& options)
    : ScalingFilterBase(prevStage, true, options)
{
}


DescalingFilter::DescalingFilter(Stage& prevStage)
    : ScalingFilterBase(prevStage, true)
{
}


DescalingFilter::DescalingFilter(Stage& prevStage, double scaleX, double offsetX, double scaleY, double offsetY, double scaleZ, double offsetZ)
    : ScalingFilterBase(prevStage, true, scaleX, offsetX, scaleY, offsetY, scaleZ, offsetZ)
{
    return;
}


const Options DescalingFilter::getDefaultOptions() const
{
    static Options options;
    return options;
}

} } // namespaces
