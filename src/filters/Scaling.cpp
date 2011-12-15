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

#include <pdal/filters/Scaling.hpp>

#include <pdal/PointBuffer.hpp>

#include <iostream>
#include <map>

#include <boost/algorithm/string.hpp>

namespace pdal { namespace filters {


// ------------------------------------------------------------------------

Scaling::Scaling(Stage& prevStage, const Options& options)
    : Filter(prevStage, options)
{
    return;
}


void Scaling::initialize()
{
    Filter::initialize();

    checkImpedance();

    return;
}


void Scaling::checkImpedance()
{
    Options& options = getOptions();
    
    std::vector<Option> dimensions = options.getOptions("dimension");
    
    std::vector<Option>::const_iterator i;
    for (i = dimensions.begin(); i != dimensions.end(); ++i)
    {
        boost::optional<Options const&> dimensionOptions = i->getOptions();

        scaling::Scaler scaler;
        scaler.name = i->getValue<std::string>();
        if (dimensionOptions)
        {
            scaler.scale = dimensionOptions->getValueOrDefault<double>("scale", 1.0);
            scaler.offset = dimensionOptions->getValueOrDefault<double>("offset", 0.0);
            scaler.type = dimensionOptions->getValueOrDefault<std::string>("type", "Integer");
            scaler.size = dimensionOptions->getValueOrDefault<boost::uint32_t>("size", 4);
        }
        
        m_scalers.push_back(scaler);
        
    }
    return;
}


void Scaling::processBuffer(const PointBuffer& srcData, PointBuffer& dstData) const
{
    // const boost::uint32_t numPoints = srcData.getNumPoints();
    // 
    // const Schema& srcSchema = srcData.getSchema();
    // 
    // const Schema& dstSchema = dstData.getSchema();
    // 
    // // rather than think about "src/dst", we will think in terms of "doubles" and "ints"
    // const Schema& schemaD = (m_isDescaling ? srcSchema : dstSchema);
    // const Schema& schemaI = (m_isDescaling ? dstSchema : srcSchema);
    // 
    // assert(schemaD.hasDimension(DimensionId::X_f64));
    // assert(schemaI.hasDimension(DimensionId::X_i32));
    // 
    // const int indexXd = schemaD.getDimensionIndex(DimensionId::X_f64);
    // const int indexYd = schemaD.getDimensionIndex(DimensionId::Y_f64);
    // const int indexZd = schemaD.getDimensionIndex(DimensionId::Z_f64);
    // const int indexXi = schemaI.getDimensionIndex(DimensionId::X_i32);
    // const int indexYi = schemaI.getDimensionIndex(DimensionId::Y_i32);
    // const int indexZi = schemaI.getDimensionIndex(DimensionId::Z_i32);
    // 
    // const Dimension& dimXd = schemaD.getDimension(DimensionId::X_f64);
    // const Dimension& dimYd = schemaD.getDimension(DimensionId::Y_f64);
    // const Dimension& dimZd = schemaD.getDimension(DimensionId::Z_f64);
    // const Dimension& dimXi = schemaI.getDimension(DimensionId::X_i32);
    // const Dimension& dimYi = schemaI.getDimension(DimensionId::Y_i32);
    // const Dimension& dimZi = schemaI.getDimension(DimensionId::Z_i32);
    // 
    // // For each dimension in the source layout, find its corresponding dimension 
    // // in the destination layout, and put its byte offset in the map for it.  
    // 
    // schema::index_by_index const& dims = srcSchema.getDimensions().get<schema::index>();
    // std::map<boost::uint32_t, boost::uint32_t> dimensions_lookup;
    // boost::uint32_t dstFieldIndex = 0;
    // boost::uint32_t i = 0;
    // for (i=0; i<dims.size(); i++)
    // {
    //     Dimension const& dim = dims[i];
    //     std::size_t src_offset = dim.getByteOffset();
    //     dstFieldIndex = dstSchema.getDimensionIndex(dim);
    //     dimensions_lookup[dstFieldIndex] = src_offset;
    // }
    // 
    // for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    // {
    // 
    //     if (m_isDescaling)
    //     {
    //         // doubles --> ints  (removeScaling)
    //         const double xd = srcData.getField<double>(pointIndex, indexXd);
    //         const double yd = srcData.getField<double>(pointIndex, indexYd);
    //         const double zd = srcData.getField<double>(pointIndex, indexZd);
    // 
    //         const boost::int32_t xi = dimXi.removeScaling<boost::int32_t>(xd);
    //         const boost::int32_t yi = dimYi.removeScaling<boost::int32_t>(yd);
    //         const boost::int32_t zi = dimZi.removeScaling<boost::int32_t>(zd);
    // 
    //         const boost::uint8_t* src_raw_data = srcData.getData(pointIndex);
    // 
    //         for (std::map<boost::uint32_t, boost::uint32_t>::const_iterator i = dimensions_lookup.begin();
    //         i != dimensions_lookup.end(); ++i)
    //         {
    //             boost::uint32_t dstFieldIndex = i->first;
    //             boost::uint32_t src_offset = i ->second;
    //             dstData.setFieldData(pointIndex, dstFieldIndex, src_raw_data+src_offset);
    //         }
    // 
    //         dstData.setField<boost::int32_t>(pointIndex, indexXi, xi);
    //         dstData.setField<boost::int32_t>(pointIndex, indexYi, yi);
    //         dstData.setField<boost::int32_t>(pointIndex, indexZi, zi);
    // 
    //     }
    //     else
    //     {
    //         // ints --> doubles  (applyScaling)
    //         const boost::int32_t xi = srcData.getField<boost::int32_t>(pointIndex, indexXi);
    //         const boost::int32_t yi = srcData.getField<boost::int32_t>(pointIndex, indexYi);
    //         const boost::int32_t zi = srcData.getField<boost::int32_t>(pointIndex, indexZi);    
    // 
    //         const double xd = dimXd.applyScaling(xi);
    //         const double yd = dimYd.applyScaling(yi);
    //         const double zd = dimZd.applyScaling(zi);
    //         
    //         const boost::uint8_t* src_raw_data = srcData.getData(pointIndex);
    //         
    //         for (std::map<boost::uint32_t, boost::uint32_t>::const_iterator i = dimensions_lookup.begin();
    //         i != dimensions_lookup.end(); ++i)
    //         {
    //             boost::uint32_t dstFieldIndex = i->first;
    //             boost::uint32_t src_offset = i ->second;
    //             dstData.setFieldData(pointIndex, dstFieldIndex, src_raw_data+src_offset);
    //         }
    // 
    //         dstData.setField<double>(pointIndex, indexXd, xd);
    //         dstData.setField<double>(pointIndex, indexYd, yd);
    //         dstData.setField<double>(pointIndex, indexZd, zd);
    //     }
    // 
    //     dstData.setNumPoints(pointIndex+1);
    // }

    return;
}


pdal::StageSequentialIterator* Scaling::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Scaling(*this, buffer);
}


const Options Scaling::getDefaultOptions() const
{
    Options options;
    return options;
}


namespace iterators { namespace sequential {


Scaling::Scaling(const pdal::filters::Scaling& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter)
    , m_scalingFilter(filter)
{
    alterSchema(buffer);
    return;
}

void Scaling::alterSchema(PointBuffer& buffer)
{
    std::vector<scaling::Scaler> const& scalers = m_scalingFilter.getScalers();
    
    std::vector<scaling::Scaler>::const_iterator i;
    
    Schema schema = buffer.getSchema();
    
    
    // Loop through the options that the filter.Scaler collected. For each 
    // dimension described, create a new dimension with the given parameters.
    // Create a map with the uuid of the new dimension that maps to the old 
    // dimension to be scaled
    for (i = scalers.begin(); i != scalers.end(); ++i)
    {
        boost::optional<Dimension const&> dim = schema.getDimensionOptional(i->name);
        if (dim)
        {
            dimension::Interpretation interp = getInterpretation(i->type);
            Dimension d2(i->name, interp, i->size, dim->getDescription());
            d2.setNumericScale(i->scale);
            d2.setNumericOffset(i->offset);
            d2.createUUID();
            std::pair<dimension::id, Dimension> p(d2.getUUID(), dim.get());
            schema.appendDimension(d2);
        }
    }
    
    buffer = PointBuffer(schema, buffer.getCapacity());
} 

dimension::Interpretation Scaling::getInterpretation(std::string const& t) const
{
    if (boost::iequals(t, "SignedInteger"))
    {
        return dimension::SignedInteger;
    }
    if (boost::iequals(t, "UnsignedInteger"))
    {
        return dimension::UnsignedInteger;
    }
    if (boost::iequals(t, "SignedByte"))
    {
        return dimension::SignedByte;
    }
    if (boost::iequals(t, "UnsignedByte"))
    {
        return dimension::UnsignedByte;
    }
    if (boost::iequals(t, "Float"))
    {
        return dimension::Float;
    }
    if (boost::iequals(t, "Pointer"))
    {
        return dimension::Pointer;
    }
    
    return dimension::Undefined;
    
}
boost::uint32_t Scaling::readBufferImpl(PointBuffer& dstData)
{
    Schema srcSchema(m_scalingFilter.getPrevStage().getSchema());
    PointBuffer srcData(srcSchema, dstData.getCapacity());
    const boost::uint32_t numRead = getPrevIterator().read(srcData);

    m_scalingFilter.processBuffer(srcData, dstData);

    return numRead;
}


boost::uint64_t Scaling::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Scaling::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

} } // iterators::sequential


} } // namespaces
