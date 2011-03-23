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

#include <libpc/filters/CropFilter.hpp>
#include <libpc/filters/CropFilterIterator.hpp>
#include <libpc/exceptions.hpp>

namespace libpc { namespace filters {


CropFilter::CropFilter(const Stage& prevStage, Bounds<double> const& bounds)
    : Filter(prevStage)
    , m_bounds(bounds)
{
    Header& header = getHeader();
    header.setBounds(bounds);

    header.setNumPoints(0);
    header.setPointCountType(PointCount_Unknown);

    return;
}


const std::string& CropFilter::getName() const
{
    static std::string name("Crop Filter");
    return name;
}


const Bounds<double>& CropFilter::getBounds() const
{
    return m_bounds;
}


// append all points from src buffer to end of dst buffer, based on the our bounds
void CropFilter::processBuffer(PointBuffer& dstData, const PointBuffer& srcData) const
{
    const SchemaLayout& schemaLayout = dstData.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();

    int fieldX = schema.getDimensionIndex(Dimension::Field_X);
    int fieldY = schema.getDimensionIndex(Dimension::Field_Y);
    int fieldZ = schema.getDimensionIndex(Dimension::Field_Z);

    const Bounds<double>& bounds = this->getBounds();

    boost::uint32_t numSrcPoints = srcData.getNumPoints();
    boost::uint32_t dstIndex = dstData.getNumPoints();

    for (boost::uint32_t srcIndex=0; srcIndex<numSrcPoints; srcIndex++)
    {
    
        double x = srcData.getField<double>(srcIndex, fieldX);
        double y = srcData.getField<double>(srcIndex, fieldY);
        double z = srcData.getField<double>(srcIndex, fieldZ);
        Vector<double> point(x,y,z);
        
        if (bounds.contains(point))
        {
            dstData.copyPointFast(dstIndex, srcIndex, srcData);
            dstData.setNumPoints(dstIndex+1);
            ++dstIndex;
            
        }
    }
    
    assert(dstIndex <= dstData.getCapacity());

    return;
}


libpc::Iterator* CropFilter::createIterator() const
{
    return new CropFilterIterator(*this);
}


} } // namespaces
