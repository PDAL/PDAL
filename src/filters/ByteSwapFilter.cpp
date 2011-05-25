/******************************************************************************
* Copyright (c) 2011, Howard Butler <hobu.inc@gmail.com>
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

#include <libpc/filters/ByteSwapFilter.hpp>

#include <libpc/filters/ByteSwapFilterIterator.hpp>
#include <libpc/Schema.hpp>
#include <libpc/PointBuffer.hpp>

namespace libpc { namespace filters {


ByteSwapFilter::ByteSwapFilter(const Stage& prevStage)
    : Filter(prevStage)
{

    this->setNumPoints(0);
    this->setPointCountType(PointCount_Unknown);

    return;
}


const std::string& ByteSwapFilter::getDescription() const
{
    static std::string name("Crop Filter");
    return name;
}

const std::string& ByteSwapFilter::getName() const
{
    static std::string name("filters.byteswap");
    return name;
}




// append all points from src buffer to end of dst buffer, based on the our bounds
boost::uint32_t ByteSwapFilter::processBuffer(PointBuffer& dstData, const PointBuffer& srcData) const
{
    // const SchemaLayout& schemaLayout = dstData.getSchemaLayout();
    //  const Schema& schema = schemaLayout.getSchema();
    // 
    //  int fieldX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Double);
    //  int fieldY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Double);
    //  int fieldZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Double);
    // 
    //  const Bounds<double>& bounds = this->getBounds();
    // 
    //  boost::uint32_t numSrcPoints = srcData.getNumPoints();
    //  boost::uint32_t dstIndex = dstData.getNumPoints();
    // 
    //  boost::uint32_t numPointsAdded = 0;
    // 
    //  for (boost::uint32_t srcIndex=0; srcIndex<numSrcPoints; srcIndex++)
    //  {
    //  
    //      double x = srcData.getField<double>(srcIndex, fieldX);
    //      double y = srcData.getField<double>(srcIndex, fieldY);
    //      double z = srcData.getField<double>(srcIndex, fieldZ);
    //      Vector<double> point(x,y,z);
    //      
    //      if (bounds.contains(point))
    //      {
    //          dstData.copyPointFast(dstIndex, srcIndex, srcData);
    //          dstData.setNumPoints(dstIndex+1);
    //          ++dstIndex;
    //          ++numPointsAdded;
    //      }
    //  }
    //  
    //  assert(dstIndex <= dstData.getCapacity());

    return 0;
}


libpc::SequentialIterator* ByteSwapFilter::createSequentialIterator() const
{
    return new ByteSwapFilterSequentialIterator(*this);
}


} } // namespaces
