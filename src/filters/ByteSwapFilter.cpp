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

#include <pdal/filters/ByteSwapFilter.hpp>

#include <pdal/filters/ByteSwapFilterIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Endian.hpp>
#include <iostream>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal { namespace filters {


IMPLEMENT_STATICS(ByteSwapFilter, "filters.byteswap", "Crop Filter")


ByteSwapFilter::ByteSwapFilter(const Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
    initialize();
    return;
}


ByteSwapFilter::ByteSwapFilter(const Stage& prevStage)
    : Filter(prevStage, Options::none())
{
    initialize();
    return;
}


void ByteSwapFilter::initialize()
{
    const Stage& stage = getPrevStage();
    this->setNumPoints(stage.getNumPoints());
    this->setPointCountType(stage.getPointCountType());

    //Schema& schema = this->getSchemaRef();

    // FIXME:  this doesn't work anymore
    // std::vector<Dimension>& dimensions = schema.getDimensions();
    // for (std::vector<Dimension>::iterator i = dimensions.begin(); i != dimensions.end(); ++i)
    // {
    //     pdal::EndianType t = i->getEndianness();
    //     if (t == Endian_Little)
    //     {
    //         i->setEndianness(Endian_Big);
    //     } else if (t == Endian_Big)
    //     {
    //         i->setEndianness(Endian_Little);
    //     } else {
    //         throw pdal_error("ByteSwapFilter can only swap big/little endian dimensions");
    //     }
    // }

    return;        
}


const Options& ByteSwapFilter::s_getDefaultOptions()
{
    static Options options;
    return options;
}


boost::uint32_t ByteSwapFilter::processBuffer(PointBuffer& dstData, const PointBuffer& srcData) const
{
    SchemaLayout& dstSchemaLayout = dstData.getSchemaLayout();
    Schema & dstSchema = dstSchemaLayout.getSchema();
    
    pdal::Schema::Dimensions const& dstDims = dstSchema.getDimensions();

    dstData.setSpatialBounds(srcData.getSpatialBounds());
    dstData.copyPointsFast(0, 0, srcData, srcData.getNumPoints());
    
    dstData.setNumPoints(srcData.getNumPoints());
    
    for (boost::uint32_t i = 0; i != dstData.getNumPoints(); ++i)
    {
        boost::uint8_t* data = dstData.getData(i);
        std::size_t position = 0;
        for (boost::uint32_t n = 0; n < dstDims.size(); ++n)
        {
            Dimension const& d = dstSchema.getDimension(n);
            std::size_t size = d.getByteSize();
            
            boost::uint8_t* pos = data + position;
            SWAP_ENDIANNESS_N(*pos, size);
            position = position + size;
        }
            
    }
    

    for (boost::uint32_t i = 0; i < dstDims.size(); ++i)
    {
        Dimension& d = dstSchema.getDimension(i);
        if (d.getEndianness() == Endian_Little)
            d.setEndianness(Endian_Big);
        if (d.getEndianness() == Endian_Big)
            d.setEndianness(Endian_Little);
    }
            
    
    


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

    // dstData.setNumPoints(dstData.getCapacity());
    return dstData.getNumPoints();
}


pdal::StageSequentialIterator* ByteSwapFilter::createSequentialIterator() const
{
    return new ByteSwapFilterSequentialIterator(*this);
}


} } // namespaces
