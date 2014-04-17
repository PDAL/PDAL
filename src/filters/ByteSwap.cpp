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

#include <pdal/filters/ByteSwap.hpp>

#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Endian.hpp>
#include <iostream>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{
namespace filters
{


ByteSwap::ByteSwap(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{
    return;
}


ByteSwap::ByteSwap(Stage& prevStage)
    : Filter(prevStage, Options::none())
{
    return;
}


void ByteSwap::initialize()
{
    Filter::initialize();

    const Stage& stage = getPrevStage();
    this->setNumPoints(stage.getNumPoints());
    this->setPointCountType(stage.getPointCountType());

    schema::index_by_index const& dimensions =
        m_schema.getDimensions().get<schema::index>();

    std::vector<Dimension> new_dimensions;
    for (schema::index_by_index::const_iterator i = dimensions.begin(); i != dimensions.end(); ++i)
    {
        pdal::Dimension d(*i);
        pdal::EndianType t = i->getEndianness();
        if (t == Endian_Little)
        {
            d.setEndianness(Endian_Big);
        }
        else if (t == Endian_Big)
        {
            d.setEndianness(Endian_Little);
        }
        else
        {
            throw pdal_error("ByteSwapFilter can only swap big/little endian dimensions");
        }
        new_dimensions.push_back(d);
    }

    m_schema = Schema(new_dimensions);
}


Options ByteSwap::getDefaultOptions()
{
    Options options;
    return options;
}


boost::uint32_t ByteSwap::processBuffer(PointBuffer& dstData, const PointBuffer& srcData) const
{
    const Schema& dstSchema = dstData.getSchema();

    schema::index_by_index const& dstDims = dstSchema.getDimensions().get<schema::index>();

    dstData.setSpatialBounds(srcData.getSpatialBounds());
    dstData.copyPointsFast(0, 0, srcData, srcData.getNumPoints());

    dstData.setNumPoints(srcData.getNumPoints());

    for (boost::uint32_t i = 0; i != dstData.getNumPoints(); ++i)
    {
        boost::uint8_t* data = dstData.getData(i);
        std::size_t position = 0;
        for (boost::uint32_t n = 0; n < dstDims.size(); ++n)
        {
            const Dimension& d = dstSchema.getDimension(n);
            std::size_t size = d.getByteSize();

            boost::uint8_t* pos = data + position;
            SWAP_ENDIANNESS_N(*pos, size);
            position = position + size;
        }

    }

    return dstData.getNumPoints();
}


pdal::StageSequentialIterator* ByteSwap::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::ByteSwap(*this, buffer);
}

namespace iterators
{
namespace sequential
{



ByteSwap::ByteSwap(const pdal::filters::ByteSwap& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_swapFilter(filter)
{
    return;
}


boost::uint64_t ByteSwap::skipImpl(boost::uint64_t count)
{
    return getPrevIterator().skip(count);
}


boost::uint32_t ByteSwap::readBufferImpl(PointBuffer& dstData)
{
    // The client has asked us for dstData.getCapacity() points.
    // We will read from our previous stage until we get that amount (or
    // until the previous stage runs out of points).

    boost::uint32_t numPointsNeeded = dstData.getCapacity();
    boost::uint32_t numPointsAchieved = 0;

    while (numPointsNeeded > 0)
    {
        // set up buffer to be filled by prev stage
        PointBuffer srcData(dstData.getSchema(), numPointsNeeded);


        // read from prev stage
        const boost::uint32_t numSrcPointsRead = getPrevIterator().read(srcData);
        assert(numSrcPointsRead == srcData.getNumPoints());
        assert(numSrcPointsRead <= numPointsNeeded);

        numPointsAchieved += numSrcPointsRead;

        // we got no data, and there is no more to get -- exit the loop
        if (numSrcPointsRead == 0) return numPointsAchieved;

        // copy points from src (prev stage) into dst (our stage),
        // based on the CropFilter's rules (i.e. its bounds)
        const boost::uint32_t numPointsProcessed = m_swapFilter.processBuffer(dstData, srcData);

        numPointsNeeded -= numPointsProcessed;

    }




    return numPointsAchieved;
}


bool ByteSwap::atEndImpl() const
{
    // we don't have a fixed point point --
    // we are at the end only when our source is at the end
    const StageSequentialIterator& iter = getPrevIterator();
    return iter.atEnd();
}

}
} // iterators::sequential

}
} // pdal::filters
