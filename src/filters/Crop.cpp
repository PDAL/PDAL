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

#include <pdal/filters/Crop.hpp>

#include <pdal/PointBuffer.hpp>
#include <sstream>

namespace pdal
{
namespace filters
{


Crop::Crop(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_bounds(options.getValueOrThrow<Bounds<double> >("bounds"))
{
    return;
}


Crop::Crop(Stage& prevStage, Bounds<double> const& bounds)
    : Filter(prevStage, Options::none())
    , m_bounds(bounds)
{
    return;
}


void Crop::initialize()
{
    Filter::initialize();

    this->setBounds(m_bounds);

    this->setNumPoints(0);
    this->setPointCountType(PointCount_Unknown);

    return;
}


const Options Crop::getDefaultOptions() const
{
    Options options;
    Option bounds("bounds",Bounds<double>(),"bounds to crop to");
    options.add(bounds);
    return options;
}


const Bounds<double>& Crop::getBounds() const
{
    return m_bounds;
}


// append all points from src buffer to end of dst buffer, based on the our bounds
boost::uint32_t Crop::processBuffer(PointBuffer& dstData, const PointBuffer& srcData) const
{
    const Schema& schema = dstData.getSchema();

    const Bounds<double>& bounds = this->getBounds();

    boost::uint32_t numSrcPoints = srcData.getNumPoints();
    boost::uint32_t dstIndex = dstData.getNumPoints();

    boost::uint32_t numPointsAdded = 0;

    boost::optional<Dimension const&> dimX = schema.getDimension("X");
    boost::optional<Dimension const&> dimY = schema.getDimension("Y");
    boost::optional<Dimension const&> dimZ = schema.getDimension("Z");

    for (boost::uint32_t srcIndex=0; srcIndex<numSrcPoints; srcIndex++)
    {
        // need to scale the values
        double x(0.0);
        double y(0.0);
        double z(0.0);

        if (dimX->getInterpretation() == dimension::SignedInteger)
        {
            boost::int32_t xi = srcData.getField<boost::int32_t>(*dimX, srcIndex);
            boost::int32_t yi = srcData.getField<boost::int32_t>(*dimY, srcIndex);
            boost::int32_t zi = srcData.getField<boost::int32_t>(*dimZ, srcIndex);

            x = dimX->applyScaling(xi);
            y = dimY->applyScaling(yi);
            z = dimZ->applyScaling(zi);

        }
        else if (dimX->getInterpretation() == dimension::UnsignedInteger)
        {
            boost::uint32_t xi = srcData.getField<boost::uint32_t>(*dimX, srcIndex);
            boost::uint32_t yi = srcData.getField<boost::uint32_t>(*dimY, srcIndex);
            boost::uint32_t zi = srcData.getField<boost::uint32_t>(*dimZ, srcIndex);

            x = dimX->applyScaling(xi);
            y = dimY->applyScaling(yi);
            z = dimZ->applyScaling(zi);

        }
        else
        {
            x = srcData.getField<double>(*dimX, srcIndex);
            y = srcData.getField<double>(*dimY, srcIndex);
            z = srcData.getField<double>(*dimZ, srcIndex);

        }

        Vector<double> point(x,y,z);

        if (bounds.contains(point))
        {
            dstData.copyPointFast(dstIndex, srcIndex, srcData);
            dstData.setNumPoints(dstIndex+1);
            ++dstIndex;
            ++numPointsAdded;
        }
    }

    assert(dstIndex <= dstData.getCapacity());

    return numPointsAdded;
}


pdal::StageSequentialIterator* Crop::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Crop(*this, buffer);
}


namespace iterators
{
namespace sequential
{


Crop::Crop(const pdal::filters::Crop& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_cropFilter(filter)
{
    return;
}


boost::uint64_t Crop::skipImpl(boost::uint64_t count)
{
    return naiveSkipImpl(count);
}


boost::uint32_t Crop::readBufferImpl(PointBuffer& dstData)
{
    // The client has asked us for dstData.getCapacity() points.
    // We will read from our previous stage until we get that amount (or
    // until the previous stage runs out of points).

    boost::uint32_t numPointsNeeded = dstData.getCapacity();
    assert(dstData.getNumPoints() == 0);

    while (numPointsNeeded > 0)
    {
        if (getPrevIterator().atEnd()) break;

        // set up buffer to be filled by prev stage
        PointBuffer srcData(dstData.getSchema(), numPointsNeeded);

        // read from prev stage
        const boost::uint32_t numSrcPointsRead = getPrevIterator().read(srcData);
        assert(numSrcPointsRead == srcData.getNumPoints());
        assert(numSrcPointsRead <= numPointsNeeded);

        // we got no data, and there is no more to get -- exit the loop
        if (numSrcPointsRead == 0) break;

        // copy points from src (prev stage) into dst (our stage),
        // based on the CropFilter's rules (i.e. its bounds)
        const boost::uint32_t numPointsProcessed = m_cropFilter.processBuffer(dstData, srcData);

        numPointsNeeded -= numPointsProcessed;
    }

    const boost::uint32_t numPointsAchieved = dstData.getNumPoints();

    return numPointsAchieved;
}


bool Crop::atEndImpl() const
{
    // we don't have a fixed point point --
    // we are at the end only when our source is at the end
    const StageSequentialIterator& iter = getPrevIterator();
    return iter.atEnd();
}


}
} // iterators::sequential

}
} // namespaces
