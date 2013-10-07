/******************************************************************************
* Copyright (c) 2013, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/drivers/buffer/Reader.hpp>

#include <pdal/PointBuffer.hpp>


namespace pdal
{
namespace drivers
{
namespace buffer
{


Reader::Reader(const Options& options, const PointBuffer& buffer)
    : pdal::Reader(options)
    , m_buffer(buffer)
{
    setNumPoints(buffer.getNumPoints());
    setBounds(buffer.getSpatialBounds());
    setSchema(buffer.getSchema());
    setPointCountType(PointCount_Fixed);

    return;
}


void Reader::initialize()
{
    pdal::Reader::initialize();

    return;
}


Options Reader::getDefaultOptions()
{
    Options options;
    return options;
}


pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::buffer::iterators::sequential::Reader(*this, buffer);
}


pdal::StageRandomIterator* Reader::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::buffer::iterators::random::Reader(*this, buffer);
}


boost::uint32_t Reader::processBuffer(PointBuffer& data, boost::uint64_t index) const
{
    const Schema& schema = data.getSchema();

    // how many are they asking for?
    boost::uint64_t numPointsWanted = data.getCapacity();

    // we can only give them as many as we have left
    boost::uint64_t numPointsAvailable = getNumPoints() - index;
    if (numPointsAvailable < numPointsWanted)
        numPointsWanted = numPointsAvailable;

    pointbuffer::DimensionMap* d = PointBuffer::mapDimensions(m_buffer, data);

    data.setNumPoints(0);

    PointBuffer::copyLikeDimensions(m_buffer, data,
                                    *d,
                                    index,
                                    0,
                                    numPointsWanted);

    data.setNumPoints(numPointsWanted);
    return numPointsWanted;
}


namespace iterators
{
namespace sequential
{



Reader::Reader(const pdal::drivers::buffer::Reader& reader, PointBuffer& buffer)
    : pdal::ReaderSequentialIterator(reader, buffer)
    , m_reader(reader)
{
    return;
}


boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{
    return count;
}


bool Reader::atEndImpl() const
{
    const boost::uint64_t numPoints = getStage().getNumPoints();
    const boost::uint64_t currPoint = getIndex();

    return currPoint >= numPoints;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    m_reader.log()->get(logDEBUG5) << "Reading a point buffer of " << data.getCapacity() << " points." << std::endl;
    return m_reader.processBuffer(data, getIndex());
}


}
} // iterators::sequential

namespace iterators
{
namespace random
{

Reader::Reader(const pdal::drivers::buffer::Reader& reader, PointBuffer& buffer)
    : pdal::ReaderRandomIterator(reader, buffer)
    , m_reader(reader)
{
    return;
}


boost::uint64_t Reader::seekImpl(boost::uint64_t count)
{
    return count;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    return m_reader.processBuffer(data, getIndex());
}

}
} // iterators::random


}
}
} // namespaces
