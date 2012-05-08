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

#include <pdal/drivers/faux/Reader.hpp>

#include <pdal/PointBuffer.hpp>

#include <boost/algorithm/string.hpp>


namespace pdal
{
namespace drivers
{
namespace faux
{


static Reader::Mode string2mode(const std::string& str)
{
    if (boost::iequals(str, "constant")) return Reader::Constant;
    if (boost::iequals(str, "random")) return Reader::Random;
    if (boost::iequals(str, "ramp")) return Reader::Ramp;
    throw pdal_error("invalid Mode option: " + str);
}


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_bounds(options.getValueOrThrow<Bounds<double> >("bounds"))
    , m_numPoints(options.getValueOrThrow<boost::uint64_t>("num_points"))
    , m_mode(string2mode(options.getValueOrThrow<std::string>("mode")))
{
    addDefaultDimensions();
    return;
}


Reader::Reader(const Bounds<double>& bounds, boost::uint64_t numPoints, Mode mode)
    : pdal::Reader(Options::none())
    , m_bounds(bounds)
    , m_numPoints(numPoints)
    , m_mode(mode)
{
    addDefaultDimensions();
    return;
}

Reader::Reader(const Bounds<double>& bounds, boost::uint64_t numPoints, Mode mode, const std::vector<Dimension>& dimensions)
    : pdal::Reader(Options::none())
    , m_bounds(bounds)
    , m_numPoints(numPoints)
    , m_mode(mode)
{
    if (dimensions.size() == 0)
    {
        throw; // BUG
    }

    for (boost::uint32_t i=0; i < dimensions.size(); i++)
    {
        const Dimension& dim = dimensions[i];
        addDefaultDimension(dim, getName());
    }
    return;
}

void Reader::addDefaultDimensions()
{
    Dimension x("X", dimension::Float, 8);
    x.setUUID("c74a80bd-8eca-4ab6-9e90-972738e122f0");
    Dimension y("Y", dimension::Float, 8);
    y.setUUID("1b102a72-daa5-4a81-8a23-8aa907350473");
    Dimension z("Z", dimension::Float, 8);
    z.setUUID("fb54cf8c-1a01-45d1-a92e-75b7d487ac54");
    Dimension t("Time", dimension::UnsignedInteger, 8);
    t.setUUID("96b94034-3d25-4e72-b474-ccbdb14d53f6");

    addDefaultDimension(x, getName());
    addDefaultDimension(y, getName());
    addDefaultDimension(z, getName());
    addDefaultDimension(t, getName());
}

void Reader::initialize()
{
    pdal::Reader::initialize();

    Schema& schema = getSchemaRef();
    schema = Schema(getDefaultDimensions());

    setNumPoints(m_numPoints);
    setPointCountType(PointCount_Fixed);

    setBounds(m_bounds);
}


const Options Reader::getDefaultOptions() const
{
    Options options;
    return options;
}


Reader::Mode Reader::getMode() const
{
    return m_mode;
}


pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::faux::iterators::sequential::Reader(*this, buffer);
}


pdal::StageRandomIterator* Reader::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::faux::iterators::random::Reader(*this, buffer);
}


boost::uint32_t Reader::processBuffer(PointBuffer& data, boost::uint64_t index) const
{
    const Schema& schema = data.getSchema();

    // make up some data and put it into the buffer

    // how many are they asking for?
    boost::uint64_t numPointsWanted = data.getCapacity();

    // we can only give them as many as we have left
    boost::uint64_t numPointsAvailable = getNumPoints() - index;
    if (numPointsAvailable < numPointsWanted)
        numPointsWanted = numPointsAvailable;

    const Bounds<double>& bounds = getBounds();
    const std::vector< Range<double> >& dims = bounds.dimensions();
    const double minX = dims[0].getMinimum();
    const double maxX = dims[0].getMaximum();
    const double minY = dims[1].getMinimum();
    const double maxY = dims[1].getMaximum();
    const double minZ = dims[2].getMinimum();
    const double maxZ = dims[2].getMaximum();

    const double numDeltas = (double)getNumPoints() - 1.0;
    const double delX = (maxX - minX) / numDeltas;
    const double delY = (maxY - minY) / numDeltas;
    const double delZ = (maxZ - minZ) / numDeltas;

    const Dimension& dimX = schema.getDimension("X", getName());
    const Dimension& dimY = schema.getDimension("Y", getName());
    const Dimension& dimZ = schema.getDimension("Z", getName());
    const Dimension& dimTime = schema.getDimension("Time", getName());

    boost::uint64_t time = index;

    const Reader::Mode mode = getMode();

    boost::uint32_t cnt = 0;
    data.setNumPoints(0);

    for (boost::uint32_t pointIndex=0; pointIndex<numPointsWanted; pointIndex++)
    {
        double x;
        double y;
        double z;
        switch (mode)
        {
            case Reader::Random:
                x = Utils::random(minX, maxX);
                y = Utils::random(minY, maxY);
                z = Utils::random(minZ, maxZ);
                break;
            case Reader::Constant:
                x = minX;
                y = minY;
                z = minZ;
                break;
            case Reader::Ramp:
                x = minX + delX * pointIndex;
                y = minY + delY * pointIndex;
                z = minZ + delZ * pointIndex;
                break;
            default:
                throw pdal_error("invalid mode in FauxReader");
                break;
        }

        data.setField<double>(dimX, pointIndex, x);
        data.setField<double>(dimY, pointIndex, y);
        data.setField<double>(dimZ, pointIndex, z);
        data.setField<boost::uint64_t>(dimTime, pointIndex, time);

        ++time;

        ++cnt;
        data.setNumPoints(cnt);
        assert(cnt <= data.getCapacity());
    }

    return cnt;
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

namespace iterators
{
namespace sequential
{



Reader::Reader(const pdal::drivers::faux::Reader& reader, PointBuffer& buffer)
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

Reader::Reader(const pdal::drivers::faux::Reader& reader, PointBuffer& buffer)
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
