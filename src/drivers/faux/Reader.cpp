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

#include <pdal/drivers/faux/Iterator.hpp>
#include <pdal/exceptions.hpp>
#include <pdal/SchemaLayout.hpp>
#include <pdal/PointBuffer.hpp>


namespace pdal { namespace drivers { namespace faux {

Reader::Reader(const Options& options)
    : pdal::Reader(options)
{
     throw not_yet_implemented("faux reader options support"); 
}


Reader::Reader(const Bounds<double>& bounds, int numPoints, Mode mode)
    : pdal::Reader(Options::none())
    , m_mode(mode)
{
    Schema& schema = getSchemaRef();

    schema.addDimension(Dimension(Dimension::Field_X, Dimension::Double));
    schema.addDimension(Dimension(Dimension::Field_Y, Dimension::Double));
    schema.addDimension(Dimension(Dimension::Field_Z, Dimension::Double));
    schema.addDimension(Dimension(Dimension::Field_Time, Dimension::Uint64));

    setNumPoints(numPoints);
    setPointCountType(PointCount_Fixed);

    setBounds(bounds);

    return;
}

Reader::Reader(const Bounds<double>& bounds, int numPoints, Mode mode, const std::vector<Dimension>& dimensions)
    : pdal::Reader( Options::none())
    , m_mode(mode)
{
    Schema& schema = getSchemaRef();
    if (dimensions.size() == 0)
    {
        throw; // BUG
    }
    schema.addDimensions(dimensions);

    setNumPoints(numPoints);
    setBounds(bounds);

    return;
}


const std::string& Reader::getDescription() const
{
    static std::string name("Faux Reader");
    return name;
}

const std::string& Reader::getName() const
{
    static std::string name("drivers.faux.reader");
    return name;
}

Reader::Mode Reader::getMode() const
{
    return m_mode;
}


pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new SequentialIterator(*this);
}


pdal::StageRandomIterator* Reader::createRandomIterator() const
{
    return new RandomIterator(*this);
}


boost::uint32_t Reader::processBuffer(PointBuffer& data, boost::uint64_t index) const
{
    const SchemaLayout& schemaLayout = data.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();

    if (schema.getDimensions().size() != 4)
        throw not_yet_implemented("need to add ability to read from arbitrary fields");

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

    const int offsetT = schema.getDimensionIndex(Dimension::Field_Time, Dimension::Uint64);
    const int offsetX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Double);
    const int offsetY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Double);
    const int offsetZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Double);

    boost::uint64_t time = index;
    
    const Reader::Mode mode = getMode();

    boost::uint32_t& cnt = data.getNumPointsRef();
    cnt = 0;
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

        data.setField<double>(pointIndex, offsetX, x);
        data.setField<double>(pointIndex, offsetY, y);
        data.setField<double>(pointIndex, offsetZ, z);
        data.setField<boost::uint64_t>(pointIndex, offsetT, time);

        ++time;
        ++cnt;
        assert(cnt <= data.getCapacity());
    }
    
    return cnt;
}


} } } // namespaces
