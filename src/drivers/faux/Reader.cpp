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

#include <cassert>
#include <iostream>

#include <libpc/drivers/faux/Reader.hpp>
#include <libpc/Utils.hpp>
#include <libpc/exceptions.hpp>

using std::vector;
using std::string;
using std::cout;

namespace libpc { namespace drivers { namespace faux {

Reader::Reader(const Bounds<double>& bounds, int numPoints, Mode mode)
    : libpc::Reader()
    , m_mode(mode)
{
    Header* header = new Header;
    Schema& schema = header->getSchema();

    schema.addDimension(Dimension(Dimension::Field_X, Dimension::Double));
    schema.addDimension(Dimension(Dimension::Field_Y, Dimension::Double));
    schema.addDimension(Dimension(Dimension::Field_Z, Dimension::Double));
    schema.addDimension(Dimension(Dimension::Field_Time, Dimension::Uint64));

    header->setNumPoints(numPoints);
    header->setBounds(bounds);

    setHeader(header);

    return;
}

Reader::Reader(const Bounds<double>& bounds, int numPoints, Mode mode, const std::vector<Dimension>& dimensions)
    : libpc::Reader()
    , m_mode(mode)
{
    Header* header = new Header;

    Schema& schema = header->getSchema();
    if (dimensions.size() == 0)
    {
        throw; // BUG
    }
    schema.addDimensions(dimensions);

    header->setNumPoints(numPoints);
    header->setBounds(bounds);

    setHeader(header);

    return;
}


const std::string& Reader::getName() const
{
    static std::string name("Faux Reader");
    return name;
}


boost::uint32_t Reader::readBuffer(PointData& data)
{
    if (data.getSchemaLayout().getSchema().getDimensions().size() != 4)
        throw not_yet_implemented("need to add ability to read from arbitrary fields");

    // make up some data and put it into the buffer

    const boost::uint32_t numPoints = data.getNumPoints();
    assert(getCurrentPointIndex() + numPoints <= getHeader().getNumPoints());

    const SchemaLayout& schemaLayout = data.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();
    Header& header = getHeader();

    const Bounds<double>& bounds = header.getBounds();
    const std::vector< Range<double> >& dims = bounds.dimensions();
    const double minX = dims[0].getMinimum();
    const double maxX = dims[0].getMaximum();
    const double minY = dims[1].getMinimum();
    const double maxY = dims[1].getMaximum();
    const double minZ = dims[2].getMinimum();
    const double maxZ = dims[2].getMaximum();

    const int offsetT = schema.getDimensionIndex(Dimension::Field_Time);
    const int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    const int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    const int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);

    boost::uint64_t time = getCurrentPointIndex();

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        double x;
        double y;
        double z;
        if (m_mode == Random)
        {
            x = (double)Utils::random(minX, maxX);
            y = (double)Utils::random(minY, maxY);
            z = (double)Utils::random(minZ, maxZ);
        }
        else
        {
            x = (double)minX;
            y = (double)minY;
            z = (double)minZ;
        }

        data.setField<double>(pointIndex, offsetX, x);
        data.setField<double>(pointIndex, offsetY, y);
        data.setField<double>(pointIndex, offsetZ, z);
        data.setField<boost::uint64_t>(pointIndex, offsetT, time);

        ++time;
    }

    incrementCurrentPointIndex(numPoints);

    return numPoints;
}


void Reader::seekToPoint(boost::uint64_t pointNumber)
{
    setCurrentPointIndex(pointNumber);
}


} } } // namespaces
