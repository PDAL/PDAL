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

static Mode string2mode(const std::string& str)
{
    if (boost::iequals(str, "constant")) return Constant;
    if (boost::iequals(str, "random")) return Random;
    if (boost::iequals(str, "ramp")) return Ramp;
    throw pdal_error("invalid Mode option: " + str);
}


Reader::Reader(const Options& options)
    : pdal::Reader(options)
{}


void Reader::processOptions(const Options& options)
{
    m_bounds = options.getValueOrThrow<Bounds<double>>("bounds");
    m_numPoints = options.getValueOrThrow<uint64_t>("num_points");
    m_mode = string2mode(options.getValueOrThrow<std::string>("mode"));
}


void Reader::buildSchema(Schema *s)
{
    m_schema = s;
    std::vector<Dimension> dims = getDefaultDimensions();
    for (auto di = dims.begin(); di != dims.end(); ++di)
        s->appendDimension(*di);
}


std::vector<Dimension> Reader::getDefaultDimensions()
{
    std::vector<Dimension> output;
    Dimension x("X", dimension::Float, 8);
    x.setUUID("c74a80bd-8eca-4ab6-9e90-972738e122f0");
    x.setNamespace(s_getName());
    output.push_back(x);

    Dimension y("Y", dimension::Float, 8);
    y.setUUID("1b102a72-daa5-4a81-8a23-8aa907350473");
    y.setNamespace(s_getName());
    output.push_back(y);

    Dimension z("Z", dimension::Float, 8);
    z.setUUID("fb54cf8c-1a01-45d1-a92e-75b7d487ac54");
    z.setNamespace(s_getName());
    output.push_back(z);

    Dimension t("Time", dimension::UnsignedInteger, 8);
    t.setUUID("96b94034-3d25-4e72-b474-ccbdb14d53f6");
    t.setNamespace(s_getName());
    output.push_back(t);

    return output;
}


Options Reader::getDefaultOptions()
{
    Options options;
    return options;
}


pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new FauxSeqIterator(m_bounds, m_schema, m_mode);
}

} // namespace faux
} // namespace drivers


FauxSeqIterator::FauxSeqIterator(const Bounds<double>& bounds, Schema *schema,
    drivers::faux::Mode mode) : m_time(0), m_mode(mode)
{
    const std::vector<Range<double>>& ranges = bounds.dimensions();
    m_minX = ranges[0].getMinimum();
    m_maxX = ranges[0].getMaximum();
    m_minY = ranges[1].getMinimum();
    m_maxY = ranges[1].getMaximum();
    m_minZ = ranges[2].getMinimum();
    m_maxZ = ranges[2].getMaximum();

    m_dimX = schema->getDimensionPtr("X", "drivers.faux.reader");
    m_dimY = schema->getDimensionPtr("Y", "drivers.faux.reader");
    m_dimZ = schema->getDimensionPtr("Z", "drivers.faux.reader");
    m_dimTime = schema->getDimensionPtr("Time", "drivers.faux.reader");
}


point_count_t FauxSeqIterator::readImpl(PointBuffer& buf, point_count_t count)
{
    using namespace pdal::drivers::faux;

    const double numDeltas = (double)count - 1.0;
    const double delX = (m_maxX - m_minX) / numDeltas;
    const double delY = (m_maxY - m_minY) / numDeltas;
    const double delZ = (m_maxZ - m_minZ) / numDeltas;

    for (PointId idx = 0; idx < count; ++idx)
    {
        double x;
        double y;
        double z;
        switch (m_mode)
        {
            case Random:
                x = Utils::random(m_minX, m_maxX);
                y = Utils::random(m_minY, m_maxY);
                z = Utils::random(m_minZ, m_maxZ);
                break;
            case Constant:
                x = m_minX;
                y = m_minY;
                z = m_minZ;
                break;
            case Ramp:
                x = m_minX + delX * idx;
                y = m_minY + delY * idx;
                z = m_minZ + delZ * idx;
                break;
        }

        buf.setField(*m_dimX, idx, x);
        buf.setField(*m_dimY, idx, y);
        buf.setField(*m_dimZ, idx, z);
        buf.setField(*m_dimTime, idx, m_time++);
    }
    return count;
}


} // pdal
