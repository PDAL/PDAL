/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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


#include <pdal/drivers/sbet/Writer.hpp>

#include <pdal/drivers/sbet/Common.hpp>
#include <pdal/PointBuffer.hpp>


namespace pdal
{
namespace drivers
{
namespace sbet
{


Writer::~Writer()
{
    FileUtils::closeFile(m_ostream);
}


void Writer::processOptions(const Options& options)
{
    m_filename = options.getOption("filename").getValue<std::string>();
}


void Writer::initialize()
{
    m_ostream = FileUtils::createFile(m_filename, true);
}


void Writer::writeBegin(boost::uint64_t)
{
    return;
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& data)
{
    if (!m_ostream->good())
    {
        throw pdal_error("sbet outstream is no good");
    }

    const Schema& schema = data.getSchema();
    const Dimension& dimTime = schema.getDimension("Time");
    const Dimension& dimY = schema.getDimension("Y");
    const Dimension& dimX = schema.getDimension("X");
    const Dimension& dimZ = schema.getDimension("Z");
    const Dimension& dimXVelocity = schema.getDimension("XVelocity");
    const Dimension& dimYVelocity = schema.getDimension("YVelocity");
    const Dimension& dimZVelocity = schema.getDimension("ZVelocity");
    const Dimension& dimRoll = schema.getDimension("Roll");
    const Dimension& dimPitch = schema.getDimension("Pitch");
    const Dimension& dimPlatformHeading = schema.getDimension("PlatformHeading");
    const Dimension& dimWanderAngle = schema.getDimension("WanderAngle");
    const Dimension& dimXBodyAccel = schema.getDimension("XBodyAccel");
    const Dimension& dimYBodyAccel = schema.getDimension("YBodyAccel");
    const Dimension& dimZBodyAccel = schema.getDimension("ZBodyAccel");
    const Dimension& dimXBodyAngRate = schema.getDimension("XBodyAngRate");
    const Dimension& dimYBodyAngRate = schema.getDimension("YBodyAngRate");
    const Dimension& dimZBodyAngRate = schema.getDimension("ZBodyAngRate");

    boost::uint8_t buf[pdal::drivers::sbet::pointByteSize];
    boost::uint32_t numPointsWritten = 0;

    for (boost::uint32_t idx = 0; idx < data.getNumPoints(); idx++)
    {
        boost::uint8_t* p = buf;

        Utils::write_field(p, data.getFieldAs<double>(dimTime, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimY, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimX, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimZ, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimXVelocity, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimYVelocity, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimZVelocity, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimRoll, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimPitch, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimPlatformHeading, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimWanderAngle, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimXBodyAccel, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimYBodyAccel, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimZBodyAccel, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimXBodyAngRate, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimYBodyAngRate, idx, false));
        Utils::write_field(p, data.getFieldAs<double>(dimZBodyAngRate, idx, false));

        // TODO should we be using StreamFactory?
        Utils::write_n(*m_ostream, buf, pdal::drivers::sbet::pointByteSize);
        ++numPointsWritten;
    }

    return numPointsWritten;
}


void Writer::writeEnd(boost::uint64_t)
{
    return;
}


}
}
} // namespace pdal::drivers::sbet
