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

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Support.hpp>
#include <pdal/drivers/las/SummaryData.hpp>


namespace pdal
{
namespace drivers
{
namespace las
{

PointDimensions::PointDimensions(const Schema& schema, std::string const& ns)
{


#   define CACHE_DIM(x) \
    x = schema.getDimensionPtr(#x, ns.c_str()); \
    if (x && x->isIgnored()) x = 0;
    
    // don't ever wipe off ignored XYZ dims.
    X = schema.getDimensionPtr("X", ns);
    if (!X)
        throw pdal_error("Unable to fetch X dimension with ns " + ns);
    Y = schema.getDimensionPtr("Y", ns);
    if (!Y)
        throw pdal_error("Unable to fetch Y dimension with ns " + ns);
    Z = schema.getDimensionPtr("Z", ns);
    if (!Z)
        throw pdal_error("Unable to fetch Z dimension with ns " + ns);
    
    CACHE_DIM(Intensity)
    CACHE_DIM(ReturnNumber)
    CACHE_DIM(NumberOfReturns)
    CACHE_DIM(ScanDirectionFlag)
    CACHE_DIM(EdgeOfFlightLine)
    CACHE_DIM(Classification)
    CACHE_DIM(ScanAngleRank)
    CACHE_DIM(UserData)
    CACHE_DIM(PointSourceId)
    CACHE_DIM(Time)
    CACHE_DIM(Red)
    CACHE_DIM(Green)
    CACHE_DIM(Blue)

#   undef CACHE_DIM

    // WavePacketDescriptorIndex = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WavePacketDescriptorIndex) : 0);
    // WaveformDataOffset = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformDataOffset) : 0);
    // ReturnPointWaveformLocation = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_ReturnPointWaveformLocation) : 0);
    // WaveformXt = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformXt) : 0);
    // WaveformYt = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformYt) : 0);
    // WaveformZt = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformZt) : 0);
}


void Support::rewriteHeader(std::ostream& stream, const SummaryData& data)
{
    // move from header start to "number of point records" field
    stream.seekp(107, std::ios_base::cur);

    // FIXME: This is only for LAS 1.3 and less!
    const int maxReturns = 5; 
    {
        boost::uint8_t buf[256];
        boost::uint8_t* p = buf;

        Utils::write_field<boost::uint32_t>(p, data.getTotalNumPoints());

        for (int i=1; i<=maxReturns; i++)
        {
            Utils::write_field<boost::uint32_t>(p, data.getReturnCount(i));
        }

        Utils::write_n(stream, buf, 4 + 4*maxReturns);
    }

    // skip over scale/offset fields
    stream.seekp(8*6, std::ios_base::cur);

    {
        boost::uint8_t buf[256];
        boost::uint8_t* p = buf;

        pdal::Bounds<double> bounds= data.getBounds();
        double minX = bounds.getMinimum(0);
        double minY = bounds.getMinimum(1);
        double minZ = bounds.getMinimum(2);
        double maxX = bounds.getMaximum(0);
        double maxY = bounds.getMaximum(1);
        double maxZ = bounds.getMaximum(2);
        Utils::write_field<double>(p, maxX);
        Utils::write_field<double>(p, minX);
        Utils::write_field<double>(p, maxY);
        Utils::write_field<double>(p, minY);
        Utils::write_field<double>(p, maxZ);
        Utils::write_field<double>(p, minZ);
        Utils::write_n(stream, buf, 6*8);
    }

    stream.seekp(0, std::ios_base::end);

    return;
}


}
}
} // namespaces
