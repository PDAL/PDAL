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

void Support::registerFields(Reader& stage, Schema& schema, PointFormat format)
{
    std::ostringstream text;

    std::vector<pdal::Dimension> const& d = stage.getDefaultDimensions();

    Schema dimensions(d);
    
    schema.appendDimension(dimensions.getDimension("X", stage.getName()));
    schema.appendDimension(dimensions.getDimension("Y", stage.getName()));
    schema.appendDimension(dimensions.getDimension("Z", stage.getName()));

    schema.appendDimension(dimensions.getDimension("Intensity", stage.getName()));
    schema.appendDimension(dimensions.getDimension("ReturnNumber", stage.getName())); // 3 bits only
    schema.appendDimension(dimensions.getDimension("NumberOfReturns", stage.getName())); // 3 bits only
    schema.appendDimension(dimensions.getDimension("ScanDirectionFlag", stage.getName()));  // 1 bit only
    schema.appendDimension(dimensions.getDimension("EdgeOfFlightLine", stage.getName())); // 1 bit only

    schema.appendDimension(dimensions.getDimension("Classification", stage.getName()));
    schema.appendDimension(dimensions.getDimension("ScanAngleRank", stage.getName()));

    schema.appendDimension(dimensions.getDimension("UserData", stage.getName()));
    schema.appendDimension(dimensions.getDimension("PointSourceId", stage.getName()));

    if (hasTime(format))
    {
        schema.appendDimension(dimensions.getDimension("Time", stage.getName()));
    }

    if (hasColor(format))
    {
        schema.appendDimension(dimensions.getDimension("Red", stage.getName()));
        schema.appendDimension(dimensions.getDimension("Green", stage.getName()));
        schema.appendDimension(dimensions.getDimension("Blue", stage.getName()));
    }

    // if (hasWave(format))
    // {
    //
    //     schema.appendDimension(Dimension(DimensionId::Las_WavePacketDescriptorIndex));
    //     schema.appendDimension(Dimension(DimensionId::Las_WaveformDataOffset));
    //     schema.appendDimension(Dimension(DimensionId::Las_ReturnPointWaveformLocation));
    //     schema.appendDimension(Dimension(DimensionId::Las_WaveformXt));
    //     schema.appendDimension(Dimension(DimensionId::Las_WaveformYt));
    //     schema.appendDimension(Dimension(DimensionId::Las_WaveformZt));
    // }

    return;
}


void Support::setScaling(Schema& schema,
                         double scaleX,
                         double scaleY,
                         double scaleZ,
                         double offsetX,
                         double offsetY,
                         double offsetZ)
{
    Dimension dimX = schema.getDimension("X");
    Dimension dimY = schema.getDimension("Y");
    Dimension dimZ = schema.getDimension("Z");

    dimX.setNumericScale(scaleX);
    dimY.setNumericScale(scaleY);
    dimZ.setNumericScale(scaleZ);

    dimX.setNumericOffset(offsetX);
    dimY.setNumericOffset(offsetY);
    dimZ.setNumericOffset(offsetZ);

    schema.setDimension(dimX);
    schema.setDimension(dimY);
    schema.setDimension(dimZ);

    return;
}


bool Support::hasTime(PointFormat format)
{
    return (format == PointFormat1) || (format == PointFormat3) || (format == PointFormat4) || (format == PointFormat5);
}


bool Support::hasColor(PointFormat format)
{
    return (format == PointFormat2) || (format == PointFormat3) || (format == PointFormat5);
}


bool Support::hasWave(PointFormat format)
{
    return (format == PointFormat4) || (format == PointFormat5);
}


boost::uint16_t Support::getPointDataSize(PointFormat pointFormat)
{
    switch (pointFormat)
    {
        case PointFormat0:
            return 20;
        case PointFormat1:
            return 28;
        case PointFormat2:
            return 26;
        case PointFormat3:
            return 34;
        default:
            throw invalid_format("point format unsupported");
    }

}

PointDimensions::PointDimensions(const Schema& schema, std::string const& ns)
{

    X = schema.getDimensionPtr("X", ns);
    Y = schema.getDimensionPtr("Y", ns);
    Z = schema.getDimensionPtr("Z", ns);

    Intensity = schema.getDimensionPtr("Intensity", ns);
    if (Intensity)
        if (Intensity->isIgnored()) Intensity = 0;

    ReturnNumber = schema.getDimensionPtr("ReturnNumber", ns);
    if (ReturnNumber)
        if (ReturnNumber->isIgnored()) ReturnNumber = 0;

    NumberOfReturns = schema.getDimensionPtr("NumberOfReturns", ns);
    if (NumberOfReturns)
        if (NumberOfReturns->isIgnored()) NumberOfReturns = 0;

    ScanDirectionFlag = schema.getDimensionPtr("ScanDirectionFlag", ns);
    if (ScanDirectionFlag)
        if (ScanDirectionFlag->isIgnored()) ScanDirectionFlag = 0;

    EdgeOfFlightLine = schema.getDimensionPtr("EdgeOfFlightLine", ns);
    if (EdgeOfFlightLine)
        if (EdgeOfFlightLine->isIgnored()) EdgeOfFlightLine = 0;


    Classification = schema.getDimensionPtr("Classification", ns);
    if (Classification)
        if (Classification->isIgnored()) Classification = 0;

    ScanAngleRank = schema.getDimensionPtr("ScanAngleRank", ns);
    if (ScanAngleRank)
        if (ScanAngleRank->isIgnored()) ScanAngleRank = 0;
        
    UserData = schema.getDimensionPtr("UserData", ns);
    if (UserData)
        if (UserData->isIgnored()) UserData = 0;

    PointSourceId = schema.getDimensionPtr("PointSourceId", ns);
    if (PointSourceId)
        if (PointSourceId->isIgnored()) PointSourceId = 0;

    Time = schema.getDimensionPtr("Time", ns);
    if (Time)
        if (Time->isIgnored()) Time = 0;

    Red = schema.getDimensionPtr("Red", ns);
    if (Red)
        if (Red->isIgnored()) Red = 0;

    Green = schema.getDimensionPtr("Green", ns);
    if (Green)
        if (Green->isIgnored()) Green = 0;

    Blue = schema.getDimensionPtr("Blue", ns);
    if (Blue)
        if (Blue->isIgnored()) Blue = 0;


    // WavePacketDescriptorIndex = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WavePacketDescriptorIndex) : 0);
    // WaveformDataOffset = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformDataOffset) : 0);
    // ReturnPointWaveformLocation = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_ReturnPointWaveformLocation) : 0);
    // WaveformXt = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformXt) : 0);
    // WaveformYt = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformYt) : 0);
    // WaveformZt = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformZt) : 0);

    return;
}


void Support::rewriteHeader(std::ostream& stream, const SummaryData& data)
{
    // move from header start to "number of point records" field
    stream.seekp(107, std::ios_base::cur);

    {
        boost::uint8_t buf[256];
        boost::uint8_t* p = buf;

        Utils::write_field<boost::uint32_t>(p, data.getTotalNumPoints());

        for (int i=1; i<=SummaryData::s_maxNumReturns; i++)
        {
            Utils::write_field<boost::uint32_t>(p, data.getReturnCount(i));
        }

        Utils::write_n(stream, buf, 4 + 4*SummaryData::s_maxNumReturns);
    }

    // skip over scale/offset fields
    stream.seekp(8*6, std::ios_base::cur);

    {
        boost::uint8_t buf[256];
        boost::uint8_t* p = buf;

        double minX, minY, minZ, maxX, maxY, maxZ;
        data.getBounds(minX, minY, minZ, maxX, maxY, maxZ);
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
