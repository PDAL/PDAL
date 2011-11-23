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

#include <pdal/drivers/las/Support.hpp>

#include <pdal/drivers/las/SummaryData.hpp>


namespace pdal { namespace drivers { namespace las {

void Support::registerFields(Schema& schema, PointFormat format)
{
    std::ostringstream text;

    Dimension x(DimensionId::X_i32);
    schema.appendDimension(x);

    Dimension y(DimensionId::Y_i32);
    schema.appendDimension(y);

    Dimension z(DimensionId::Z_i32);
    schema.appendDimension(z);

    Dimension intensity(DimensionId::Las_Intensity);
    schema.appendDimension(intensity);

    Dimension return_no(DimensionId::Las_ReturnNumber); // 3 bits only
    schema.appendDimension(return_no);

    Dimension no_returns(DimensionId::Las_NumberOfReturns); // 3 bits only
    schema.appendDimension(no_returns);

    Dimension scan_dir(DimensionId::Las_ScanDirectionFlag); // 1 bit only
    schema.appendDimension(scan_dir);

    Dimension edge(DimensionId::Las_EdgeOfFlightLine); // 1 bit only
    schema.appendDimension(edge);

    Dimension classification(DimensionId::Las_Classification);
    schema.appendDimension(classification);

    Dimension scan_angle(DimensionId::Las_ScanAngleRank);
    schema.appendDimension(scan_angle);

    Dimension user_data(DimensionId::Las_UserData);
    schema.appendDimension(user_data);

    Dimension point_source_id(DimensionId::Las_PointSourceId);
    schema.appendDimension(point_source_id);

    if (hasTime(format))
    {
        Dimension t(DimensionId::Las_Time);
        schema.appendDimension(t);
    }

    if (hasColor(format))
    {
        Dimension red(DimensionId::Red_u16);
        schema.appendDimension(red);

        Dimension green(DimensionId::Green_u16);
        schema.appendDimension(green);

        Dimension blue(DimensionId::Blue_u16);
        schema.appendDimension(blue);
    }

    if (hasWave(format))
    {
        schema.appendDimension(Dimension(DimensionId::Las_WavePacketDescriptorIndex));
        schema.appendDimension(Dimension(DimensionId::Las_WaveformDataOffset));
        schema.appendDimension(Dimension(DimensionId::Las_ReturnPointWaveformLocation));
        schema.appendDimension(Dimension(DimensionId::Las_WaveformXt));
        schema.appendDimension(Dimension(DimensionId::Las_WaveformYt));
        schema.appendDimension(Dimension(DimensionId::Las_WaveformZt));
    }
    
    return;
}


void Support::setScaling(   Schema& schema, 
                            double scaleX, 
                            double scaleY, 
                            double scaleZ, 
                            double offsetX, 
                            double offsetY, 
                            double offsetZ)
{
    Dimension dimX = schema.getDimension(DimensionId::X_i32);
    Dimension dimY = schema.getDimension(DimensionId::Y_i32);
    Dimension dimZ = schema.getDimension(DimensionId::Z_i32);

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
    case PointFormat0: return 20;
    case PointFormat1: return 28;
    case PointFormat2: return 26;
    case PointFormat3: return 34;
    default:
        throw invalid_format("point format unsupported");
    }

}


// --------------------------------------------------------------------------
// LasPointIndexes
// --------------------------------------------------------------------------

PointIndexes::PointIndexes(const Schema& schema, PointFormat format)
{
    X = schema.getDimensionIndex(DimensionId::X_i32);
    Y = schema.getDimensionIndex(DimensionId::Y_i32);
    Z = schema.getDimensionIndex(DimensionId::Z_i32);
    
    Intensity = schema.getDimensionIndex(DimensionId::Las_Intensity);
    ReturnNumber = schema.getDimensionIndex(DimensionId::Las_ReturnNumber);
    NumberOfReturns = schema.getDimensionIndex(DimensionId::Las_NumberOfReturns);
    ScanDirectionFlag = schema.getDimensionIndex(DimensionId::Las_ScanDirectionFlag);
    EdgeOfFlightLine = schema.getDimensionIndex(DimensionId::Las_EdgeOfFlightLine);
    Classification = schema.getDimensionIndex(DimensionId::Las_Classification);
    ScanAngleRank = schema.getDimensionIndex(DimensionId::Las_ScanAngleRank);
    UserData = schema.getDimensionIndex(DimensionId::Las_UserData);
    PointSourceId = schema.getDimensionIndex(DimensionId::Las_PointSourceId);
    
    Time = (Support::hasTime(format) ? schema.getDimensionIndex(DimensionId::Las_Time) : 0);
    
    Red = (Support::hasColor(format) ? schema.getDimensionIndex(DimensionId::Red_u16) : 0);
    Green = (Support::hasColor(format) ? schema.getDimensionIndex(DimensionId::Green_u16) : 0);
    Blue = (Support::hasColor(format) ? schema.getDimensionIndex(DimensionId::Blue_u16) : 0);
        
    // WavePacketDescriptorIndex = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WavePacketDescriptorIndex) : 0);
    // WaveformDataOffset = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformDataOffset) : 0);
    // ReturnPointWaveformLocation = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_ReturnPointWaveformLocation) : 0);
    // WaveformXt = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformXt) : 0);
    // WaveformYt = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformYt) : 0);
    // WaveformZt = (Support::hasWave(format) ? schema.getDimensionIndex(DimensionId::Las_WaveformZt) : 0);

    return;
}

PointPositions::PointPositions(const Schema& schema, PointFormat format)
{

    X = schema.getDimension(DimensionId::X_i32).getByteOffset();
    Y = schema.getDimension(DimensionId::Y_i32).getByteOffset();
    Z = schema.getDimension(DimensionId::Z_i32).getByteOffset();
    
    Intensity = schema.getDimension(DimensionId::Las_Intensity).getByteOffset();
    ReturnNumber = schema.getDimension(DimensionId::Las_ReturnNumber).getByteOffset();
    NumberOfReturns = schema.getDimension(DimensionId::Las_NumberOfReturns).getByteOffset();
    ScanDirectionFlag = schema.getDimension(DimensionId::Las_ScanDirectionFlag).getByteOffset();
    EdgeOfFlightLine = schema.getDimension(DimensionId::Las_EdgeOfFlightLine).getByteOffset();
    Classification = schema.getDimension(DimensionId::Las_Classification).getByteOffset();
    ScanAngleRank = schema.getDimension(DimensionId::Las_ScanAngleRank).getByteOffset();
    UserData = schema.getDimension(DimensionId::Las_UserData).getByteOffset();
    PointSourceId = schema.getDimension(DimensionId::Las_PointSourceId).getByteOffset();
    
    Time = (Support::hasTime(format) ? schema.getDimension(DimensionId::Las_Time).getByteOffset() : 0);
    
    Red = (Support::hasColor(format) ? schema.getDimension(DimensionId::Red_u16).getByteOffset() : 0);
    Green = (Support::hasColor(format) ? schema.getDimension(DimensionId::Green_u16).getByteOffset() : 0);
    Blue = (Support::hasColor(format) ? schema.getDimension(DimensionId::Blue_u16).getByteOffset() : 0);
        
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


} } } // namespaces
