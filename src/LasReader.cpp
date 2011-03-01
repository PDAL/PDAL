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

#include "libpc/LasReader.hpp"
#include "libpc/LasHeaderReader.hpp"

namespace libpc
{


LasReader::LasReader(std::istream& istream)
    : Reader()
    , m_istream(istream)
{
    LasHeader* lasHeader = new LasHeader;
    setHeader(lasHeader);
    
    LasHeaderReader lasHeaderReader(*lasHeader, istream);
    lasHeaderReader.read();


    return;
}


const std::string& LasReader::getName() const
{
    static std::string name("Las Reader");
    return name;
}


const LasHeader& LasReader::getLasHeader() const
{
    return (const LasHeader&)getHeader();
}


LasHeader& LasReader::getLasHeader()
{
    return (LasHeader&)getHeader();
}


void LasReader::seekToPoint(boost::uint64_t pointNum)
{
    reset();

    boost::uint32_t chunk = (boost::uint32_t)pointNum; // BUG: this needs to be done in blocks if pointNum is large

    PointData pointData(getHeader().getSchema(), chunk);
    readPoints(pointData);

    // just drop the points on the floor and return
    return;
}


void LasReader::reset()
{
    m_currentPointIndex = 0;
    m_numPointsRead = 0;
}


boost::uint32_t LasReader::readPoints(PointData& pointData)
{
    boost::uint32_t numPoints = pointData.getNumPoints();

    const LasHeader& lasHeader = getLasHeader();
    const SchemaLayout& schemaLayout = pointData.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();
    LasHeader::PointFormatId pointFormat = lasHeader.getDataFormatId();

    const std::size_t fieldIndexX = schema.getDimensionIndex(Dimension::Field_X);
    const std::size_t fieldIndexY = schema.getDimensionIndex(Dimension::Field_Y);
    const std::size_t fieldIndexZ = schema.getDimensionIndex(Dimension::Field_Z);
    const std::size_t fieldIndexIntensity = schema.getDimensionIndex(Dimension::Field_Intensity);
    const std::size_t fieldIndexReturnNum = schema.getDimensionIndex(Dimension::Field_ReturnNumber);
    const std::size_t fieldIndexNumReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns);
    const std::size_t fieldIndexScanDir = schema.getDimensionIndex(Dimension::Field_ScanDirectionFlag);
    const std::size_t fieldIndexFlight = schema.getDimensionIndex(Dimension::Field_EdgeOfFlightLine);
    const std::size_t fieldIndexClassification = schema.getDimensionIndex(Dimension::Field_Classification);
    const std::size_t fieldIndexScanAngle = schema.getDimensionIndex(Dimension::Field_ScanAngleRank);
    const std::size_t fieldIndexUserData = schema.getDimensionIndex(Dimension::Field_UserData);
    const std::size_t fieldIndexPointSource = schema.getDimensionIndex(Dimension::Field_PointSourceId);
    const std::size_t fieldIndexTime = schema.getDimensionIndex(Dimension::Field_GpsTime);
    const std::size_t fieldIndexRed = schema.getDimensionIndex(Dimension::Field_Red);
    const std::size_t fieldIndexGreen = schema.getDimensionIndex(Dimension::Field_Green);
    const std::size_t fieldIndexBlue = schema.getDimensionIndex(Dimension::Field_Blue);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        boost::uint8_t buf[34];

        if (pointFormat == LasHeader::ePointFormat0)
        {
            Utils::read_n(buf, m_istream, LasHeader::ePointSize0);
        }
        else if (pointFormat == LasHeader::ePointFormat1)
        {
            Utils::read_n(buf, m_istream, LasHeader::ePointSize1);
        }
        else if (pointFormat == LasHeader::ePointFormat2)
        {
            Utils::read_n(buf, m_istream, LasHeader::ePointSize2);
        }
        else if (pointFormat == LasHeader::ePointFormat3)
        {
            Utils::read_n(buf, m_istream, LasHeader::ePointSize3);

            boost::uint8_t* p = buf;

            const boost::uint32_t x = Utils::read_field<boost::uint32_t>(p);
            const boost::uint32_t y = Utils::read_field<boost::uint32_t>(p);
            const boost::uint32_t z = Utils::read_field<boost::uint32_t>(p);
            const boost::uint16_t intensity = Utils::read_field<boost::uint16_t>(p);
            const boost::uint8_t flags = Utils::read_field<boost::uint8_t>(p);
            const boost::uint8_t classification = Utils::read_field<boost::uint8_t>(p);
            const boost::int8_t scanAngleRank = Utils::read_field<boost::int8_t>(p);
            const boost::uint8_t user = Utils::read_field<boost::uint8_t>(p);
            const boost::uint16_t pointSourceId = Utils::read_field<boost::uint16_t>(p);
            const double gpsTime = Utils::read_field<double>(p);
            const boost::uint16_t red = Utils::read_field<boost::uint16_t>(p);
            const boost::uint16_t green = Utils::read_field<boost::uint16_t>(p);
            const boost::uint16_t blue = Utils::read_field<boost::uint16_t>(p);

            const boost::uint8_t returnNum = flags & 0x03;
            const boost::uint8_t numReturns = (flags >> 3) & 0x03;
            const boost::uint8_t scanDirFlag = (flags >> 6) & 0x01;
            const boost::uint8_t flight = (flags >> 7) & 0x01;

            pointData.setField<boost::uint32_t>(pointIndex, fieldIndexX, x);
            pointData.setField<boost::uint32_t>(pointIndex, fieldIndexY, y);
            pointData.setField<boost::uint32_t>(pointIndex, fieldIndexZ, z);
            pointData.setField<boost::uint16_t>(pointIndex, fieldIndexIntensity, intensity);
            pointData.setField<boost::uint8_t>(pointIndex, fieldIndexReturnNum, returnNum);
            pointData.setField<boost::uint8_t>(pointIndex, fieldIndexNumReturns, numReturns);
            pointData.setField<boost::uint8_t>(pointIndex, fieldIndexScanDir, scanDirFlag);
            pointData.setField<boost::uint8_t>(pointIndex, fieldIndexFlight, flight);
            pointData.setField<boost::uint8_t>(pointIndex, fieldIndexClassification, classification);
            pointData.setField<boost::int8_t>(pointIndex, fieldIndexScanAngle, scanAngleRank);
            pointData.setField<boost::uint8_t>(pointIndex, fieldIndexUserData, user);
            pointData.setField<boost::uint16_t>(pointIndex, fieldIndexPointSource, pointSourceId);
            pointData.setField<double>(pointIndex, fieldIndexTime, gpsTime);
            pointData.setField<boost::uint16_t>(pointIndex, fieldIndexRed, red);
            pointData.setField<boost::uint16_t>(pointIndex, fieldIndexGreen, green);
            pointData.setField<boost::uint16_t>(pointIndex, fieldIndexBlue, blue);

            pointData.setValid(pointIndex);
        }
        else
        {
            throw;
        }
    }

    return numPoints;
}

} // namespace libpc
