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

#include <libpc/drivers/las/Iterator.hpp>
#include <libpc/drivers/las/Reader.hpp>
#include "LasHeaderReader.hpp"
#include <libpc/exceptions.hpp>

namespace libpc { namespace drivers { namespace las {


Iterator::Iterator(const LasReader& reader)
    : libpc::Iterator(reader)
    , m_stageAsDerived(reader)
{
    return;
}


void Iterator::skip(boost::uint64_t pointNum)
{
    const LasReader& reader = m_stageAsDerived;
    const Header& header = reader.getHeader();

    incrementCurrentPointIndex(pointNum);

    boost::uint32_t chunk = (boost::uint32_t)pointNum; // BUG: this needs to be done in blocks if pointNum is large

    // BUG: we can move the stream a constant amount
    PointBuffer PointBuffer(header.getSchema(), chunk);
    read(PointBuffer);
    // just drop the points on the floor and return
    
    return;
}


bool Iterator::atEnd() const
{
    return getCurrentPointIndex() >= getStage().getNumPoints();
}


boost::uint32_t Iterator::read(PointBuffer& PointBuffer)
{
    LasReader& reader = const_cast<LasReader&>(m_stageAsDerived);       // BUG BUG BUG

    boost::uint32_t numPoints = PointBuffer.getCapacity();

    const LasHeader& lasHeader = reader.getLasHeader();
    const SchemaLayout& schemaLayout = PointBuffer.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();
    LasHeader::PointFormatId pointFormat = lasHeader.getDataFormatId();

    bool hasTimeData = false;
    bool hasColorData = false;
    bool hasWaveData = false;
    switch (pointFormat)
    {
    case LasHeader::ePointFormat0:
        break;
    case LasHeader::ePointFormat1:
        hasTimeData = true;
        break;
    case LasHeader::ePointFormat2:
        hasColorData = true;
        break;
    case LasHeader::ePointFormat3:
        hasTimeData = true;
        hasColorData = true;
        break;
    case LasHeader::ePointFormat4:
        hasTimeData = true;
        hasWaveData = true;
        break;
    case LasHeader::ePointFormat5:
        hasColorData = true;
        hasTimeData = true;
        hasWaveData = true;
        break;
    case LasHeader::ePointFormatUnknown:
        throw not_yet_implemented("Unknown point format encountered");
    }

    const int fieldIndexX = schema.getDimensionIndex(Dimension::Field_X);
    const int fieldIndexY = schema.getDimensionIndex(Dimension::Field_Y);
    const int fieldIndexZ = schema.getDimensionIndex(Dimension::Field_Z);
    
    const int fieldIndexIntensity = schema.getDimensionIndex(Dimension::Field_Intensity);
    const int fieldIndexReturnNum = schema.getDimensionIndex(Dimension::Field_ReturnNumber);
    const int fieldIndexNumReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns);
    const int fieldIndexScanDir = schema.getDimensionIndex(Dimension::Field_ScanDirectionFlag);
    const int fieldIndexFlight = schema.getDimensionIndex(Dimension::Field_EdgeOfFlightLine);
    const int fieldIndexClassification = schema.getDimensionIndex(Dimension::Field_Classification);
    const int fieldIndexScanAngle = schema.getDimensionIndex(Dimension::Field_ScanAngleRank);
    const int fieldIndexUserData = schema.getDimensionIndex(Dimension::Field_UserData);
    const int fieldIndexPointSource = schema.getDimensionIndex(Dimension::Field_PointSourceId);

    const int fieldIndexTime = (hasTimeData ? schema.getDimensionIndex(Dimension::Field_Time) : 0);

    const int fieldIndexRed = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Red) : 0);
    const int fieldIndexGreen = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Green) : 0);
    const int fieldIndexBlue = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Blue) : 0);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        boost::uint8_t buf[34];

        if (pointFormat == LasHeader::ePointFormat0)
        {
            Utils::read_n(buf, reader.m_istream, LasHeader::ePointSize0);

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

            const boost::uint8_t returnNum = flags & 0x03;
            const boost::uint8_t numReturns = (flags >> 3) & 0x03;
            const boost::uint8_t scanDirFlag = (flags >> 6) & 0x01;
            const boost::uint8_t flight = (flags >> 7) & 0x01;

            PointBuffer.setField<boost::uint32_t>(pointIndex, fieldIndexX, x);
            PointBuffer.setField<boost::uint32_t>(pointIndex, fieldIndexY, y);
            PointBuffer.setField<boost::uint32_t>(pointIndex, fieldIndexZ, z);
            PointBuffer.setField<boost::uint16_t>(pointIndex, fieldIndexIntensity, intensity);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexReturnNum, returnNum);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexNumReturns, numReturns);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexScanDir, scanDirFlag);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexFlight, flight);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexClassification, classification);
            PointBuffer.setField<boost::int8_t>(pointIndex, fieldIndexScanAngle, scanAngleRank);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexUserData, user);
            PointBuffer.setField<boost::uint16_t>(pointIndex, fieldIndexPointSource, pointSourceId);

        }
        else if (pointFormat == LasHeader::ePointFormat1)
        {
            throw;
            //Utils::read_n(buf, m_istream, LasHeader::ePointSize1);
        }
        else if (pointFormat == LasHeader::ePointFormat2)
        {
            throw;
            //Utils::read_n(buf, m_istream, LasHeader::ePointSize2);
        }
        else if (pointFormat == LasHeader::ePointFormat3)
        {
            Utils::read_n(buf, reader.m_istream, LasHeader::ePointSize3);

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
            const double time = Utils::read_field<double>(p);
            const boost::uint16_t red = Utils::read_field<boost::uint16_t>(p);
            const boost::uint16_t green = Utils::read_field<boost::uint16_t>(p);
            const boost::uint16_t blue = Utils::read_field<boost::uint16_t>(p);

            const boost::uint8_t returnNum = flags & 0x03;
            const boost::uint8_t numReturns = (flags >> 3) & 0x03;
            const boost::uint8_t scanDirFlag = (flags >> 6) & 0x01;
            const boost::uint8_t flight = (flags >> 7) & 0x01;

            PointBuffer.setField<boost::uint32_t>(pointIndex, fieldIndexX, x);
            PointBuffer.setField<boost::uint32_t>(pointIndex, fieldIndexY, y);
            PointBuffer.setField<boost::uint32_t>(pointIndex, fieldIndexZ, z);
            PointBuffer.setField<boost::uint16_t>(pointIndex, fieldIndexIntensity, intensity);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexReturnNum, returnNum);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexNumReturns, numReturns);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexScanDir, scanDirFlag);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexFlight, flight);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexClassification, classification);
            PointBuffer.setField<boost::int8_t>(pointIndex, fieldIndexScanAngle, scanAngleRank);
            PointBuffer.setField<boost::uint8_t>(pointIndex, fieldIndexUserData, user);
            PointBuffer.setField<boost::uint16_t>(pointIndex, fieldIndexPointSource, pointSourceId);
            PointBuffer.setField<double>(pointIndex, fieldIndexTime, time);
            PointBuffer.setField<boost::uint16_t>(pointIndex, fieldIndexRed, red);
            PointBuffer.setField<boost::uint16_t>(pointIndex, fieldIndexGreen, green);
            PointBuffer.setField<boost::uint16_t>(pointIndex, fieldIndexBlue, blue);
            
        }
        
        else
        {
            throw;
        }
        PointBuffer.setNumPoints(pointIndex+1);
        
    }

    incrementCurrentPointIndex(numPoints);

    return numPoints;
}


} } } // namespaces
