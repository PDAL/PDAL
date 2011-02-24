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

#include "libpc/LasWriter.hpp"
#include "libpc/LasHeaderWriter.hpp"

namespace libpc
{


LasWriter::LasWriter(Stage& prevStage, std::ostream& ostream)
    : Writer(prevStage)
    , m_ostream(ostream)
{
    LasHeader* lasHeader = new LasHeader;
    setHeader(lasHeader);

    return;
}


void LasWriter::writeBegin()
{
    Header& baseHeader = getHeader();
    LasHeader& lasHeader = (LasHeader&)baseHeader;

    // need to set properties of the header here, based on this->getHeader() and on the user's preferences
    lasHeader.setBounds( baseHeader.getBounds() );
    lasHeader.SetOffset(0,0,0);
    lasHeader.SetScale(1,1,1);
    
    boost::uint32_t cnt = static_cast<boost::uint32_t>(m_targetNumPointsToWrite);
    lasHeader.SetPointRecordsCount(cnt);

    LasHeaderWriter lasHeaderWriter(lasHeader, m_ostream);
    lasHeaderWriter.write();

    return;
}


void LasWriter::writeEnd()
{
    return;
}


boost::uint32_t LasWriter::writeBuffer(const PointData& pointData)
{
    Header& baseHeader = getHeader();
    LasHeader& lasHeader = (LasHeader&)baseHeader;

    const SchemaLayout& schemaLayout = pointData.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();
    LasHeader::PointFormatId pointFormat = lasHeader.getDataFormatId();

    const std::size_t fieldIndexX = schema.getDimensionIndex(Dimension::Field_X);
    const std::size_t fieldIndexY = schema.getDimensionIndex(Dimension::Field_Y);
    const std::size_t fieldIndexZ = schema.getDimensionIndex(Dimension::Field_Z);
    const std::size_t fieldIndexIntensity = schema.getDimensionIndex(Dimension::Field_Intensity);
    const std::size_t fieldIndexReturnNum = schema.getDimensionIndex(Dimension::Field_ReturnNumber);
    const std::size_t fieldIndexNumReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns);
    const std::size_t fieldIndexScanDir = schema.getDimensionIndex(Dimension::Field_ScanDirection);
    const std::size_t fieldIndexFlight = schema.getDimensionIndex(Dimension::Field_FlightLineEdge);
    const std::size_t fieldIndexClassification = schema.getDimensionIndex(Dimension::Field_Classification);
    const std::size_t fieldIndexScanAngle = schema.getDimensionIndex(Dimension::Field_ScanAngleRank);
    const std::size_t fieldIndexUserData = schema.getDimensionIndex(Dimension::Field_UserData);
    const std::size_t fieldIndexPointSource = schema.getDimensionIndex(Dimension::Field_PointSourceId);
    const std::size_t fieldIndexTime = schema.getDimensionIndex(Dimension::Field_Time);
    const std::size_t fieldIndexRed = schema.getDimensionIndex(Dimension::Field_Red);
    const std::size_t fieldIndexGreen = schema.getDimensionIndex(Dimension::Field_Green);
    const std::size_t fieldIndexBlue = schema.getDimensionIndex(Dimension::Field_Blue);

    boost::uint32_t numValidPoints = 0;

    for (boost::uint32_t pointIndex=0; pointIndex<pointData.getNumPoints(); pointIndex++)
    {
        if (pointData.isValid(pointIndex) == false) continue;

        boost::uint8_t buf[100]; // BUG: fixed size

        if (pointFormat == LasHeader::ePointFormat0)
        {
            Utils::write_n(m_ostream, buf, LasHeader::ePointSize0);
        }
        else if (pointFormat == LasHeader::ePointFormat1)
        {
            Utils::write_n(m_ostream, buf, LasHeader::ePointSize1);
        }
        else if (pointFormat == LasHeader::ePointFormat2)
        {
            Utils::write_n(m_ostream, buf, LasHeader::ePointSize2);
        }
        else if (pointFormat == LasHeader::ePointFormat3)
        {
            boost::uint8_t* p = buf;

            const boost::uint32_t x = pointData.getField<boost::uint32_t>(pointIndex, fieldIndexX);
            const boost::uint32_t y = pointData.getField<boost::uint32_t>(pointIndex, fieldIndexY);
            const boost::uint32_t z = pointData.getField<boost::uint32_t>(pointIndex, fieldIndexZ);
            const boost::uint16_t intensity = pointData.getField<boost::uint16_t>(pointIndex, fieldIndexIntensity);
            
            const boost::uint8_t returnNum = pointData.getField<boost::uint8_t>(pointIndex, fieldIndexReturnNum);
            const boost::uint8_t numReturns = pointData.getField<boost::uint8_t>(pointIndex, fieldIndexNumReturns);
            const boost::uint8_t scanDirFlag = pointData.getField<boost::uint8_t>(pointIndex, fieldIndexScanDir);
            const boost::uint8_t flight = pointData.getField<boost::uint8_t>(pointIndex, fieldIndexFlight);

            const boost::uint8_t bits = returnNum & (numReturns<<3) & (scanDirFlag << 6) && (flight << 7);

            const boost::uint8_t classification = pointData.getField<boost::uint8_t>(pointIndex, fieldIndexClassification);
            const boost::int8_t scanAngleRank = pointData.getField<boost::int8_t>(pointIndex, fieldIndexScanAngle);
            const boost::uint8_t user = pointData.getField<boost::uint8_t>(pointIndex, fieldIndexUserData);
            const boost::uint16_t pointSourceId = pointData.getField<boost::uint16_t>(pointIndex, fieldIndexPointSource);
            const double gpsTime = pointData.getField<double>(pointIndex, fieldIndexTime);
            const boost::uint16_t red = pointData.getField<boost::uint16_t>(pointIndex, fieldIndexRed);
            const boost::uint16_t green = pointData.getField<boost::uint16_t>(pointIndex, fieldIndexGreen);
            const boost::uint16_t blue = pointData.getField<boost::uint16_t>(pointIndex, fieldIndexBlue);


            Utils::write_field<boost::uint32_t>(p, x);
            Utils::write_field<boost::uint32_t>(p, y);
            Utils::write_field<boost::uint32_t>(p, z);
            Utils::write_field<boost::uint16_t>(p, intensity);
            Utils::write_field<boost::uint8_t>(p, bits);
            Utils::write_field<boost::uint8_t>(p, classification);
            Utils::write_field<boost::int8_t>(p, scanAngleRank);
            Utils::write_field<boost::uint8_t>(p, user);
            Utils::write_field<boost::uint16_t>(p, pointSourceId);
            Utils::write_field<double>(p, gpsTime);
            Utils::write_field<boost::uint16_t>(p, red);
            Utils::write_field<boost::uint16_t>(p, green);
            Utils::write_field<boost::uint16_t>(p, blue);

            Utils::write_n(m_ostream, buf, LasHeader::ePointSize3);
        }
        else
        {
            throw;
        }

        ++numValidPoints;
    }

    //std::vector<boost::uint8_t> const& data = point.GetData();    
    //detail::write_n(m_ofs, data.front(), m_header->GetDataRecordLength());

    return numValidPoints;
}

} // namespace libpc
