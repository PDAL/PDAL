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

#include <libpc/drivers/las/Writer.hpp>
#include "LasHeaderWriter.hpp"
#include <libpc/exceptions.hpp>

namespace libpc { namespace drivers { namespace las {



LasWriter::LasWriter(Stage& prevStage, std::ostream& ostream)
    : Writer(prevStage)
    , m_ostream(ostream)
{
    return;
}


const std::string& LasWriter::getName() const
{
    static std::string name("Liblas Writer");
    return name;
}


void LasWriter::writeBegin()
{
    // need to set properties of the header here, based on prev->getHeader() and on the user's preferences
    m_lasHeader.setBounds( getPrevStage().getHeader().getBounds() );
    m_lasHeader.SetOffset(0,0,0);
    m_lasHeader.SetScale(1,1,1);
    
    boost::uint32_t cnt = static_cast<boost::uint32_t>(m_targetNumPointsToWrite);
    m_lasHeader.SetPointRecordsCount(cnt);

    LasHeaderWriter lasHeaderWriter(m_lasHeader, m_ostream);
    lasHeaderWriter.write();

    return;
}


void LasWriter::writeEnd()
{
    return;
}


boost::uint32_t LasWriter::writeBuffer(const PointBuffer& PointBuffer)
{
    const SchemaLayout& schemaLayout = PointBuffer.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();
    LasHeader::PointFormatId pointFormat = m_lasHeader.getDataFormatId();

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

    boost::uint32_t numValidPoints = 0;

    boost::uint8_t buf[1024]; // BUG: fixed size

    for (boost::uint32_t pointIndex=0; pointIndex<PointBuffer.getNumPoints(); pointIndex++)
    {

        if (pointFormat == LasHeader::ePointFormat0)
        {
            boost::uint8_t* p = buf;

            const boost::uint32_t x = PointBuffer.getField<boost::uint32_t>(pointIndex, fieldIndexX);
            const boost::uint32_t y = PointBuffer.getField<boost::uint32_t>(pointIndex, fieldIndexY);
            const boost::uint32_t z = PointBuffer.getField<boost::uint32_t>(pointIndex, fieldIndexZ);
            const boost::uint16_t intensity = PointBuffer.getField<boost::uint16_t>(pointIndex, fieldIndexIntensity);
            
            const boost::uint8_t returnNum = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexReturnNum);
            const boost::uint8_t numReturns = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexNumReturns);
            const boost::uint8_t scanDirFlag = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexScanDir);
            const boost::uint8_t flight = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexFlight);

            const boost::uint8_t bits = returnNum & (numReturns<<3) & (scanDirFlag << 6) && (flight << 7);

            const boost::uint8_t classification = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexClassification);
            const boost::int8_t scanAngleRank = PointBuffer.getField<boost::int8_t>(pointIndex, fieldIndexScanAngle);
            const boost::uint8_t user = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexUserData);
            const boost::uint16_t pointSourceId = PointBuffer.getField<boost::uint16_t>(pointIndex, fieldIndexPointSource);

            Utils::write_field<boost::uint32_t>(p, x);
            Utils::write_field<boost::uint32_t>(p, y);
            Utils::write_field<boost::uint32_t>(p, z);
            Utils::write_field<boost::uint16_t>(p, intensity);
            Utils::write_field<boost::uint8_t>(p, bits);
            Utils::write_field<boost::uint8_t>(p, classification);
            Utils::write_field<boost::int8_t>(p, scanAngleRank);
            Utils::write_field<boost::uint8_t>(p, user);
            Utils::write_field<boost::uint16_t>(p, pointSourceId);

            Utils::write_n(m_ostream, buf, LasHeader::ePointSize0);
        }
        else if (pointFormat == LasHeader::ePointFormat1)
        {
            throw;
            //Utils::write_n(m_ostream, buf, LasHeader::ePointSize1);
        }
        else if (pointFormat == LasHeader::ePointFormat2)
        {
            throw;
            //Utils::write_n(m_ostream, buf, LasHeader::ePointSize2);
        }
        else if (pointFormat == LasHeader::ePointFormat3)
        {
            boost::uint8_t* p = buf;

            const boost::uint32_t x = PointBuffer.getField<boost::uint32_t>(pointIndex, fieldIndexX);
            const boost::uint32_t y = PointBuffer.getField<boost::uint32_t>(pointIndex, fieldIndexY);
            const boost::uint32_t z = PointBuffer.getField<boost::uint32_t>(pointIndex, fieldIndexZ);
            const boost::uint16_t intensity = PointBuffer.getField<boost::uint16_t>(pointIndex, fieldIndexIntensity);
            
            const boost::uint8_t returnNum = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexReturnNum);
            const boost::uint8_t numReturns = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexNumReturns);
            const boost::uint8_t scanDirFlag = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexScanDir);
            const boost::uint8_t flight = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexFlight);

            const boost::uint8_t bits = returnNum & (numReturns<<3) & (scanDirFlag << 6) && (flight << 7);

            const boost::uint8_t classification = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexClassification);
            const boost::int8_t scanAngleRank = PointBuffer.getField<boost::int8_t>(pointIndex, fieldIndexScanAngle);
            const boost::uint8_t user = PointBuffer.getField<boost::uint8_t>(pointIndex, fieldIndexUserData);
            const boost::uint16_t pointSourceId = PointBuffer.getField<boost::uint16_t>(pointIndex, fieldIndexPointSource);
            const double time = PointBuffer.getField<double>(pointIndex, fieldIndexTime);
            const boost::uint16_t red = PointBuffer.getField<boost::uint16_t>(pointIndex, fieldIndexRed);
            const boost::uint16_t green = PointBuffer.getField<boost::uint16_t>(pointIndex, fieldIndexGreen);
            const boost::uint16_t blue = PointBuffer.getField<boost::uint16_t>(pointIndex, fieldIndexBlue);

            Utils::write_field<boost::uint32_t>(p, x);
            Utils::write_field<boost::uint32_t>(p, y);
            Utils::write_field<boost::uint32_t>(p, z);
            Utils::write_field<boost::uint16_t>(p, intensity);
            Utils::write_field<boost::uint8_t>(p, bits);
            Utils::write_field<boost::uint8_t>(p, classification);
            Utils::write_field<boost::int8_t>(p, scanAngleRank);
            Utils::write_field<boost::uint8_t>(p, user);
            Utils::write_field<boost::uint16_t>(p, pointSourceId);
            Utils::write_field<double>(p, time);
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

} } } // namespaces
