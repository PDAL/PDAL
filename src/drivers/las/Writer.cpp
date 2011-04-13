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

#include <libpc/drivers/las/Writer.hpp>

#include "LasHeaderWriter.hpp"
#include <libpc/exceptions.hpp>
#include <libpc/Stage.hpp>
#include <libpc/SchemaLayout.hpp>
#include <libpc/PointBuffer.hpp>

#include <iostream>

namespace libpc { namespace drivers { namespace las {



LasWriter::LasWriter(Stage& prevStage, std::ostream& ostream)
    : Writer(prevStage)
    , m_ostream(ostream)
    , m_numPointsWritten(0)
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
    m_lasHeader.setBounds( getPrevStage().getBounds() );

    const Schema& schema = getPrevStage().getSchema();

    int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    const Dimension& dimX = schema.getDimension(indexX);
    const Dimension& dimY = schema.getDimension(indexY);
    const Dimension& dimZ = schema.getDimension(indexZ);

    m_lasHeader.SetScale(dimX.getNumericScale(), dimY.getNumericScale(), dimZ.getNumericScale());
    m_lasHeader.SetOffset(dimX.getNumericOffset(), dimY.getNumericOffset(), dimZ.getNumericOffset());


    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(8);

    std::cout << "scale: " << dimX.getNumericScale() <<" " << dimY.getNumericScale() << " " << dimZ.getNumericScale() << std::endl;
    boost::uint32_t cnt = static_cast<boost::uint32_t>(m_targetNumPointsToWrite);
    m_lasHeader.SetPointRecordsCount(cnt);

    LasHeaderWriter lasHeaderWriter(m_lasHeader, m_ostream);
    lasHeaderWriter.write();

    return;
}


void LasWriter::writeEnd()
{
    m_lasHeader.SetPointRecordsCount(m_numPointsWritten);

    std::streamsize const dataPos = 107; 
    m_ostream.seekp(dataPos, std::ios::beg);
    LasHeaderWriter lasHeaderWriter(m_lasHeader, m_ostream);
    Utils::write_n(m_ostream, m_numPointsWritten, sizeof(m_numPointsWritten));
        
    return;
}


boost::uint32_t LasWriter::writeBuffer(const PointBuffer& PointBuffer)
{
    const SchemaLayout& schemaLayout = PointBuffer.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();
    PointFormat pointFormat = m_lasHeader.getPointFormat();

    const PointIndexes indexes(schema, pointFormat);

    boost::uint32_t numValidPoints = 0;

    boost::uint8_t buf[1024]; // BUG: fixed size

    for (boost::uint32_t pointIndex=0; pointIndex<PointBuffer.getNumPoints(); pointIndex++)
    {

        if (pointFormat == PointFormat0)
        {
            boost::uint8_t* p = buf;

            const boost::uint32_t x = PointBuffer.getField<boost::uint32_t>(pointIndex, indexes.X);
            const boost::uint32_t y = PointBuffer.getField<boost::uint32_t>(pointIndex, indexes.Y);
            const boost::uint32_t z = PointBuffer.getField<boost::uint32_t>(pointIndex, indexes.Z);
            const boost::uint16_t intensity = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.Intensity);
            
            const boost::uint8_t returnNum = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.ReturnNumber);
            const boost::uint8_t numReturns = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.NumberOfReturns);
            const boost::uint8_t scanDirFlag = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.ScanDirectionFlag);
            const boost::uint8_t flight = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.EdgeOfFlightLine);

            const boost::uint8_t bits = returnNum & (numReturns<<3) & (scanDirFlag << 6) && (flight << 7);

            const boost::uint8_t classification = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.Classification);
            const boost::int8_t scanAngleRank = PointBuffer.getField<boost::int8_t>(pointIndex, indexes.ScanAngleRank);
            const boost::uint8_t user = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.UserData);
            const boost::uint16_t pointSourceId = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.PointSourceId);

            Utils::write_field<boost::uint32_t>(p, x);
            Utils::write_field<boost::uint32_t>(p, y);
            Utils::write_field<boost::uint32_t>(p, z);
            Utils::write_field<boost::uint16_t>(p, intensity);
            Utils::write_field<boost::uint8_t>(p, bits);
            Utils::write_field<boost::uint8_t>(p, classification);
            Utils::write_field<boost::int8_t>(p, scanAngleRank);
            Utils::write_field<boost::uint8_t>(p, user);
            Utils::write_field<boost::uint16_t>(p, pointSourceId);

            Utils::write_n(m_ostream, buf, Support::getPointDataSize(pointFormat));
        }
        else if (pointFormat == PointFormat1)
        {
            throw;
            //Utils::write_n(m_ostream, buf, LasHeader::ePointSize1);
        }
        else if (pointFormat == PointFormat2)
        {
            throw;
            //Utils::write_n(m_ostream, buf, LasHeader::ePointSize2);
        }
        else if (pointFormat == PointFormat3)
        {
            boost::uint8_t* p = buf;

            const boost::uint32_t x = PointBuffer.getField<boost::uint32_t>(pointIndex, indexes.X);
            const boost::uint32_t y = PointBuffer.getField<boost::uint32_t>(pointIndex, indexes.Y);
            const boost::uint32_t z = PointBuffer.getField<boost::uint32_t>(pointIndex, indexes.Z);
            const boost::uint16_t intensity = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.Intensity);
            
            const boost::uint8_t returnNum = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.ReturnNumber);
            const boost::uint8_t numReturns = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.NumberOfReturns);
            const boost::uint8_t scanDirFlag = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.ScanDirectionFlag);
            const boost::uint8_t flight = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.EdgeOfFlightLine);

            const boost::uint8_t bits = returnNum & (numReturns<<3) & (scanDirFlag << 6) && (flight << 7);

            const boost::uint8_t classification = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.Classification);
            const boost::int8_t scanAngleRank = PointBuffer.getField<boost::int8_t>(pointIndex, indexes.ScanAngleRank);
            const boost::uint8_t user = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.UserData);
            const boost::uint16_t pointSourceId = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.PointSourceId);
            const double time = PointBuffer.getField<double>(pointIndex, indexes.Time);
            const boost::uint16_t red = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.Red);
            const boost::uint16_t green = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.Green);
            const boost::uint16_t blue = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.Blue);

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

            Utils::write_n(m_ostream, buf, PointSize3);
        }
        else
        {
            throw;
        }

        ++numValidPoints;
    }

    //std::vector<boost::uint8_t> const& data = point.GetData();    
    //detail::write_n(m_ofs, data.front(), m_header->GetDataRecordLength());
    
    m_numPointsWritten = m_numPointsWritten+numValidPoints;
    return numValidPoints;
}

} } } // namespaces
