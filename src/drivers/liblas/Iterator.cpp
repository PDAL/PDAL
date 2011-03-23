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


#include <libpc/drivers/liblas/Iterator.hpp>
#include <libpc/drivers/liblas/Reader.hpp>

#include <liblas/factory.hpp>
#include <liblas/bounds.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/drivers/liblas/header.hpp>

namespace libpc { namespace drivers { namespace liblas {


Iterator::Iterator(const LiblasReader& reader)
    : libpc::Iterator(reader)
    , m_stageAsDerived(reader)
{
    return;
}


void Iterator::skip(boost::uint64_t n)
{
    LiblasReader& reader = const_cast<LiblasReader&>(m_stageAsDerived); // BUG BUG BUG

    boost::uint64_t pos = getCurrentPointIndex() + n;

    size_t nn = (size_t)pos;
    reader.m_externalReader->Seek(nn);

    setCurrentPointIndex(nn);

    return;
}



bool Iterator::atEnd() const
{
    return getCurrentPointIndex() >= getStage().getNumPoints();
}


boost::uint32_t Iterator::read(PointBuffer& PointBuffer)
{
    LiblasReader& reader = const_cast<LiblasReader&>(m_stageAsDerived);     // BUG BUG BUG

    boost::uint32_t numPoints = PointBuffer.getCapacity();
    boost::uint32_t i = 0;

    const Schema& schema = PointBuffer.getSchema();

    const int indexX = schema.getDimensionIndex(Dimension::Field_X);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z);
    
    const int indexIntensity = schema.getDimensionIndex(Dimension::Field_Intensity);
    const int indexReturnNumber = schema.getDimensionIndex(Dimension::Field_ReturnNumber);
    const int indexNumberOfReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns);
    const int indexScanDirectionFlag = schema.getDimensionIndex(Dimension::Field_ScanDirectionFlag);
    const int indexEdgeOfFlightLine = schema.getDimensionIndex(Dimension::Field_EdgeOfFlightLine);
    const int indexClassification = schema.getDimensionIndex(Dimension::Field_Classification);
    const int indexScanAngleRank = schema.getDimensionIndex(Dimension::Field_ScanAngleRank);
    const int indexUserData = schema.getDimensionIndex(Dimension::Field_UserData);
    const int indexPointSourceId = schema.getDimensionIndex(Dimension::Field_PointSourceId);
    
    const int indexTime = (reader.m_hasTimeData ? schema.getDimensionIndex(Dimension::Field_Time) : 0);

    const int indexRed = (reader.m_hasColorData ? schema.getDimensionIndex(Dimension::Field_Red) : 0);
    const int indexGreen = (reader.m_hasColorData ? schema.getDimensionIndex(Dimension::Field_Green) : 0);
    const int indexBlue = (reader.m_hasColorData ? schema.getDimensionIndex(Dimension::Field_Blue) : 0);

    //const int indexWavePacketDescriptorIndex = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WavePacketDescriptorIndex) : 0);
    //const int indexWaveformDataOffset = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformDataOffset) : 0);
    //const int indexReturnPointWaveformLocation = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_ReturnPointWaveformLocation) : 0);
    //const int indexWaveformXt = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformXt) : 0);
    //const int indexWaveformYt = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformYt) : 0);
    //const t indexWaveformZt = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformZt) : 0);

    for (i=0; i<numPoints; i++)
    {
        bool ok = reader.m_externalReader->ReadNextPoint();
        if (!ok)
        {
            throw libpc_error("liblas reader failed to retrieve point");
        }

        const ::liblas::Point& pt = reader.m_externalReader->GetPoint();

        const boost::int32_t x = pt.GetRawX();
        const boost::int32_t y = pt.GetRawY();
        const boost::int32_t z = pt.GetRawZ();

        const boost::uint16_t intensity = pt.GetIntensity();
        const boost::int8_t returnNumber = (boost::int8_t)pt.GetReturnNumber();
        const boost::int8_t numberOfReturns = (boost::int8_t)pt.GetNumberOfReturns();
        const boost::int8_t scanDirFlag = (boost::int8_t)pt.GetScanDirection();
        const boost::int8_t edgeOfFlightLine = (boost::int8_t)pt.GetFlightLineEdge();
        const boost::uint8_t classification = pt.GetClassification().GetClass();
        const boost::int8_t scanAngleRank = pt.GetScanAngleRank();
        const boost::uint8_t userData = pt.GetUserData();
        const boost::uint16_t pointSourceId = pt.GetPointSourceID();
        
        PointBuffer.setField(i, indexX, x);
        PointBuffer.setField(i, indexY, y);
        PointBuffer.setField(i, indexZ, z);

        PointBuffer.setField(i, indexIntensity, intensity);
        PointBuffer.setField(i, indexReturnNumber, returnNumber);
        PointBuffer.setField(i, indexNumberOfReturns, numberOfReturns);
        PointBuffer.setField(i, indexScanDirectionFlag, scanDirFlag);
        PointBuffer.setField(i, indexEdgeOfFlightLine, edgeOfFlightLine);
        PointBuffer.setField(i, indexClassification, classification);
        PointBuffer.setField(i, indexScanAngleRank, scanAngleRank);
        PointBuffer.setField(i, indexUserData, userData);
        PointBuffer.setField(i, indexPointSourceId, pointSourceId);

        if (reader.m_hasTimeData)
        {
            const double time = pt.GetTime();
            
            PointBuffer.setField(i, indexTime, time);
        }

        if (reader.m_hasColorData)
        {
            const ::liblas::Color color = pt.GetColor();
            const boost::uint16_t red = color.GetRed();
            const boost::uint16_t green = color.GetGreen();
            const boost::uint16_t blue = color.GetBlue();

            PointBuffer.setField(i, indexRed, red);
            PointBuffer.setField(i, indexGreen, green);
            PointBuffer.setField(i, indexBlue, blue);
        }
        
        PointBuffer.setNumPoints(i+1);
        if (reader.m_hasWaveData)
        {
            throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
        }
        
    }

    incrementCurrentPointIndex(numPoints);

    return numPoints;
}



} } } // namespaces
