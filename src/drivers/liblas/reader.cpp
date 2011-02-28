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


#include "reader.hpp"

#include <liblas/factory.hpp>
#include <liblas/bounds.hpp>

#include <libpc/exceptions.hpp>
#include "header.hpp"

namespace libpc
{
LiblasReader::LiblasReader(std::istream& istream)
    : Reader()
    , m_istream(istream)
    , m_externalReader(NULL)
    , m_versionMajor(0)
    , m_versionMinor(0)
    , m_scaleX(0.0)
    , m_scaleY(0.0)
    , m_scaleZ(0.0)
    , m_offsetX(0.0)
    , m_offsetY(0.0)
    , m_offsetZ(0.0)
    , m_isCompressed(false)
    , m_pointFormat(liblas::ePointFormatUnknown)
    , m_hasTimeData(false)
    , m_hasColorData(false)
    , m_hasWaveData(false)
{
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(m_istream);
    m_externalReader = new liblas::Reader(reader);

    LiblasHeader* myHeader = new LiblasHeader;
    setHeader(myHeader);

    processExternalHeader();

    registerFields();

    return;
}


LiblasReader::~LiblasReader()
{
    // BUG: this might be a smart pointer, so delete might not be needed
    delete m_externalReader;
}


void LiblasReader::processExternalHeader()
{
    const liblas::Header& externalHeader = m_externalReader->GetHeader();
    LiblasHeader& internalHeader = getLiblasHeader();

    internalHeader.setNumPoints( externalHeader.GetPointRecordsCount() );

    const liblas::Bounds<double>& externalBounds = externalHeader.GetExtent();
    const Bounds<double> internalBounds(externalBounds.minx(), externalBounds.miny(), externalBounds.minz(), externalBounds.maxx(), externalBounds.maxy(), externalBounds.maxz());
    internalHeader.setBounds(internalBounds);

    m_versionMajor = externalHeader.GetVersionMajor();
    m_versionMinor = externalHeader.GetVersionMinor();

    m_scaleX = externalHeader.GetScaleX();
    m_scaleY = externalHeader.GetScaleY();
    m_scaleZ = externalHeader.GetScaleZ();
    m_offsetX = externalHeader.GetOffsetX();
    m_offsetY = externalHeader.GetOffsetY();
    m_offsetZ = externalHeader.GetOffsetZ();

    m_isCompressed = externalHeader.Compressed();

    m_pointFormat = externalHeader.GetDataFormatId();

    m_hasTimeData = m_hasColorData = m_hasWaveData = false;
    switch (m_pointFormat)
    {
    case liblas::ePointFormat0:
        break;
    case liblas::ePointFormat1:
        m_hasTimeData = true;
        break;
    case liblas::ePointFormat2:
        m_hasColorData = true;
        break;
    case liblas::ePointFormat3:
        m_hasTimeData = true;
        m_hasColorData = true;
        break;
    case liblas::ePointFormat4:
        m_hasTimeData = true;
        m_hasWaveData = true;
        break;
    case liblas::ePointFormat5:
        m_hasColorData = true;
        m_hasTimeData = true;
        m_hasWaveData = true;
        break;
    }

    if (m_hasWaveData)
    {
        throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
    }

    return;
}

void LiblasReader::registerFields()
{
    const liblas::Header& externalHeader = m_externalReader->GetHeader();
    LiblasHeader& internalHeader = getLiblasHeader();
    Schema& schema = internalHeader.getSchema();

    Dimension xDim(Dimension::Field_X, Dimension::Int32);
    Dimension yDim(Dimension::Field_Y, Dimension::Int32);
    Dimension zDim(Dimension::Field_Z, Dimension::Int32);
    xDim.setNumericScale(externalHeader.GetScaleX());
    yDim.setNumericScale(externalHeader.GetScaleY());
    zDim.setNumericScale(externalHeader.GetScaleZ());
    xDim.setNumericOffset(externalHeader.GetOffsetX());
    yDim.setNumericOffset(externalHeader.GetOffsetY());
    zDim.setNumericOffset(externalHeader.GetOffsetZ());

    schema.addDimension(xDim);
    schema.addDimension(yDim);
    schema.addDimension(zDim);

    schema.addDimension(Dimension(Dimension::Field_Intensity, Dimension::Int16));
    schema.addDimension(Dimension(Dimension::Field_ReturnNumber, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_NumberOfReturns, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_ScanDirectionFlag, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_EdgeOfFlightLine, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_Classification, Dimension::Uint8));
    schema.addDimension(Dimension(Dimension::Field_ScanAngleRank, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_UserData, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_PointSourceId, Dimension::Uint16));

    if (m_hasTimeData)
    {
        schema.addDimension(Dimension(Dimension::Field_GpsTime, Dimension::Double));
    }

    if (m_hasColorData)
    {
        schema.addDimension(Dimension(Dimension::Field_Red, Dimension::Uint16));
        schema.addDimension(Dimension(Dimension::Field_Green, Dimension::Uint16));
        schema.addDimension(Dimension(Dimension::Field_Blue, Dimension::Uint16));
    }

    //if (m_hasWaveData)
    //{
    //    schema.addDimension(Dimension(Dimension::Field_WavePacketDescriptorIndex, Dimension::Uint8));
    //    schema.addDimension(Dimension(Dimension::Field_WaveformDataOffset, Dimension::Uint64));
    //    schema.addDimension(Dimension(Dimension::Field_ReturnPointWaveformLocation, Dimension::Uint32));
    //    schema.addDimension(Dimension(Dimension::Field_WaveformXt, Dimension::Float));
    //    schema.addDimension(Dimension(Dimension::Field_WaveformYt, Dimension::Float));
    //    schema.addDimension(Dimension(Dimension::Field_WaveformZt, Dimension::Float));
    //}
    
    return;
}


const LiblasHeader& LiblasReader::getLiblasHeader() const
{
    return (const LiblasHeader&)getHeader();
}


LiblasHeader& LiblasReader::getLiblasHeader()
{
    return (LiblasHeader&)getHeader();
}


void LiblasReader::seekToPoint(boost::uint64_t n)
{
    m_externalReader->Seek(n);
    return;
}


void LiblasReader::reset()
{
    m_externalReader->Reset();
}


boost::uint32_t LiblasReader::readPoints(PointData& pointData)
{
    boost::uint32_t numPoints = pointData.getNumPoints();
    boost::uint32_t i = 0;

    const std::size_t indexX = pointData.getDimensionIndex(Dimension::Field_X);
    const std::size_t indexY = pointData.getDimensionIndex(Dimension::Field_Y);
    const std::size_t indexZ = pointData.getDimensionIndex(Dimension::Field_Z);
    
    const std::size_t indexIntensity = pointData.getDimensionIndex(Dimension::Field_Intensity);
    const std::size_t indexReturnNumber = pointData.getDimensionIndex(Dimension::Field_ReturnNumber);
    const std::size_t indexNumberOfReturns = pointData.getDimensionIndex(Dimension::Field_NumberOfReturns);
    const std::size_t indexScanDirectionFlag = pointData.getDimensionIndex(Dimension::Field_ScanDirectionFlag);
    const std::size_t indexEdgeOfFlightLine = pointData.getDimensionIndex(Dimension::Field_EdgeOfFlightLine);
    const std::size_t indexClassification = pointData.getDimensionIndex(Dimension::Field_Classification);
    const std::size_t indexScanAngleRank = pointData.getDimensionIndex(Dimension::Field_ScanAngleRank);
    const std::size_t indexUserData = pointData.getDimensionIndex(Dimension::Field_UserData);
    const std::size_t indexPointSourceId = pointData.getDimensionIndex(Dimension::Field_PointSourceId);
    
    const std::size_t indexGpsTime = (m_hasTimeData ? pointData.getDimensionIndex(Dimension::Field_GpsTime) : 0);

    const std::size_t indexRed = (m_hasColorData ? pointData.getDimensionIndex(Dimension::Field_Red) : 0);
    const std::size_t indexGreen = (m_hasColorData ? pointData.getDimensionIndex(Dimension::Field_Green) : 0);
    const std::size_t indexBlue = (m_hasColorData ? pointData.getDimensionIndex(Dimension::Field_Blue) : 0);

    //const std::size_t indexWavePacketDescriptorIndex = (m_hasWaveData ? pointData.getDimensionIndex(Dimension::Field_WavePacketDescriptorIndex) : 0);
    //const std::size_t indexWaveformDataOffset = (m_hasWaveData ? pointData.getDimensionIndex(Dimension::Field_WaveformDataOffset) : 0);
    //const std::size_t indexReturnPointWaveformLocation = (m_hasWaveData ? pointData.getDimensionIndex(Dimension::Field_ReturnPointWaveformLocation) : 0);
    //const std::size_t indexWaveformXt = (m_hasWaveData ? pointData.getDimensionIndex(Dimension::Field_WaveformXt) : 0);
    //const std::size_t indexWaveformYt = (m_hasWaveData ? pointData.getDimensionIndex(Dimension::Field_WaveformYt) : 0);
    //const std::size_t indexWaveformZt = (m_hasWaveData ? pointData.getDimensionIndex(Dimension::Field_WaveformZt) : 0);

    for (i=0; i<numPoints; i++)
    {
        bool ok = m_externalReader->ReadNextPoint();
        assert(ok); // BUG: add error check
        const liblas::Point& pt = m_externalReader->GetPoint();

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
        
        pointData.setField(i, indexX, x);
        pointData.setField(i, indexY, y);
        pointData.setField(i, indexZ, z);

        pointData.setField(i, indexIntensity, intensity);
        pointData.setField(i, indexReturnNumber, returnNumber);
        pointData.setField(i, indexNumberOfReturns, numberOfReturns);
        pointData.setField(i, indexScanDirectionFlag, scanDirFlag);
        pointData.setField(i, indexEdgeOfFlightLine, edgeOfFlightLine);
        pointData.setField(i, indexClassification, classification);
        pointData.setField(i, indexScanAngleRank, scanAngleRank);
        pointData.setField(i, indexUserData, userData);
        pointData.setField(i, indexPointSourceId, pointSourceId);

        if (m_hasTimeData)
        {
            const double gpsTime = pt.GetTime();
            
            pointData.setField(i, indexGpsTime, gpsTime);
        }

        if (m_hasColorData)
        {
            const liblas::Color color = pt.GetColor();
            const boost::uint16_t red = color.GetRed();
            const boost::uint16_t green = color.GetGreen();
            const boost::uint16_t blue = color.GetBlue();

            pointData.setField(i, indexRed, red);
            pointData.setField(i, indexGreen, green);
            pointData.setField(i, indexBlue, blue);
        }

        if (m_hasWaveData)
        {
            throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
        }
        
    }

    return numPoints;
}

} // namespace libpc
