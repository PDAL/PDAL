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


#include <libpc/drivers/liblas/reader.hpp>

#include <liblas/factory.hpp>
#include <liblas/bounds.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/drivers/liblas/header.hpp>

namespace libpc { namespace drivers { namespace liblas {

LiblasReader::LiblasReader(std::istream& istream)
    : Stage()
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
    , m_pointFormatNumber(-1)
    , m_hasTimeData(false)
    , m_hasColorData(false)
    , m_hasWaveData(false)
{
    ::liblas::ReaderFactory f;
    ::liblas::Reader reader = f.CreateWithStream(m_istream);
    m_externalReader = new ::liblas::Reader(reader);

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


const std::string& LiblasReader::getName() const
{
    static std::string name("Liblas Reader");
    return name;
}


boost::int8_t LiblasReader::getPointFormatNumber() const
{
    return m_pointFormatNumber;
}

void LiblasReader::processExternalHeader()
{
    const ::liblas::Header& externalHeader = m_externalReader->GetHeader();
    LiblasHeader& internalHeader = getLiblasHeader();

    internalHeader.setNumPoints( externalHeader.GetPointRecordsCount() );

    const ::liblas::Bounds<double>& externalBounds = externalHeader.GetExtent();
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

    m_pointFormatNumber = (boost::int8_t)externalHeader.GetDataFormatId();

    m_hasTimeData = m_hasColorData = m_hasWaveData = false;
    switch (m_pointFormatNumber)
    {
    case 0:
        break;
    case 1:
        m_hasTimeData = true;
        break;
    case 2:
        m_hasColorData = true;
        break;
    case 3:
        m_hasTimeData = true;
        m_hasColorData = true;
        break;
    case 4:
        m_hasTimeData = true;
        m_hasWaveData = true;
        break;
    case 5:
        m_hasColorData = true;
        m_hasTimeData = true;
        m_hasWaveData = true;
        break;
    default:
        throw not_yet_implemented("Unknown point format encountered");
    }

    if (m_hasWaveData)
    {
        throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
    }

    return;
}

void LiblasReader::registerFields()
{
    const ::liblas::Header& externalHeader = m_externalReader->GetHeader();
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

    schema.addDimension(Dimension(Dimension::Field_Intensity, Dimension::Uint16));
    schema.addDimension(Dimension(Dimension::Field_ReturnNumber, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_NumberOfReturns, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_ScanDirectionFlag, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_EdgeOfFlightLine, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_Classification, Dimension::Uint8));
    schema.addDimension(Dimension(Dimension::Field_ScanAngleRank, Dimension::Int8));
    schema.addDimension(Dimension(Dimension::Field_UserData, Dimension::Uint8));
    schema.addDimension(Dimension(Dimension::Field_PointSourceId, Dimension::Uint16));

    if (m_hasTimeData)
    {
        schema.addDimension(Dimension(Dimension::Field_Time, Dimension::Double));
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



libpc::Iterator* LiblasReader::createIterator()
{
    return new Iterator(*this);
}


Iterator::Iterator(LiblasReader& reader)
    : libpc::Iterator(reader)
    , m_stageAsDerived(reader)
{
    return;
}


void Iterator::seekToPoint(boost::uint64_t n)
{
    LiblasReader& reader = m_stageAsDerived;

    size_t nn = (size_t)n;
    if (n != nn) throw; // BUG
    reader.m_externalReader->Seek(nn);
    return;
}


boost::uint32_t Iterator::readBuffer(PointData& pointData)
{
    LiblasReader& reader = m_stageAsDerived;

    boost::uint32_t numPoints = pointData.getCapacity();
    boost::uint32_t i = 0;

    const Schema& schema = pointData.getSchema();

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

        if (reader.m_hasTimeData)
        {
            const double time = pt.GetTime();
            
            pointData.setField(i, indexTime, time);
        }

        if (reader.m_hasColorData)
        {
            const ::liblas::Color color = pt.GetColor();
            const boost::uint16_t red = color.GetRed();
            const boost::uint16_t green = color.GetGreen();
            const boost::uint16_t blue = color.GetBlue();

            pointData.setField(i, indexRed, red);
            pointData.setField(i, indexGreen, green);
            pointData.setField(i, indexBlue, blue);
        }
        
        pointData.setNumPoints(i+1);
        if (reader.m_hasWaveData)
        {
            throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
        }
        
    }

    return numPoints;
}



} } } // namespaces
