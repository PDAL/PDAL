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

#include "writer.hpp"
#include "header.hpp"
#include <liblas/Writer.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/libpc_config.hpp>


namespace libpc
{


LiblasWriter::LiblasWriter(Stage& prevStage, std::ostream& ostream)
    : Writer(prevStage)
    , m_ostream(ostream)
    , m_externalWriter(NULL)
{
    m_externalHeader = new liblas::Header;
    m_externalHeader->SetCompressed(false);

    setupExternalHeader();

    // make our own header
    LiblasHeader* internalHeader = new LiblasHeader;
    setHeader(internalHeader);

    //const liblas::Bounds<double>& extBounds = extHeader.GetExtent();
    //const Bounds<double> bounds(extBounds.minx(), extBounds.miny(), extBounds.minz(), extBounds.maxx(), extBounds.maxy(), extBounds.maxz());
    //myHeader->setBounds(bounds);

    return;
}


LiblasWriter::~LiblasWriter()
{
    delete m_externalHeader;
    return;
}


void LiblasWriter::setupExternalHeader()
{
    setFormatVersion(1,2);
    setPointFormat(0);

    setCompressed(false);

    setSystemIdentifier("libPC");
    setGeneratingSoftware(GetVersionString());

    std::size_t indexX = getHeader().getSchema().getDimensionIndex(Dimension::Field_X);
    std::size_t indexY = getHeader().getSchema().getDimensionIndex(Dimension::Field_Y);
    std::size_t indexZ = getHeader().getSchema().getDimensionIndex(Dimension::Field_Z);
    const Dimension& dimX = getHeader().getSchema().getDimension(indexX);
    const Dimension& dimY = getHeader().getSchema().getDimension(indexY);
    const Dimension& dimZ = getHeader().getSchema().getDimension(indexZ);
    m_externalHeader->SetScale(dimX.getNumericScale(), dimY.getNumericScale(), dimZ.getNumericScale());
    m_externalHeader->SetOffset(dimX.getNumericOffset(), dimY.getNumericOffset(), dimZ.getNumericOffset());

    return;
}


void LiblasWriter::setCompressed(bool v)
{
    m_externalHeader->SetCompressed(v);
}


void LiblasWriter::setFormatVersion(boost::uint8_t majorVersion, boost::uint8_t minorVersion)
{
    m_externalHeader->SetVersionMajor(majorVersion);
    m_externalHeader->SetVersionMinor(minorVersion);
}


void LiblasWriter::setPointFormat(boost::int8_t pointFormat)
{
    m_externalHeader->SetDataFormatId((liblas::PointFormatName)pointFormat);
}


void LiblasWriter::setDate(boost::uint16_t dayOfYear, boost::uint16_t year)
{
    m_externalHeader->SetCreationDOY(dayOfYear);
    m_externalHeader->SetCreationYear(year);
}


void LiblasWriter::setSystemIdentifier(const std::string& systemId) 
{
    m_externalHeader->SetSystemId(systemId);
}


void LiblasWriter::setGeneratingSoftware(const std::string& softwareId)
{
    m_externalHeader->SetSoftwareId(softwareId);
}


void LiblasWriter::writeBegin()
{
    m_externalHeader->SetPointRecordsCount(99); // BUG

    m_externalWriter = new liblas::Writer(m_ostream, *m_externalHeader);
    return;
}


void LiblasWriter::writeEnd()
{
    delete m_externalWriter;
    m_externalWriter = NULL;
    return;
}


boost::uint32_t LiblasWriter::writeBuffer(const PointData& pointData)
{
    bool hasTimeData = false;
    bool hasColorData = false;
    bool hasWaveData = false;

    const liblas::PointFormatName pointFormat = m_externalHeader->GetDataFormatId();
    switch (pointFormat)
    {
    case liblas::ePointFormat0:
        break;
    case liblas::ePointFormat1:
        hasTimeData = true;
        break;
    case liblas::ePointFormat2:
        hasColorData = true;
        break;
    case liblas::ePointFormat3:
        hasTimeData = true;
        hasColorData = true;
        break;
    case liblas::ePointFormat4:
        hasTimeData = true;
        hasWaveData = true;
        break;
    case liblas::ePointFormat5:
        hasColorData = true;
        hasTimeData = true;
        hasWaveData = true;
        break;
    case liblas::ePointFormatUnknown:
        throw not_yet_implemented("Unknown point format encountered");
    }

    if (hasWaveData)
    {
        throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
    }

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
    
    const int indexGpsTime = (hasTimeData ? schema.getDimensionIndex(Dimension::Field_GpsTime) : 0);

    const int indexRed = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Red) : 0);
    const int indexGreen = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Green) : 0);
    const int indexBlue = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Blue) : 0);

    //const int indexWavePacketDescriptorIndex = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WavePacketDescriptorIndex) : 0);
    //const int indexWaveformDataOffset = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformDataOffset) : 0);
    //const int indexReturnPointWaveformLocation = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_ReturnPointWaveformLocation) : 0);
    //const int indexWaveformXt = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformXt) : 0);
    //const int indexWaveformYt = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformYt) : 0);
    //const int indexWaveformZt = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformZt) : 0);

    liblas::Point pt;

    boost::uint32_t numPoints = pointData.getNumPoints();
    for (boost::uint32_t i=0; i<numPoints; i++)
    {
        const boost::int32_t x = pointData.getField<boost::int32_t>(i, indexX);
        const boost::int32_t y = pointData.getField<boost::int32_t>(i, indexY);
        const boost::int32_t z = pointData.getField<boost::int32_t>(i, indexZ);
        pt.SetRawX(x);
        pt.SetRawY(y);
        pt.SetRawZ(z);

        const boost::uint16_t intensity = pointData.getField<boost::uint16_t>(i, indexIntensity);
        const boost::int8_t returnNumber = pointData.getField<boost::int8_t>(i, indexReturnNumber);
        const boost::int8_t numberOfReturns = pointData.getField<boost::int8_t>(i, indexNumberOfReturns);
        const boost::int8_t scanDirFlag = pointData.getField<boost::int8_t>(i, indexScanDirectionFlag);
        const boost::int8_t edgeOfFlightLine = pointData.getField<boost::int8_t>(i, indexEdgeOfFlightLine);
        const boost::uint8_t classification = pointData.getField<boost::uint8_t>(i, indexClassification);
        const boost::int8_t scanAngleRank = pointData.getField<boost::int8_t>(i, indexScanAngleRank);
        const boost::uint8_t userData = pointData.getField<boost::uint8_t>(i, indexUserData);
        const boost::uint16_t pointSourceId = pointData.getField<boost::uint16_t>(i, indexPointSourceId);
        pt.SetIntensity(intensity);
        pt.SetReturnNumber(returnNumber);
        pt.SetNumberOfReturns(numberOfReturns);
        pt.SetScanDirection(scanDirFlag);
        pt.SetFlightLineEdge(edgeOfFlightLine);
        pt.SetClassification(classification);
        pt.SetScanAngleRank(scanAngleRank);
        pt.SetUserData(userData);
        pt.SetPointSourceID(pointSourceId);

        if (hasTimeData)
        {
            const double gpsTime = pointData.getField<double>(i, indexGpsTime);
            pt.SetTime(gpsTime);
        }

        if (hasColorData)
        {
            const boost::uint16_t red = pointData.getField<boost::uint16_t>(i, indexRed);
            const boost::uint16_t green = pointData.getField<boost::uint16_t>(i, indexGreen);
            const boost::uint16_t blue = pointData.getField<boost::uint16_t>(i, indexBlue);
            liblas::Color color(red, green, blue);
            pt.SetColor(color);
        }

        if (hasWaveData)
        {
            assert(false);
        }

        bool ok = m_externalWriter->WritePoint(pt);
        assert(ok); // BUG
    }

    return numPoints;
}

} // namespace libpc
