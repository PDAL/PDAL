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

#include <libpc/drivers/liblas/Writer.hpp>

#include <liblas/header.hpp>
#include <liblas/Writer.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/libpc_config.hpp>
#include <libpc/Schema.hpp>
#include <libpc/Stage.hpp>
#include <libpc/PointBuffer.hpp>


namespace libpc { namespace drivers { namespace liblas {



LiblasWriter::LiblasWriter(Stage& prevStage, std::ostream& ostream)
    : Writer(prevStage)
    , m_ostream(ostream)
    , m_externalWriter(NULL)
{
    m_externalHeader = new ::liblas::Header;
    m_externalHeader->SetCompressed(false);

    setupExternalHeader();

    return;
}


LiblasWriter::~LiblasWriter()
{
    delete m_externalHeader;
    return;
}


const std::string& LiblasWriter::getName() const
{
    static std::string name("Liblas Writer");
    return name;
}


void LiblasWriter::setupExternalHeader()
{
    setFormatVersion(1,2);
    setPointFormat(3);

    setCompressed(false);

    setSystemIdentifier("libPC");
    setGeneratingSoftware(GetVersionString());

    const Schema& schema = getPrevStage().getSchema();

    int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    const Dimension& dimX = schema.getDimension(indexX);
    const Dimension& dimY = schema.getDimension(indexY);
    const Dimension& dimZ = schema.getDimension(indexZ);
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
    m_externalHeader->SetDataFormatId((::liblas::PointFormatName)pointFormat);
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

    m_externalWriter = new ::liblas::Writer(m_ostream, *m_externalHeader);
    return;
}


void LiblasWriter::writeEnd()
{
    delete m_externalWriter;
    m_externalWriter = NULL;
    return;
}


boost::uint32_t LiblasWriter::writeBuffer(const PointBuffer& PointBuffer)
{
    bool hasTimeData = false;
    bool hasColorData = false;
    bool hasWaveData = false;

    const ::liblas::PointFormatName pointFormat = m_externalHeader->GetDataFormatId();
    switch (pointFormat)
    {
    case ::liblas::ePointFormat0:
        break;
    case ::liblas::ePointFormat1:
        hasTimeData = true;
        break;
    case ::liblas::ePointFormat2:
        hasColorData = true;
        break;
    case ::liblas::ePointFormat3:
        hasTimeData = true;
        hasColorData = true;
        break;
    case ::liblas::ePointFormat4:
        hasTimeData = true;
        hasWaveData = true;
        break;
    case ::liblas::ePointFormat5:
        hasColorData = true;
        hasTimeData = true;
        hasWaveData = true;
        break;
    case ::liblas::ePointFormatUnknown:
        throw not_yet_implemented("Unknown point format encountered");
    }

    if (hasWaveData)
    {
        throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
    }

    const Schema& schema = PointBuffer.getSchema();

    const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    
    const int indexIntensity = schema.getDimensionIndex(Dimension::Field_Intensity, Dimension::Uint16);
    const int indexReturnNumber = schema.getDimensionIndex(Dimension::Field_ReturnNumber, Dimension::Uint8);
    const int indexNumberOfReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns, Dimension::Uint8);
    const int indexScanDirectionFlag = schema.getDimensionIndex(Dimension::Field_ScanDirectionFlag, Dimension::Uint8);
    const int indexEdgeOfFlightLine = schema.getDimensionIndex(Dimension::Field_EdgeOfFlightLine, Dimension::Uint8);
    const int indexClassification = schema.getDimensionIndex(Dimension::Field_Classification, Dimension::Uint8);
    const int indexScanAngleRank = schema.getDimensionIndex(Dimension::Field_ScanAngleRank, Dimension::Int8);
    const int indexUserData = schema.getDimensionIndex(Dimension::Field_UserData, Dimension::Uint8);
    const int indexPointSourceId = schema.getDimensionIndex(Dimension::Field_PointSourceId, Dimension::Uint16);
    
    const int indexTime = (hasTimeData ? schema.getDimensionIndex(Dimension::Field_Time, Dimension::Double) : 0);

    const int indexRed = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Red, Dimension::Uint16) : 0);
    const int indexGreen = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Green, Dimension::Uint16) : 0);
    const int indexBlue = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Blue, Dimension::Uint16) : 0);

    //const int indexWavePacketDescriptorIndex = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WavePacketDescriptorIndex) : 0);
    //const int indexWaveformDataOffset = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformDataOffset) : 0);
    //const int indexReturnPointWaveformLocation = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_ReturnPointWaveformLocation) : 0);
    //const int indexWaveformXt = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformXt) : 0);
    //const int indexWaveformYt = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformYt) : 0);
    //const int indexWaveformZt = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformZt) : 0);

    ::liblas::Point pt;

    boost::uint32_t numPoints = PointBuffer.getNumPoints();
    for (boost::uint32_t i=0; i<numPoints; i++)
    {
        const boost::int32_t x = PointBuffer.getField<boost::int32_t>(i, indexX);
        const boost::int32_t y = PointBuffer.getField<boost::int32_t>(i, indexY);
        const boost::int32_t z = PointBuffer.getField<boost::int32_t>(i, indexZ);
        pt.SetRawX(x);
        pt.SetRawY(y);
        pt.SetRawZ(z);

        const boost::uint16_t intensity = PointBuffer.getField<boost::uint16_t>(i, indexIntensity);
        const boost::int8_t returnNumber = PointBuffer.getField<boost::int8_t>(i, indexReturnNumber);
        const boost::int8_t numberOfReturns = PointBuffer.getField<boost::int8_t>(i, indexNumberOfReturns);
        const boost::int8_t scanDirFlag = PointBuffer.getField<boost::int8_t>(i, indexScanDirectionFlag);
        const boost::int8_t edgeOfFlightLine = PointBuffer.getField<boost::int8_t>(i, indexEdgeOfFlightLine);
        const boost::uint8_t classification = PointBuffer.getField<boost::uint8_t>(i, indexClassification);
        const boost::int8_t scanAngleRank = PointBuffer.getField<boost::int8_t>(i, indexScanAngleRank);
        const boost::uint8_t userData = PointBuffer.getField<boost::uint8_t>(i, indexUserData);
        const boost::uint16_t pointSourceId = PointBuffer.getField<boost::uint16_t>(i, indexPointSourceId);
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
            const double time = PointBuffer.getField<double>(i, indexTime);
            pt.SetTime(time);
        }

        if (hasColorData)
        {
            const boost::uint16_t red = PointBuffer.getField<boost::uint16_t>(i, indexRed);
            const boost::uint16_t green = PointBuffer.getField<boost::uint16_t>(i, indexGreen);
            const boost::uint16_t blue = PointBuffer.getField<boost::uint16_t>(i, indexBlue);
            ::liblas::Color color(red, green, blue);
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

} } } // namespaces
