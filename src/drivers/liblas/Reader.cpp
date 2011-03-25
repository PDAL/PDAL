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

#include <libpc/drivers/liblas/Reader.hpp>

#include <liblas/factory.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/drivers/liblas/Iterator.hpp>
#include <libpc/drivers/liblas/Header.hpp>

namespace libpc { namespace drivers { namespace liblas {

LiblasReader::LiblasReader(const std::string& filename)
    : Stage()
    , m_filename(filename)
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
    std::istream* str = Utils::openFile(m_filename);

    {
        ::liblas::ReaderFactory f;
        ::liblas::Reader reader = f.CreateWithStream(*str);

        LiblasHeader* myHeader = new LiblasHeader;
        setHeader(myHeader);

        processExternalHeader(reader);

        registerFields(reader);
    }

    Utils::closeFile(str);

    return;
}


LiblasReader::~LiblasReader()
{
    return;
}


const std::string& LiblasReader::getName() const
{
    static std::string name("Liblas Reader");
    return name;
}



const std::string& LiblasReader::getFileName() const
{
    return m_filename;
}


bool LiblasReader::hasTimeData() const
{
    return m_hasTimeData;
}


bool LiblasReader::hasColorData() const
{
    return m_hasColorData;
}


bool LiblasReader::hasWaveData() const
{
    return m_hasWaveData;
}


boost::int8_t LiblasReader::getPointFormatNumber() const
{
    return m_pointFormatNumber;
}

void LiblasReader::processExternalHeader(::liblas::Reader& externalReader)
{
    const ::liblas::Header& externalHeader = externalReader.GetHeader();
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

void LiblasReader::registerFields(::liblas::Reader& externalReader)
{
    const ::liblas::Header& externalHeader = externalReader.GetHeader();
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



libpc::SequentialIterator* LiblasReader::createSequentialIterator() const
{
    return new SequentialIterator(*this);
}


libpc::RandomIterator* LiblasReader::createRandomIterator() const
{
    return new RandomIterator(*this);
}


} } } // namespaces
