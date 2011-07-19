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

#include <pdal/drivers/liblas/Reader.hpp>

#include <liblas/factory.hpp>
#include <liblas/variablerecord.hpp>

#include <pdal/exceptions.hpp>
#include <pdal/drivers/liblas/Iterator.hpp>
#include <pdal/drivers/las/VariableLengthRecord.hpp>


namespace pdal { namespace drivers { namespace liblas {

LiblasReader::LiblasReader(const Options& options)
    : LasReaderBase(options)
    , m_versionMajor(0)
    , m_versionMinor(0)
    , m_scaleX(0.0)
    , m_scaleY(0.0)
    , m_scaleZ(0.0)
    , m_offsetX(0.0)
    , m_offsetY(0.0)
    , m_offsetZ(0.0)
    , m_isCompressed(false)
    , m_pointFormat(::pdal::drivers::las::PointFormatUnknown)
{
    m_filename = options.getOption<std::string>("filename").getValue();

    std::istream* str = Utils::openFile(m_filename);

    {
        ::liblas::ReaderFactory factory;
        ::liblas::Reader externalReader = factory.CreateWithStream(*str);

        processExternalHeader(externalReader);

        registerFields(externalReader);
    }

    Utils::closeFile(str);

    return;
}


LiblasReader::~LiblasReader()
{
    return;
}


const std::string& LiblasReader::getDescription() const
{
    static std::string name("Liblas Reader");
    return name;
}

const std::string& LiblasReader::getName() const
{
    static std::string name("drivers.liblas.reader");
    return name;
}



const std::string& LiblasReader::getFileName() const
{
    return m_filename;
}


::pdal::drivers::las::PointFormat LiblasReader::getPointFormat() const
{
    return m_pointFormat;
}


boost::uint8_t LiblasReader::getVersionMajor() const
{
    return m_versionMajor;
}


boost::uint8_t LiblasReader::getVersionMinor() const
{
    return m_versionMinor;
}


const SpatialReference& LiblasReader::getSpatialReference() const
{
    throw not_yet_implemented("SRS support in liblas reader not yet implemented");
}


void LiblasReader::processExternalHeader(::liblas::Reader& externalReader)
{
    const ::liblas::Header& externalHeader = externalReader.GetHeader();

    this->setNumPoints( externalHeader.GetPointRecordsCount() );

    const ::liblas::Bounds<double>& externalBounds = externalHeader.GetExtent();
    const Bounds<double> internalBounds(externalBounds.minx(), externalBounds.miny(), externalBounds.minz(), externalBounds.maxx(), externalBounds.maxy(), externalBounds.maxz());
    this->setBounds(internalBounds);

    m_versionMajor = externalHeader.GetVersionMajor();
    m_versionMinor = externalHeader.GetVersionMinor();

    m_scaleX = externalHeader.GetScaleX();
    m_scaleY = externalHeader.GetScaleY();
    m_scaleZ = externalHeader.GetScaleZ();
    m_offsetX = externalHeader.GetOffsetX();
    m_offsetY = externalHeader.GetOffsetY();
    m_offsetZ = externalHeader.GetOffsetZ();

    m_isCompressed = externalHeader.Compressed();

    m_pointFormat = (::pdal::drivers::las::PointFormat)externalHeader.GetDataFormatId();

    if (::pdal::drivers::las::Support::hasWave(m_pointFormat))
    {
        throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
    }

    return;
}


int LiblasReader::getMetadataRecordCount() const
{
    return m_metadataRecords.size();
}


const MetadataRecord& LiblasReader::getMetadataRecord(int index) const
{
    return m_metadataRecords[index];
}


MetadataRecord& LiblasReader::getMetadataRecordRef(int index)
{
    return m_metadataRecords[index];
}


void LiblasReader::registerFields(::liblas::Reader& externalReader)
{
    const ::liblas::Header& externalHeader = externalReader.GetHeader();
    Schema& schema = getSchemaRef();

    ::pdal::drivers::las::Support::registerFields(schema, getPointFormat());

    ::pdal::drivers::las::Support::setScaling(schema, 
        externalHeader.GetScaleX(),
        externalHeader.GetScaleY(),
        externalHeader.GetScaleZ(),
        externalHeader.GetOffsetX(),
        externalHeader.GetOffsetY(),
        externalHeader.GetOffsetZ());

    return;
}


pdal::StageSequentialIterator* LiblasReader::createSequentialIterator() const
{
    return new SequentialIterator(*this);
}


pdal::StageRandomIterator* LiblasReader::createRandomIterator() const
{
    return new RandomIterator(*this);
}


} } } // namespaces
