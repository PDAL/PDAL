/******************************************************************************
* Copyright (c) 2011, Kirk McKelvey <kirkoman@gmail.com>
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

#include <libpc/drivers/mrsid/Reader.hpp>
#include <libpc/exceptions.hpp>
#include "lidar/MG4PointReader.h"

namespace libpc
{
namespace drivers
{
namespace mrsid
{

LT_USE_LIDAR_NAMESPACE

static libpc::Dimension::Field FieldEnumOf(const char *);
static libpc::Dimension::DataType TypeEnumOf(LizardTech::DataType);

Reader::Reader(const char * filepath)
    : libpc::Stage(), m_reader(NULL)
{
    try
    {
        // open a MG4 file
        m_reader = MG4PointReader::create();
        m_reader->init(filepath);
    }
    catch(...)
    {
        RELEASE(m_reader);
        throw;
    }

    Header* header = new Header;
    Schema& schema = header->getSchema();

    header->setNumPoints(m_reader->getNumPoints());
    Bounds<double> bounds(m_reader->getBounds().x.min, m_reader->getBounds().y.min, m_reader->getBounds().z.min,
                          m_reader->getBounds().x.max, m_reader->getBounds().y.max, m_reader->getBounds().z.max);

    header->setBounds(bounds);

    size_t numChannels = m_reader->getNumChannels();
    const PointInfo &pointInfo = m_reader->getPointInfo();
    for(size_t i = 0; i < numChannels; i += 1)
        schema.addDimension(Dimension(FieldEnumOf(pointInfo.getChannel(i).getName()), TypeEnumOf(pointInfo.getChannel(i).getDataType())));

    setHeader(header);

    return;
}

libpc::Dimension::Field
FieldEnumOf(const char * fieldname)
{
    if (!strcmp(CHANNEL_NAME_X, fieldname))
        return Dimension::Field_X;
    if (!strcmp(CHANNEL_NAME_Y, fieldname))
        return Dimension::Field_Y;
    if (!strcmp(CHANNEL_NAME_Y, fieldname))
        return Dimension::Field_Z;
    if (!strcmp(CHANNEL_NAME_Intensity, fieldname))
        return Dimension::Field_Intensity;
    if (!strcmp(CHANNEL_NAME_ReturnNum, fieldname))
        return Dimension::Field_ReturnNumber;
    if (!strcmp(CHANNEL_NAME_NumReturns, fieldname))
        return Dimension::Field_NumberOfReturns;
    if (!strcmp(CHANNEL_NAME_ScanDir, fieldname))
        return Dimension::Field_ScanDirectionFlag;
    if (!strcmp(CHANNEL_NAME_EdgeFlightLine, fieldname))
        return Dimension::Field_EdgeOfFlightLine;
    if (!strcmp(CHANNEL_NAME_ClassId, fieldname))
        return Dimension::Field_Classification;
    if (!strcmp(CHANNEL_NAME_ScanAngle, fieldname))
        return Dimension::Field_ScanAngleRank;
    if (!strcmp(CHANNEL_NAME_UserData, fieldname))
        return Dimension::Field_UserData;
    if (!strcmp(CHANNEL_NAME_SourceId, fieldname))
        return Dimension::Field_PointSourceId;
    if (!strcmp(CHANNEL_NAME_GPSTime, fieldname))
        return Dimension::Field_Time;
    if (!strcmp(CHANNEL_NAME_Red, fieldname))
        return Dimension::Field_Red;
    if (!strcmp(CHANNEL_NAME_Green, fieldname))
        return Dimension::Field_Green;
    if (!strcmp(CHANNEL_NAME_Blue, fieldname))
        return Dimension::Field_Blue;
    // @todo
    // Field_WavePacketDescriptorIndex
    // Field_WaveformDataOffset
    // Field_ReturnPointWaveformLocation
    // Field_WaveformXt
    // Field_WaveformYt
    // Field_WaveformZt
    // ...
}

libpc::Dimension::DataType
TypeEnumOf(LizardTech::DataType ldt)
{
    switch (ldt)
    {
    case DATATYPE_UINT8:
        return Dimension::Uint8;
    case DATATYPE_SINT8:
        return Dimension::Int8;
    case DATATYPE_UINT16:
        return Dimension::Uint16;
    case DATATYPE_SINT16:
        return Dimension::Int16;
    case DATATYPE_UINT32:
        return Dimension::Uint32;
    case DATATYPE_SINT32:
        return Dimension::Int32;
    case DATATYPE_UINT64:
        return Dimension::Uint64;
    case DATATYPE_SINT64:
        return Dimension::Int64;
    case DATATYPE_FLOAT32:
        return Dimension::Float;
    case DATATYPE_FLOAT64:
        return Dimension::Double;
    case DATATYPE_INVALID:
    default:
        return Dimension::Uint8;
    }
}

const std::string& Reader::getName() const
{
    static std::string name("MrSID/LiDAR Reader");
    return name;
}

boost::uint32_t Reader::readBuffer(PointData& pointData)
{
    return 0;
}


}
}
} // namespaces
