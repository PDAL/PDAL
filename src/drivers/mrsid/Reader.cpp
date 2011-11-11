/******************************************************************************
* Copyright (c) 2011, Michael S. Rosen (michael.rosen@gmail.com)
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

#include <pdal/drivers/MrSID/Reader.hpp>

#include <pdal/drivers/MrSID/Iterator.hpp>
#include <pdal/PointBuffer.hpp>

#include <boost/algorithm/string.hpp>


namespace pdal { namespace drivers { namespace MrSID {


Reader::Reader(const Options& options)
    : pdal::Reader(options)
{
    return;
}


Reader::Reader(LizardTech::PointSource *ps)
    : pdal::Reader(Options::none())
    , m_PS(ps), m_iter(NULL)
{
    m_PS->retain();
    return;
}

Reader::~Reader()
{
    m_iter->release();
    m_PS->release();
}
Dimension Reader::LTChannelToPDalDimension(const LizardTech::ChannelInfo & channel) const
{
    Dimension retval(DimensionId::X_f64);
    if (!strcmp(CHANNEL_NAME_X, channel.getName()))
    {
        Dimension dim(DimensionId::X_f64);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_Y, channel.getName()))
    {
        Dimension dim(DimensionId::Y_f64);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_Z, channel.getName()))
    {
        Dimension dim(DimensionId::Z_f64);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_GPSTime, channel.getName()))
    {
        Dimension dim(DimensionId::Time_u64);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_Blue, channel.getName()))
    {
        Dimension dim(DimensionId::Blue_u16);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_Red, channel.getName()))
    {
        Dimension dim(DimensionId::Red_u16);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_Green, channel.getName()))
    {
        Dimension dim(DimensionId::Green_u16);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_ClassId, channel.getName()))
    {
        Dimension dim(DimensionId::Las_Classification);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_EdgeFlightLine, channel.getName()))
    {
        Dimension dim(DimensionId::Las_EdgeOfFlightLine);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_Intensity, channel.getName()))
    {
        Dimension dim(DimensionId::Las_Intensity);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_NumReturns, channel.getName()))
    {
        Dimension dim(DimensionId::Las_NumberOfReturns);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_ReturnNum, channel.getName()))
    {
        Dimension dim(DimensionId::Las_ReturnNumber);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_ScanAngle, channel.getName()))
    {
        Dimension dim(DimensionId::Las_ScanAngleRank);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_ScanDir, channel.getName()))
    {
        Dimension dim(DimensionId::Las_ScanDirectionFlag);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_SourceId, channel.getName()))
    {
        Dimension dim(DimensionId::Las_PointSourceId);
        retval = dim;
    }
    else if (!strcmp(CHANNEL_NAME_UserData, channel.getName()))
    {
        Dimension dim(DimensionId::Las_UserData);
        retval = dim;
    }
    else
    {
        Dimension dim(DimensionId::Undefined); // throws
        retval = dim;
    }

    return retval;
}
void Reader::initialize()
{
    pdal::Reader::initialize();

    Schema& schema = getSchemaRef();
    const LizardTech::PointInfo& pointinfo = m_PS->getPointInfo();
    for (unsigned int i=0; i<pointinfo.getNumChannels(); i++)
    {
        const LizardTech::ChannelInfo &channel = pointinfo.getChannel(i);
        Dimension dim = LTChannelToPDalDimension(channel);
        schema.appendDimension(dim);
    }

    setNumPoints(m_PS->getNumPoints());
    setPointCountType(PointCount_Fixed);
    pdal::Bounds<double> b(m_PS->getBounds().x.min, m_PS->getBounds().x.max, m_PS->getBounds().y.min, m_PS->getBounds().y.max, m_PS->getBounds().z.min,m_PS->getBounds().z.max);
    setBounds(b);
    m_iter = m_PS->createIterator(m_PS->getBounds(), 1.0, m_PS->getPointInfo(), NULL);
}


const Options Reader::getDefaultOptions() const
{
    Options options;
    return options;
}

pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new SequentialIterator(*this);
}

int Reader::SchemaToPointInfo(const Schema &schema, LizardTech::PointInfo &pointInfo) const
{
    pointInfo.init(schema.getDimensions().size());
    for (unsigned int idx=0; idx<schema.getDimensions().size(); idx++)
    {
       switch (schema.getDimensions()[idx].getId())
       {
         case (DimensionId::X_f64):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_X, LizardTech::DATATYPE_FLOAT64, 64);
            break;
         case (DimensionId::X_i32):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_X, LizardTech::DATATYPE_SINT32, 32);
            break;
         case (DimensionId::Y_f64):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Y, LizardTech::DATATYPE_FLOAT64, 64);
            break;
         case (DimensionId::Y_i32):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Y, LizardTech::DATATYPE_SINT32, 32);
            break;
         case (DimensionId::Z_f64):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Z, LizardTech::DATATYPE_FLOAT64, 64);
            break;
         case (DimensionId::Z_i32):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Z, LizardTech::DATATYPE_SINT32, 32);
            break;
         case (DimensionId::Blue_u8):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Blue, LizardTech::DATATYPE_UINT8, 8);
            break;
         case (DimensionId::Blue_u16):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Blue, LizardTech::DATATYPE_UINT16, 16);
            break;
         case (DimensionId::Red_u8):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Red, LizardTech::DATATYPE_UINT8, 8);
            break;
         case (DimensionId::Red_u16):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Red, LizardTech::DATATYPE_UINT16, 16);
            break;
         case (DimensionId::Green_u8):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Green, LizardTech::DATATYPE_UINT8, 8);
            break;
         case (DimensionId::Green_u16):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_Green, LizardTech::DATATYPE_UINT16, 16);
            break;
         case (DimensionId::Las_Classification): /// BUG!  Adjust # of bits per las spec
            pointInfo.getChannel(idx).init(CHANNEL_NAME_ClassId, LizardTech::DATATYPE_UINT8, 8);
            break;
         case (DimensionId::Las_EdgeOfFlightLine):
            pointInfo.getChannel(idx).init(CHANNEL_NAME_EdgeFlightLine, LizardTech::DATATYPE_UINT8, 8);
            break;
         // bug.  finish this.
       }
    }
    return 0; //bug, do error checking
}


boost::uint32_t Reader::processBuffer(PointBuffer& data, boost::uint64_t index) const
{
    const Schema& schema = data.getSchema();

    // how many are they asking for?
    boost::uint64_t numPointsWanted = data.getCapacity();

    // we can only give them as many as we have left
    boost::uint64_t numPointsAvailable = getNumPoints() - index;
    if (numPointsAvailable < numPointsWanted)
        numPointsWanted = numPointsAvailable;


    LizardTech::PointData points;
    // to do:  specify a PointInfo structure that reads only the channels we will output.
    points.init(m_PS->getPointInfo(), (size_t)numPointsWanted);
    size_t count = m_iter->getNextPoints(points);

    boost::uint32_t cnt = 0;
    data.setNumPoints(0);

    for (boost::uint32_t pointIndex=0; pointIndex<count; pointIndex++)
    {
        ++cnt;
        for (unsigned int i=0; i < schema.getDimensions().size(); i++)
        {
            Dimension d = schema.getDimensions()[i];

            if (d.getId() == DimensionId::X_f64 && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_X))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_X)->getData());
                double value = static_cast<double>(pData[pointIndex]);
                data.setField<double>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::X_i32 && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_X))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_X)->getData());
                boost::int32_t value = static_cast<boost::int32_t>(pData[pointIndex]);
                data.setField<boost::int32_t>(pointIndex, schema.getDimensionIndex(d), value);
            }
            else if (d.getId() == DimensionId::Y_f64 && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Y))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_Y)->getData());
                double value = static_cast<double>(pData[pointIndex]);
                data.setField<double>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::Y_i32 && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Y))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_Y)->getData());
                boost::int32_t value = static_cast<boost::int32_t>(pData[pointIndex]);
                data.setField<boost::int32_t>(pointIndex, schema.getDimensionIndex(d), value);
            }
            else if (d.getId() == DimensionId::Z_f64 && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Z))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_Z)->getData());
                double value = static_cast<double>(pData[pointIndex]);
                data.setField<double>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::Z_i32 && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Z))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_Z)->getData());
                boost::int32_t value = static_cast<boost::int32_t>(pData[pointIndex]);
                data.setField<boost::int32_t>(pointIndex, schema.getDimensionIndex(d), value);
            }
            else if (d.getId() == DimensionId::Time_u64 && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_GPSTime))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_GPSTime)->getData());
                boost::uint64_t  value = static_cast<boost::uint64_t>(pData[pointIndex]);
                data.setField<boost::uint64_t>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::Las_Intensity && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Intensity))
            {
                uint16_t *pData = static_cast<uint16_t*>(points.getChannel(CHANNEL_NAME_Intensity)->getData());
                boost::uint16_t value = static_cast<boost::uint16_t>(pData[pointIndex]);
                data.setField<boost::uint16_t>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::Las_ReturnNumber && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_ReturnNum))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_ReturnNum)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::Las_NumberOfReturns && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_NumReturns))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::Las_ScanDirectionFlag && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_ScanDir))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::Las_ScanAngleRank && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_ScanAngle))
            {
                boost::int8_t *pData = static_cast<int8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::int8_t value = static_cast<boost::int8_t>(pData[pointIndex]);
                data.setField<boost::int8_t>(pointIndex, schema.getDimensionIndex(d), value);
            }   
            else if (d.getId() == DimensionId::Las_EdgeOfFlightLine && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_EdgeFlightLine))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::Las_Classification && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_ClassId))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(pointIndex, schema.getDimensionIndex(d), value);
            }            
            else if (d.getId() == DimensionId::Las_UserData && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_UserData))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(pointIndex, schema.getDimensionIndex(d), value);
            }    
            else if (d.getId() == DimensionId::Las_PointSourceId && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_SourceId))
            {
                uint16_t *pData = static_cast<uint16_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint16_t value = static_cast<boost::uint16_t>(pData[pointIndex]);
                data.setField<boost::uint16_t>(pointIndex, schema.getDimensionIndex(d), value);
            }    


        }        
    }
    data.setNumPoints(cnt);
    assert(cnt <= data.getCapacity());
    
    return cnt;
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    // add stuff here specific to this stage type

    return tree;
}


} } } // namespaces
