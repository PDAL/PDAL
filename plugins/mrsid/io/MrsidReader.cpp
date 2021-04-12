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

#include "MrsidReader.hpp"

#include <lidar/MG4PointReader.h>
#include <lidar/FileIO.h>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/DimUtil.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "readers.mrsid",
    "MrSID Reader [deprecated]",
    "http://pdal.io/stages/readers.mrsid.html"
};

CREATE_SHARED_STAGE(MrsidReader, s_info)

std::string MrsidReader::getName() const { return s_info.name; }

MrsidReader::MrsidReader()
    : pdal::Reader()
    , m_PS(0), m_iter(NULL)
    , m_initialized(false)
{}


void MrsidReader::addArgs(ProgramArgs& args)
{
}


void MrsidReader::done(PointTableRef)
{
    m_iter->release();
    m_PS->release();
    m_initialized = false;
    m_PS = 0;
    m_iter = 0;
    getMetadata().addList("filename", m_filename);
}

pdal::Dimension::Type getPDALType(LizardTech::DataType t)
{

   using namespace Dimension;
   using namespace LizardTech;
   switch (t)
   {
       case DATATYPE_SINT8:
           return Type::Signed8;
       case DATATYPE_UINT8:
           return Type::Unsigned8;
       case DATATYPE_SINT16:
           return Type::Signed16;
       case DATATYPE_UINT16:
           return Type::Unsigned16;
       case DATATYPE_SINT32:
           return Type::Signed32;
       case DATATYPE_UINT32:
           return Type::Unsigned32;
       case DATATYPE_SINT64:
           return Type::Signed64;
       case DATATYPE_UINT64:
           return Type::Unsigned64;
       case DATATYPE_FLOAT32:
           return Type::Float;
       case DATATYPE_FLOAT64:
           return Type::Double;

       default:
           return Type::Double;
   }
}

void MrsidReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;

    if (!m_PS)
        throwError("MrSID object not initialized.");
    const LizardTech::PointInfo& pointinfo = m_PS->getPointInfo();

    // add a map for PDAL names that aren't the same as LT ones (GPSTime vs Time)
    std::map<std::string, std::string> dimensionTranslations;
    dimensionTranslations.insert(std::pair<std::string, std::string>("Time", "GPSTime"));
    dimensionTranslations.insert(std::pair<std::string, std::string>("NumReturns", "NumberOfReturns"));
    dimensionTranslations.insert(std::pair<std::string, std::string>("ReturnNum", "ReturnNumber"));
    dimensionTranslations.insert(std::pair<std::string, std::string>("ScanDir", "ScanDirectionFlag"));
    dimensionTranslations.insert(std::pair<std::string, std::string>("EdgeFlightLine", "EdgeOfFlightLine"));
    dimensionTranslations.insert(std::pair<std::string, std::string>("ScanAngle", "ScanAngleRank"));
    dimensionTranslations.insert(std::pair<std::string, std::string>("ClassId", "Classification"));
    dimensionTranslations.insert(std::pair<std::string, std::string>("SourceId", "PointSourceId"));

    for (unsigned int i=0; i<pointinfo.getNumChannels(); i++)
    {
        const LizardTech::ChannelInfo &channel = pointinfo.getChannel(i);
        const char* name = channel.getName();
        auto translated = dimensionTranslations.find(name);
        if (translated != dimensionTranslations.end())
            name = translated->second.c_str();

        LizardTech::DataType t = channel.getDataType();
        Dimension::Type pdal_type = getPDALType(t);
        layout->registerOrAssignDim(name, pdal_type);
    }
    m_layout = layout;

}

void MrsidReader::initialize()
{
    LT_USE_LIDAR_NAMESPACE

    if (m_initialized)
        return;

    FileIO *file = FileIO::create();
    file->init(m_filename.c_str(), "r");
    m_PS = MG4PointReader::create();
    m_PS->init(file);
    file->release();
    file = NULL;

    m_iter = m_PS->createIterator(m_PS->getBounds(), 1.0,
        m_PS->getPointInfo(), NULL);
    m_initialized = true;
}

void MrsidReader::ready(PointTableRef table, MetadataNode& m)
{
    m_index = 0;
}

QuickInfo MrsidReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    MrsidReader::initialize();
    addDimensions(layout.get());

    BOX3D b(m_PS->getBounds().x.min, m_PS->getBounds().x.max,
        m_PS->getBounds().y.min, m_PS->getBounds().y.max,
        m_PS->getBounds().z.min,m_PS->getBounds().z.max);

    Dimension::IdList dims = layout->dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
        qi.m_dimNames.push_back(layout->dimName(*di));
    if (!Utils::numericCast(m_PS->getNumPoints(), qi.m_pointCount))
        qi.m_pointCount = (std::numeric_limits<point_count_t>::max)();
    qi.m_bounds = b;
    qi.m_srs = pdal::SpatialReference(m_PS->getWKT());
    qi.m_valid = true;

    PointTable table;
    done(table);

    return qi;
}

void MrsidReader::LayoutToPointInfo(const PointLayout &layout, LizardTech::PointInfo &pointInfo) const
{
    using namespace pdal::Dimension;
    const Dimension::IdList& dims = layout.dims();

    pointInfo.init(dims.size());
    for (unsigned int idx=0; idx<dims.size(); idx++)
    {
        std::string name = layout.dimName(dims[idx]);
        Dimension::Type t = layout.dimType(dims[idx]);
        size_t size = layout.dimSize(dims[idx]);

        if (Utils::iequals(name, "EdgeOfFlightLine")) name = CHANNEL_NAME_EdgeFlightLine;
        if (Utils::iequals(name, "Classification")) name = CHANNEL_NAME_ClassId;
        if (Utils::iequals(name, "ScanAngleRank")) name = CHANNEL_NAME_ScanAngle;
        if (Utils::iequals(name, "ScanDirectionFlag")) name = CHANNEL_NAME_ScanDir;
#ifdef CHANNEL_NAME_GPSTime
        if (Utils::iequals(name, "GpsTime")) name = CHANNEL_NAME_GPSTime;
#endif
#ifdef CHANNEL_NAME_GPSTime_Week
        if (Utils::iequals(name, "GpsTime")) name = CHANNEL_NAME_GPSTime_Week;
#endif
#ifdef CHANNEL_NAME_GPSTime_Adjusted
        //FIXME: We should account for header metadata if we have it
        if (Utils::iequals(name, "GpsTime")) name = CHANNEL_NAME_GPSTime_Adjusted;
#endif
        if (Utils::iequals(name, "PointSourceId")) name = CHANNEL_NAME_SourceId;
        if (Utils::iequals(name, "ReturnNumber")) name = CHANNEL_NAME_ReturnNum;
        if (Utils::iequals(name, "NumberOfReturns")) name = CHANNEL_NAME_NumReturns;

        if (t == Type::Double)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_FLOAT64, 64);
        else if (t == Type::Float)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_FLOAT32, 32);
        else if (t == Type::Signed64)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_SINT64, 64);
        else if (t == Type::Signed32)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_SINT32, 32);
        else if (t == Type::Signed16)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_SINT16, 16);
        else if (t == Type::Signed8)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_SINT8, 8);
        else if (t == Type::Unsigned64)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_UINT64, 64);
        else if (t == Type::Unsigned32)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_UINT32, 32);
        else if (t == Type::Unsigned16)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_UINT16, 16);
        else if (t == Type::Unsigned8)
            pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_UINT8, 8);
    }
}

template<typename T>
T getData(LizardTech::PointData& points, const char* channelId, point_count_t pointIndex)
{
    T *pData = static_cast<T*>(points.getChannel(channelId)->getData());
    T value = static_cast<T>(pData[pointIndex]);
    return value;
}

point_count_t MrsidReader::read(PointViewPtr view, point_count_t count)
{
    using namespace pdal::Dimension;

    LizardTech::PointData points;
    LayoutToPointInfo(*m_layout, m_pointInfo);

    points.init(m_pointInfo, (size_t)count);
    count = m_iter->getNextPoints(points);
    Dimension::IdList dims = view->dims();

    for (point_count_t pointIndex=0; pointIndex<count; pointIndex++)
    {
        for (Dimension::IdList::size_type i=0; i < dims.size(); i++)
        {
            Dimension::Id const& d = dims[i];
            std::string name = m_layout->dimName(d);
            Dimension::Type t = m_layout->dimType(d);

            if (Utils::iequals(name, "X") &&
                m_pointInfo.hasChannel(CHANNEL_NAME_X))
            {
                view->setField(d, pointIndex,
                    getData<double>(points, CHANNEL_NAME_X, pointIndex));
            }
            else if (Utils::iequals(name, "Y") &&
                m_pointInfo.hasChannel(CHANNEL_NAME_Y))
            {
                view->setField(d, pointIndex,
                    getData<double>(points, CHANNEL_NAME_Y, pointIndex));
            }
            else if (Utils::iequals(name, "Z") &&
                m_pointInfo.hasChannel(CHANNEL_NAME_Z))
            {
                view->setField(d, pointIndex,
                    getData<double>(points, CHANNEL_NAME_Z, pointIndex));
            }
#ifdef CHANNEL_NAME_GPSTime
            else if (Utils::iequals(name, "GpsTime") &&
                       m_pointInfo.hasChannel(CHANNEL_NAME_GPSTime))
            {
                view->setField<double>( d,
                                        pointIndex,
                                        getData<double>(points, CHANNEL_NAME_GPSTime, pointIndex));
            }
#endif
#ifdef CHANNEL_NAME_GPSTime_Week
            else if (Utils::iequals(name, "GpsTime") &&
                       m_pointInfo.hasChannel(CHANNEL_NAME_GPSTime_Week))
            {
                view->setField<double>( d,
                                        pointIndex,
                                        getData<double>(points, CHANNEL_NAME_GPSTime_Week, pointIndex));
            }
#endif
#ifdef CHANNEL_NAME_GPSTime_Adjusted
            else if (Utils::iequals(name, "GpsTime") &&
                       m_pointInfo.hasChannel(CHANNEL_NAME_GPSTime_Week))
            {
                view->setField<double>( d,
                                        pointIndex,
                                        getData<double>(points, CHANNEL_NAME_GPSTime_Adjusted, pointIndex));
            }
#endif
            else if (Utils::iequals(name, "Intensity") &&
                     m_pointInfo.hasChannel(CHANNEL_NAME_Intensity))
            {
                view->setField<uint16_t>(   d,
                                            pointIndex,
                                            getData<uint16_t>(points, CHANNEL_NAME_Intensity, pointIndex));
            }
            else if (Utils::iequals(name, "ReturnNumber") &&
                     m_pointInfo.hasChannel(CHANNEL_NAME_ReturnNum))
            {
                view->setField<uint8_t>(d,
                                        pointIndex,
                                        getData<uint8_t>(points, CHANNEL_NAME_ReturnNum, pointIndex));
            }
            else if (Utils::iequals(name, "NumberOfReturns") &&
                     m_pointInfo.hasChannel(CHANNEL_NAME_NumReturns))
            {
                view->setField<uint8_t>(d,
                                        pointIndex,
                                        getData<uint8_t>(points, CHANNEL_NAME_NumReturns, pointIndex));
            }
            else if (Utils::iequals(name, "ScanDirectionFlag") &&
                     m_pointInfo.hasChannel(CHANNEL_NAME_ScanDir))
            {
                view->setField<uint8_t>(d,
                                        pointIndex,
                                        getData<uint8_t>(points, CHANNEL_NAME_ScanDir, pointIndex));
            }
            else if (Utils::iequals(name, "ScanAngleRank") &&
                     m_pointInfo.hasChannel(CHANNEL_NAME_ScanAngle))
            {
                view->setField<int8_t>( d,
                                        pointIndex,
                                        getData<int8_t>(points, CHANNEL_NAME_ScanAngle, pointIndex));
            }
            else if (Utils::iequals(name, "EdgeOfFlightLine") &&
                     m_pointInfo.hasChannel(CHANNEL_NAME_EdgeFlightLine))
            {
                view->setField<uint8_t>(d,
                                        pointIndex,
                                        getData<uint8_t>(points, CHANNEL_NAME_EdgeFlightLine, pointIndex));
            }
            else if (Utils::iequals(name, "Classification") &&
                     m_pointInfo.hasChannel(CHANNEL_NAME_ClassId))
            {
                view->setField<uint8_t>(d,
                                        pointIndex,
                                        getData<uint8_t>(points, CHANNEL_NAME_ClassId, pointIndex));
            }
            else if (Utils::iequals(name, "UserData") &&
                     m_pointInfo.hasChannel(CHANNEL_NAME_UserData))
            {
                view->setField<uint8_t>(d,
                                        pointIndex,
                                        getData<uint8_t>(points, CHANNEL_NAME_UserData, pointIndex));
            }
            else if (Utils::iequals(name, "PointSourceId") &&
                     m_pointInfo.hasChannel(CHANNEL_NAME_SourceId))
            {
                view->setField<uint16_t>(   d,
                                            pointIndex,
                                            getData<uint16_t>(points, CHANNEL_NAME_SourceId, pointIndex));
            }
        }
    }

    return count;
}

} // namespaces
