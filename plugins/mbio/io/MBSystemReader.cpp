/******************************************************************************
* Copyright (c) 2017, Howard Butler (hobu@hob.co)
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

#include "MBSystemReader.hpp"

#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/DimUtil.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.mbio",
    "MBSystem Reader",
    "http://www.pdal.io/stages/readers.mbio.html" );

CREATE_SHARED_PLUGIN(1, 0, MBSystemReader, Reader, s_info)

std::string MBSystemReader::getName() const { return s_info.name; }

MBSystemReader::MBSystemReader()
    : pdal::Reader()
    , m_index(0)
    , m_initialized(false)
{}


void MBSystemReader::addArgs(ProgramArgs& args)
{
}


void MBSystemReader::done(PointTableRef)
{
    m_initialized = false;
    getMetadata().addList("filename", m_filename);
}

void MBSystemReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;


//     for (unsigned int i=0; i<pointinfo.getNumChannels(); i++)
//     {
//         const LizardTech::ChannelInfo &channel = pointinfo.getChannel(i);
//         const char* name = channel.getName();
//         auto translated = dimensionTranslations.find(name);
//         if (translated != dimensionTranslations.end())
//             name = translated->second.c_str();
//
//         LizardTech::DataType t = channel.getDataType();
//         Dimension::Type pdal_type = getPDALType(t);
//         layout->registerOrAssignDim(name, pdal_type);
//     }
//     m_layout = layout;

}


void MBSystemReader::ready(PointTableRef table, MetadataNode& m)
{
    m_index = 0;

    int verbose = 0;
    int pings = 0;  // Perhaps an argument for this?
    int lonflip = 0; // Longitude -180 -> 180
    double bounds[4] { -180, 180, -90, 90 };
    int btime_i[7];
    int etime_i[7];
    double speedmin;
    double timegap;
    char *mbio_ptr;
    double btime_d;
    double etime_d;
    int beams_bath;
    int beams_amp;
    int pixels_ss;
    int error;

    // Need to set filename and format from args.

    mb_read_init(verbose, m_filename.data(), m_format, bounds, btime_i,
        etime_i, speedmin, timegap, &mbio_ptr, &btime_d, &etime_d, &beams_bath,
        &beams_amp, &pixels_ss, &error);
}


QuickInfo MBSystemReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    MBSystemReader::initialize();
    addDimensions(layout.get());

//     BOX3D b(m_PS->getBounds().x.min, m_PS->getBounds().x.max,
//         m_PS->getBounds().y.min, m_PS->getBounds().y.max,
//         m_PS->getBounds().z.min,m_PS->getBounds().z.max);

    Dimension::IdList dims = layout->dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
        qi.m_dimNames.push_back(layout->dimName(*di));
//     if (!Utils::numericCast(m_PS->getNumPoints(), qi.m_pointCount))
//         qi.m_pointCount = std::numeric_limits<point_count_t>::max();
//     qi.m_bounds = b;
//     qi.m_srs = pdal::SpatialReference(m_PS->getWKT());
    qi.m_valid = true;

    PointTable table;
    done(table);

    return qi;
}


point_count_t MBSystemReader::read(PointViewPtr view, point_count_t count)
{
    using namespace pdal::Dimension;


    Dimension::IdList dims = view->dims();

//     for (point_count_t pointIndex=0; pointIndex<count; pointIndex++)
//     {
//     }

    return 0;
}

} // namespaces
