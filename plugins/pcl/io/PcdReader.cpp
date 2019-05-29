/******************************************************************************
* Copyright (c) 2014, Brad Chambers (brad.chambers@gmail.com)
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

#include "PcdReader.hpp"
#include "point_types.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include <pdal/PointView.hpp>
#include "../PCLConversions.hpp"
#include "PcdHeader.hpp"

namespace pdal
{

static PluginInfo const s_info
{
    "readers.pcd",
    "Read data in the Point Cloud Library (PCL) format.",
    "http://pdal.io/stages/readers.pcd.html"
};

CREATE_SHARED_STAGE(PcdReader, s_info)

std::string PcdReader::getName() const
{
    return s_info.name;
}

QuickInfo PcdReader::inspect()
{
    QuickInfo qi;

    initialize();

    for (auto i : m_header.m_fields)
        qi.m_dimNames.push_back(i.m_label);
    qi.m_pointCount = m_header.m_pointCount;
    qi.m_valid = true;

    return qi;
}

void PcdReader::ready(PointTableRef table)
{
    // initialize();
}


void PcdReader::addDimensions(PointLayoutPtr layout)
{
    for (auto i : m_header.m_fields)
    {
        Dimension::BaseType base = Dimension::BaseType::None;
        if (i.m_type == PcdFieldType::U)
            base = Dimension::BaseType::Unsigned;
        else if (i.m_type == PcdFieldType::I)
            base = Dimension::BaseType::Signed;
        else if (i.m_type == PcdFieldType::F)
            base = Dimension::BaseType::Floating;
        Dimension::Type t = static_cast<Dimension::Type>(unsigned(base) | i.m_size);
        layout->registerOrAssignDim(i.m_label, t);
    }
}


point_count_t PcdReader::read(PointViewPtr view, point_count_t /*count*/)
{
    pcl::PointCloud<XYZIRGBA>::Ptr cloud(new pcl::PointCloud<XYZIRGBA>);

    pcl::PCDReader r;
    r.read<XYZIRGBA>(m_filename, *cloud);

    pclsupport::PCDtoPDAL(*cloud, view);

    return cloud->points.size();
}

void PcdReader::initialize()
{
    std::istream *fs;
    try {
        fs = Utils::openFile(m_filename, true);
        *fs >> m_header;
        // std::cout << m_header; // echo back for testing
    } catch (...) {
        Utils::closeFile(fs);
        throw;
    }

    Utils::closeFile(fs);
}

} // namespace pdal
