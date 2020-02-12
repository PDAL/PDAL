/******************************************************************************
* Copyright (c) 2014, Connor Manning, connor@hobu.co
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

#include "HdfReader.hpp"
#include <pdal/util/FileUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <map>

namespace pdal
{

static PluginInfo const s_info
{
    "readers.hdf",
    "HDF Reader (WIP)",
    "http://pdal.io/"
};

CREATE_SHARED_STAGE(HdfReader, s_info)

std::string HdfReader::getName() const { return s_info.name; }

void HdfReader::addDimensions(PointLayoutPtr layout)
{
    m_hdf5Handler.setLog(log());
    m_hdf5Handler.initialize(m_filename, m_pathDimMap);

    m_infos = m_hdf5Handler.getDimensionInfos();
    for(auto& info : m_infos) {
        info.id = layout->registerOrAssignDim(info.name, info.pdal_type);
    }
}

void HdfReader::prepared(PointTableRef table) {
    for(auto& info : m_infos) {
        m_bufs.push_back((uint8_t *)nullptr);
    }
}

void HdfReader::ready(PointTableRef table)
{
    m_index = 0;
}


point_count_t HdfReader::read(PointViewPtr view, point_count_t count)
{
    PointId startId = view->size();
    point_count_t remaining = m_hdf5Handler.getNumPoints() - m_index;
    count = (std::min)(count, remaining);
    PointId nextId = startId;

    log()->get(LogLevel::Info) << "num infos: " << m_infos.size() << std::endl;
    log()->get(LogLevel::Info) << "num points: " << m_hdf5Handler.getNumPoints() << std::endl;

    for(uint64_t pi = 0; pi < m_hdf5Handler.getNumPoints(); pi++) {
        for(uint64_t index = 0; index < m_infos.size(); ++index) {
            auto& info = m_infos.at(index);
            uint64_t bufIndex = pi % info.chunkSize;
            if(bufIndex == 0) {
                m_bufs.at(index) = m_hdf5Handler.getNextChunk(index);
            }
            uint8_t *p = m_bufs.at(index) + bufIndex*info.size;
            view->setField(info.id, info.pdal_type, nextId, (void*) p);
        }
        nextId++;
    }

    return count;
}

void HdfReader::addArgs(ProgramArgs& args)
{
    args.add("map", "Map of HDF path to PDAL dimension", m_pathDimMap);
}

void HdfReader::initialize()
{
    log()->get(LogLevel::Info) << "***JSON TESTING***" << std::endl;
    log()->get(LogLevel::Info) << m_pathDimMap << std::endl;
    log()->get(LogLevel::Info) << "----------------" << std::endl;
    for(const auto& [key, value] : m_pathDimMap.items()) {
        log()->get(LogLevel::Info) << "Key: " << key << ", Val: " << value <<std::endl;
    }

    // Data are WGS84 (4326) with ITRF2000 datum (6656)
    // See http://nsidc.org/data/docs/daac/icebridge/ilvis2/index.html for
    // background
    setSpatialReference(SpatialReference("EPSG:4326"));
}

void HdfReader::done(PointTableRef table)
{
    m_hdf5Handler.close();
}


bool HdfReader::eof()
{
    return m_index >= m_hdf5Handler.getNumPoints();
}

} // namespace pdal
