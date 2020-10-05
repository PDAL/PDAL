/******************************************************************************
* Copyright (c) 2020, Ryan Pals, ryan@hobu.co
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
#include <pdal/pdal_types.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include "Hdf5Handler.hpp"

#include <map>



namespace pdal
{

static PluginInfo const s_info
{
    "readers.hdf",
    "HDF Reader",
    "http://pdal.io/stages/readers.hdf.html"
};

CREATE_SHARED_STAGE(HdfReader, s_info)

std::string HdfReader::getName() const { return s_info.name; }

HdfReader::HdfReader()
    : m_hdf5Handler(new hdf5::Handler())
    { }


void HdfReader::addDimensions(PointLayoutPtr layout)
{
    m_hdf5Handler->setLog(log());
    m_hdf5Handler->initialize(m_filename, m_pathDimMap);

    for (hdf5::DimInfo& dim : m_hdf5Handler->getDimensions())
    {
        dim.setId(layout->registerOrAssignDim(dim.getName(), dim.getPdalType()));
    }
}


void HdfReader::ready(PointTableRef table)
{
    m_index = 0;
}


point_count_t HdfReader::read(PointViewPtr view, point_count_t count)
{
    PointId startId = view->size();
    point_count_t remaining = m_hdf5Handler->getNumPoints() - m_index;
    count = (std::min)(count, remaining);
    PointId nextId = startId;

    for(uint64_t pi = 0; pi < m_hdf5Handler->getNumPoints(); pi++) {
        for(hdf5::DimInfo& dim : m_hdf5Handler->getDimensions()) {
            uint8_t *p = dim.getValue(pi);
            view->setField(dim.getId(), dim.getPdalType(), nextId, (void*) p);
        }
        m_index++;
        nextId++;
    }

    return count;
}


bool HdfReader::processOne(PointRef& point)
{
    for(hdf5::DimInfo& dim : m_hdf5Handler-> getDimensions()) {
        uint8_t *p = dim.getValue(m_index);
        point.setField(dim.getId(), dim.getPdalType(), p);
    }

    m_index++;
    return m_index <= m_hdf5Handler->getNumPoints();
}

void HdfReader::addArgs(ProgramArgs& args)
{
    args.add("dimensions", "Map of HDF path to PDAL dimension", m_pathDimJson);
}

void HdfReader::initialize()
{
    parseDimensions();
}

void HdfReader::parseDimensions()
{
    log()->get(LogLevel::Info) << "**JSON map**" << std::endl;
    log()->get(LogLevel::Info) << m_pathDimJson << std::endl;

    if(m_pathDimJson.is_null()) {
        throw pdal_error("Required option 'dimensions' was not set");
    } else if(!m_pathDimJson.is_object()) {
        throw pdal_error("Option 'dimensions' must be a JSON object, not a " +
            std::string(m_pathDimJson.type_name()));
    }

    for(auto& entry : m_pathDimJson.items()) {
        std::string dimName = entry.key();
        auto datasetName = entry.value();
        log()->get(LogLevel::Info) << "Key: " << dimName << ", Value: "
            << datasetName << ", Type: " << datasetName.type_name() << std::endl;

        if(!datasetName.is_string()) {
            throw pdal_error("Every value in 'dimensions' must be a string. Key '"
                + dimName + "' has value with type '" +
                std::string(datasetName.type_name()) + "'");
        } else {
            m_pathDimMap[dimName] = datasetName.get<std::string>();
        }
    }
}

void HdfReader::done(PointTableRef table)
{
    m_hdf5Handler->close();
}


} // namespace pdal
