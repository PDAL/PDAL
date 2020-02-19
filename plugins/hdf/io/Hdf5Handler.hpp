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

#pragma once

#include <pdal/pdal_export.hpp>  // Suppresses windows 4251 messages
#include <pdal/Dimension.hpp>
#include <pdal/Log.hpp>
#include "H5Cpp.h"
#include <nlohmann/json.hpp>

#include <memory>
#include <vector>
#include <string>
#include <map>

namespace pdal
{

namespace hdf5
{

class DimInfo
{
public:
    DimInfo(
        const std::string& dimName,
        const std::string& datasetName,
        H5::H5File *file);

    uint8_t *getValue(pdal::point_count_t pointIndex);
    //setters
    void setId(Dimension::Id id);
    //getters
    Dimension::Id getId();
    Dimension::Type getPdalType();
    std::string getName();
    hsize_t getNumPoints();

private:
    std::vector<uint8_t> m_buffer;
    std::string m_name;
    Dimension::Type m_pdalType;
    Dimension::Id m_pdalId = Dimension::Id::Unknown;
    hsize_t chunkUpperBound = 0,
            chunkLowerBound = 0,
            m_numPoints = 0,
            m_chunkSize;
    H5::DataSet m_dset;
    size_t m_size;
};

class Handler
{
public:
    void initialize(
            const std::string& filename,
            const std::map<std::string,std::string>& map);
    void close();

    hsize_t getNumPoints() const;
    std::vector<pdal::hdf5::DimInfo>& getDimensions();

    void setLog(pdal::LogPtr log);

private:
    std::vector<pdal::hdf5::DimInfo> m_dimInfos;

    pdal::LogPtr m_logger;

    std::unique_ptr<H5::H5File> m_h5File;
    hsize_t m_numPoints = 0;
};

} //namespace hdf5

} // namespace pdal

