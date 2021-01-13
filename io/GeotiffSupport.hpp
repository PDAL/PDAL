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

#pragma once

#include <pdal/Log.hpp>
#include <pdal/SpatialReference.hpp>

namespace pdal
{

namespace Geotiff
{
    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };
}

struct Entry;

class GeotiffSrs
{
public:
    GeotiffSrs(const std::vector<uint8_t>& directoryRec,
        const std::vector<uint8_t>& doublesRec,
        const std::vector<uint8_t>& asciiRec, LogPtr log);
    SpatialReference srs() const
        { return m_srs; }

    std::string const& gtiffPrintString()
        { return m_gtiff_print_string; }

private:
    SpatialReference m_srs;
    LogPtr m_log;
    std::string m_gtiff_print_string;

    void validateDirectory(const Entry *ent, size_t numEntries,
        size_t numDoubles, size_t asciiSize);
};

class GeotiffTags
{
public:
    GeotiffTags(const SpatialReference& srs);

    std::vector<uint8_t>& directoryData()
        { return m_directoryRec; }
    std::vector<uint8_t>& doublesData()
        { return m_doublesRec; }
    std::vector<uint8_t>& asciiData()
        { return m_asciiRec; }

private:
    std::vector<uint8_t> m_directoryRec;
    std::vector<uint8_t> m_doublesRec;
    std::vector<uint8_t> m_asciiRec;
};

} // namespace pdal
