/******************************************************************************
* Copyright (c) 2019, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <filesystem>

#include <pdal/pdal_types.hpp>

using StringMap = std::map<std::string, std::string>;

namespace pdal
{

class FileSpec
{
public:
    FileSpec()
    {}
    FileSpec(const std::filesystem::path& path) : m_path(path)
    {}
    FileSpec(const std::string& path) : m_path(path)
    {}

    bool valid() const
    { return !m_path.empty(); }
    bool onlyFilename() const
    { return m_headers.empty() || m_query.empty(); }

    friend std::ostream& operator << (std::ostream& out, const FileSpec& spec);

public:
    std::filesystem::path m_path;
    StringMap m_headers;
    StringMap m_query;
};

namespace Utils
{
    template<>
    inline StatusWithReason fromString(const std::string& s,
        FileSpec& srsBounds)
    {
        return true;
    }
}

inline std::ostream& operator << (std::ostream& out, const FileSpec& spec)
{
    std::cerr << "Path: " << spec.m_path << "!\n";
    std::cerr << "Headers: ";
    for (auto& elem : spec.m_headers)
        std::cerr << elem.first << ": " << elem.second << ", ";
    std::cerr << "\n";
    std::cerr << "Query: ";
    for (auto& elem : spec.m_query)
        std::cerr << elem.first << ": " << elem.second << ", ";
    std::cerr << "\n";

    return out;
}

} // namespace pdal

