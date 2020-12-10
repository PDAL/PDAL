/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (info@hobu.co)
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

#include <sstream>

#include <pdal/StageExtensions.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{

namespace
{

// NOTE: Only extensions for dynamic stages go here.  Static stage extensions
//  are defined in the stage files themselves.

using Extensions = std::map<std::string, StringList>;

static const Extensions readerExtensions =
{
  { "readers.draco", { "drc" } },
  { "readers.icebridge", { "icebridge", "h5" } },
  { "readers.matlab", { "mat" } },
  { "readers.nitf", { "nitf", "nsf", "ntf" } },
  { "readers.pcd", { "pcd" } },
  { "readers.rdb", { "rdbx" } },
  { "readers.mrsid", { "sid" } },
  { "readers.rxp", { "rxp" } },
  { "readers.fbx", { "fbx" } },
  { "readers.slpk", { "slpk" } },
  { "readers.i3s", { "i3s" } },
  { "readers.obj", { "obj" } },
  { "readers.e57", { "e57" } }
};

static const Extensions writerExtensions =
{
  { "writers.draco", { "drc" } },
  { "writers.matlab", { "mat" } },
  { "writers.nitf", { "nitf", "nsf", "ntf" } },
  { "writers.pcd", { "pcd" } },
  { "writers.e57", { "e57" } },
  { "writers.fbx", { "fbx" } }
};

} // unnamed namespace

StageExtensions::StageExtensions(LogPtr log) : m_log(log)
{}


void StageExtensions::load()
{
    static bool loaded(false);

    if (loaded)
        return;
    loaded = true;

    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& p : readerExtensions)
    {
        const std::string& stage = p.first;
        for (auto& ext : p.second)
            m_readers[ext] = stage;
    }
    for (auto& p : writerExtensions)
    {
        const std::string& stage = p.first;
        for (auto& ext : p.second)
            m_writers[ext] = stage;
    }
}

PDAL_DLL void StageExtensions::set(const std::string& stage,
    const StringList& exts)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (Utils::startsWith(stage, "readers."))
        for (const std::string& ext : exts)
            m_readers[ext] = stage;
    else if (Utils::startsWith(stage, "writers."))
        for (const std::string& ext : exts)
            m_writers[ext] = stage;
}

// Get the default reader associated with an extension.  Extensions
// are specified without the leading '.'
std::string StageExtensions::defaultReader(const std::string& extension)
{
    load();
    std::lock_guard<std::mutex> lock(m_mutex);
    return (m_readers[extension]);
}


// Get the default writer associated with an extension.  Extensions
// are specified without the leading '.'
std::string StageExtensions::defaultWriter(const std::string& extension)
{
    load();
    std::lock_guard<std::mutex> lock(m_mutex);
    return (m_writers[extension]);
}


StringList StageExtensions::extensions(const std::string& stage)
{
    StringList exts;

    std::lock_guard<std::mutex> lock(m_mutex);

    if (Utils::startsWith(stage, "readers."))
    {
        for (auto& entry : m_readers)
            if (entry.second == stage)
                exts.push_back(entry.first);
    }
    else if (Utils::startsWith(stage, "writers."))
    {
        for (auto& entry : m_writers)
            if (entry.second == stage)
                exts.push_back(entry.first);
    }
    return exts;
}

} // namespace pdal
