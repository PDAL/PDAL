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

#include <nlohmann/json.hpp>

#include <pdal/StageExtensions.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{

namespace
{

StringList parse(const std::string& val)
{
    // This should yeild POSIX-compatible extensions
    auto issplit = [](char c)
    {
        return (!std::isalnum(c) && c != '_');
    };
    return Utils::split2(val, issplit);
}

// This is here as JSON because I was going to parse it from a file at
// runtime.  The advantage of that is that it allows people who make
// their own plugins to add to the list.  For now we decided that managing
// a configuration file like this wasn't worth it.

// NOTE: Only extensions for dynamic stages go here.  Static stage extensions
//  are defined in the stage files themselves.
const std::string extension_json (
R"PDALEXTENSIONS(

{
    "readers.icebridge" : "icebridge h5",
    "readers.matlab" : "mat",
    "writers.matlab" : "mat",
    "readers.numpy" : "npy",
    "readers.nitf" : "nitf, nsf, ntf",
    "writers.nitf" : "nitf, nsf, ntf",
    "readers.pcd" : "pcd",
    "writers.pcd" : "pcd",
    "readers.rdb" : "rdbx",
    "readers.sqlite" : "sqlite, gpkg",
    "writers.sqlite" : "sqlite, gpkg",
    "readers.mrsid" : "sid",
    "readers.rxp" : "rxp",
    "readers.fbx" : "fbx",
    "readers.slpk" : "slpk",
    "readers.i3s" : "i3s",
    "readers.e57" : "e57",
    "writers.e57" : "e57"
}

)PDALEXTENSIONS"
);

} // unnamed namespace

StageExtensions::StageExtensions(LogPtr log) : m_log(log)
{}

void StageExtensions::load()
{
    static bool loaded(false);

    if (loaded)
        return;
    loaded = true;

    NL::json root;

    try
    {
        root = NL::json::parse(extension_json);
    }
    catch (const NL::json::parse_error& err)
    {
        std::string e = "Unable to parse 'pdal_extensions.json': " +
            std::string(err.what()) +
            " Plugins will not be able to be inferred from filenames.";
        throw pdal_error(e);
    }

    if (!root.is_object())
    {
        throw pdal_error("Invalid root object in 'pdal_extensions.json'. "
            "Must be an object.");
    }

    for (auto it : root.items())
    {
        const std::string& stage = it.key();
        const NL::json& val = it.value();

        if (!Utils::startsWith(stage, "readers") &&
            !Utils::startsWith(stage, "writers"))
        {
            throw pdal_error("Only readers and writers may define "
                "extensions. '" +  stage + "' invalid.");
        }
        if (!val.is_string())
            throw pdal_error("Extension value for '" + stage +
                "' must be a string.");
        set(stage, parse(val.get<std::string>()));
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
