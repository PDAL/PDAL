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

#include <pdal/StageFactory.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/util/FileUtils.hpp>

#include <sstream>
#include <string>

namespace pdal
{

StringList StageFactory::extensions(const std::string& driver)
{
    std::map<std::string, StringList> exts =
        PluginManager<Stage>::get().extensions();

    exts["readers.pcd"] = {"pcd"};
    exts["writers.pcd"] = {"pcd"};
    exts["readers.greyhound"] = {"greyhound"};
    exts["writers.greyhound"] = {"greyhound"};
    exts["readers.matlab"] = {"mat"};
    exts["writers.matlab"] = {"mat"};
    exts["readers.icebridge"] = {"h5", "icebridge"};
    exts["readers.nitf"] = {"nitf", "nsf", "ntf"};
    exts["writers.nitf"] = {"nitf", "nsf", "ntf"};
    exts["readers.sqlite"] = {"sqlite", "gpkg"};
    exts["writers.sqlite"] = {"sqlite", "gpkg"};
    exts["readers.mrsid"] = {"sid"};
    exts["readers.rxp"] = {"rxp"};

    return exts[driver];
}

std::string StageFactory::inferReaderDriver(const std::string& filename)
{
    std::map<std::string, std::string> drivers =
        PluginManager<Stage>::get().inferredReaders();

    drivers["pcd"] = "readers.pcd";
    drivers["greyhound"] = "readers.greyhound";
    drivers["mat"] = "readers.matlab";
    drivers["h5"] = "readers.icebridge";
    drivers["icebridge"] = "readers.icebridge";
    drivers["nitf"] = "readers.nitf";
    drivers["ntf"] = "readers.nitf";
    drivers["nsf"] = "readers.nitf";
    drivers["sqlite"] = "readers.sqlite";
    drivers["gpkg"] = "readers.sqlite";
    drivers["sid"] = "readers.mrsid";
    drivers["rxp"] = "readers.rxp";

    static const std::string ghPrefix("greyhound://");

    std::string ext;
    // filename may actually be a greyhound uri + pipelineId
    if (Utils::iequals(filename.substr(0, ghPrefix.size()), ghPrefix))
        ext = ".greyhound";      // Make it look like an extension.
    else
        ext = FileUtils::extension(filename);

    // Strip off '.' and make lowercase.
    if (ext.length())
        ext = Utils::tolower(ext.substr(1, ext.length() - 1));

    return drivers[ext]; // will be "" if not found
}


std::string StageFactory::inferWriterDriver(const std::string& filename)
{
    std::string ext;

    static const std::string ghPrefix("greyhound://");

    if (filename == "STDOUT")
        ext = ".txt";
    else if (Utils::iequals(filename.substr(0, ghPrefix.size()), ghPrefix))
        ext = ".greyhound";      // Make it look like an extension.
    else
        ext = Utils::tolower(FileUtils::extension(filename));
    // Strip off '.' and make lowercase.
    if (ext.length())
        ext = Utils::tolower(ext.substr(1, ext.length() - 1));

    std::map<std::string, std::string> drivers =
        PluginManager<Stage>::get().inferredWriters();
    drivers["pcd"] = "writers.pcd";
    drivers["greyhound"] = "writers.greyhound";
    drivers["mat"] = "writers.matlab";
    drivers["nitf"] = "writers.nitf";
    drivers["ntf"] = "writers.nitf";
    drivers["nsf"] = "writers.nitf";
    drivers["sqlite"] = "writers.sqlite";
    drivers["gpkg"] = "writers.sqlite";

    return drivers[ext];
}


StageFactory::StageFactory(bool ignored)
{
    (void)ignored;
}


Stage *StageFactory::createStage(std::string const& stage_name)
{
    static_assert(0 < sizeof(Stage), "");
    Stage *s = PluginManager<Stage>::createObject(stage_name);
    if (s)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_ownedStages.push_back(std::unique_ptr<Stage>(s));
    }
    return s;
}


void StageFactory::destroyStage(Stage *s)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto it = m_ownedStages.begin(); it != m_ownedStages.end(); ++it)
    {
        if (s == it->get())
        {
            m_ownedStages.erase(it);
            break;
        }
    }
}

} // namespace pdal
