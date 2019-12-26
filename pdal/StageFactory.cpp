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

#include <algorithm>
#include <sstream>
#include <string>

namespace pdal
{

namespace
{

const std::vector<std::string> protocols { "ept", "i3s" };

std::string getDriverProtocol(std::string filename)
{
    const auto protocol = std::find_if(protocols.begin(), protocols.end(),
            [&filename](std::string protocol)
            {
                const std::string search(protocol + "://");
                return Utils::startsWith(filename, search);
            });

    if (protocol != protocols.end())
        return *protocol;
    return "";
}

}

/**
  Find the default reader for a file.

  \param filename  Filename for which to infer a reader.
  \return  Name of the reader driver associated with the file.
*/
std::string StageFactory::inferReaderDriver(const std::string& filename)
{
    std::string ext;

    const std::string driverProtocol = getDriverProtocol(filename);

    if (!driverProtocol.empty())
        ext = "." + driverProtocol;
    else
        ext = FileUtils::extension(filename);
    // Strip off '.' and make lowercase.
    if (ext.length())
        ext = Utils::tolower(ext.substr(1));

    if (ext == "json" && Utils::endsWith(filename, "ept.json"))
        return "readers.ept";

    return PluginManager<Stage>::extensions().defaultReader(ext);
}


/**
  Find the default writer for a file.

  \param filename  Filename for which to infer a writer.
  \return  Name of the writer driver associated with the file.
*/
std::string StageFactory::inferWriterDriver(const std::string& filename)
{
    const std::string driverProtocol = getDriverProtocol(filename);

    std::string lFilename = Utils::tolower(filename);
    if (lFilename == "devnull" || lFilename == "/dev/null")
        return "writers.null";

    std::string ext;
    if (lFilename == "stdout")
        ext = ".txt";
    else if (!driverProtocol.empty())
        ext = "." + driverProtocol;
    else
        ext = Utils::tolower(FileUtils::extension(lFilename));
    // Strip off '.' and make lowercase.
    if (ext.length())
        ext = Utils::tolower(ext.substr(1));

    return PluginManager<Stage>::extensions().defaultWriter(ext);
}


StageFactory::StageFactory(bool /* legacy */)
{
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
