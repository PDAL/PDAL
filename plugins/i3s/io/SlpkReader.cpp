/******************************************************************************
* Copyright (c) 2018, Kyle Mann (kyle@hobu.co)
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

#include "SlpkReader.hpp"
#include <pdal/util/FileUtils.hpp>

#include "pool.hpp"
#include "EsriUtil.hpp"
#include "SlpkExtractor.hpp"

namespace pdal
{

static PluginInfo const slpkInfo
{
    "readers.slpk",
    "SLPK Reader",
    "http://pdal.io/stages/readers.slpk.html"
};

CREATE_SHARED_STAGE(SlpkReader, slpkInfo)

std::string SlpkReader::getName() const { return slpkInfo.name; }

void SlpkReader::initInfo()
{
    // create temp path
    std::string path = arbiter::getTempPath();

    // use arbiter to create new directory if doesn't already exist
    std::string fullPath(path + FileUtils::stem(
        FileUtils::getFilename(m_filename)));
    arbiter::mkdirp(fullPath);

    // un-archive the slpk archive
    SlpkExtractor slpk(m_filename, fullPath);
    slpk.extract();
    m_filename = fullPath;
    log()->get(LogLevel::Debug) << "Making directory at: " <<
        fullPath << std::endl;

    // unarchive and decompress the 3dscenelayer and create json info object
    auto compressed = m_arbiter->get(m_filename + "/3dSceneLayer.json.gz");
    std::string jsonString;

    m_decomp.decompress(jsonString, compressed.data(), compressed.size());
    m_info = EsriUtil::parse(jsonString);
    if (m_info.empty())
        throwError(std::string("Incorrect Json object"));
}


NL::json SlpkReader::fetchJson(std::string filepath)
{
    std::string output;
    auto compressed = m_arbiter->get(filepath + ".json.gz");
    m_decomp.decompress<std::string>(output, compressed.data(),
        compressed.size());
    return EsriUtil::parse(output);

}

// fetch data using arbiter to get a char vector
std::vector<char> SlpkReader::fetchBinary(std::string url, std::string attNum,
    std::string ext) const
{
    url += attNum + ext;

    auto data(m_arbiter->getBinary(url));

    if (FileUtils::extension(url) != ".gz")
        return data;

    std::vector<char> decomp;
    m_decomp.decompress<std::vector<char>>(decomp, data.data(),
            data.size());
    return decomp;
}

} //namespace pdal
