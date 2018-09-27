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
#include "../lepcc/src/include/lepcc_c_api.h"
#include "../lepcc/src/include/lepcc_types.h"
#include "pool.hpp"
#include "SlpkExtractor.hpp"

#include <istream>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/pdal_features.hpp>
#include <pdal/compression/LazPerfCompression.hpp>
#include <gdal.h>

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
    //create temp path
    std::string path = arbiter::fs::getTempPath();

    //use arbiter to create new directory if doesn't already exist
    std::string fullPath(path+ FileUtils::stem(
                FileUtils::getFilename(m_filename)));
    arbiter::fs::mkdirp(fullPath);

    //un-archive the slpk archive
    SlpkExtractor slpk(m_filename, fullPath);
    slpk.extract();
    m_filename = fullPath;
    log()->get(LogLevel::Debug) << "Making directory at: " <<
        fullPath << std::endl;

    //unarchive and decompress the 3dscenelayer
    //and create json info object
    auto compressed = m_arbiter->get(m_filename
            + "/3dSceneLayer.json.gz");
    std::string jsonString;

    m_decomp.decompress(jsonString, compressed.data(),
            compressed.size());
    m_info = parse(jsonString);
    if (m_info.empty())
        throwError(std::string("Incorrect Json object"));

}

//Traverse tree through nodepages. Create a nodebox for each node in
//the tree and test if it overlaps with the bounds created by user.
//If it's a leaf node(the highest resolution) and it overlaps, add
//it to the list of nodes to be pulled later.
void SlpkReader::buildNodeList(std::vector<int>& nodes, int pageIndex)
{
    std::string nodeUrl = m_filename + "/nodepages/"
        + std::to_string(pageIndex);

    std::string ext = ".json.gz";

    if(!FileUtils::fileExists(nodeUrl + ext))
    {
        return;
    }

    std::string output;
    auto compressed = m_arbiter->get(nodeUrl+ext);
    m_decomp.decompress<std::string>(
            output,
            compressed.data(),
            compressed.size());
    const Json::Value nodeIndexJson = parse(output);

    if(nodeIndexJson.empty() || !nodeIndexJson.isMember("nodes"))
        throwError(std::string("Could not find node information"));

    int pageSize = nodeIndexJson["nodes"].size();
    int initialNode = nodeIndexJson["nodes"][0]["resourceId"].asInt();

    for (int i = 0; i < pageSize; i++)
    {
        BOX3D nodeBox = parseBox(nodeIndexJson["nodes"][i]);
        int cCount = nodeIndexJson["nodes"][i]["childCount"].asInt();

        //density calculated as (number of points in node) / (lod threshold)
        int pCount =  nodeIndexJson["nodes"][i]["vertexCount"].asInt();
        double lodThreshold =
            nodeIndexJson["nodes"][i]["lodThreshold"].asDouble();
        double density = (double)pCount / lodThreshold;
        bool overlap = m_bounds.overlaps(nodeBox);

        //if density is within desired lod and the bounds overlap with node
        //lod is default at -1, meaning no user input. This will grab only
        //leaf nodes, or the highest resolution.
        if(m_args.lod == -1 && overlap && cCount == 0)
        {
            int name = nodeIndexJson["nodes"][i]["resourceId"].asInt();
            nodes.push_back(name);
        }
        else if (density > m_args.lod &&
            density < (m_args.lod + 0.5) &&
            overlap)
        {
            int name = nodeIndexJson["nodes"][i]["resourceId"].asInt();
            nodes.push_back(name);
        }
    }
    buildNodeList(nodes, ++pageIndex);
}

std::vector<char> SlpkReader::fetchBinary(std::string url,
        std::string attNum, std::string ext) const
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
