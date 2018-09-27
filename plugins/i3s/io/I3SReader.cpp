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

#include "I3SReader.hpp"

namespace pdal
{

static PluginInfo const i3sInfo
{
    "readers.i3s",
    "I3S Reader",
    "http://pdal.io/stages/readers.i3s.html"
};

CREATE_SHARED_STAGE(I3SReader, i3sInfo)

std::string I3SReader::getName() const { return i3sInfo.name; }

void I3SReader::initInfo()
{
    try
    {
        m_info = parse(m_arbiter->get(m_filename));

        if (m_info.empty())
            throwError(std::string("Incorrect Json object"));
        if (!m_info.isMember("layers"))
            throwError(std::string("Json object contains no layers"));

        m_info = m_info["layers"][0];
    }catch(pdal_error& e)
    {
        throwError(std::string("Error parsing Json object: ")+e.what());
    }

    m_filename += "/layers/0";
}

//Traverse tree through nodepages. Create a nodebox for each node in
//the tree and test if it overlaps with the bounds created by user.
//If it's a leaf node(the highest resolution) and it overlaps, add
//it to the list of nodes to be pulled later.
void I3SReader::buildNodeList(std::vector<int>& nodes, int pageIndex)
{
    std::string nodeUrl = m_filename + "/nodepages/"
        + std::to_string(pageIndex);

    const Json::Value nodeIndexJson = parse(m_arbiter->get(nodeUrl));


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
        if (m_args.lod == -1 && overlap && cCount == 0)
        {
            int name = nodeIndexJson["nodes"][i]["resourceId"].asInt();
            nodes.push_back(name);
        }
        //0.5 represents a large enough gap to find the related nodes
        //while also separating from different resolution sets
        else if (density > m_args.lod &&
            density < (m_args.lod + 0.5) &&
            overlap)
        {
            int name = nodeIndexJson["nodes"][i]["resourceId"].asInt();
            nodes.push_back(name);
        }

        //keeps track of largest node so recursive loop knows when to stop
        if ((nodeIndexJson["nodes"][i]["firstChild"].asInt() +
                cCount - 1) > m_maxNode)
        {
            m_maxNode = nodeIndexJson["nodes"][i]["firstChild"].asInt() +
                cCount - 1;
        }
        else if (initialNode + i == m_maxNode)
            return;
    }
    buildNodeList(nodes, ++pageIndex);
}

std::vector<char> I3SReader::fetchBinary(std::string url,
        std::string attNum, std::string ext) const
{
    // For the REST I3S endpoint there are no file extensions.
    return m_arbiter->getBinary(url + attNum);
}

} //namespace pdal
