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

#include <pdal/PipelineManager.hpp>

#include <pdal/Filter.hpp>
#include <pdal/MultiFilter.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Writer.hpp>

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/drivers/liblas/Reader.hpp>
#include <pdal/drivers/liblas/Writer.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

namespace pdal
{
    
PipelineManager::PipelineManager()
{

    return;
}


PipelineManager::~PipelineManager()
{
    while (m_stages.size())
    {
        m_stages.pop_back();
    }
    return;
}


ReaderPtr PipelineManager::addReader(const std::string& type, const Options& options)
{
    ReaderPtr stage = m_factory.createReader(type, options);
    m_stages.push_back(stage);
    return stage;
}


FilterPtr PipelineManager::addFilter(const std::string& type, const DataStagePtr& prevStage, const Options& options)
{
    FilterPtr stage = m_factory.createFilter(type, prevStage, options);
    m_stages.push_back(stage);
    return stage;
}


MultiFilterPtr PipelineManager::addMultiFilter(const std::string& type, const std::vector<const DataStagePtr>& prevStages, const Options& options)
{
    MultiFilterPtr stage = m_factory.createMultiFilter(type, prevStages, options);
    m_stages.push_back(stage);
    return stage;
}


WriterPtr PipelineManager::addWriter(const std::string& type, const DataStagePtr& prevStage, const Options& options)
{
    WriterPtr stage = m_factory.createWriter(type, prevStage, options);
    m_stages.push_back(stage);
    return stage;
}


Option<std::string> PipelineManager::parseOption(const boost::property_tree::ptree& tree)
{
    // cur is an option element, such as this:
    //     <option>
    //       <name>myname</name>
    //       <description>my descr</description>
    //       <value>17</value>
    //     </option>
    // this function will process the element and return an Option from it

    Option<std::string> option(tree);

    return option;
}


std::string PipelineManager::parseType(const boost::property_tree::ptree& tree)
{
    // tree is this:
    //     <type>drivers.foo.writer</type>

    std::string s = tree.get_value("Type");

    return s;
}


ReaderPtr PipelineManager::parseReader(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type;

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        if (iter->first == "Type")
        {
            const boost::property_tree::ptree subtree = iter->second;
            type = parseType(subtree);
        }
        else if (iter->first == "Option")
        {
            const boost::property_tree::ptree subtree = iter->second;
            Option<std::string> option = parseOption(subtree);
            options.add(option);
        }
        else
        {
            throw pdal_error("xml reader invalid child of Reader element");
        }
        ++iter;
    }

    ReaderPtr ptr = addReader(type, options);

    return ptr;
}


FilterPtr PipelineManager::parseFilter(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type;
    DataStagePtr prevStage;

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        if (iter->first == "Type")
        {
            const boost::property_tree::ptree subtree = iter->second;
            type = parseType(subtree);
        }
        else if (iter->first == "Option")
        {
            const boost::property_tree::ptree subtree = iter->second;
            Option<std::string> option = parseOption(subtree);
            options.add(option);
        }
        else if (iter->first == "Filter")
        {
            const boost::property_tree::ptree subtree = iter->second;
            prevStage = parseFilter(subtree);
        }
        else if (iter->first == "MultiFilter")
        {
            const boost::property_tree::ptree subtree = iter->second;
            prevStage = parseMultiFilter(subtree);
        }
        else if (iter->first == "Reader")
        {
            const boost::property_tree::ptree subtree = iter->second;
            prevStage = parseReader(subtree);
        }
        else
        {
            throw pdal_error("xml reader invalid child of Reader element");
        }
        ++iter;
    }

    FilterPtr ptr = addFilter(type, prevStage, options);

    return ptr;
}


MultiFilterPtr PipelineManager::parseMultiFilter(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type;
    std::vector<const DataStagePtr> prevStages;

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        if (iter->first == "Type")
        {
            const boost::property_tree::ptree subtree = iter->second;
            type = parseType(subtree);
        }
        else if (iter->first == "Option")
        {
            const boost::property_tree::ptree subtree = iter->second;
            Option<std::string> option = parseOption(subtree);
            options.add(option);
        }
        else if (iter->first == "Filter")
        {
            const boost::property_tree::ptree subtree = iter->second;
            DataStagePtr prevStage = parseFilter(subtree);
            prevStages.push_back(prevStage);
        }
        else if (iter->first == "MultiFilter")
        {
            const boost::property_tree::ptree subtree = iter->second;
            DataStagePtr prevStage = parseMultiFilter(subtree);
            prevStages.push_back(prevStage);
        }
        else if (iter->first == "Reader")
        {
            const boost::property_tree::ptree subtree = iter->second;
            DataStagePtr prevStage = parseReader(subtree);
            prevStages.push_back(prevStage);
        }
        else
        {
            throw pdal_error("xml reader invalid child of Reader element");
        }
        ++iter;
    }

    MultiFilterPtr ptr = addMultiFilter(type, prevStages, options);

    return ptr;
}


WriterPtr PipelineManager::parseWriter(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type;
    DataStagePtr prevStage;

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        if (iter->first == "Type")
        {
            const boost::property_tree::ptree subtree = iter->second;
            type = parseType(subtree);
        }
        else if (iter->first == "Option")
        {
            const boost::property_tree::ptree subtree = iter->second;
            Option<std::string> option = parseOption(subtree);
            options.add(option);
        }
        else if (iter->first == "Filter")
        {
            const boost::property_tree::ptree subtree = iter->second;
            prevStage = parseFilter(subtree);
        }
        else if (iter->first == "MultiFilter")
        {
            const boost::property_tree::ptree subtree = iter->second;
            prevStage = parseMultiFilter(subtree);
        }
        else if (iter->first == "Reader")
        {
            const boost::property_tree::ptree subtree = iter->second;
            prevStage = parseReader(subtree);
        }
        else
        {
            throw pdal_error("xml reader invalid child of Reader element");
        }
        ++iter;
    }

    WriterPtr ptr = addWriter(type, prevStage, options);

    return ptr;
}


StagePtr PipelineManager::parsePipeline(const boost::property_tree::ptree& tree)
{
    StagePtr stage;

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        if (iter->first == "Reader")
        {
            const boost::property_tree::ptree subtree = iter->second;
            stage = parseReader(subtree);
        }
        else if (iter->first == "Filter")
        {
            const boost::property_tree::ptree subtree = iter->second;
            stage = parseFilter(subtree);
        }
        else if (iter->first == "MultiFilter")
        {
            const boost::property_tree::ptree subtree = iter->second;
            stage = parseMultiFilter(subtree);
        }
        else if (iter->first == "Writer")
        {
            const boost::property_tree::ptree subtree = iter->second;
            stage = parseWriter(subtree);
        }
        else
        {
            throw pdal_error("xml reader invalid child of Pipeline element");
        }
        ++iter;
    }

    return stage;
}


void PipelineManager::readXml(const std::string& filename)
{
    boost::property_tree::ptree tree;
    boost::property_tree::xml_parser::read_xml(filename, tree);

    boost::property_tree::ptree pipeline = tree.get_child("Pipeline"); // err check

    parsePipeline(pipeline);

    return;
}





} // namespace pdal
