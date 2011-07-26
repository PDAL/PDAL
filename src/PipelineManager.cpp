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

#include <boost/property_tree/xml_parser.hpp>

namespace pdal
{
    
PipelineManager::PipelineManager()
{

    return;
}


PipelineManager::~PipelineManager()
{
    while (m_readers.size())
    {
        Reader* reader = m_readers.back();
        m_readers.pop_back();
        delete reader;
    }

    while (m_filters.size())
    {
        Filter* filter = m_filters.back();
        m_filters.pop_back();
        delete filter;
    }

    while (m_multifilters.size())
    {
        MultiFilter* multifilter = m_multifilters.back();
        m_multifilters.pop_back();
        delete multifilter;
    }

    while (m_writers.size())
    {
        Writer* writer = m_writers.back();
        m_writers.pop_back();
        delete writer;
    }

    return;
}


Reader* PipelineManager::addReader(const std::string& type, const Options& options)
{
    Reader* stage = m_factory.createReader(type, options);
    m_readers.push_back(stage);
    return stage;
}


Filter* PipelineManager::addFilter(const std::string& type, const Stage& prevStage, const Options& options)
{
    Filter* stage = m_factory.createFilter(type, prevStage, options);
    m_filters.push_back(stage);
    return stage;
}


MultiFilter* PipelineManager::addMultiFilter(const std::string& type, const std::vector<const Stage*>& prevStages, const Options& options)
{
    MultiFilter* stage = m_factory.createMultiFilter(type, prevStages, options);
    m_multifilters.push_back(stage);
    return stage;
}


Writer* PipelineManager::addWriter(const std::string& type, const Stage& prevStage, const Options& options)
{
    Writer* stage = m_factory.createWriter(type, prevStage, options);
    m_writers.push_back(stage);
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


Reader* PipelineManager::parseReader(const boost::property_tree::ptree& tree)
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

    Reader* ptr = addReader(type, options);

    return ptr;
}


Filter* PipelineManager::parseFilter(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type = "";
    Stage* prevStage = NULL;

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

    Filter* ptr = addFilter(type, *prevStage, options);

    return ptr;
}


MultiFilter* PipelineManager::parseMultiFilter(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type = "";
    std::vector<const Stage*> prevStages;

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
            Stage* prevStage = parseFilter(subtree);
            prevStages.push_back(prevStage);
        }
        else if (iter->first == "MultiFilter")
        {
            const boost::property_tree::ptree subtree = iter->second;
            Stage* prevStage = parseMultiFilter(subtree);
            prevStages.push_back(prevStage);
        }
        else if (iter->first == "Reader")
        {
            const boost::property_tree::ptree subtree = iter->second;
            Stage* prevStage = parseReader(subtree);
            prevStages.push_back(prevStage);
        }
        else
        {
            throw pdal_error("xml reader invalid child of Reader element");
        }
        ++iter;
    }

    MultiFilter* ptr = addMultiFilter(type, prevStages, options);

    return ptr;
}


Writer* PipelineManager::parseWriter(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type = "";
    Stage* prevStage = NULL;

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

    Writer* ptr = addWriter(type, *prevStage, options);

    return ptr;
}


void PipelineManager::parsePipeline(const boost::property_tree::ptree& tree, Writer*& writer, Stage*& stage)
{
    writer = NULL;
    stage = NULL;

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
            writer = parseWriter(subtree);
        }
        else
        {
            throw pdal_error("xml reader invalid child of Pipeline element");
        }
        ++iter;
    }

    return;
}


void PipelineManager::readXml(const std::string& filename)
{
    boost::property_tree::ptree tree;
    boost::property_tree::xml_parser::read_xml(filename, tree);

    boost::property_tree::ptree pipeline = tree.get_child("Pipeline"); // err check

    Writer* writer = NULL;
    Stage* stage = NULL;
    parsePipeline(pipeline, writer, stage);

    return;
}


} // namespace pdal
