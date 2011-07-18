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
#include <pdal/exceptions.hpp>

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/drivers/liblas/Reader.hpp>
#include <pdal/drivers/liblas/Writer.hpp>

#include <boost/shared_ptr.hpp>

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


static bool isElement(xmlNodePtr cur)
{
    return (cur->type == XML_ELEMENT_NODE);
}


static bool isText(xmlNodePtr cur)
{
    return (cur->type == XML_TEXT_NODE);
}


static bool isElement(xmlNodePtr cur, const char* str)
{
    if (!isElement(cur)) return false;
    return (xmlStrcmp(cur->name, (const xmlChar*)str)==0);
}


static bool isDataStageElement(xmlNodePtr cur)
{
    return (isElement(cur, "Reader") || isElement(cur, "Filter") || isElement(cur, "MultiFilter"));
}


static bool isStageElement(xmlNodePtr cur)
{
    return (isDataStageElement(cur) || isElement(cur, "Writer"));
}


static const char* fromXmlChar(const xmlChar* str)
{
    return (const char*)str;
}


static const xmlChar* toXmlChar(const char* str)
{
    return BAD_CAST(str);
}


void PipelineManager::parseOption(xmlDocPtr doc, xmlNodePtr cur, Options& options)
{
    // cur is an option element, such as this:
    //    <filename>foo.bar</filename>
    // we assume the body of the element is just text
    // this function will process the element and return an Option from it

    assert(isElement(cur));

    const char* name = fromXmlChar(cur->name);
    const char* text = NULL;

    cur = cur->children;

    while (cur)
    {
        if (isText(cur))
        {
            text = fromXmlChar(cur->content);
        }
        cur = cur->next;
    }

    Option<std::string> option(name, text, "");

    options.add(option);

    return;
}


Options PipelineManager::parseOptions(xmlDocPtr doc, xmlNodePtr cur)
{
    // cur is a Reader element
    // this function will process the children and return a Reader
    assert(isElement(cur, "Options"));

    cur = cur->children;

    Options options;

    while (cur)
    {
        if (isElement(cur))
        {
            parseOption(doc, cur, options);
        }
        cur = cur->next;
    }

    return options;
}


StagePtr PipelineManager::parseStage(xmlDocPtr doc, xmlNodePtr cur)
{
    // cur is a Reader, Filter, MultiFilter, or Writer element
    assert(isStageElement(cur));

    if (isElement(cur, "Reader"))
    {
        return parseReader(doc, cur);
    }
     
    if (isElement(cur, "Filter"))
    {
        return parseFilter(doc, cur);
    }

    if (isElement(cur, "MultiFilter"))
    {
        return parseMultiFilter(doc, cur);
    }

    if (isElement(cur, "Writer"))
    {
        return parseWriter(doc, cur);
    }

    throw pdal_error("xml reader expected Stage element");
}


DataStagePtr PipelineManager::parseDataStage(xmlDocPtr doc, xmlNodePtr cur)
{
    // cur is a Reader, Filter, MultiFilter, or Writer element
    assert(isDataStageElement(cur));

    if (isElement(cur, "Reader"))
    {
        return parseReader(doc, cur);
    }
     
    if (isElement(cur, "Filter"))
    {
        return parseFilter(doc, cur);
    }

    if (isElement(cur, "MultiFilter"))
    {
        return parseMultiFilter(doc, cur);
    }

    throw pdal_error("xml reader expected DataStage element");
}


ReaderPtr PipelineManager::parseReader(xmlDocPtr doc, xmlNodePtr cur)
{
    // cur is a Reader element
    // this function will process the children and return a Reader
    assert(isElement(cur, "Reader"));

    const char* type = fromXmlChar(xmlGetProp(cur, toXmlChar("type")));
    printf("type=%s\n", type);

    cur = cur->children;

    Options options;

    while (cur)
    {
        if (isElement(cur))
        {
            if (isElement(cur, "Options"))
            {
                options = parseOptions(doc, cur);
            }
            else
            {
                throw pdal_error("xml reader unknown element");
            }
        }
        cur = cur->next;
    }

    ReaderPtr ptr = addReader(type, options);

    return ptr;
}


FilterPtr PipelineManager::parseFilter(xmlDocPtr doc, xmlNodePtr cur)
{
    // cur is a Filter element
    // this function will process the children and return a Filter
    assert(isElement(cur, "Filter"));

    const char* type = fromXmlChar(xmlGetProp(cur, toXmlChar("type")));
    printf("type=%s\n", type);

    cur = cur->children;

    bool gotPrevStage = false;
    DataStagePtr prevStage;
    Options options;

    while (cur)
    {
        if (isElement(cur))
        {
            if (isElement(cur, "Options"))
            {
                options = parseOptions(doc, cur);
            }
            else if (isDataStageElement(cur))
            {
                if (gotPrevStage)
                {
                    throw pdal_error("xml reader found extra stage element");
                }
                prevStage = parseDataStage(doc, cur);
                gotPrevStage = true;
            }
            else
            {
                throw pdal_error("xml reader unknown element");
            }
        }
        cur = cur->next;
    }

    if (!gotPrevStage)
    {
        throw pdal_error("xml reader did not find child stage element");
    }

    FilterPtr ptr = addFilter(type, prevStage, options);

    return ptr;
}


MultiFilterPtr PipelineManager::parseMultiFilter(xmlDocPtr doc, xmlNodePtr cur)
{
    // cur is a Filter element
    // this function will process the children and return a Filter
    assert(isElement(cur, "MultiFilter"));

    const char* type = fromXmlChar(xmlGetProp(cur, toXmlChar("type")));
    printf("type=%s\n", type);

    cur = cur->children;

    std::vector<const DataStagePtr> prevStages;
    Options options;

    while (cur)
    {
        if (isElement(cur))
        {
            if (isElement(cur, "Options"))
            {
                options = parseOptions(doc, cur);
            }
            else if (isDataStageElement(cur))
            {
                DataStagePtr prevStage = parseDataStage(doc, cur);
                prevStages.push_back(prevStage);
            }
            else
            {
                throw pdal_error("xml reader unknown element");
            }
        }
        cur = cur->next;
    }

    if (prevStages.size() == 0)
    {
        throw pdal_error("xml reader did not find child stage element");
    }

    MultiFilterPtr ptr = addMultiFilter(type, prevStages, options);

    return ptr;
}


WriterPtr PipelineManager::parseWriter(xmlDocPtr doc, xmlNodePtr cur)
{
    // cur is a Writer element
    // this function will process the children and return a Writer
    assert(isElement(cur, "Writer"));

    const char* type = fromXmlChar(xmlGetProp(cur, toXmlChar("type")));
    printf("type=%s\n", type);

    cur = cur->children;

    bool gotPrevStage = false;
    DataStagePtr prevStage;
    Options options;

    while (cur)
    {
        if (isElement(cur))
        {
            if (isElement(cur, "Options"))
            {
                options = parseOptions(doc, cur);
            }
            else if (isDataStageElement(cur))
            {
                if (gotPrevStage)
                {
                    throw pdal_error("xml reader found extra stage element");
                }
                prevStage = parseDataStage(doc, cur);
                gotPrevStage = true;
            }
            else
            {
                throw pdal_error("xml reader unknown element");
            }
        }
        cur = cur->next;
    }

    if (!gotPrevStage)
    {
        throw pdal_error("xml reader did not find child stage element");
    }

    WriterPtr ptr = addWriter(type, prevStage, options);

    return ptr;
}


StagePtr PipelineManager::parsePipeline(xmlDocPtr doc, xmlNodePtr cur)
{
    assert(isElement(cur, "Pipeline"));

    cur = cur->children;

    StagePtr stage;

    while (cur)
    {
        if (isElement(cur))
        {
            if (isElement(cur, "Reader"))
            {
                stage = parseReader(doc, cur);
            }
            else if (isElement(cur, "Filter"))
            {
                stage = parseFilter(doc, cur);
            }
            else if (isElement(cur, "MultiFilter"))
            {
                stage = parseMultiFilter(doc, cur);
            }
            else if (isElement(cur, "Writer"))
            {
                stage = parseWriter(doc, cur);
            }
            else
            {
                throw pdal_error("xml reader invalid child of root element");
            }
        }
        cur = cur->next;
    }

    return stage;
}


void PipelineManager::readXml(const std::string& filename)
{
    xmlDocPtr doc;
    xmlNodePtr cur;

    doc = xmlParseFile(filename.c_str());
    if (!doc)
    {
        throw pdal_error("xml reader unable to parse");
    }

    cur = xmlDocGetRootElement(doc);
    if (!cur)
    {
        xmlFreeDoc(doc);
        throw pdal_error("xml reader empty document");
    }

    if (!isElement(cur, "Pipeline"))
    {
        xmlFreeDoc(doc);
        throw pdal_error("xml reader invalid root element");
    }

    parsePipeline(doc, cur);

    return;
}





} // namespace pdal
