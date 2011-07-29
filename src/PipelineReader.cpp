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

#include <pdal/PipelineReader.hpp>

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
#include <boost/optional.hpp>

namespace pdal
{
    
// ------------------------------------------------------------------------

// this class helps keep tracks of what child nodes we've seen, so we
// can keep all the error checking in one place
class PipelineReader::StageParserContext
{
public:
    enum Cardinality { None, One, Many };

    StageParserContext(Cardinality cardinality)
        : m_numTypes(0)
        , m_cardinality(cardinality)
        , m_numStages(0)
    {
        return;
    }

    void addType()
    {
        ++m_numTypes;
    }

    void addStage()
    {
        ++m_numStages;
    }
    
    void addUnknown(const std::string& name)
    {
        throw pipeline_xml_error("unknown child of element: " + name);
    }

    void validate()
    {
        if (m_numTypes == 0)
            throw pipeline_xml_error("expected Type element missing");
        if (m_numTypes > 1)
            throw pipeline_xml_error("extra Type element found");

        if (m_cardinality == None)
        {
            if (m_numStages != 0)
                throw pipeline_xml_error("found child stages where none expected");
        }
        if (m_cardinality == One)
        {
            if (m_numStages == 0)
                throw pipeline_xml_error("expected child stage missing");
            if (m_numStages > 1)
                throw pipeline_xml_error("extra child stages found");
        }
        if (m_cardinality == Many)
        {
            if (m_numStages == 0)
                throw pipeline_xml_error("expected child stage missing");
        }
    }

private:
    int m_numTypes;
    Cardinality m_cardinality; // num child stages allowed
    int m_numStages;
};


// ------------------------------------------------------------------------


PipelineReader::PipelineReader(PipelineManager& manager)
    : m_manager(manager)
{

    return;
}


PipelineReader::~PipelineReader()
{
    return;
}


Option<std::string> PipelineReader::parseElement_Option(const boost::property_tree::ptree& tree)
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


std::string PipelineReader::parseElement_Type(const boost::property_tree::ptree& tree)
{
    // tree is this:
    //     <type>drivers.foo.writer</type>

    std::string s = tree.get_value("Type");

    return s;
}


Stage* PipelineReader::parseElement_anystage(const std::string& name, const boost::property_tree::ptree& subtree)
{
    Stage* stage = NULL;

    if (name == "Filter")
    {
        stage = parseElement_Filter(subtree);
    }
    else if (name == "MultiFilter")
    {
        stage = parseElement_MultiFilter(subtree);
    }
    else if (name == "Reader")
    {
        stage = parseElement_Reader(subtree);
    }
    else
    {
        throw pipeline_xml_error("encountered unknown stage type");
    }

    return stage;
}


Reader* PipelineReader::parseElement_Reader(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type;
    
    StageParserContext context(StageParserContext::None);

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const boost::property_tree::ptree& subtree = iter->second;

        if (name == "Type")
        {
            context.addType();
            type = parseElement_Type(subtree);
        }
        else if (name == "Option")
        {
            Option<std::string> option = parseElement_Option(subtree);
            options.add(option);
        }
        else
        {
            context.addUnknown(name);
        }
        ++iter;
    }

    context.validate();

    Reader* ptr = m_manager.addReader(type, options);

    return ptr;
}


Filter* PipelineReader::parseElement_Filter(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type = "";
    Stage* prevStage = NULL;

    StageParserContext context(StageParserContext::One);

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const boost::property_tree::ptree& subtree = iter->second;

        if (name == "Type")
        {
            context.addType();
            type = parseElement_Type(subtree);
        }
        else if (name == "Option")
        {
            Option<std::string> option = parseElement_Option(subtree);
            options.add(option);
        }
        else if (name == "Filter" || name == "MultiFilter" || name == "Reader")
        {
            context.addStage();
            prevStage = parseElement_anystage(name, subtree);
        }
        else
        {
            context.addUnknown(name);
        }
        ++iter;
    }

    context.validate();

    Filter* ptr = m_manager.addFilter(type, *prevStage, options);

    return ptr;
}


MultiFilter* PipelineReader::parseElement_MultiFilter(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type = "";
    std::vector<const Stage*> prevStages;
    StageParserContext context(StageParserContext::Many);

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const boost::property_tree::ptree& subtree = iter->second;

        if (name == "Type")
        {
            context.addType();
            type = parseElement_Type(subtree);
        }
        else if (name == "Option")
        {
            Option<std::string> option = parseElement_Option(subtree);
            options.add(option);
        }
        else if (name == "Filter" || name == "MultiFilter" || name == "Reader")
        {
            context.addStage();
            Stage* prevStage = parseElement_anystage(name, subtree);
            prevStages.push_back(prevStage);
        }
        else
        {
            context.addUnknown(name);
        }
        ++iter;
    }

    context.validate();

    MultiFilter* ptr = m_manager.addMultiFilter(type, prevStages, options);

    return ptr;
}


Writer* PipelineReader::parseElement_Writer(const boost::property_tree::ptree& tree)
{
    Options options;
    std::string type = "";
    Stage* prevStage = NULL;
    StageParserContext context(StageParserContext::One);

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const boost::property_tree::ptree& subtree = iter->second;

        if (name == "Type")
        {
            context.addType();
            type = parseElement_Type(subtree);
        }
        else if (name == "Option")
        {
            Option<std::string> option = parseElement_Option(subtree);
            options.add(option);
        }
        else if (name == "Filter" || name == "MultiFilter" || name == "Reader")
        {
            context.addStage();
            prevStage = parseElement_anystage(name, subtree);
        }
        else
        {
            context.addUnknown(name);
        }
        ++iter;
    }

    context.validate();

    Writer* ptr = m_manager.addWriter(type, *prevStage, options);

    return ptr;
}


Writer* PipelineReader::parseElement_WriterPipeline(const boost::property_tree::ptree& tree)
{
    Writer* writer = NULL;

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    
    {
        if (iter->first == "Writer")
        {
            const boost::property_tree::ptree subtree = iter->second;
            writer = parseElement_Writer(subtree);
        }
        else
        {
            throw pipeline_xml_error("xml reader invalid child of Pipeline element");
        }
    }
    
    ++iter;
    if (iter != tree.end())
    {
        throw pipeline_xml_error("extra nodes at front of writer pipeline");
    }

    return writer;
}


Stage* PipelineReader::parseElement_ReaderPipeline(const boost::property_tree::ptree& tree)
{
    Stage* stage = NULL;

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    const std::string& name = iter->first;
    const boost::property_tree::ptree subtree = iter->second;

    {
        if (name == "Reader" || name == "Filter" || name == "MultiFilter")
        {
            stage = parseElement_anystage(name, subtree);
        }
        else
        {
            throw pipeline_xml_error("xml reader invalid child of ReaderPipeline element");
        }
    }
    
    ++iter;
    if (iter != tree.end())
    {
        throw pipeline_xml_error("extra nodes at front of writer pipeline");
    }

    return stage;
}


void PipelineReader::readWriterPipeline(const std::string& filename)
{
    boost::property_tree::ptree tree;
    boost::property_tree::xml_parser::read_xml(filename, tree);

    boost::optional<boost::property_tree::ptree> opt( tree.get_child_optional("WriterPipeline") );
    if (!opt.is_initialized())
    {
        throw pipeline_xml_error("root element is not WriterPipeline");
    }

    boost::property_tree::ptree subtree = opt.get();

    (void)parseElement_WriterPipeline(subtree);

    return;
}


void PipelineReader::readReaderPipeline(const std::string& filename)
{
    boost::property_tree::ptree tree;
    boost::property_tree::xml_parser::read_xml(filename, tree);

    boost::optional<boost::property_tree::ptree> opt( tree.get_child_optional("ReaderPipeline") );
    if (!opt.is_initialized())
    {
        throw pipeline_xml_error("root element is not ReaderPipeline");
    }

    boost::property_tree::ptree subtree = opt.get();

    (void)parseElement_ReaderPipeline(subtree);

    return;
}




} // namespace pdal
