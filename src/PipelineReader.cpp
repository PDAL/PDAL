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
#include <pdal/Options.hpp>
#include <pdal/FileUtils.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/trim.hpp>

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


PipelineReader::PipelineReader(PipelineManager& manager, bool isDebug, boost::uint32_t verboseLevel)
    : m_manager(manager)
    , m_isDebug(isDebug)
    , m_verboseLevel(verboseLevel)
{
    if (m_isDebug)
    {
        Option opt("debug", true);
        m_baseOptions.add(opt);
    }
    if (m_verboseLevel)
    {
        Option opt("verbose", m_verboseLevel);
        m_baseOptions.add(opt);
    }

    return;
}


PipelineReader::~PipelineReader()
{
    return;
}


Option PipelineReader::parseElement_Option(const boost::property_tree::ptree& tree)
{
    // cur is an option element, such as this:
    //     <option>
    //       <name>myname</name>
    //       <description>my descr</description>
    //       <value>17</value>
    //     </option>
    // this function will process the element and return an Option from it

    // boost::property_tree::xml_parser::write_xml(std::cout, tree);
    map_t attrs;
    collect_attributes(attrs, tree);

    std::string name = attrs["name"];
    std::string value = tree.get_value<std::string>();
    boost::algorithm::trim(value);
    Option option(name, value);

    using namespace boost::property_tree;
    using namespace boost;


    optional<ptree const&> moreOptions = tree.get_child_optional("Options");

    if (moreOptions)
    {
        ptree::const_iterator iter = moreOptions->begin();

        Options options;
        while (iter != moreOptions->end())
        {

            if (iter->first == "Option")
            {
                Option o2 = parseElement_Option(iter->second);
                options.add(o2);
            }

            ++iter;
        }
        option.setOptions(options);
    }


    // filenames in the XML are fixed up as follows:
    //   - if absolute path, leave it alone
    //   - if relative path, make it absolute using the XML file's directory
    // The toAbsolutePath function does exactly that magic for us.
    if (option.getName() == "filename")
    {
        const std::string oldpath = option.getValue<std::string>();
        if (!FileUtils::isAbsolutePath(oldpath))
        {
            const std::string abspath = FileUtils::toAbsolutePath(m_inputXmlFile);
            const std::string absdir = FileUtils::getDirectory(abspath);
            const std::string newpath = FileUtils::toAbsolutePath(oldpath, absdir);
            assert(FileUtils::isAbsolutePath(newpath));
            option.setValue(newpath);
        }
    }

    return option;
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
    else if (name == "<xmlattr>")
    {
        // ignore: will parse later
    }
    else
    {
        throw pipeline_xml_error("encountered unknown stage type");
    }

    return stage;
}


Reader* PipelineReader::parseElement_Reader(const boost::property_tree::ptree& tree)
{
    Options options(m_baseOptions);

    StageParserContext context(StageParserContext::None);

    map_t attrs;
    collect_attributes(attrs, tree);

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const boost::property_tree::ptree& subtree = iter->second;

        if (name == "<xmlattr>")
        {
            // already parsed
        }
        else if (name == "Option")
        {
            Option option = parseElement_Option(subtree);
            options.add(option);
        }
        else if (name == "Metadata")
        {
            // FIXME ignored for now
        }
        else
        {
            context.addUnknown(name);
        }
        ++iter;
    }

    std::string type;
    if (attrs.count("type"))
    {
        type = attrs["type"];
        context.addType();
    }

    context.validate();

    Reader* ptr = m_manager.addReader(type, options);

    return ptr;
}


Filter* PipelineReader::parseElement_Filter(const boost::property_tree::ptree& tree)
{
    Options options(m_baseOptions);
    Stage* prevStage = NULL;

    StageParserContext context(StageParserContext::One);

    map_t attrs;
    collect_attributes(attrs, tree);

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const boost::property_tree::ptree& subtree = iter->second;

        if (name == "<xmlattr>")
        {
            // already parsed
        }
        else if (name == "Option")
        {
            Option option = parseElement_Option(subtree);
            options.add(option);
        }
        else if (name == "Metadata")
        {
            // FIXME: ignored
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

    std::string type;
    if (attrs.count("type"))
    {
        type = attrs["type"];
        context.addType();
    }

    context.validate();

    Filter* ptr = m_manager.addFilter(type, *prevStage, options);

    return ptr;
}


MultiFilter* PipelineReader::parseElement_MultiFilter(const boost::property_tree::ptree& tree)
{
    Options options(m_baseOptions);
    std::vector<Stage*> prevStages;
    StageParserContext context(StageParserContext::Many);

    map_t attrs;
    collect_attributes(attrs, tree);

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const boost::property_tree::ptree& subtree = iter->second;

        if (name == "<xmlattr>")
        {
            // already parsed
        }
        else if (name == "Option")
        {
            Option option = parseElement_Option(subtree);
            options.add(option);
        }
        else if (name == "Metadata")
        {
            // FIXME: ignored
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

    std::string type;
    if (attrs.count("type"))
    {
        type = attrs["type"];
        context.addType();
    }

    context.validate();

    MultiFilter* ptr = m_manager.addMultiFilter(type, prevStages, options);

    return ptr;
}


void PipelineReader::parse_attributes(map_t& attrs, const boost::property_tree::ptree& tree)
{
    for (boost::property_tree::ptree::const_iterator iter = tree.begin();
            iter != tree.end();
            ++iter)
    {
        std::string name = iter->first;
        std::string value = tree.get<std::string>(name);
        boost::algorithm::trim(value);

        attrs[name] = value;
    }

    return;
}


void PipelineReader::collect_attributes(map_t& attrs, const boost::property_tree::ptree& tree)
{
    if (tree.count("<xmlattr>"))
    {
        const boost::property_tree::ptree& subtree = tree.get_child("<xmlattr>");
        parse_attributes(attrs, subtree);
    }

    return;
}


Writer* PipelineReader::parseElement_Writer(const boost::property_tree::ptree& tree)
{
    Options options(m_baseOptions);
    Stage* prevStage = NULL;
    StageParserContext context(StageParserContext::One);

    map_t attrs;
    collect_attributes(attrs, tree);

    boost::property_tree::ptree::const_iterator iter = tree.begin();
    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const boost::property_tree::ptree& subtree = iter->second;

        if (name == "<xmlattr>")
        {
            // already parsed -- ignore it
        }
        else if (name == "Option")
        {
            Option option = parseElement_Option(subtree);
            options.add(option);
        }
        else if (name == "Metadata")
        {
            // FIXME
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

    std::string type;
    if (attrs.count("type"))
    {
        type = attrs["type"];
        context.addType();
    }

    context.validate();

    Writer* ptr = m_manager.addWriter(type, *prevStage, options);

    return ptr;
}


bool PipelineReader::parseElement_Pipeline(const boost::property_tree::ptree& tree)
{
    Stage* stage = NULL;
    Writer* writer = NULL;

    map_t attrs;
    collect_attributes(attrs, tree);

    std::string version = "";
    if (attrs.count("version"))
    {
        version = attrs["version"];
    }
    if (version != "1.0")
    {
        throw pipeline_xml_error("unsupported pipeline xml version");
    }

    boost::property_tree::ptree::const_iterator iter = tree.begin();

    bool isWriter = false;

    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const boost::property_tree::ptree subtree = iter->second;

        if (name == "Reader" || name == "Filter" || name == "MultiFilter")
        {
            stage = parseElement_anystage(name, subtree);
        }
        else if (name == "Writer")
        {
            writer = parseElement_Writer(subtree);
            isWriter = true;
        }
        else if (name == "<xmlattr>")
        {
            // ignore it, already parsed
        }
        else
        {
            throw pipeline_xml_error("xml reader invalid child of ReaderPipeline element");
        }

        ++iter;
    }

    if (writer && stage)
    {
        throw pipeline_xml_error("extra nodes at front of writer pipeline");
    }

    return isWriter;
}

bool PipelineReader::readPipeline(std::istream& input)
{

    boost::property_tree::ptree tree;
    boost::property_tree::xml_parser::read_xml(input, tree,
            boost::property_tree::xml_parser::no_comments);

    boost::optional<boost::property_tree::ptree> opt(tree.get_child_optional("Pipeline"));
    if (!opt.is_initialized())
    {
        throw pipeline_xml_error("root element is not Pipeline");
    }

    boost::property_tree::ptree subtree = opt.get();

    bool isWriter = parseElement_Pipeline(subtree);

    return isWriter;

}


bool PipelineReader::readPipeline(const std::string& filename)
{
    m_inputXmlFile = filename;

    std::istream* input = FileUtils::openFile(filename);

    bool isWriter = false;

    try
    {
        isWriter = readPipeline(*input);
    }
    catch (...)
    {
        FileUtils::closeFile(input);
        throw;
    }

    FileUtils::closeFile(input);

    m_inputXmlFile = "";

    return isWriter;
}


} // namespace pdal
