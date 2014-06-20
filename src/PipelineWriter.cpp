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

#include <pdal/PipelineWriter.hpp>

#include <pdal/Filter.hpp>
#include <pdal/MultiFilter.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Writer.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointBuffer.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <pdal/FileUtils.hpp>

namespace pdal
{

PipelineWriter::PipelineWriter(const PipelineManager& manager)
    : m_manager(manager)
    , m_buffer(0)
{

    return;
}


PipelineWriter::~PipelineWriter()
{
    return;
}


static boost::property_tree::ptree generateTreeFromStage(const Stage& stage)
{
    boost::property_tree::ptree subtree = stage.serializePipeline();

    boost::property_tree::ptree tree;

    boost::property_tree::ptree& attrtree = tree.add_child("Pipeline", subtree);

    attrtree.put("<xmlattr>.version", "1.0");

    return tree;
}


void PipelineWriter::write_option_ptree(boost::property_tree::ptree& tree, const Options& opts)
{
    boost::property_tree::ptree m_tree = opts.toPTree();

    boost::property_tree::ptree::const_iterator iter = m_tree.begin();

    while (iter != m_tree.end())
    {
        if (iter->first != "Option")
            throw pdal_error("malformed Options ptree");
        const boost::property_tree::ptree& optionTree = iter->second;

        // we want to create this:
        //      ...
        //      <Option name="file">foo.las</Option>
        //      ...

        const std::string& name = optionTree.get_child("Name").get_value<std::string>();
        const std::string& value = optionTree.get_child("Value").get_value<std::string>();

        boost::property_tree::ptree& subtree = tree.add("Option", value);
        subtree.put("<xmlattr>.name", name);

        using namespace boost::property_tree;
        using namespace boost;

        optional<ptree const&> moreOptions = optionTree.get_child_optional("Options");

        if (moreOptions)
        {
            ptree newOpts;
            write_option_ptree(newOpts, moreOptions.get());
            subtree.put_child("Options", newOpts);
        }
        ++iter;
    }

    return;
}

boost::property_tree::ptree PipelineWriter::get_metadata_entry(boost::property_tree::ptree const& input)
{
    using namespace boost;

    property_tree::ptree entry;

    // std::ostream* ofile = FileUtils::createFile("metadata.xml");
    // boost::property_tree::write_xml(*ofile, input);
    // FileUtils::closeFile(ofile);
    const std::string& name = input.get_child("name").get_value<std::string>();
    const std::string& value = input.get_child("value").get_value<std::string>();
    const std::string& tname = input.get_child("type").get_value<std::string>();

    entry.put_value(value);
    entry.put("<xmlattr>.name", name);
    entry.put("<xmlattr>.type", tname);

    boost::optional<property_tree::ptree const&> entries = input.get_child_optional("metadata");
    if (entries)
    {
        property_tree::ptree::const_iterator iter = entries->begin();
        while (iter != entries->end())
        {

            property_tree::ptree e = get_metadata_entry(iter->second);
            entry.add_child("Metadata", e);

            ++iter;
        }

    }
    return entry;


}

void PipelineWriter::write_metadata_ptree(boost::property_tree::ptree& tree, const Metadata& m)
{
    using namespace boost;

    property_tree::ptree t = m.toPTree();

    property_tree::ptree output = get_metadata_entry(t);

    tree.add_child("Metadata", output);

    return;
}


void PipelineWriter::writePipeline(const std::string& filename) const
{
    const Stage* stage = m_manager.isWriterPipeline() ? (Stage*)m_manager.getWriter() : (Stage*)m_manager.getStage();

    boost::property_tree::ptree tree = generateTreeFromStage(*stage);

    if (m_buffer)
    {
        boost::property_tree::ptree metadata_tree;

        write_metadata_ptree(metadata_tree, m_buffer->getMetadata());
        boost::property_tree::ptree & child = tree.get_child("Pipeline");
        child.add_child("PointBuffer", metadata_tree);
    }

    const boost::property_tree::xml_parser::xml_writer_settings<char> settings(' ', 4);

    if (boost::iequals(filename, "STDOUT"))
        boost::property_tree::xml_parser::write_xml(std::cout, tree);
    else
        boost::property_tree::xml_parser::write_xml(filename, tree, std::locale(), settings);

    return;
}


} // namespace pdal
