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
#include <pdal/Reader.hpp>
#include <pdal/Writer.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/PDALUtils.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <pdal/FileUtils.hpp>

using namespace boost::property_tree;

namespace pdal
{

static ptree generateTreeFromStage(const Stage& stage)
{
    ptree tree;
    ptree& attrtree = tree.add_child("Pipeline", stage.serializePipeline());
    attrtree.put("<xmlattr>.version", "1.0");
    return tree;
}


void PipelineWriter::write_option_ptree(ptree& tree, const Options& opts)
{
    ptree m_tree = pdal::utils::toPTree(opts);

    auto iter = m_tree.begin();
    while (iter != m_tree.end())
    {
        if (iter->first != "Option")
            throw pdal_error("malformed Options ptree");
        const ptree& optionTree = iter->second;

        // we want to create this:
        //      ...
        //      <Option name="file">foo.las</Option>
        //      ...

        const std::string& name =
            optionTree.get_child("Name").get_value<std::string>();
        const std::string& value =
            optionTree.get_child("Value").get_value<std::string>();

        ptree& subtree = tree.add("Option", value);
        subtree.put("<xmlattr>.name", name);

        boost::optional<ptree const&> moreOptions =
            optionTree.get_child_optional("Options");

        if (moreOptions)
        {
            ptree newOpts;
            write_option_ptree(newOpts, moreOptions.get());
            subtree.put_child("Options", newOpts);
        }
        ++iter;
    }
}


ptree PipelineWriter::getMetadataEntry(const MetadataNode& input)
{
    ptree entry;

    entry.put_value(input.value());
    entry.put("<xmlattr>.name", input.name());
    entry.put("<xmlattr>.type", input.type());

    std::vector<MetadataNode> children = input.children();
    for (auto ci = children.begin(); ci != children.end(); ++ci)
        entry.add_child("Metadata", getMetadataEntry(*ci));
    return entry;
}


void PipelineWriter::writeMetadata(boost::property_tree::ptree& tree,
    const MetadataNode& input)
{
    tree.add_child("Metadata", getMetadataEntry(input));
}


void PipelineWriter::writePipeline(const std::string& filename) const
{
    const Stage* stage = m_manager.isWriterPipeline() ?
        (Stage*)m_manager.getWriter() :
        (Stage*)m_manager.getStage();

    ptree tree = generateTreeFromStage(*stage);
    const xml_parser::xml_writer_settings<char> settings(' ', 4);

    if (boost::iequals(filename, "STDOUT"))
        xml_parser::write_xml(std::cout, tree);
    else
        xml_parser::write_xml(filename, tree, std::locale(), settings);
}

} // namespace pdal

