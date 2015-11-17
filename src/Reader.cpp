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

#include <pdal/Reader.hpp>
#include <pdal/PipelineWriter.hpp>

namespace pdal
{

void Reader::readerProcessOptions(const Options& options)
{
    if (options.hasOption("filename"))
        m_filename = options.getValueOrThrow<std::string>("filename");
    if (options.hasOption("count"))
        m_count = options.getValueOrThrow<point_count_t>("count");
}


boost::property_tree::ptree Reader::serializePipeline() const
{
    boost::property_tree::ptree tree;

    tree.add("<xmlattr>.type", getName());

    PipelineWriter::write_option_ptree(tree, getOptions());
    PipelineWriter::writeMetadata(tree, m_metadata);

    boost::property_tree::ptree root;
    root.add_child("Reader", tree);

    return root;
}


void Reader::read(
        const std::string path,
        std::function<void(BasePointTable&)> onInit,
        std::function<void(PointView&)> onData,
        const std::size_t chunkBytes,
        Options options)
{
    PointLayout layout;
    read(path, layout, onInit, onData, chunkBytes, options);
}

void Reader::read(
        const std::string path,
        PointLayout& layout,
        std::function<void(BasePointTable&)> onInit,
        std::function<void(PointView&)> onData,
        const std::size_t chunkBytes,
        Options options)
{
    options.add(Option("filename", path));
    setOptions(options);

    VectorPointTable table(layout);
    prepare(table);

    const std::size_t pointSize(layout.pointSize());
    std::size_t maxIndex(
            chunkBytes / pointSize + (chunkBytes % pointSize ? 1 : 0));

    table.reserve(maxIndex + 1);

    setReadCb(
            [&onData, &table, &maxIndex]
            (pdal::PointView& view, pdal::PointId index)
    {
        if (index >= maxIndex && table.numPoints() == index + 1)
        {
            onData(view);

            table.clear();
            view.clear();
        }
    });

    onInit(table);
    onData(**execute(table).begin());
}

} // namespace pdal

