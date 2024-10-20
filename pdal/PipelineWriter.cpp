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

#include <pdal/Metadata.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/Stage.hpp>

namespace pdal
{

namespace
{

std::string generateTag(Stage *stage, PipelineWriter::TagMap& tags)
{
    auto tagExists = [tags](const std::string& tag)
    {
        for (auto& t : tags)
        {
            if (t.second == tag)
                return true;
        }
        return false;
    };

    std::string tag = stage->tag();
    if (tag.empty())
    {
        for (size_t i = 1; ; ++i)
        {
            tag = stage->getName() + std::to_string(i);
            tag = Utils::replaceAll(tag, ".", "_");
            if (!tagExists(tag))
                break;
        }
    }
    return tag;
}

void generateTags(Stage *stage, PipelineWriter::TagMap& tags)
{
    for (Stage *s : stage->getInputs())
        generateTags(s, tags);
    tags[stage] = generateTag(stage, tags);
}

} // anonymous namespace

namespace PipelineWriter
{

PDAL_EXPORT void writePipeline(Stage *stage, const std::string& filename)
{
    std::ostream *out = Utils::createFile(filename, false);
    writePipeline(stage, *out);
    Utils::closeFile(out);
}

PDAL_EXPORT void writePipeline(Stage *stage, std::ostream& strm)
{
    TagMap tags;
    generateTags(stage, tags);

    MetadataNode root;
    stage->serialize(root, tags);
    Utils::toJSON(root, strm);
}

} // namespace PipelineWriter

} // namespace pdal

