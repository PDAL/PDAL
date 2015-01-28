/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#pragma once

#include <pdal/Kernel.hpp>

#include <pdal/Stage.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>


#ifdef __clang__
#pragma GCC diagnostic ignored "-Wtautological-constant-out-of-range-compare"
#endif

#include <boost/property_tree/json_parser.hpp>
#include <boost/tokenizer.hpp>

#include <pdal/KernelSupport.hpp>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

namespace pdal
{

class PDAL_DLL InfoKernel : public Kernel
{
public:
    SET_KERNEL_NAME ("info", "Info Kernel")
    SET_KERNEL_LINK ("http://pdal.io/kernels/kernels.info.html")

    InfoKernel();
    int execute(); // overrride

private:
    void addSwitches(); // overrride
    void validateSwitches(); // overrride

    void dump(std::ostream& o);

    MetadataNode dumpPoints(PointBufferPtr buf) const;
    MetadataNode dumpStats() const;
    void dumpPipeline() const;
    MetadataNode dumpSummary(const QuickInfo& qi);
    MetadataNode dumpQuery(PointBufferPtr buf) const;

    std::string m_inputFile;
    bool m_showStats;
    bool m_showSchema;
    bool m_showMetadata;
    bool m_boundary;
    pdal::Options m_options;
    std::string m_pointIndexes;
    bool m_useJSON;
    std::string m_Dimensions;
    std::string m_QueryPoint;
    double m_QueryDistance;
    std::string m_pipelineFile;
    bool m_showSummary;

    Stage *m_statsStage;
    Stage *m_hexbinStage;
    Reader *m_reader;

    MetadataNode m_tree;
    std::unique_ptr<PipelineManager> m_manager;
};

} // namespace pdal
