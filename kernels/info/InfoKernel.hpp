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
#include <pdal/KernelSupport.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Stage.hpp>
#include <pdal/util/FileUtils.hpp>

#ifdef __clang__
#pragma GCC diagnostic ignored "-Wtautological-constant-out-of-range-compare"
#endif

#include <boost/property_tree/json_parser.hpp>
#include <boost/tokenizer.hpp>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

extern "C" int32_t InfoKernel_ExitFunc();
extern "C" PF_ExitFunc InfoKernel_InitPlugin();

namespace pdal
{

class PDAL_DLL InfoKernel : public Kernel
{
public:
    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;
    int execute(); // overrride


    void setup(const std::string& filename);
    MetadataNode run(const std::string& filename);

    inline bool showAll() { return m_showAll; }
    inline void doShowAll(bool value) { m_showAll = value; }
    inline void doComputeSummary(bool value) { m_showSummary = value; }
    inline void doComputeBoundary(bool value) { m_boundary = value; }

private:
    InfoKernel();
    void addSwitches(); // overrride
    void validateSwitches(); // overrride

    void dump(MetadataNode& root);
    MetadataNode dumpPoints(PointViewPtr inView) const;
    MetadataNode dumpStats() const;
    void dumpPipeline() const;
    MetadataNode dumpSummary(const QuickInfo& qi);
    MetadataNode dumpQuery(PointViewPtr inView) const;

    std::string m_inputFile;
    bool m_showStats;
    bool m_showSchema;
    bool m_showAll;
    bool m_showMetadata;
    bool m_boundary;
    pdal::Options m_options;
    std::string m_pointIndexes;
    std::string m_dimensions;
    std::string m_queryPoint;
    std::string m_pipelineFile;
    bool m_showSummary;
    bool m_needPoints;
    std::string m_PointCloudSchemaOutput;

    Stage *m_statsStage;
    Stage *m_hexbinStage;
    Stage *m_reader;

    MetadataNode m_tree;
    std::unique_ptr<PipelineManager> m_manager;
};

} // namespace pdal
