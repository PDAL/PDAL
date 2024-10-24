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
#include <pdal/PipelineManager.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Stage.hpp>
#include <pdal/util/FileUtils.hpp>

#include "private/PointlessLas.hpp"

#ifdef __clang__
#pragma GCC diagnostic ignored "-Wtautological-constant-out-of-range-compare"
#endif

namespace pdal
{

class PDAL_EXPORT InfoKernel : public Kernel
{
public:
    std::string getName() const;
    int execute(); // overrride

    InfoKernel();
    void setup(const std::string& filename);
    MetadataNode run(const std::string& filename);

    inline bool showAll() { return m_showAll; }
    inline void doShowAll(bool value) { m_showAll = value; }
    inline void doComputeSummary(bool value) { m_showSummary = value; }
    inline void doComputeBoundary(bool value) { m_boundary = value; }

private:
    void addSwitches(ProgramArgs& args);
    void validateSwitches(ProgramArgs& args);
    void makeReader(const std::string& filename);
    void makePipeline();
    void dump(MetadataNode& root);
    MetadataNode dumpSummary(const QuickInfo& qi);

    std::string m_inputFile;
    bool m_showStats;
    bool m_showSchema;
    bool m_showAll;
    bool m_showMetadata;
    bool m_boundary;
    bool m_stac;
    std::string m_breakoutDimension;
    std::string m_pointIndexes;
    std::string m_dimensions;
    std::string m_enumerate;
    std::string m_queryPoint;
    std::string m_pipelineFile;
    std::string m_pcType;
    bool m_showSummary;
    bool m_needPoints;
    bool m_usestdin;

    Stage *m_statsStage;
    Stage *m_expressionStatsStage;
    Stage *m_hexbinStage;
    Stage *m_infoStage;
    Stage *m_reader;
    Stage *m_stacStage;

    MetadataNode m_tree;
};

} // namespace pdal
