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

#pragma once

#include <pdal/pdal_internal.hpp>
#include <pdal/StageFactory.hpp>

#include <vector>
#include <string>

namespace pdal
{

class Options;

class PDAL_DLL PipelineManager
{
public:
    PipelineManager() : m_tablePtr(new PointTable()), m_table(*m_tablePtr),
            m_progressFd(-1), m_input(nullptr)
        {}
    PipelineManager(int progressFd) : m_tablePtr(new PointTable()),
            m_table(*m_tablePtr), m_progressFd(progressFd), m_input(nullptr)
        {}
    PipelineManager(PointTableRef table) : m_table(table), m_progressFd(-1),
            m_input(nullptr)
        {}
    PipelineManager(PointTableRef table, int progressFd) : m_table(table),
            m_progressFd(progressFd), m_input(nullptr)
        {}
    ~PipelineManager();

    void readPipeline(std::istream& input, bool debug=false,
                   uint32_t verbose = 0);
    void readPipeline(const std::string& filename, bool debug=false,
                   uint32_t verbose = 0);

    // Use these to manually add stages into the pipeline manager.
    Stage& addReader(const std::string& type);
    Stage& addFilter(const std::string& type);
    Stage& addWriter(const std::string& type);

    // returns true if the pipeline endpoint is a writer
    bool isWriterPipeline() const
        { return (bool)getStage(); }

    // return the pipeline reader endpoint (or nullptr, if not a reader
    // pipeline)
    Stage* getStage() const
        { return m_stages.empty() ? nullptr : m_stages.back(); }

    void prepare() const;
    point_count_t execute();

    // Get the resulting point views.
    const PointViewSet& views() const
        { return m_viewSet; }

    // Get the point table data.
    PointTableRef pointTable() const
        { return m_table; }

    MetadataNode getMetadata() const;

private:
    StageFactory m_factory;
    std::unique_ptr<PointTable> m_tablePtr;
    PointTableRef m_table;

    PointViewSet m_viewSet;

    std::vector<Stage*> m_stages; // stage observer, never owner
    int m_progressFd;
    std::istream *m_input;

    PipelineManager& operator=(const PipelineManager&); // not implemented
    PipelineManager(const PipelineManager&); // not implemented
};
typedef std::unique_ptr<PipelineManager> PipelineManagerPtr;

} // namespace pdal
