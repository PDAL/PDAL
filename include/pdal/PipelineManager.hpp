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
    PipelineManager()
        {}

    ~PipelineManager();

    // Use these to manually add stages into the pipeline manager.
    Stage* addReader(const std::string& type);
    Stage* addFilter(const std::string& type, Stage *prevStage);
    Stage* addFilter(const std::string& type,
        const std::vector<Stage *>& prevStages);
    Stage* addWriter(const std::string& type, Stage *prevStage);

    // returns true if the pipeline endpoint is a writer
    bool isWriterPipeline() const
        { return (bool)getStage(); }

    // return the pipeline reader endpoint (or NULL, if not a reader pipeline)
    Stage* getStage() const
        { return m_stages.empty() ? NULL : m_stages.back(); }

    void prepare() const;
    point_count_t execute();

    // Get the resulting point buffers.
    const PointBufferSet& buffers() const
        { return m_pbSet; }

    // Get the point context;
    PointContextRef context() const
        { return m_context; }

    MetadataNode getMetadata() const;

private:
    StageFactory m_factory;
    PointContext m_context;
    PointBufferSet m_pbSet;

    typedef std::vector<Stage *> StagePtrList;
    StagePtrList m_stages;

    PipelineManager& operator=(const PipelineManager&); // not implemented
    PipelineManager(const PipelineManager&); // not implemented
};


} // namespace pdal

