/******************************************************************************
* Copyright (c) 2016, Howard Butler (howard@hobu.co)
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

#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/pdal_export.hpp>

#include <string>

namespace pdal
{

/**
  An executor hides the management of constructing, executing, and
  fetching data from a PipelineManager.

  It is constructed with JSON defining a pipeline.
*/


/* Don't use this anymore. Use PipelineManager directly yourself */
class PDAL_EXPORT_DEPRECATED PipelineExecutor {
public:

    /**
      Construct a PipelineExecutor

      \param json Pipeline JSON defining the PDAL operations
    */
    PipelineExecutor(std::string const& json);

    /**
      dtor
    */
    ~PipelineExecutor(){};

    /**
      Execute the pipeline

      \return total number of points produced by the pipeline.
    */
    int64_t execute();

    /**
      Validate the pipeline

      \return does PDAL think the pipeline is valid?
    */
    bool validate();

    /**
      \return the transliterated pipeline
    */
    std::string getPipeline() const;

    /**
      \return computed metadata for the pipeline and all stages
    */
    std::string getMetadata() const;

    /**
      \return computed schema for the pipeline
    */
    std::string getSchema() const;

    /**
      \return log output for the executed pipeline. use
      setLogLevel to adjust verbosity.
    */
    std::string getLog() const;

    /**
      set the log verbosity. Use values 0-8.
    */
    void setLogLevel(int level);

    /**
      \return log verbosity
    */
    int getLogLevel() const;

    /**
      \return has the pipeline been executed
    */
    inline bool executed() const
    {
        return m_executed;
    }

    /**
      \return Whether the pipeline has been read (prepare or execute has been called)
    */
    bool pipelineRead() const;

    /**
      \return a const reference to the pipeline manager
    */
    PipelineManager const& getManagerConst() const { return m_manager; }

    /**
      \return a reference to the pipeline manager
    */
    PipelineManager & getManager() { return m_manager; }

private:
    void readPipeline();
    void setLogStream(std::ostream& strm);

    std::string m_json;
    pdal::PipelineManager m_manager;
    bool m_executed;
    std::stringstream m_logStream;
    pdal::LogLevel m_logLevel;
    bool m_pipelineRead;
};

}
