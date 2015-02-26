/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include <istream>
#include <string>
#include <vector>

#include <pdal/Filter.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Options.hpp>

#include <pdal/PipelineManager.hpp>


namespace pdal
{


class PipelineParser
{
public:

    PipelineParser(PipelineManager& manager, const Options& baseOptions)
        : m_manager(manager)
        , m_baseOptions(baseOptions)
    {}

    virtual bool parse(std::istream& stream) = 0;
    virtual bool parse(const std::string& filename) = 0;

    std::string getInputFile() const { return m_inputFile; }
    void setInputFile(const std::string& filename) { m_inputFile = filename; }
    Options getBaseOptions() const { return m_baseOptions; }

    Reader* addReader(const std::string& type)
    {
        return m_manager.addReader(type);
    }

    Filter* addFilter(const std::string& type, Stage* prevStage)
    {
        return m_manager.addFilter(type, prevStage);
    }

    Filter* addFilter(const std::string& type,
            const std::vector<Stage*>& prevStages)
    {
        return m_manager.addFilter(type, prevStages);
    }

    Writer* addWriter(const std::string& type, Stage* prevStage)
    {
        return m_manager.addWriter(type, prevStage);
    }

private:

    std::string m_inputFile;
    PipelineManager& m_manager;
    Options m_baseOptions;

};


}
