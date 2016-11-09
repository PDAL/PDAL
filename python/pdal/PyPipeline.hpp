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
#include <pdal/plang/Array.hpp>
#include <pdal/PipelineExecutor.hpp>

#include <string>
#include <sstream>
#undef toupper
#undef tolower
#undef isspace

#define PY_ARRAY_UNIQUE_SYMBOL LIBPDALPYTHON_ARRAY_API
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <numpy/arrayobject.h>


namespace libpdalpython
{

class python_error : public std::runtime_error
{
public:
    inline python_error(std::string const& msg) : std::runtime_error(msg)
        {}
};

    typedef pdal::plang::Array* PArray;

class Pipeline {
public:
    Pipeline(std::string const& xml);
    ~Pipeline();

    int64_t execute();
    bool validate();
    inline std::string getPipeline() const
    {
        return m_executor.getPipeline();
    }
    inline std::string getMetadata() const
    {
        return m_executor.getMetadata();
    }
    inline std::string getSchema() const
    {
        return m_executor.getSchema();
    }
    inline std::string getLog() const
    {
        return m_executor.getLog();
    }
    std::vector<PArray> getArrays() const;


    void setLogLevel(int level);
    int getLogLevel() const;

private:

    pdal::PipelineExecutor m_executor;

};

}
