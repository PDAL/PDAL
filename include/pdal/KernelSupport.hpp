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

#include <string>

#include <pdal/Options.hpp>
#include <pdal/Stage.hpp>
#include <pdal/PipelineManager.hpp>

namespace pdal
{

class app_usage_error : public pdal::pdal_error
{
public:
    inline app_usage_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


class app_runtime_error : public pdal::pdal_error
{
public:
    inline app_runtime_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


// this is a static class with some helper functions the cmd line apps need
class PDAL_DLL KernelSupport
{
public:
    /**
      Make a pipeline given a filename.

      \param filename  A input filespec from which the reader can be inferred
        or the name of a pipeline file itself.
      \param noPoints  When a single-reader pipeline is created, add an
        option to eliminate the reading of points.
    */
    static PipelineManagerPtr makePipeline(const std::string& filename,
        bool noPoints);

private:
    KernelSupport& operator=(const KernelSupport&); // not implemented
    KernelSupport(const KernelSupport&); // not implemented
};

} // namespace pdal
