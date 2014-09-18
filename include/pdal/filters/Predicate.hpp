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
#ifdef PDAL_HAVE_PYTHON

#include <pdal/Filter.hpp>

#include <pdal/plang/BufferedInvocation.hpp>


namespace pdal
{
namespace filters
{

class PDAL_DLL Predicate : public Filter
{
public:
    SET_STAGE_NAME("filters.predicate", "Predicate Filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.predicate.html")  
#ifdef PDAL_HAVE_PYTHON
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif
    
    Predicate(const Options& options) : Filter(options), m_script(NULL)
        {}

    static Options getDefaultOptions();

private:
    plang::BufferedInvocation* m_pythonMethod;
    plang::Script* m_script;
    std::string m_source;
    std::string m_module;
    std::string m_function;

    virtual void processOptions(const Options& options);
    virtual void ready(PointContext ctx);
    virtual PointBufferSet run(PointBufferPtr buf);
    virtual void done(PointContext ctx);

    Predicate& operator=(const Predicate&); // not implemented
    Predicate(const Predicate&); // not implemented
};

} // namespace filters
} // namespace pdal

#endif // PDAL_HAVE_PYTHON
