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

#include "PyPipeline.hpp"
#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

#ifndef _WIN32
#include <dlfcn.h>
#endif

#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include "PyArray.hpp"

namespace libpdalpython
{

using namespace pdal::python;

Pipeline::Pipeline(std::string const& json)
    : m_executor(json)
{
    // Make the symbols in pdal_base global so that they're accessible
    // to PDAL plugins.  Python dlopen's this extension with RTLD_LOCAL,
    // which means that without this, symbols in libpdal_base aren't available
    // for resolution of symbols on future runtime linking.  This is an issue
    // on Apline and other Linux variants that doesn't use UNIQUE symbols
    // for C++ template statics. only
#ifndef _WIN32
    ::dlopen("libpdal_base.so", RTLD_NOLOAD | RTLD_GLOBAL);
#endif

    import_array();
}

Pipeline::~Pipeline()
{
}

void Pipeline::setLogLevel(int level)
{
    m_executor.setLogLevel(level);
}

int Pipeline::getLogLevel() const
{
    return static_cast<int>(m_executor.getLogLevel());
}

int64_t Pipeline::execute()
{

    int64_t count = m_executor.execute();
    return count;
}

bool Pipeline::validate()
{
    return m_executor.validate();
}

std::vector<Array *> Pipeline::getArrays() const
{
    std::vector<Array *> output;

    if (!m_executor.executed())
        throw python_error("call execute() before fetching arrays");

    const pdal::PointViewSet& pvset = m_executor.getManagerConst().views();

    for (auto i: pvset)
    {
        //ABELL - Leak?
        Array *array = new pdal::python::Array;
        array->update(i);
        output.push_back(array);
    }
    return output;
}
} //namespace libpdalpython

