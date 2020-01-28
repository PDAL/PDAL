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

#include "Script.hpp"
#include "Environment.hpp"

#include <pdal/Dimension.hpp>
#include <pdal/PointView.hpp>

namespace pdal
{
namespace plang
{

class PDAL_DLL Invocation
{
public:
    Invocation(const Script&, MetadataNode m, const std::string& pdalArgs);
    Invocation& operator=(Invocation const& rhs) = delete;
    Invocation(const Invocation& other) = delete;
    ~Invocation()
    {}

    bool execute(PointViewPtr& v, MetadataNode stageMetadata);

    PyObject* m_function;

private:
    void compile();
    PyObject *prepareData(PointViewPtr& view);
    void extractData(PointViewPtr& view, PyObject *outArrays);
    PyObject *addArray(std::string const& name, uint8_t* data,
        Dimension::Type t, point_count_t count);
    void *extractArray(PyObject *array, const std::string& name,
        Dimension::Type dataType, size_t& arrSize);
    PointViewPtr maskData(PointViewPtr& view, PyObject *maskArray);
    void extractMetadata(MetadataNode stageMetadata);

    Script m_script;

    PyObject* m_module;
    // Pointer to the function in the module.  Owned by the module.

    // Pointers to numpy arrays and contained data buffers for cleanup.
    std::vector<PyObject*> m_pyInputArrays;
    std::vector<void *> m_numpyBuffers;

    MetadataNode m_inputMetadata;
    std::string m_pdalargs;
};

} // namespace plang
} // namespace pdal

