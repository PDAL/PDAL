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

namespace pdal
{
namespace plang
{


class PDAL_DLL Invocation
{
public:
    Invocation(const Script&);
    ~Invocation();

    void compile();

    void resetArguments();


    // creates a Python variable pointing to a (one dimensional) C array
    // adds the new variable to the arguments dictionary
    void insertArgument(std::string const& name,
                        uint8_t* data,
                        Dimension::Type::Enum t,
                        point_count_t count);
    void *extractResult(const std::string& name,
                        Dimension::Type::Enum dataType);

    bool hasOutputVariable(const std::string& name) const;

    // returns true iff the called python function returns true,
    // as would be used for a predicate function
    // (that is, the return value is NOT an error indicator)
    bool execute();

    // after a call to execute, this function will return you a list of
    // the names in the 'outs' dictionary (this is used by the
    // BufferedInvocation class to find the returned data -- faster to
    // examine what's already in there than it is to iterate over all the
    // possible names from the schema)
    void getOutputNames(std::vector<std::string>& names);

protected:
    PyObject* m_metaIn;
    PyObject* m_metaOut;

private:
    void cleanup();

    Script m_script;

    PyObject* m_bytecode;
    PyObject* m_module;
    PyObject* m_dictionary;
    PyObject* m_function;

    PyObject* m_varsIn;
    PyObject* m_varsOut;
    PyObject* m_scriptArgs;
    PyObject* m_scriptResult;
    std::vector<PyObject*> m_pyInputArrays;

    Invocation& operator=(Invocation const& rhs); // nope
};

} // namespace plang
} // namespace pdal

