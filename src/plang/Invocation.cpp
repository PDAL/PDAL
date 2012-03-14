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

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include <pdal/plang/Invocation.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127) // conditional expression is constant
#  pragma warning(disable: 4505) // unreferenced local function has been removed
#endif

#include <Python.h>
#include <numpy/arrayobject.h>


namespace pdal { namespace plang {




bool Invocation::execute()
{
    if (!m_compile)
    {
        throw python_error("no code has been compiled");
    }

    Py_INCREF(m_varsIn);
    Py_INCREF(m_varsOut);
    m_scriptArgs = PyTuple_New(2);
    PyTuple_SetItem(m_scriptArgs, 0, m_varsIn);
    PyTuple_SetItem(m_scriptArgs, 1, m_varsOut);

    m_scriptResult = PyObject_CallObject(m_func, m_scriptArgs);
    if (!m_scriptResult) m_env.handleError();

    if (!PyBool_Check(m_scriptResult))
    {
        throw python_error("user function return value not a boolean type");
    }
    bool sts = false;
    if (m_scriptResult == Py_True)
    {
        sts = true;
    }

    return sts;
}



} } //namespaces

#endif
