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

#include <pdal/plang/PythonEnvironment.hpp>
#include <pdal/plang/Invocation.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

#include <Python.h>
#include "Redirector.hpp"

namespace pdal
{
namespace plang
{


static std::string s_buffer;
static void mywriterfunction(const std::string& s)
{
    s_buffer += s;
}


PythonEnvironment::PythonEnvironment()
    : m_tracebackModule(NULL)
    , m_tracebackDictionary(NULL)
    , m_tracebackFunction(NULL)
{
    PyImport_AppendInittab("redirector", redirector_init);

    Py_Initialize();

    Invocation::numpy_init();

    // load traceback stuff we need for error handling
    m_tracebackModule = PyImport_ImportModule("traceback");
    if (!m_tracebackModule)
    {
        throw python_error("unable to load traceback module");
    }

    m_tracebackDictionary = PyModule_GetDict(m_tracebackModule);

    m_tracebackFunction = PyDict_GetItemString(m_tracebackDictionary, "format_exception");
    if (!m_tracebackFunction)
    {
        throw python_error("unable to find traceback function");
    }

    if (!PyCallable_Check(m_tracebackFunction))
    {
        throw python_error("invalid traceback function");
    }

    PyImport_ImportModule("redirector");

#if 0
    PyRun_SimpleString("print(\'hello to console\')");

    {
        // switch sys.stdout to custom handler
        Redirector::stdout_write_type* write = mywriterfunction;
        Redirector::set_stdout(write);
        PyRun_SimpleString("print(\'hello to buffer\')");
        Redirector::reset_stdout();
    }

    PyRun_SimpleString("print(\'hello to console again\')");

    // output what was written to buffer object
    std::clog << s_buffer << std::endl;
#endif

    return;
}


PythonEnvironment::~PythonEnvironment()
{
    Py_XDECREF(m_tracebackFunction);

    Py_XDECREF(m_tracebackModule);

    Py_Finalize();

    return;
}


void PythonEnvironment::set_stdout(std::ostream* s)
{
    return;
}


void PythonEnvironment::restore_stdout()
{
    return;
}


void PythonEnvironment::handleError()
{
    // get exception info
    PyObject *type, *value, *traceback;
    PyErr_Fetch(&type, &value, &traceback);
    PyErr_NormalizeException(&type, &value, &traceback);

    std::string mssg = "";
    if (traceback)
    {
        // create an argument for "format exception"
        PyObject* args = PyTuple_New(3);
        PyTuple_SetItem(args, 0, type);
        PyTuple_SetItem(args, 1, value);
        PyTuple_SetItem(args, 2, traceback);

        // get a list of string describing what went wrong
        PyObject* output = PyObject_CallObject(m_tracebackFunction, args);

        // print error message
        int i, n = PyList_Size(output);
        for (i=0; i<n; i++) mssg += PyString_AsString(PyList_GetItem(output, i));

        // clean up
        Py_XDECREF(args);
        Py_XDECREF(output);
    }
    else if (value != NULL)
    {
        PyObject *s = PyObject_Str(value);
        const char* text = PyString_AS_STRING(s);
        Py_DECREF(s);
        mssg += text;
    }
    else
    {
        mssg = "unknown error";
    }

    Py_XDECREF(value);
    Py_XDECREF(type);
    Py_XDECREF(traceback);

    throw python_error(mssg);

    return;
}


}
} //namespaces

#endif
