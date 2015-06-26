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

#include <pdal/plang/PythonEnvironment.hpp>
#include <pdal/plang/Invocation.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

#include <Python.h>
#include <pystate.h>

#include "Redirector.hpp"

// http://www.linuxjournal.com/article/3641
// http://www.codeproject.com/Articles/11805/Embedding-Python-in-C-C-Part-I
// http://stackoverflow.com/questions/6596016/python-threads-in-c

namespace pdal
{
namespace plang
{


PythonEnvironment::PythonEnvironment()
    : m_redirector(new Redirector())
{
    PyImport_AppendInittab(const_cast<char*>("redirector"), redirector_init);

    Py_Initialize();

    Invocation::numpy_init();

    PyImport_ImportModule("redirector");

    return;
}


PythonEnvironment::~PythonEnvironment()
{
    delete m_redirector;

    Py_Finalize();

    return;
}


void PythonEnvironment::set_stdout(std::ostream* ostr)
{
    m_redirector->set_stdout(ostr);

    return;
}


void PythonEnvironment::reset_stdout()
{
    m_redirector->reset_stdout();

    return;
}

void PythonEnvironment::gil_lock()
{
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    m_gilstate = (int)gstate;
}


void PythonEnvironment::gil_unlock()
{
    PyGILState_STATE gstate = (PyGILState_STATE)m_gilstate;
    PyGILState_Release(gstate);
}


std::string getPythonTraceback()
{

    // get exception info
    PyObject *type, *value, *traceback;
    PyErr_Fetch(&type, &value, &traceback);
    PyErr_NormalizeException(&type, &value, &traceback);

    std::ostringstream mssg;
    if (traceback)
    {
        PyObject* tracebackModule;
        PyObject* tracebackDictionary;
        PyObject* tracebackFunction;
    
        tracebackModule = PyImport_ImportModule("traceback");
        if (!tracebackModule)
            throw error("Unable to load traceback module while "
                "importing numpy inside PDAL.");

        tracebackDictionary = PyModule_GetDict(tracebackModule);

        tracebackFunction =
            PyDict_GetItemString(tracebackDictionary, "format_exception");
        if (!tracebackFunction)
            throw error("Unable to find traceback function while "
                "importing numpy inside PDAL.");

        if (!PyCallable_Check(tracebackFunction))
            throw error("Invalid traceback function while importing numpy "
                "inside PDAL.");
        
        // create an argument for "format exception"
        PyObject* args = PyTuple_New(3);
        PyTuple_SetItem(args, 0, type);
        PyTuple_SetItem(args, 1, value);
        PyTuple_SetItem(args, 2, traceback);

        // get a list of string describing what went wrong
        PyObject* output = PyObject_CallObject(tracebackFunction, args);

        // print error message
        int i, n = PyList_Size(output);

#if PY_MAJOR_VERSION >= 3
        for (i=0; i<n; i++) 
        {
            PyObject* u = PyUnicode_AsUTF8String(PyList_GetItem(output, i));
            const char* p = PyBytes_AsString(u);
            
            mssg << p;
        }
        
#else
        for (i=0; i<n; i++) mssg << PyString_AsString(PyList_GetItem(output, i));
#endif
        
        // clean up
        Py_XDECREF(args);
        Py_XDECREF(output);
    }
    else if (value != NULL)
    {
        PyObject *s = PyObject_Str(value);
#if PY_MAJOR_VERSION >= 3
        // const char* text = PyUnicode_AS_DATA(s);
        PyObject* u = PyUnicode_AsUTF8String(s);
        const char* text = PyBytes_AsString(u);        
#else
        const char* text = PyString_AS_STRING(s);
#endif
        Py_DECREF(s);
        mssg << text;
    }
    else
    {
        mssg << "unknown error that we are unable to get a traceback for. Was it already printed/taken?";
    }

    Py_XDECREF(value);
    Py_XDECREF(type);
    Py_XDECREF(traceback);

    return mssg.str();
}

} // namespace plang
} // namespace pdal

