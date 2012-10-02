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
#include <pdal/GlobalEnvironment.hpp>

#ifdef PDAL_HAVE_PYTHON

#include <pdal/plang/Invocation.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127) // conditional expression is constant
#endif

#include <Python.h>

// This file can only be included once, otherwise we get wierd runtime errors,
// even if we define NO_IMPORT and stuff, so we include it only here, and provide
// a backdoor function numpy_init() which gets called from GlobalEnvironment::startup().
#include <numpy/arrayobject.h>

namespace pdal
{
namespace plang
{


void Invocation::numpy_init()
{
    // this macro is defined be NumPy and must be included
    if (_import_array() < 0)
    {
        PyObject* tracebackModule;
        PyObject* tracebackDictionary;
        PyObject* tracebackFunction;
        tracebackModule = PyImport_ImportModule("traceback");
        if (!tracebackModule)
        {
            throw python_error("unable to load traceback module while importing numpy inside PDAL");
        }

        tracebackDictionary = PyModule_GetDict(tracebackModule);

        tracebackFunction = PyDict_GetItemString(tracebackDictionary, "format_exception");
        if (!tracebackFunction)
        {
            throw python_error("unable to find traceback function while importing numpy inside PDAL");
        }

        if (!PyCallable_Check(tracebackFunction))
        {
            throw python_error("invalid traceback function while importing numpy inside PDAL");
        }
        
        
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
            PyObject* output = PyObject_CallObject(tracebackFunction, args);

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


        std::ostringstream oss;
        oss << "unable to initialize NumPy with error '" << std::string(mssg) << "'";
        throw python_error(oss.str());
    }
}


Invocation::Invocation(const Script& script)
    : m_script(script)
    , m_environment(pdal::GlobalEnvironment::get().getPythonEnvironment())
    , m_bytecode(NULL)
    , m_module(NULL)
    , m_dictionary(NULL)
    , m_function(NULL)
    , m_varsIn(NULL)
    , m_varsOut(NULL)
    , m_scriptArgs(NULL)
    , m_scriptResult(NULL)
{
    resetArguments();

    return;
}


Invocation::~Invocation()
{
    cleanup();
    return;
}


void Invocation::compile()
{
    m_bytecode = Py_CompileString(m_script.source(), m_script.module(), Py_file_input);
    if (!m_bytecode) m_environment.handleError();

    assert(m_bytecode);
    m_module = PyImport_ExecCodeModule(const_cast<char*>(m_script.module()), m_bytecode);
    if (!m_module) m_environment.handleError();

    m_dictionary = PyModule_GetDict(m_module);

    m_function = PyDict_GetItemString(m_dictionary, m_script.function());
    if (!m_function)
    {
        std::ostringstream oss;
        oss << "unable to find target function '" << m_script.function() << "' in module";
        throw python_error(oss.str());
    }

    if (!PyCallable_Check(m_function)) m_environment.handleError();

    return;
}


void Invocation::cleanup()
{
    Py_XDECREF(m_varsIn);
    Py_XDECREF(m_varsOut);

    Py_XDECREF(m_scriptResult);

    Py_XDECREF(m_scriptArgs); // also decrements script and vars

    for (unsigned int i=0; i<m_pyInputArrays.size(); i++)
    {
        PyObject* obj = m_pyInputArrays[i];
        Py_XDECREF(obj);
    }
    m_pyInputArrays.clear();

    return;
}


void Invocation::resetArguments()
{
    cleanup();

    m_varsIn = PyDict_New();
    m_varsOut = PyDict_New();

    return;
}


void Invocation::insertArgument(const std::string& name,
                                boost::uint8_t* data,
                                boost::uint32_t data_len,
                                boost::uint32_t data_stride,
                                dimension::Interpretation dataType,
                                boost::uint32_t numBytes)
{
    npy_intp mydims = data_len;
    int nd = 1;
    npy_intp* dims = &mydims;
    npy_intp stride = data_stride;
    npy_intp* strides = &stride;
    int flags = NPY_CARRAY; // NPY_BEHAVED

    const int pyDataType = getPythonDataType(dataType, numBytes);

    PyObject* pyArray = PyArray_New(&PyArray_Type, nd, dims, pyDataType, strides, data, 0, flags, NULL);

    m_pyInputArrays.push_back(pyArray);

    PyDict_SetItemString(m_varsIn, name.c_str(), pyArray);

    return;
}


void Invocation::extractResult(const std::string& name,
                               boost::uint8_t* dst,
                               boost::uint32_t data_len,
                               boost::uint32_t data_stride,
                               dimension::Interpretation dataType,
                               boost::uint32_t numBytes)
{
    PyObject* xarr = PyDict_GetItemString(m_varsOut, name.c_str());
    if (!xarr)
    {
        throw python_error("plang output variable '" + name + "' not found");
    }
    if (!PyArray_Check(xarr))
    {
        throw python_error("plang output variable  '" + name + "' is not a numpy array");
    }

    PyArrayObject* arr = (PyArrayObject*)xarr;

    npy_intp one=0;
    const int pyDataType = getPythonDataType(dataType, numBytes);

    boost::uint8_t* p = dst;

    if (pyDataType == PyArray_DOUBLE)
    {
        double* src = (double*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(double*)p = src[i];
            p += data_stride;
        }
    }
    else if (pyDataType == PyArray_FLOAT)
    {
        float* src = (float*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(float*)p = src[i];
            p += data_stride;
        }
    }
    else if (pyDataType == PyArray_BYTE)
    {
        boost::int8_t* src = (boost::int8_t*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(boost::int8_t*)p = src[i];
            p += data_stride;
        }
    }
    else if (pyDataType == PyArray_UBYTE)
    {
        boost::uint8_t* src = (boost::uint8_t*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(boost::uint8_t*)p = src[i];
            p += data_stride;
        }
    }
    else if (pyDataType == PyArray_SHORT)
    {
        boost::int16_t* src = (boost::int16_t*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(boost::int16_t*)p = src[i];
            p += data_stride;
        }
    }
    else if (pyDataType == PyArray_USHORT)
    {
        boost::uint16_t* src = (boost::uint16_t*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(boost::uint16_t*)p = src[i];
            p += data_stride;
        }
    }
    else if (pyDataType == PyArray_INT)
    {
        boost::int32_t* src = (boost::int32_t*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(boost::int32_t*)p = src[i];
            p += data_stride;
        }
    }
    else if (pyDataType == PyArray_UINT)
    {
        boost::uint32_t* src = (boost::uint32_t*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(boost::uint32_t*)p = src[i];
            p += data_stride;
        }
    }
    else if (pyDataType == PyArray_LONGLONG)
    {
        boost::int64_t* src = (boost::int64_t*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(boost::int64_t*)p = src[i];
            p += data_stride;
        }
    }
    else if (pyDataType == PyArray_ULONGLONG)
    {
        boost::uint64_t* src = (boost::uint64_t*)PyArray_GetPtr(arr, &one);
        for (unsigned int i=0; i<data_len; i++)
        {
            *(boost::uint64_t*)p = src[i];
            p += data_stride;
        }
    }
    else
    {
        assert(0);
    }

    return;
}


void Invocation::getOutputNames(std::vector<std::string>& names)
{
    names.clear();

    PyObject *key, *value;
    Py_ssize_t pos = 0;

    while (PyDict_Next(m_varsOut, &pos, &key, &value))
    {
        char* p = PyString_AsString(key);
        names.push_back(p);
    }

    return;
}


int Invocation::getPythonDataType(dimension::Interpretation datatype, boost::uint32_t siz)
{
    switch (datatype)
    {
        case dimension::SignedByte:
            switch (siz)
            {
                case 1:
                    return PyArray_BYTE;
            }
            break;
        case dimension::UnsignedByte:
            switch (siz)
            {
                case 1:
                    return PyArray_UBYTE;
            }
            break;
        case dimension::Float:
            switch (siz)
            {
                case 4:
                    return PyArray_FLOAT;
                case 8:
                    return PyArray_DOUBLE;
            }
            break;
        case dimension::SignedInteger:
            switch (siz)
            {
                case 1:
                    return PyArray_BYTE;
                case 2:
                    return PyArray_SHORT;
                case 4:
                    return PyArray_INT;
                case 8:
                    return PyArray_LONGLONG;
            }
            break;
        case dimension::UnsignedInteger:
            switch (siz)
            {
                case 1:
                    return PyArray_UBYTE;
                case 2:
                    return PyArray_USHORT;
                case 4:
                    return PyArray_UINT;
                case 8:
                    return PyArray_ULONGLONG;
            }
            break;
        default:
            return -1;
    }

    assert(0);

    return -1;
}


bool Invocation::hasOutputVariable(const std::string& name) const
{
    PyObject* obj = PyDict_GetItemString(m_varsOut, name.c_str());

    return (obj!=NULL);
}


bool Invocation::execute()
{
    if (!m_bytecode)
    {
        throw python_error("no code has been compiled");
    }
    
    m_environment.gil_lock();

    Py_INCREF(m_varsIn);
    Py_INCREF(m_varsOut);
    m_scriptArgs = PyTuple_New(2);
    PyTuple_SetItem(m_scriptArgs, 0, m_varsIn);
    PyTuple_SetItem(m_scriptArgs, 1, m_varsOut);

    m_scriptResult = PyObject_CallObject(m_function, m_scriptArgs);
    if (!m_scriptResult) m_environment.handleError();

    if (!PyBool_Check(m_scriptResult))
    {
        throw python_error("user function return value not a boolean type");
    }
    bool sts = false;
    if (m_scriptResult == Py_True)
    {
        sts = true;
    }

    m_environment.gil_unlock();

    return sts;
}


}
} //namespaces

#endif
