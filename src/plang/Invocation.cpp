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

#include <pdal/plang/Invocation.hpp>
#include <pdal/plang/Environment.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127) // conditional expression is constant
#endif

#include <Python.h>
#undef toupper
#undef tolower
#undef isspace

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#define NO_IMPORT_ARRAY
#define PY_ARRAY_UNIQUE_SYMBOL PDAL_ARRAY_API
#include <numpy/arrayobject.h>

namespace
{

int argCount(PyObject *function)
{
    PyObject *module = PyImport_ImportModule("inspect");
    if (!module)
        return false;
    PyObject *dictionary = PyModule_GetDict(module);
    PyObject *getargFunc = PyDict_GetItemString(dictionary, "getargspec");
    PyObject *inArgs = PyTuple_New(1);
    PyTuple_SetItem(inArgs, 0, function);
    PyObject *outArgs = PyObject_CallObject(getargFunc, inArgs);
    PyObject *arglist = PyTuple_GetItem(outArgs, 0);
    return PyList_Size(arglist);
}

}

namespace pdal
{
namespace plang
{

Invocation::Invocation(const Script& script) :
    m_metaIn(NULL)
    , m_metaOut(NULL)
    , m_script(script)
    , m_bytecode(NULL)
    , m_module(NULL)
    , m_dictionary(NULL)
    , m_function(NULL)
    , m_varsIn(NULL)
    , m_varsOut(NULL)
    , m_scriptArgs(NULL)
    , m_scriptResult(NULL)
{
    plang::Environment::get();
    resetArguments();
}


Invocation::~Invocation()
{
    cleanup();
}


void Invocation::compile()
{
    m_bytecode = Py_CompileString(m_script.source(), m_script.module(),
        Py_file_input);
    if (!m_bytecode)
        throw pdal::pdal_error(getTraceback());

    Py_INCREF(m_bytecode);

    m_module = PyImport_ExecCodeModule(const_cast<char*>(m_script.module()),
        m_bytecode);
    if (!m_module)
        throw pdal::pdal_error(getTraceback());

    m_dictionary = PyModule_GetDict(m_module);
    m_function = PyDict_GetItemString(m_dictionary, m_script.function());
    if (!m_function)
    {
        std::ostringstream oss;
        oss << "unable to find target function '" << m_script.function() <<
            "' in module.";
        throw pdal::pdal_error(oss.str());
    }
    if (!PyCallable_Check(m_function))
        throw pdal::pdal_error(getTraceback());
}


void Invocation::cleanup()
{
    Py_XDECREF(m_varsIn);
    Py_XDECREF(m_varsOut);
    Py_XDECREF(m_scriptResult);
    Py_XDECREF(m_scriptArgs); // also decrements script and vars
    for (size_t i = 0; i < m_pyInputArrays.size(); i++)
        Py_XDECREF(m_pyInputArrays[i]);
    m_pyInputArrays.clear();
    Py_XDECREF(m_bytecode);
    Py_XDECREF(m_metaIn);
    Py_XDECREF(m_metaOut);
}


void Invocation::resetArguments()
{
    cleanup();
    m_varsIn = PyDict_New();
    m_varsOut = PyDict_New();
    m_metaIn = PyList_New(0);
    m_metaOut = PyList_New(0);
}


void Invocation::insertArgument(std::string const& name, uint8_t* data,
    Dimension::Type::Enum t, point_count_t count)
{
    npy_intp mydims = count;
    int nd = 1;
    npy_intp* dims = &mydims;
    npy_intp stride = Dimension::size(t);
    npy_intp* strides = &stride;

#ifdef NPY_ARRAY_CARRAY
    int flags = NPY_ARRAY_CARRAY;
#else
    int flags = NPY_CARRAY;
#endif

    const int pyDataType = plang::Environment::getPythonDataType(t);

    PyObject* pyArray = PyArray_New(&PyArray_Type, nd, dims, pyDataType,
        strides, data, 0, flags, NULL);
    m_pyInputArrays.push_back(pyArray);
    PyDict_SetItemString(m_varsIn, name.c_str(), pyArray);
}


void *Invocation::extractResult(std::string const& name,
    Dimension::Type::Enum t)
{
    PyObject* xarr = PyDict_GetItemString(m_varsOut, name.c_str());
    if (!xarr)
        throw pdal::pdal_error("plang output variable '" + name + "' not found.");
    if (!PyArray_Check(xarr))
        throw pdal::pdal_error("Plang output variable  '" + name +
            "' is not a numpy array");

    PyArrayObject* arr = (PyArrayObject*)xarr;

    npy_intp one = 0;
    const int pyDataType = pdal::plang::Environment::getPythonDataType(t);
    PyArray_Descr *dtype = PyArray_DESCR(arr);

    if (static_cast<uint32_t>(dtype->elsize) != Dimension::size(t))
    {
        std::ostringstream oss;
        oss << "dtype of array has size " << dtype->elsize
            << " but PDAL dimension '" << name << "' has byte size of "
            << Dimension::size(t) << " bytes.";
        throw pdal::pdal_error(oss.str());
    }

    using namespace Dimension;
    BaseType::Enum b = Dimension::base(t);
    if (dtype->kind == 'i' && b != BaseType::Signed)
    {
        std::ostringstream oss;
        oss << "dtype of array has a signed integer type but the " <<
            "dimension data type of '" << name <<
            "' is not pdal::Signed.";
        throw pdal::pdal_error(oss.str());
    }

    if (dtype->kind == 'u' && b != BaseType::Unsigned)
    {
        std::ostringstream oss;
        oss << "dtype of array has a unsigned integer type but the " <<
            "dimension data type of '" << name <<
            "' is not pdal::Unsigned.";
        throw pdal::pdal_error(oss.str());
    }

    if (dtype->kind == 'f' && b != BaseType::Floating)
    {
        std::ostringstream oss;
        oss << "dtype of array has a float type but the " <<
            "dimension data type of '" << name << "' is not pdal::Floating.";
        throw pdal::pdal_error(oss.str());
    }
    return PyArray_GetPtr(arr, &one);
}


void Invocation::getOutputNames(std::vector<std::string>& names)
{
    names.clear();

    PyObject *key, *value;
    Py_ssize_t pos = 0;

    while (PyDict_Next(m_varsOut, &pos, &key, &value))
    {
        const char* p(0);
#if PY_MAJOR_VERSION >= 3
        p = PyBytes_AsString(PyUnicode_AsUTF8String(key));
#else
        p = PyString_AsString(key);
#endif
        if (p)
            names.push_back(p);
    }
}


bool Invocation::hasOutputVariable(const std::string& name) const
{
    return (PyDict_GetItemString(m_varsOut, name.c_str()) != NULL);
}


bool Invocation::execute()
{
    if (!m_bytecode)
        throw pdal::pdal_error("No code has been compiled");

    Py_INCREF(m_varsIn);
    Py_INCREF(m_varsOut);
    Py_ssize_t numArgs = argCount(m_function);
    m_scriptArgs = PyTuple_New(numArgs);
    PyTuple_SetItem(m_scriptArgs, 0, m_varsIn);
    if (numArgs > 1)
        PyTuple_SetItem(m_scriptArgs, 1, m_varsOut);
    if (numArgs > 2)
        PyTuple_SetItem(m_scriptArgs, 2, m_metaIn);
    if (numArgs > 3)
        PyTuple_SetItem(m_scriptArgs, 3, m_metaOut);

    m_scriptResult = PyObject_CallObject(m_function, m_scriptArgs);
    if (!m_scriptResult)
        throw pdal::pdal_error(getTraceback());

    if (!PyBool_Check(m_scriptResult))
        throw pdal::pdal_error("User function return value not a boolean type.");

    return (m_scriptResult == Py_True);
}

} // namespace plang
} // namespace pdal

