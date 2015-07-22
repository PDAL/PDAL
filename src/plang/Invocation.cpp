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

#include <pdal/GlobalEnvironment.hpp>

#include <pdal/plang/Invocation.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127) // conditional expression is constant
#endif

#include <Python.h>

//#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

// This file can only be included once, otherwise we get wierd runtime errors,
// even if we define NO_IMPORT and stuff, so we include it only here, and
// provide a backdoor function numpy_init() which gets called from
// GlobalEnvironment::startup().
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
        std::ostringstream oss;
        oss << "unable to initialize NumPy with error '" <<
            getPythonTraceback() << "'";
        throw error(oss.str());
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
        throw error(getPythonTraceback());

    Py_INCREF(m_bytecode);

    m_module = PyImport_ExecCodeModule(const_cast<char*>(m_script.module()),
        m_bytecode);
    if (!m_module)
        throw error(getPythonTraceback());

    m_dictionary = PyModule_GetDict(m_module);
    m_function = PyDict_GetItemString(m_dictionary, m_script.function());
    if (!m_function)
    {
        std::ostringstream oss;
        oss << "unable to find target function '" << m_script.function() <<
            "' in module.";
        throw error(oss.str());
    }
    if (!PyCallable_Check(m_function))
        throw error(getPythonTraceback());
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
}


void Invocation::resetArguments()
{
    cleanup();
    m_varsIn = PyDict_New();
    m_varsOut = PyDict_New();
}


void Invocation::insertArgument(std::string const& name, uint8_t* data,
    Dimension::Type::Enum t, point_count_t count)
{
    npy_intp mydims = count;
    int nd = 1;
    npy_intp* dims = &mydims;
    npy_intp stride = Dimension::size(t);
    npy_intp* strides = &stride;
    int flags = NPY_CARRAY; // NPY_BEHAVED

    const int pyDataType = getPythonDataType(t);

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
        throw error("plang output variable '" + name + "' not found.");
    if (!PyArray_Check(xarr))
        throw error("Plang output variable  '" + name +
            "' is not a numpy array");

    PyArrayObject* arr = (PyArrayObject*)xarr;

    npy_intp one = 0;
    const int pyDataType = getPythonDataType(t);
    PyArray_Descr *dtype = PyArray_DESCR(arr);

    if (static_cast<uint32_t>(dtype->elsize) != Dimension::size(t))
    {
        std::ostringstream oss;
        oss << "dtype of array has size " << dtype->elsize
            << " but PDAL dimension '" << name << "' has byte size of "
            << Dimension::size(t) << " bytes.";
        throw error(oss.str());
    }

    using namespace Dimension;
    BaseType::Enum b = Dimension::base(t);
    if (dtype->kind == 'i' && b != BaseType::Signed)
    {
        std::ostringstream oss;
        oss << "dtype of array has a signed integer type but the " <<
            "dimension data type of '" << name <<
            "' is not pdal::Signed.";
        throw error(oss.str());
    }

    if (dtype->kind == 'u' && b != BaseType::Unsigned)
    {
        std::ostringstream oss;
        oss << "dtype of array has a unsigned integer type but the " <<
            "dimension data type of '" << name <<
            "' is not pdal::Unsigned.";
        throw error(oss.str());
    }

    if (dtype->kind == 'f' && b != BaseType::Floating)
    {
        std::ostringstream oss;
        oss << "dtype of array has a float type but the " <<
            "dimension data type of '" << name << "' is not pdal::Floating.";
        throw error(oss.str());
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


int Invocation::getPythonDataType(Dimension::Type::Enum t)
{
    using namespace Dimension;

    switch (t)
    {
    case Type::Float:
        return PyArray_FLOAT;
    case Type::Double:
        return PyArray_DOUBLE;
    case Type::Signed8:
        return PyArray_BYTE;
    case Type::Signed16:
        return PyArray_SHORT;
    case Type::Signed32:
        return PyArray_INT;
    case Type::Signed64:
        return PyArray_LONGLONG;
    case Type::Unsigned8:
        return PyArray_UBYTE;
    case Type::Unsigned16:
        return PyArray_USHORT;
    case Type::Unsigned32:
        return PyArray_UINT;
    case Type::Unsigned64:
        return PyArray_ULONGLONG;
    default:
        return -1;
    }
    assert(0);

    return -1;
}


bool Invocation::hasOutputVariable(const std::string& name) const
{
    return (PyDict_GetItemString(m_varsOut, name.c_str()) != NULL);
}


bool Invocation::execute()
{
    if (!m_bytecode)
        throw error("No code has been compiled");

    m_environment.gil_lock();

    Py_INCREF(m_varsIn);
    Py_INCREF(m_varsOut);
    m_scriptArgs = PyTuple_New(2);
    PyTuple_SetItem(m_scriptArgs, 0, m_varsIn);
    PyTuple_SetItem(m_scriptArgs, 1, m_varsOut);

    m_scriptResult = PyObject_CallObject(m_function, m_scriptArgs);
    if (!m_scriptResult)
        throw error(getPythonTraceback());

    if (!PyBool_Check(m_scriptResult))
        throw error("User function return value not a boolean type.");
    m_environment.gil_unlock();

    return (m_scriptResult == Py_True);
}

} // namespace plang
} // namespace pdal

