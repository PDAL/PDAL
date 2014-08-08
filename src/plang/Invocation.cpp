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
        std::ostringstream oss;
        oss << "unable to initialize NumPy with error '" << getPythonTraceback() << "'";
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
    if (!m_bytecode) throw python_error(getPythonTraceback());

    Py_INCREF(m_bytecode);

    m_module = PyImport_ExecCodeModule(const_cast<char*>(m_script.module()), m_bytecode);
    if (!m_module) throw python_error(getPythonTraceback());

    m_dictionary = PyModule_GetDict(m_module);

    m_function = PyDict_GetItemString(m_dictionary, m_script.function());
    if (!m_function)
    {
        std::ostringstream oss;
        oss << "unable to find target function '" << m_script.function() << "' in module";
        throw python_error(oss.str());
    }

    if (!PyCallable_Check(m_function)) throw python_error(getPythonTraceback());

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

    Py_XDECREF(m_bytecode);

    return;
}


void Invocation::resetArguments()
{
    cleanup();

    m_varsIn = PyDict_New();
    m_varsOut = PyDict_New();

    return;
}


void Invocation::insertArgument(boost::uint8_t* data,
                                Dimension::Type::Enum t,
                                point_count_t count)
{
    std::string name = Dimension::Type::name(t);
    npy_intp mydims = data_len;
    int nd = 1;
    npy_intp* dims = &mydims;
    npy_intp stride = Dimension::Type::size(t);
    npy_intp* strides = &stride;
    int flags = NPY_CARRAY; // NPY_BEHAVED

    const int pyDataType = getPythonDataType(t);

    PyObject* pyArray = PyArray_New(&PyArray_Type, nd, dims, pyDataType,
        strides, data, 0, flags, NULL);
    m_pyInputArrays.push_back(pyArray);
    PyDict_SetItemString(m_varsIn, name.c_str(), pyArray);
}


void *Invocation::extractResult(const std::string& name,
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
        throw python_error("plang output variable  '" + name +
            "' is not a numpy array");
    }

    PyArrayObject* arr = (PyArrayObject*)xarr;

    npy_intp one=0;
    const int pyDataType = getPythonDataType(t);
    
    PyArray_Descr *dtype = PyArray_DESCR(arr);
    
    if (static_cast<boost::uint32_t>(dtype->elsize) != numBytes)
    {
        std::ostringstream oss;
        oss << "dtype of array has size " << dtype->elsize 
            << " but PDAL dimension '" << name << "' has byte size of "
            << numBytes << " bytes";
        throw python_error(oss.str());
    }
    
    if (dtype->kind == 'i' && dataType != dimension::SignedInteger)
    {
        std::ostringstream oss;
        oss << "dtype of array has a signed integer type but the " <<
            "dimension data type of '" << name <<
            "' is not pdal::SignedInteger"; 
        throw python_error(oss.str());
    }

    if (dtype->kind == 'u' && dataType != dimension::UnsignedInteger)
    {
        std::ostringstream oss;
        oss << "dtype of array has a unsigned integer type but the " <<
            "dimension data type of '" << name <<
            "' is not pdal::UnsignedInteger"; 
        throw python_error(oss.str());
    }

    if (dtype->kind == 'f' && dataType != dimension::Float)
    {
        std::ostringstream oss;
        oss << "dtype of array has a float type but the " <<
            "dimension data type of '" << name << "' is not pdal::Float"; 
        throw python_error(oss.str());
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
        PyObject* u = PyUnicode_AsUTF8String(key);
        p = PyBytes_AsString(u);
#else
        p = PyString_AsString(key);
#endif
        if (p)
            names.push_back(p);
    }

    return;
}


int Invocation::getPythonDataType(Dimension::Type::Enum t)
{
    Dimension::BaseType b = Dimension::Type::base(t);
    size_t size = Dimension::Type::size(t)
    switch (t)
    {
        case Dimension::BaseType::Enum::Floating:
            switch (size)
            {
                case 4:
                    return PyArray_FLOAT;
                case 8:
                    return PyArray_DOUBLE;
            }
            break;
        case Dimension::BaseType::Enum::Signed:
            switch (size)
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
        case Dimension::BaseType::Enum::Unsigned:
            switch (size)
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
    if (!m_scriptResult) throw python_error(getPythonTraceback());

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
