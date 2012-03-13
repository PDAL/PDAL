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

#include <pdal/plang/PythonSupport.hpp>

#include <string>
#include <iostream>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

#include <Python.h>
#include <numpy/arrayobject.h>


namespace pdal { namespace plang {


PythonEnvironment::PythonEnvironment()
    : m_tracebackModule(NULL)
    , m_tracebackDictionary(NULL)
    , m_tracebackFunction(NULL)
{
    return;
}


PythonEnvironment::~PythonEnvironment()
{
    return;
}


void PythonEnvironment::startup()
{
    Py_Initialize();

    // this macro is defined be NumPy and must be included
    if (_import_array() < 0)
    {
        throw python_error("unable to initialize NumPy");
    }

    // load traceback stuff we need for error handling
    m_tracebackModule = PyImport_ImportModule( "traceback" );
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

    return;
}


void PythonEnvironment::shutdown()
{
    Py_XDECREF(m_tracebackFunction);

    Py_XDECREF(m_tracebackModule);

    Py_Finalize();

    return;
}


void PythonEnvironment::dumpObject(PyObject* obj)
{
    if (!obj)
    {
        throw python_error("unable dump null object");
    }

    // output scalar result
    if (PyFloat_Check(obj)) 
    {
        printf("result: %f\n", PyFloat_AsDouble(obj));
        return;
    }

    // we only support arrays for now
    assert(PyArray_Check(obj));

    PyArrayObject* arr = PyArray_GETCONTIGUOUS((PyArrayObject*)obj);
    int ndims = arr->nd;
    npy_intp* dims = arr->dimensions; // not copying data
    double* data = (double*) arr->data; // not copying data
    int i, j, k = 0;

    if( ndims == 1 ) 
    {
        // output vector result
        for( i=0; i<dims[0]; i++ )
            printf( "element: %i value: %f\n", i, data[k++] );
        printf("\n");
    }
    else if( ndims == 2 ) 
    {
        // output matrix result
        for( i=0; i<dims[0]; i++ ) {
            for( j=0; j<dims[1]; j++ )
                printf( "%f ", data[k++] );
            printf( "\n" );
        }
    }
    else
    {
        // output N-D result
        for( i=0; i<ndims; i++ )
            for( j=0; j<dims[i]; j++ )
                printf( "dimension: %i element: %i value: %f\n", i, j, data[k++] );
    }

    // clean
    Py_XDECREF(obj);
}


void PythonEnvironment::handleError()
{
    // get exception info
    PyObject *type, *value, *traceback;
    PyErr_Fetch(&type, &value, &traceback);
    PyErr_NormalizeException(&type, &value, &traceback);

    // create a argument for "format exception"
    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, type);
    PyTuple_SetItem(args, 1, value);
    PyTuple_SetItem(args, 2, traceback);

    // get a list of string describing what went wrong
    PyObject* output = PyObject_CallObject(m_tracebackFunction, args);

    // print error message
    int i, n = PyList_Size(output);
    for (i=0; i<n; i++) printf("%s", PyString_AsString(PyList_GetItem(output, i)));

    // clean up
    Py_XDECREF(args);
    Py_XDECREF(output);

    return;
}


// ==========================================================================


PythonMethodX::PythonMethodX(PythonEnvironment& env, const std::string& source)
    : m_env(env)
    , m_source(source)
    , m_scriptSource(NULL)
    , m_varsIn(NULL)
    , m_varsOut(NULL)
    , m_scriptArgs(NULL)
    , m_scriptResult(NULL)
{
    return;
}


void PythonMethodX::compile()
{
    return;
}


inline
static int getPythonDataType(dimension::Interpretation datatype, boost::uint32_t siz)
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
        case 4:
            return PyArray_INT;
        case 8:
            return PyArray_LONGLONG;
        }
        break;
    case dimension::UnsignedInteger:
        switch (siz)
        {
        case 4:
            return PyArray_UINT;
        case 8:
            return PyArray_ULONGLONG;
        }
        break;
    }

    assert(0);

    return -1;
}


void PythonMethodX::resetArguments()
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

    m_varsIn = PyDict_New();
    m_varsOut = PyDict_New();
    
    return;
}


void PythonMethodX::insertArgument(const std::string& name, 
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


bool PythonMethodX::hasOutputVariable(const std::string& name) const
{
   PyObject* obj = PyDict_GetItemString(m_varsOut, name.c_str());

   return (obj!=NULL);
}


void PythonMethodX::extractResult(const std::string& name, 
                                  boost::uint8_t* dst, 
                                  boost::uint32_t data_len, 
                                  boost::uint32_t data_stride,                                  
                                  dimension::Interpretation dataType, 
                                  boost::uint32_t numBytes)
{      
    PyObject* xarr = PyDict_GetItemString(m_varsOut, name.c_str());
    assert(xarr);
    assert(PyArray_Check(xarr));
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


void PythonMethodX::execute()
{
    PyObject* compiled = Py_CompileString(m_source.c_str(), "YowModule", Py_file_input);
    if (!compiled) m_env.handleError();

    PyObject* module = PyImport_ExecCodeModule("YowModule", compiled);
    if (!module) m_env.handleError();

    PyObject* dict = PyModule_GetDict(module);

    PyObject* func = PyDict_GetItemString(dict, "yow");
    if (!func) m_env.handleError();
    if (!PyCallable_Check(func)) m_env.handleError();
  
    Py_INCREF(m_varsIn);
    Py_INCREF(m_varsOut);
    m_scriptArgs = PyTuple_New(2);
    PyTuple_SetItem(m_scriptArgs, 0, m_varsIn);
    PyTuple_SetItem(m_scriptArgs, 1, m_varsOut);

    m_scriptResult = PyObject_CallObject(func, m_scriptArgs);
    if (!m_scriptResult) m_env.handleError();
    
    return;
}


//////////////////////////////////////////////////////////////////////////////////


PythonPDALMethod::PythonPDALMethod(PythonEnvironment& env, const std::string& source)
    : PythonMethodX(env, source)
{
    return;
}



void PythonPDALMethod::beginChunk(PointBuffer& buffer)
{
    const Schema& schema = buffer.getSchema();

    schema::Map const& map = schema.getDimensions();
    schema::index_by_index const& idx = map.get<schema::index>();
    for (schema::index_by_index::const_iterator iter = idx.begin(); iter != idx.end(); ++iter)
    {
        const Dimension& dim = *iter;
        const std::string& name = dim.getName();

        boost::uint8_t* data = buffer.getData(0) + dim.getByteOffset();

        const boost::uint32_t numPoints = buffer.getNumPoints();
        const boost::uint32_t stride = buffer.getSchema().getByteSize();
        const dimension::Interpretation datatype = dim.getInterpretation();
        const boost::uint32_t numBytes = dim.getByteSize();
        this->insertArgument(name, data, numPoints, stride, datatype, numBytes);
    }

    return;
}


void PythonPDALMethod::endChunk(PointBuffer& buffer)
{
    const Schema& schema = buffer.getSchema();

    schema::Map const& map = schema.getDimensions();
    schema::index_by_index const& idx = map.get<schema::index>();
    for (schema::index_by_index::const_iterator iter = idx.begin(); iter != idx.end(); ++iter)
    {
        const Dimension& dim = *iter;
        const std::string& name = dim.getName();
        
        if (hasOutputVariable(name))
        {
            boost::uint8_t* data = buffer.getData(0) + dim.getByteOffset();
            const boost::uint32_t numPoints = buffer.getNumPoints();
            const boost::uint32_t stride = buffer.getSchema().getByteSize();
            const dimension::Interpretation datatype = dim.getInterpretation();
            const boost::uint32_t numBytes = dim.getByteSize();
            extractResult(name, data, numPoints, stride, datatype, numBytes);
        }
    }

    return;
}



} } //namespaces

#endif
