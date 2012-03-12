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

#include "C:\Utils\Python27\include\Python.h"
#include "C:\Utils\Python27\Lib\site-packages\numpy\core\include\numpy\arrayobject.h"


namespace pdal { namespace plang {


PythonEnvironment::PythonEnvironment()
    : m_mod1(NULL)
    , m_mod2(NULL)
    , m_dict1(NULL)
    , m_dict2(NULL)
    , m_func1(NULL)
    , m_func2(NULL)
    , m_fexcp(NULL)
{
    return;
}


PythonEnvironment::~PythonEnvironment()
{
    return;
}


bool PythonEnvironment::startup()
{
    // launch the python interpreter
    Py_Initialize();

    // this macro is defined be NumPy and must be included
    if (_import_array() < 0)
        die(20);

    // load module "traceback" (for error handling)
    m_mod1 = PyImport_ImportModule( "traceback" );
    if (!m_mod1) die(6);

    PyObject* compiled = Py_CompileString(
        "def run_expression( expr, vars ):\n"
        "  return eval( expr, {}, vars )\n"
        "def run_script( expr, vars ):\n"
        "  exec( expr, {}, vars );\n"
        "  return locals()['vars']\n",
        "Test",
        Py_file_input);
    m_mod2 = PyImport_ExecCodeModule("Test", compiled);
    if (!m_mod2) die(6);

    // get dictionary of available items in the modules
    m_dict1 = PyModule_GetDict(m_mod1);
    m_dict2 = PyModule_GetDict(m_mod2);

    // grab the functions we are interested in
    m_fexcp = PyDict_GetItemString(m_dict1, "format_exception");
    m_func1 = PyDict_GetItemString(m_dict2, "run_expression");
    m_func2 = PyDict_GetItemString(m_dict2, "run_script");
    if(! ( m_func1 && m_func2 && m_fexcp )) die(5);

    if(! ( PyCallable_Check(m_func1) && PyCallable_Check(m_func2) && PyCallable_Check(m_fexcp)) ) die(4);

    return true;
}


bool PythonEnvironment::shutdown()
{

    Py_XDECREF(m_func1);
    Py_XDECREF(m_func2);
    Py_XDECREF(m_fexcp);

    Py_XDECREF(m_mod1);
    Py_XDECREF(m_mod2);

    Py_Finalize();

    return true;
}


void PythonEnvironment::die(int i)
{
    printf("error: %d\n", i);
    abort();
}


void PythonEnvironment::output_result( PyObject* rslt )
{
    if( !rslt ) {
        printf("output_result: argument must be a valid pointer\n");
        return;
    }

    // output scalar result
    if( PyFloat_Check(rslt) ) {
        printf( "result: %f\n", PyFloat_AsDouble(rslt) );
        return;
    }

    if( !PyArray_Check(rslt) ) {
        die(10);
    }

    PyArrayObject* obj = PyArray_GETCONTIGUOUS( (PyArrayObject*) rslt );
    int ndims = obj->nd;
    int* dims = obj->dimensions; // not copying data
    double* data = (double*) obj->data; // not copying data
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


void PythonEnvironment::handle_error( PyObject* fe )
{
    // get exception info
    PyObject *type, *value, *traceback;
    PyErr_Fetch( &type, &value, &traceback );
    PyErr_NormalizeException( &type, &value, &traceback );

    // create a argument for "format exception"
    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem( args, 0, type );
    PyTuple_SetItem( args, 1, value );
    PyTuple_SetItem( args, 2, traceback );

    // get a list of string describing what went wrong
    PyObject* output = PyObject_CallObject( fe, args );

    // print error message
    int i, n = PyList_Size( output );
    for( i=0; i<n; i++ ) printf( "%s", PyString_AsString( PyList_GetItem( output, i ) ) );

    // clean up
    Py_XDECREF( args );
    Py_XDECREF( output );
}


// ==========================================================================


PythonMethod::PythonMethod(PythonEnvironment& env, const std::string& source)
    : m_env(env)
    , m_source(source)
{
    return;
}


bool PythonMethod::compile()
{
    return true;
}


bool PythonMethod::setVariable_Float64Array(const std::string& name, double* data)
{
    return true;
}


bool PythonMethod::execute()
{
//        *vars, *args2, *rslt2, *rslt3;

    // define a python expression and a python script
    PyObject* script = PyString_FromString( 
        "import numpy\n"
        "print '===inside script==='\n"
        "value = 42.0\n"
        "arr1 = arr1 * 2\n"
        "print arr1[3]\n"
        "print arr2[3]\n"
        "print arr3[3]\n"
        "print arr4[3]\n"
        "print '==================='\n"
        "print value\n" );
    


    const int nelem = 10;
    struct Record
    {
        double x;
        double y;
        double z;
        unsigned char t;
    };

    Record* arr = new Record[nelem];

    for (int i=0; i<nelem; i++)
    {
        arr[i].x = i*i;
        arr[i].y = i*10;
        arr[i].z = i*100;
        arr[i].t = (i%17);
    }

    int mydims = { nelem };

    int nd = 1;
    npy_intp* dims = &mydims;
    int stride = sizeof(Record);
    npy_intp* strides = &stride;
    int flags = 0;//NPY_BEHAVED; //NPY_CARRAY;
    
    char* data = (char*)arr;
    PyObject* py_array1 = PyArray_New(&PyArray_Type, nd, dims, PyArray_DOUBLE, strides, data, 0, flags, NULL);
    PyObject* py_array2 = PyArray_New(&PyArray_Type, nd, dims, PyArray_DOUBLE, strides, data+8, 0, flags, NULL);
    PyObject* py_array3 = PyArray_New(&PyArray_Type, nd, dims, PyArray_DOUBLE, strides, data+16, 0, flags, NULL);
    PyObject* py_array4 = PyArray_New(&PyArray_Type, nd, dims, PyArray_UBYTE, strides, data+24, 0, flags, NULL);

    printf("BEFORE\n");
    printf("%f\n", arr[3].x);
    printf("%f\n", arr[3].y);
    printf("%f\n", arr[3].z);
    printf("%d\n", (int)arr[3].t);

    // put variables into dictionary of local variables for python
    PyObject* vars = PyDict_New();
    PyDict_SetItemString( vars, "arr1", py_array1 );
    PyDict_SetItemString( vars, "arr2", py_array2 );
    PyDict_SetItemString( vars, "arr3", py_array3 );
    PyDict_SetItemString( vars, "arr4", py_array4 );

    // create argument for "run_script"
    Py_INCREF(vars);
    PyObject* args2 = PyTuple_New(2);
    PyTuple_SetItem(args2, 0, script);
    PyTuple_SetItem(args2, 1, vars);

    // execute script
    PyObject* rslt2 = PyObject_CallObject(m_env.m_func2, args2);
    if( !rslt2 ) m_env.die(2);

    PyObject* rslt3 = PyDict_GetItemString( rslt2, "value" );
    if( !rslt3 ) m_env.die(1);
    printf("======value out=========\n");
    m_env.output_result( rslt3 );
    printf("========================\n");
    Py_XDECREF(rslt3);

    PyObject* rslt4 = PyDict_GetItemString( rslt2, "arr1" );
    if( !rslt4 ) m_env.die(1);
    printf("========arr1 out=======\n");
    m_env.output_result( rslt4 );
    printf("=======================\n");

    PyObject* rslt5 = PyDict_GetItemString( rslt2, "arr2" );
    if( !rslt5 ) m_env.die(1);
    printf("========arr2 out=======\n");
    m_env.output_result( rslt5 );
    printf("=======================\n");

    PyObject* rslt6 = PyDict_GetItemString( rslt2, "arr3" );
    if( !rslt6 ) m_env.die(1);
    printf("========arr3 out=======\n");
    m_env.output_result( rslt6 );
    printf("=======================\n");

    PyObject* rslt7 = PyDict_GetItemString( rslt2, "arr4" );
    if( !rslt7 ) m_env.die(1);
    //printf("========arr4 out=======\n");
    //m_env.output_result( rslt7 );
    //printf("=======================\n");

    {
        double* data = (double*)PyArray_DATA(rslt4);
        printf("%f\n", data[3]);
    }

    Py_XDECREF(rslt4);
    Py_XDECREF(rslt5);

    Py_XDECREF(rslt2);

    printf("AFTER\n");
    printf("%f\n", arr[3].x);
    printf("%f\n", arr[3].y);
    printf("%f\n", arr[3].z);
    printf("%d\n", (int)arr[3].t);

    Py_XDECREF(args2); // also decrements script and vars
    Py_XDECREF(py_array1);
    Py_XDECREF(py_array2);
    Py_XDECREF(py_array3);
    Py_XDECREF(py_array4);
    delete[] arr;
    Py_XDECREF(script);

    return true;
}




} } //namespaces

#endif
