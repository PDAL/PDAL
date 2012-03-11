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

    // load module "traceback" (for error handling) and module from file "Test.py"
    m_mod1 = PyImport_ImportModule( "traceback" );
    m_mod2 = PyImport_ImportModule( "Test" );
    if(! (m_mod1 && m_mod2 )) die(6);

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
    PyObject *script,
        *mat,
        *vars, *args2, *rslt2, *rslt3;

    // define a python expression and a python script
    script = PyString_FromString( 
        "import numpy\n"
        "print 'DURING'\n"
        "value = 42.0\n"
        "m[1,1]=987654321\n"
        "mm[1,2]=123456789\n"
        "print m[1,1]\n"
        "print mm[1,2]\n"
        "print value\n" );
    


    // define a matrix using the classical C-format:
    // pointer-to-pointer-to-numeric type, last dimension running fastest
    //
    // setup pointers
    const int nrow = 3, ncol = 4, nelem = nrow*ncol;
    double** m = new double*[nrow];
    m[0] = new double[nelem];
    m[1] = m[0] + ncol;
    m[2] = m[1] + ncol;

    // fill in values
    m[0][0] = 1.0; m[0][1] = 2.0; m[0][2] = 5.0; m[0][3] = 34;
    m[1][0] = 5.0; m[1][1] = 1.0; m[1][2] = 8.0; m[1][3] = 64;
    m[2][0] = 8.0; m[2][1] = 0.0; m[2][2] = 3.0; m[2][3] = 12;
    int mdim[] = { nrow, ncol };
    mat = PyArray_SimpleNewFromData( 2, mdim, PyArray_DOUBLE, m[0] );

    printf("BEFORE\n");
    printf("%f\n", m[1][2]);

    // put variables into dictionary of local variables for python
    vars = PyDict_New();
    PyDict_SetItemString( vars, "m", mat );
    PyDict_SetItemString( vars, "mm", mat );

    // create argument for "run_script"
    Py_INCREF(vars);
    args2 = PyTuple_New(2);
    PyTuple_SetItem(args2, 0, script);
    PyTuple_SetItem(args2, 1, vars);

    // execute script
    rslt2 = PyObject_CallObject(m_env.m_func2, args2);
    if( !rslt2 ) m_env.die(2);

    rslt3 = PyDict_GetItemString( rslt2, "value" );
    if( !rslt3 ) m_env.die(1);
    printf("===============\n");
    m_env.output_result( rslt3 );
    printf("===============\n");
    Py_XDECREF(rslt3);

    Py_XDECREF(rslt2);

    printf("AFTER\n");
    printf("%f\n", m[1][1]);
    printf("%f\n", m[1][2]);

    Py_XDECREF(args2); // also decrements script and vars
    Py_XDECREF(mat);
    delete[] m[0];
    delete[] m;
    Py_XDECREF(script);

    return true;
}




} } //namespaces

#endif
