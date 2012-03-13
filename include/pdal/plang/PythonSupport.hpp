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

#ifndef PYTHONSUPPORT_H
#define PYTHONSUPPORT_H

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include <pdal/pdal_internal.hpp>
#include <pdal/PointBuffer.hpp>

#include <boost/cstdint.hpp>
#include <boost/variant.hpp>

#include <vector>
#include <iostream>

// forward declare PyObject so we don't need the python headers everywhere
// see: http://mail.python.org/pipermail/python-dev/2003-August/037601.html
#ifndef PyObject_HEAD
struct _object;
typedef _object PyObject;
#endif

namespace pdal { namespace plang {


// this is a singleton: only create it once, and keep it around forever
class PDAL_DLL PythonEnvironment
{
public:
    static void startup();
    static void shutdown();

    void dumpObject(PyObject*);
    void handleError();

private:
    PythonEnvironment();
    ~PythonEnvironment();

    PyObject* m_tracebackModule;
    PyObject* m_tracebackDictionary;
    PyObject *m_tracebackFunction;
};


class PDAL_DLL PythonMethodX
{
public:
    PythonMethodX(const std::string& source);
    void compile();

    void resetArguments();


    // creates a Python variable pointing to a (one dimensional) C array
    // adds the new variable to the arguments dictionary
    void insertArgument(const std::string& name, 
                        boost::uint8_t* data, 
                        boost::uint32_t data_len, 
                        boost::uint32_t data_stride,                                  
                        dimension::Interpretation dataType, 
                        boost::uint32_t numBytes);
    void extractResult(const std::string& name, 
                       boost::uint8_t* data, 
                       boost::uint32_t data_len, 
                       boost::uint32_t data_stride,                                  
                       dimension::Interpretation dataType, 
                       boost::uint32_t numBytes);

    bool hasOutputVariable(const std::string& name) const;

    // returns true iff the called python function returns true,
    // as would be used for a predicate function
    // (that is, the return value is NOT an error indicator)
    bool execute();

private:
    PythonEnvironment& m_env;
    std::string m_source;

    PyObject* m_compile;
    PyObject* m_module;
    PyObject* m_dict;
    PyObject* m_func;

    PyObject* m_scriptSource;
    PyObject* m_varsIn;
    PyObject* m_varsOut;
    PyObject* m_scriptArgs;
    PyObject* m_scriptResult;
    std::vector<PyObject*> m_pyInputArrays;

    PythonMethodX& operator=(PythonMethodX const& rhs); // nope
};


class PDAL_DLL PythonPDALMethod : public PythonMethodX
{
public:
    PythonPDALMethod(const std::string& source);

    void beginChunk(PointBuffer&);
    void endChunk(PointBuffer&);

private:
    PythonPDALMethod& operator=(PythonPDALMethod const& rhs); // nope
};


} } // namespaces

#endif

#endif
