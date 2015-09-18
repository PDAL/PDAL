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

#include <pdal/plang/Environment.hpp>
#include <pdal/plang/Redirector.hpp>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define PY_ARRAY_UNIQUE_SYMBOL PDAL_ARRAY_API
#include <numpy/arrayobject.h>

#include <sstream>
#include <mutex>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

#include <Python.h>
#include <pystate.h>
#undef toupper
#undef tolower
#undef isspace


// http://www.linuxjournal.com/article/3641
// http://www.codeproject.com/Articles/11805/Embedding-Python-in-C-C-Part-I
// http://stackoverflow.com/questions/6596016/python-threads-in-c

namespace pdal
{
namespace plang
{

static Environment* g_environment=0;

EnvironmentPtr Environment::get()
{
    static std::once_flag flag;

    auto init = []()
    {
        g_environment = new Environment();
    };

    std::call_once(flag, init);

    return g_environment;
}


Environment::Environment()
{
    // This awfulness is due to the import_array MACRO that returns a value
    // in some cases and returns nothing at other times.  Defining
    // NUMPY_IMPORT_ARRAY_RETVAL to nothing makes it so we don't ever return
    // a value.  The function needs to be stuck in a function to deal with
    // the return.
    auto initNumpy = []()
    {
#undef NUMPY_IMPORT_ARRAY_RETVAL
#define NUMPY_IMPORT_ARRAY_RETVAL
        import_array();
    };

    PyImport_AppendInittab(const_cast<char*>("redirector"), redirector_init);

    Py_Initialize();
    initNumpy();
    PyImport_ImportModule("redirector");
}


Environment::~Environment()
{
    Py_Finalize();
}


void Environment::set_stdout(std::ostream* ostr)
{
    m_redirector.set_stdout(ostr);
}


void Environment::reset_stdout()
{
    m_redirector.reset_stdout();
}


std::string getTraceback()
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
            throw pdal::pdal_error("Unable to load traceback module.");

        tracebackDictionary = PyModule_GetDict(tracebackModule);
        if (!tracebackDictionary)
            throw pdal::pdal_error("Unable to load traceback dictionary.");
        tracebackFunction =
            PyDict_GetItemString(tracebackDictionary, "format_exception");
        if (!tracebackFunction)
            throw pdal::pdal_error("Unable to find traceback function.");

        if (!PyCallable_Check(tracebackFunction))
            throw pdal::pdal_error("Invalid traceback function.");

        // create an argument for "format exception"
        PyObject* args = PyTuple_New(3);
        PyTuple_SetItem(args, 0, type);
        PyTuple_SetItem(args, 1, value);
        PyTuple_SetItem(args, 2, traceback);

        // get a list of string describing what went wrong
        PyObject* output = PyObject_CallObject(tracebackFunction, args);

        // print error message
        int n = PyList_Size(output);

        for (int i = 0; i < n; i++)
        {
            PyObject* l = PyList_GetItem(output, i);
            if (!l)
                throw pdal::pdal_error("unable to get list item in getTraceback");
            PyObject* r = PyObject_Repr(l);
            if (!r)
                throw pdal::pdal_error("unable to get repr in getTraceback");
            Py_ssize_t size;
#if PY_MAJOR_VERSION >= 3
            char* d = PyUnicode_AsUTF8AndSize(r, &size);
#else
            char* d = PyString_AsString(r);
#endif
            mssg << d;
        }

        // clean up
        Py_XDECREF(args);
        Py_XDECREF(output);
    }
    else if (value != NULL)
    {
        PyObject* r = PyObject_Repr(value);
        if (!r)
            throw pdal::pdal_error("couldn't make string representation of traceback value");
        Py_ssize_t size;
#if PY_MAJOR_VERSION >= 3
            char* d = PyUnicode_AsUTF8AndSize(r, &size);
#else
            char* d = PyString_AsString(r);
#endif
        mssg << d;
    }
    else
        mssg << "unknown error that we are unable to get a traceback for."
            "Was it already printed/taken?";

    Py_XDECREF(value);
    Py_XDECREF(type);
    Py_XDECREF(traceback);

    return mssg.str();
}

PyObject *fromMetadata(MetadataNode m)
{
    std::string name = m.name();
    std::string value = m.value();
    std::string type = m.type();
    std::string description = m.description();

    MetadataNodeList children = m.children();
    PyObject *submeta = NULL;
    if (children.size())
    {
        submeta = PyList_New(0);
        for (MetadataNode& child : children)
            PyList_Append(submeta, fromMetadata(child));
    }
    PyObject *data = PyTuple_New(5);
    PyTuple_SetItem(data, 0, PyUnicode_FromString(name.data()));
    PyTuple_SetItem(data, 1, PyUnicode_FromString(value.data()));
    PyTuple_SetItem(data, 2, PyUnicode_FromString(type.data()));
    PyTuple_SetItem(data, 3, PyUnicode_FromString(description.data()));
    PyTuple_SetItem(data, 4, submeta);

    return data;
}

std::string readPythonString(PyObject* list, Py_ssize_t index)
{
    std::stringstream ss;

    PyObject* o = PyTuple_GetItem(list, index);
    if (!o)
    {
        std::stringstream oss;
        oss << "Unable to get list item number " << index << " for list of length " << PyTuple_Size(list);
        throw pdal_error(oss.str());
    }
    PyObject* r = PyObject_Repr(o);
    if (!r)
        throw pdal::pdal_error("unable to get repr in readPythonString");
    Py_ssize_t size;
#if PY_MAJOR_VERSION >= 3
            char* d = PyUnicode_AsUTF8AndSize(r, &size);
#else
            char* d = PyString_AsString(r);
#endif
    ss << d;

    return ss.str();
}
void addMetadata(PyObject *list, MetadataNode m)
{

    if (!PyList_Check(list))
        return;

    for (Py_ssize_t i = 0; i < PyList_Size(list); ++i)
    {
        PyObject *tuple = PyList_GetItem(list, i);
        if (!PyTuple_Check(tuple) || PyTuple_Size(tuple) != 5)
            continue;

        std::string name = readPythonString(tuple, 0);
        std::string value = readPythonString(tuple, 1);

        std::string type = readPythonString(tuple, 2);
        if (type.empty())
            type = Metadata::inferType(value);

        std::string description = readPythonString(tuple, 3);

        PyObject *submeta = PyTuple_GetItem(tuple, 4);
        MetadataNode child =  m.addWithType(name, value, type, description);
        if (submeta)
            addMetadata(submeta, child);
    }
}

int Environment::getPythonDataType(Dimension::Type::Enum t)
{
    using namespace Dimension;

    switch (t)
    {
    case Type::Float:
        return NPY_FLOAT;
    case Type::Double:
        return NPY_DOUBLE;
    case Type::Signed8:
        return NPY_BYTE;
    case Type::Signed16:
        return NPY_SHORT;
    case Type::Signed32:
        return NPY_INT;
    case Type::Signed64:
        return NPY_LONGLONG;
    case Type::Unsigned8:
        return NPY_UBYTE;
    case Type::Unsigned16:
        return NPY_USHORT;
    case Type::Unsigned32:
        return NPY_UINT;
    case Type::Unsigned64:
        return NPY_ULONGLONG;
    default:
        return -1;
    }
    assert(0);

    return -1;
}



} // namespace plang
} // namespace pdal

