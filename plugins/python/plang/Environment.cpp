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

#ifndef _WIN32
#include <dlfcn.h>
#endif

#include "Environment.hpp"
#include "Redirector.hpp"

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define PY_ARRAY_UNIQUE_SYMBOL PDAL_ARRAY_API
#include <numpy/arrayobject.h>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

#include <sstream>
#include <mutex>

#pragma warning(disable: 4127)  // conditional expression is constant

#include <pystate.h>
#undef toupper
#undef tolower
#undef isspace

// See ticket #1010.  This function runs when a shared lib containing this
// object is loaded.  It makes sure python symbols can be found by extention
// module .so's since on some platforms (notably Ubuntu), they aren't linked
// with libpython even though they depend on it.  If a platform doesn't support
// __attribute__ ((constructor)) this does nothing.  We'll have to deal with
// those as they come up.
#ifndef _WIN32
__attribute__((constructor))
static void loadPython()
{
    std::string libname;

    pdal::Utils::getenv("PDAL_PYTHON_LIBRARY", libname);

// PDAL_PYTHON_LIBRARY below is the result of the cmake FindPython script's
// PYTHON_LIBRARY.

    if (libname.empty())
        libname = PDAL_PYTHON_LIBRARY;
    libname = pdal::FileUtils::getFilename(libname);
    ::dlopen(libname.data(), RTLD_LAZY | RTLD_GLOBAL);
}
#endif

// http://www.linuxjournal.com/article/3641
// http://www.codeproject.com/Articles/11805/Embedding-Python-in-C-C-Part-I
// http://stackoverflow.com/questions/6596016/python-threads-in-c

namespace pdal
{
namespace plang
{
static Environment* g_environment = 0;
//
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
//

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

    if (!Py_IsInitialized())
    {
        PyImport_AppendInittab(const_cast<char*>("redirector"),
            redirector_init);
        Py_Initialize();
    }
    else
    {
        m_redirector.init();
        PyObject* added = PyImport_AddModule("redirector");
        if (!added)
            throw pdal_error("unable to add redirector module!");
    }

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
        Py_ssize_t n = PyList_Size(output);

        for (Py_ssize_t i = 0; i < n; i++)
        {
            PyObject* l = PyList_GetItem(output, i);
            if (!l)
                throw pdal::pdal_error("unable to get list item in getTraceback");
            PyObject* r = PyObject_Repr(l);
            if (!r)
                throw pdal::pdal_error("unable to get repr in getTraceback");
            Py_ssize_t size;
            const char *d = PyUnicode_AsUTF8AndSize(r, &size);
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
        const char *d = PyUnicode_AsUTF8AndSize(r, &size);
        mssg << d;
    }
    else
        mssg << "unknown error that we are unable to get a traceback for."
        "Was it already printed/taken?";

    Py_XDECREF(value);
    Py_XDECREF(type);
    Py_XDECREF(traceback);
    PyErr_Clear();
    return mssg.str();
}

// Returns a new reference.
PyObject *fromMetadata(MetadataNode m)
{
    std::string name = m.name();
    std::string value = m.value();
    std::string type = m.type();
    std::string description = m.description();

    MetadataNodeList children = m.children();
    PyObject *submeta(0);
    if (children.size())
    {
        submeta = PyList_New(0);
        for (MetadataNode& child : children)
            PyList_Append(submeta, fromMetadata(child));
    }
    PyObject *data = PyDict_New();
    PyDict_SetItemString(data, "name", PyUnicode_FromString(name.data()));
    PyDict_SetItemString(data, "value", PyUnicode_FromString(value.data()));
    PyDict_SetItemString(data, "type", PyUnicode_FromString(type.data()));
    PyDict_SetItemString(data, "description",
        PyUnicode_FromString(description.data()));

    if (children.size())
        PyDict_SetItemString(data, "children", submeta);
    return data;
}

std::string readPythonString(PyObject* dict, const std::string& key)
{
    std::stringstream ss;

    PyObject* o = PyDict_GetItemString(dict, key.c_str());
    if (!o)
    {
        std::stringstream oss;
        oss << "Unable to get dictionary item '" << key << "'";
        throw pdal_error(oss.str());
    }

    PyObject* r = PyObject_Str(o);
    if (!r)
        throw pdal::pdal_error("unable to get repr in readPythonString");
    Py_ssize_t size;
    const char *d = PyUnicode_AsUTF8AndSize(r, &size);
    ss << d;

    return ss.str();
}
void addMetadata(PyObject *dict, MetadataNode m)
{
    if (!dict)
    {
        return;
    }

    if (!PyDict_Check(dict))
        throw pdal::pdal_error("'metadata' member must be a dictionary!");

    std::string name = readPythonString(dict, "name");
    std::string value = readPythonString(dict, "value");

    std::string type = readPythonString(dict, "type");
    if (type.empty())
        type = Metadata::inferType(value);

    std::string description = readPythonString(dict, "description");

    PyObject *submeta = PyDict_GetItemString(dict, "children");
    if (submeta)
    {
        if (!PyList_Check(submeta))
            throw pdal::pdal_error("'children' metadata member must be a list!");

        for (Py_ssize_t i = 0; i < PyList_Size(submeta); ++i)
        {
            PyObject* p = PyList_GetItem(submeta, i);
            addMetadata(p, m);
        }
        MetadataNode child = m.addWithType(name, value, type, description);
    }
}

int Environment::getPythonDataType(Dimension::Type t)
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

Dimension::Type Environment::getPDALDataType(int t)
{
    using namespace Dimension;

    switch (t)
    {
    case NPY_FLOAT32:
        return Type::Float;
    case NPY_FLOAT64:
        return Type::Double;
    case NPY_INT8:
        return Type::Signed8;
    case NPY_INT16:
        return Type::Signed16;
    case NPY_INT32:
        return Type::Signed32;
    case NPY_INT64:
        return Type::Signed64;
    case NPY_UINT8:
        return Type::Unsigned8;
    case NPY_UINT16:
        return Type::Unsigned16;
    case NPY_UINT32:
        return Type::Unsigned32;
    case NPY_UINT64:
        return Type::Unsigned64;
    default:
        return Type::None;
    }
    assert(0);

    return Type::None;
}
} // namespace plang
} // namespace pdal
