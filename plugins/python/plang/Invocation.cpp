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

#include "Invocation.hpp"

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
    PyObject *arglist = PyTuple_GetItem(outArgs, (Py_ssize_t)0);
    return (int)PyList_Size(arglist);
}
}

namespace pdal
{
namespace plang
{
Invocation::Invocation(const Script& script)
    : m_script(script)
    , m_bytecode(NULL)
    , m_module(NULL)
    , m_dictionary(NULL)
    , m_function(NULL)
    , m_varsIn(NULL)
    , m_varsOut(NULL)
    , m_scriptArgs(NULL)
    , m_scriptResult(NULL)
    , m_metadata_PyObject(NULL)
    , m_schema_PyObject(NULL)
    , m_srs_PyObject(NULL)
    , m_pdalargs_PyObject(NULL)
{
    plang::Environment::get();
    resetArguments();
}

Invocation::~Invocation()
{
    cleanup();
}

void Printobject(PyObject* o)
{
    PyObject* r = PyObject_Repr(o);
    if (!r)
        throw pdal::pdal_error("couldn't make string representation of traceback value");
    Py_ssize_t size;
    const char *d = PyUnicode_AsUTF8AndSize(r, &size);
    std::cout << "raw_json" << d << std::endl;
}
void Invocation::compile()
{
    Py_XDECREF(m_bytecode);
    m_bytecode = Py_CompileString(m_script.source(), m_script.module(),
        Py_file_input);
    if (!m_bytecode)
        throw pdal::pdal_error(getTraceback());

    Py_INCREF(m_bytecode);

    Py_XDECREF(m_module);
    m_module = PyImport_ExecCodeModule(const_cast<char*>(m_script.module()),
        m_bytecode);
    if (!m_module)
        throw pdal::pdal_error(getTraceback());
    Py_INCREF(m_module);

    Py_XDECREF(m_dictionary);
    m_dictionary = PyModule_GetDict(m_module);
    if (!m_dictionary)
    {
        std::ostringstream oss;
        oss << "unable to fetch module dictionary";
        throw pdal::pdal_error(oss.str());
    }
    Py_INCREF(m_dictionary);

    // PyDict_GetItemString returns borrowed reference
    // we need to increment the count after we fetch this
    // to keep it around
    m_function = PyDict_GetItemString(m_dictionary,
        m_script.function());
    if (!m_function)
    {
        std::ostringstream oss;
        oss << "unable to find target function '" << m_script.function() <<
            "' in module.";
        throw pdal::pdal_error(oss.str());
    }
    Py_INCREF(m_function);

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
    Py_XDECREF(m_module);

    //	Py_XDECREF(m_function);
    Py_XDECREF(m_dictionary);

    Py_XDECREF(m_metadata_PyObject);
    Py_XDECREF(m_srs_PyObject);
    Py_XDECREF(m_pdalargs_PyObject);
}

void Invocation::resetArguments()
{
    cleanup();
    m_varsIn = PyDict_New();
    m_varsOut = PyDict_New();
}

void Invocation::insertArgument(std::string const& name, uint8_t* data,
    Dimension::Type t, point_count_t count)
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
    Dimension::Type t, size_t& num_elements)
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

    npy_intp nDims = PyArray_NDIM(arr);
    npy_intp* shape = PyArray_SHAPE(arr);
    npy_intp nPoints(1);
    for (int i = 0; i < nDims; ++i)
        nPoints *= shape[i];

    num_elements = (size_t)nPoints;

    if (static_cast<uint32_t>(dtype->elsize) != Dimension::size(t))
    {
        std::ostringstream oss;
        oss << "dtype of array has size " << dtype->elsize
            << " but PDAL dimension '" << name << "' has byte size of "
            << Dimension::size(t) << " bytes.";
        throw pdal::pdal_error(oss.str());
    }

    using namespace Dimension;
    BaseType b = Dimension::base(t);
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
        p = PyBytes_AsString(PyUnicode_AsUTF8String(key));
        if (p)
            names.push_back(p);
    }
}

bool Invocation::hasOutputVariable(const std::string& name) const
{
    return (PyDict_GetItemString(m_varsOut, name.c_str()) != NULL);
}

PyObject* addGlobalObject(PyObject* module,
    PyObject* obj,
    const std::string& name)
{
    PyObject* output(NULL);
    PyObject* mod_vars = PyModule_GetDict(module);
    if (!mod_vars)
        throw pdal::pdal_error("Unable to get module dictionary");

    PyObject* nameObj = PyUnicode_FromString(name.c_str());
    if (PyDict_Contains(mod_vars, nameObj) == 1)
        output = PyDict_GetItem(mod_vars, nameObj);
    else
    {
        int success = PyModule_AddObject(module, name.c_str(), obj);
        if (success)
            throw pdal::pdal_error("unable to set" + name + "global");
        Py_INCREF(obj);
        output = obj;
    }
    return output;
}

bool Invocation::execute()
{
    if (!m_bytecode)
        throw pdal::pdal_error("No code has been compiled");

    Py_INCREF(m_varsIn);
    Py_ssize_t numArgs = argCount(m_function);
    m_scriptArgs = PyTuple_New(numArgs);

    if (numArgs > 2)
        throw pdal::pdal_error("Only two arguments -- ins and outs "
            "numpy arrays -- can be passed!");

    PyTuple_SetItem(m_scriptArgs, 0, m_varsIn);
    if (numArgs > 1)
    {
        Py_INCREF(m_varsOut);
        PyTuple_SetItem(m_scriptArgs, 1, m_varsOut);
    }

    int success(0);

    if (m_metadata_PyObject)
    {
        addGlobalObject(m_module, m_metadata_PyObject, "metadata");
    }

    if (m_schema_PyObject)
    {
        addGlobalObject(m_module, m_schema_PyObject, "schema");
    }

    if (m_srs_PyObject)
    {
        addGlobalObject(m_module, m_srs_PyObject, "spatialreference");
    }

    if (m_pdalargs_PyObject)
    {
        addGlobalObject(m_module, m_pdalargs_PyObject, "pdalargs");
    }

    m_scriptResult = PyObject_CallObject(m_function, m_scriptArgs);
    if (!m_scriptResult)
        throw pdal::pdal_error(getTraceback());
    if (!PyBool_Check(m_scriptResult))
        throw pdal::pdal_error("User function return value not boolean.");

    PyObject* b = PyUnicode_FromString("metadata");
    if (PyDict_Contains(m_dictionary, PyUnicode_FromString("metadata")) == 1)
        m_metadata_PyObject = PyDict_GetItem(m_dictionary, b);

    return (m_scriptResult == Py_True);
}

PyObject* getPyJSON(std::string const& s)
{
    PyObject* raw_json = PyUnicode_FromString(s.c_str());
    if (!raw_json)
        throw pdal::pdal_error(getTraceback());

    PyObject* json_module = PyImport_ImportModule("json");
    if (!json_module)
        throw pdal::pdal_error(getTraceback());

    PyObject* json_mod_dict = PyModule_GetDict(json_module);
    if (!json_mod_dict)
        throw pdal::pdal_error(getTraceback());

    PyObject* loads_func = PyDict_GetItemString(json_mod_dict, "loads");
    if (!loads_func)
        throw pdal::pdal_error(getTraceback());
    Py_INCREF(loads_func);

    PyObject* json_args = PyTuple_New(1);
    if (!json_args)
        throw pdal::pdal_error(getTraceback());

    int bFail = PyTuple_SetItem(json_args, 0, raw_json);
    if (bFail)
        throw pdal::pdal_error(getTraceback());

    PyObject* strict = PyDict_New();
    if (!strict)
        throw pdal::pdal_error(getTraceback());

    bFail = PyDict_SetItemString(strict, "strict", Py_False);
    if (bFail)
        throw pdal::pdal_error(getTraceback());

    PyObject* json = PyObject_Call(loads_func, json_args, strict);
    if (!json)
        throw pdal::pdal_error(getTraceback());
    return json;
}

void Invocation::setKWargs(std::string const& s)
{
    Py_XDECREF(m_pdalargs_PyObject);
    m_pdalargs_PyObject = getPyJSON(s);
}

void Invocation::begin(PointView& view, MetadataNode m)
{
    PointLayoutPtr layout(view.m_pointTable.layout());
    Dimension::IdList const& dims = layout->dims();

    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        Dimension::Id d = *di;
        const Dimension::Detail *dd = layout->dimDetail(d);
        void *data = malloc(dd->size() * view.size());
        m_buffers.push_back(data);  // Hold pointer for deallocation
        char *p = (char *)data;
        for (PointId idx = 0; idx < view.size(); ++idx)
        {
            view.getFieldInternal(d, idx, (void *)p);
            p += dd->size();
        }
        std::string name = layout->dimName(*di);
        insertArgument(name, (uint8_t *)data, dd->type(), view.size());
    }

    // Put pipeline 'metadata' variable into module scope
    Py_XDECREF(m_metadata_PyObject);
    m_metadata_PyObject = plang::fromMetadata(m);

    // Put 'schema' dict into module scope
    MetadataNode s = view.layout()->toMetadata();
    std::ostringstream ostrm;
    Utils::toJSON(s, ostrm);
    Py_XDECREF(m_schema_PyObject);
    m_schema_PyObject = getPyJSON(ostrm.str());
    ostrm.str("");

    MetadataNode srs = view.spatialReference().toMetadata();
    Utils::toJSON(srs, ostrm);
    Py_XDECREF(m_srs_PyObject);
    m_srs_PyObject = getPyJSON(ostrm.str());
}

void Invocation::end(PointView& view, MetadataNode m)
{
    // for each entry in the script's outs dictionary,
    // look up that entry's name in the schema and then
    // copy the data into the right dimension spot in the
    // buffer

    std::vector<std::string> names;
    getOutputNames(names);

    PointLayoutPtr layout(view.m_pointTable.layout());
    Dimension::IdList const& dims = layout->dims();

    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        Dimension::Id d = *di;
        const Dimension::Detail *dd = layout->dimDetail(d);
        std::string name = layout->dimName(*di);
        auto found = std::find(names.begin(), names.end(), name);
        if (found == names.end()) continue; // didn't have this dim in the names

        assert(name == *found);
        assert(hasOutputVariable(name));

        size_t size = dd->size();
        size_t arrSize(0);
        void *data = extractResult(name, dd->type(), arrSize);
        char *p = (char *)data;
        for (PointId idx = 0; idx < arrSize; ++idx)
        {
            view.setField(d, dd->type(), idx, (void *)p);
            p += size;
        }
    }
    for (auto bi = m_buffers.begin(); bi != m_buffers.end(); ++bi)
        free(*bi);
    m_buffers.clear();
    if (m_metadata_PyObject)
        addMetadata(m_metadata_PyObject, m);
}
} // namespace plang
} // namespace pdal
