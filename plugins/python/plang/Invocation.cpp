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

#include <pdal/util/Algorithm.hpp>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define NO_IMPORT_ARRAY
#define PY_ARRAY_UNIQUE_SYMBOL PDAL_ARRAY_API
#include <numpy/arrayobject.h>

namespace
{
int argCount(PyObject *function)
{
    // New object.
    PyObject *module = PyImport_ImportModule("inspect");
    if (!module)
        return false;

    // Owned by module.
    PyObject *dictionary = PyModule_GetDict(module);
    // Owned by dictionary.
    PyObject *getargFunc = PyDict_GetItemString(dictionary, "getargspec");

    // New object.
    PyObject *inArgs = PyTuple_New(1);

    // SetItem takes reference.  Increment so that the function argument isn't
    // lost.
    Py_INCREF(function);
    PyTuple_SetItem(inArgs, 0, function);

    // OutArgs returns a new object.
    PyObject *outArgs = PyObject_CallObject(getargFunc, inArgs);
    // Owned by tuple.
    PyObject *arglist = PyTuple_GetItem(outArgs, (Py_ssize_t)0);

    int count = (int)PyList_Size(arglist);

    Py_DECREF(module);
    Py_DECREF(inArgs);
    Py_DECREF(outArgs);

    return count;
}


pdal::StringList dictKeys(PyObject *arrays)
{
    pdal::StringList keys;
    PyObject *key, *value;
    Py_ssize_t pos = 0;

    if (arrays)
    {
        while (PyDict_Next(arrays, &pos, &key, &value))
        {
            PyObject *utf8 = PyUnicode_AsUTF8String(key);
            const char* p = PyBytes_AsString(utf8);
            if (p)
                keys.push_back(p);
            Py_DECREF(utf8);
        }
    }
    return keys;
}



// Add an object to the module if it doesn't aready have it.
// Module takes ownership of the added object.
void addGlobalObject(PyObject* module, PyObject* obj, const std::string& name)
{
    if (!module || !obj)
        return;

    if (PyModule_AddObject(module, name.c_str(), obj))
        throw pdal::pdal_error("Unable to set" + name + "global");
}

} // unnamed namespace

namespace pdal
{
namespace plang
{
Invocation::Invocation(const Script& script, MetadataNode m,
        const std::string& pdalArgs) :
    m_script(script), m_inputMetadata(m), m_pdalargs(pdalArgs)
{
    Environment::get();
    compile();
}


void Invocation::compile()
{
    PyObject *bytecode = Py_CompileString(m_script.source(), m_script.module(),
        Py_file_input);
    if (!bytecode)
        throw pdal_error(getTraceback());

    m_module = PyImport_ExecCodeModule(const_cast<char*>(m_script.module()),
        bytecode);

    Py_DECREF(bytecode);
    if (!m_module)
        throw pdal_error(getTraceback());

    PyObject *dictionary = PyModule_GetDict(m_module);
    if (!dictionary)
        throw pdal_error("Unable to fetch module dictionary");

    // Owned by the module through the dictionary.
    m_function = PyDict_GetItemString(dictionary, m_script.function());
    if (!m_function)
    {
        std::ostringstream oss;
        oss << "unable to find target function '" << m_script.function() <<
            "' in module.";
        throw pdal_error(oss.str());
    }

    if (!PyCallable_Check(m_function))
        throw pdal_error(getTraceback());
}


// creates a Python variable pointing to a (one dimensional) C array
// Returns a new reference to the numpy array.
PyObject * Invocation::addArray(std::string const& name, uint8_t* data,
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
    return PyArray_New(&PyArray_Type, nd, dims, pyDataType, strides, data,
        0, flags, NULL);
}


bool Invocation::execute(PointViewPtr& v, MetadataNode stageMetadata)
{
    if (!m_module)
        throw pdal_error("No code has been compiled");

    PyObject *inArrays = prepareData(v);
    PyObject *outArrays(nullptr);

    // New object.
    Py_ssize_t numArgs = argCount(m_function);
    PyObject *scriptArgs = PyTuple_New(numArgs);

    if (numArgs > 2)
        throw pdal_error("Only two arguments -- ins and outs "
            "numpy arrays -- can be passed!");

    // inArrays and outArrays are owned by scriptArgs here.
    PyTuple_SetItem(scriptArgs, 0, inArrays);
    if (numArgs > 1)
    {
        outArrays = PyDict_New();
        PyTuple_SetItem(scriptArgs, 1, outArrays);
    }

    PyObject *scriptResult = PyObject_CallObject(m_function, scriptArgs);
    if (!scriptResult)
        throw pdal_error(getTraceback());
    if (!PyBool_Check(scriptResult))
        throw pdal_error("User function return value not boolean.");

    PyObject *maskArray = PyDict_GetItemString(outArrays, "Mask");
    if (maskArray)
    {
        if (PyDict_Size(outArrays) > 1)
            throw pdal_error("'Mask' output array must be the only "
                "output array.");
        v = maskData(v, maskArray);
    }
    else
        extractData(v, outArrays);
    extractMetadata(stageMetadata);

    // This looks weird, but booleans are implemented as static objects,
    // allowing this comparison (Py_True is a pointer to the "true" object.)
    bool ok = scriptResult == Py_True;

    Py_DECREF(scriptArgs);
    Py_DECREF(scriptResult);
    return ok;
}


// Returns a pointer to the data in Numpy array, 'array'.
void *Invocation::extractArray(PyObject *array, std::string const& name,
    Dimension::Type t, size_t& num_elements)
{
    if (!array)
        throw pdal_error("plang output variable '" + name + "' not found.");
    if (!PyArray_Check(array))
        throw pdal_error("Plang output variable  '" + name +
            "' is not a numpy array");

    PyArrayObject* arr = (PyArrayObject*)array;

    npy_intp zero = 0;
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
        throw pdal_error(oss.str());
    }

    using namespace Dimension;
    BaseType b = Dimension::base(t);
    if (dtype->kind == 'i' && b != BaseType::Signed)
    {
        std::ostringstream oss;
        oss << "dtype of array has a signed integer type but the " <<
            "dimension data type of '" << name <<
            "' is not pdal::Signed.";
        throw pdal_error(oss.str());
    }

    if (dtype->kind == 'u' && b != BaseType::Unsigned)
    {
        std::ostringstream oss;
        oss << "dtype of array has a unsigned integer type but the " <<
            "dimension data type of '" << name <<
            "' is not pdal::Unsigned.";
        throw pdal_error(oss.str());
    }

    if (dtype->kind == 'f' && b != BaseType::Floating)
    {
        std::ostringstream oss;
        oss << "dtype of array has a float type but the " <<
            "dimension data type of '" << name << "' is not pdal::Floating.";
        throw pdal_error(oss.str());
    }
    return PyArray_GetPtr(arr, &zero);
}


// Wrap a string in a python JSON object.
// Returns a new reference.
PyObject* getPyJSON(std::string const& s)
{
    if (s.empty())
        return nullptr;

    // New ref. Transferred to tuple below.
    PyObject* raw_json = PyUnicode_FromString(s.c_str());
    if (!raw_json)
        throw pdal_error(getTraceback());

    PyObject* json_module = PyImport_ImportModule("json");
    if (!json_module)
        throw pdal_error(getTraceback());

    // Owned by module.
    PyObject* json_mod_dict = PyModule_GetDict(json_module);
    if (!json_mod_dict)
        throw pdal_error(getTraceback());

    // Owned by dict.
    PyObject* loads_func = PyDict_GetItemString(json_mod_dict, "loads");
    if (!loads_func)
        throw pdal_error(getTraceback());

    // New ref.
    PyObject* json_args = PyTuple_New(1);
    if (!json_args)
        throw pdal_error(getTraceback());

    // Ref transferred to tuple.
    if (PyTuple_SetItem(json_args, 0, raw_json))
        throw pdal_error(getTraceback());

    // New ref.
    PyObject* strict = PyDict_New();
    if (!strict)
        throw pdal_error(getTraceback());

    if (PyDict_SetItemString(strict, "strict", Py_False))
        throw pdal_error(getTraceback());

    // New reference.
    PyObject* json = PyObject_Call(loads_func, json_args, strict);

    // Yes, these will leak with a "throw" above.  Not worth doing anything
    // about it.
    Py_DECREF(json_args);
    Py_DECREF(strict);
    if (!json)
        throw pdal_error(getTraceback());

    return json;
}


// Returns a new reference to a dictionary of numpy arrays/names.
PyObject *Invocation::prepareData(PointViewPtr& view)
{
    PointLayoutPtr layout(view->m_pointTable.layout());
    Dimension::IdList const& dims = layout->dims();

    PyObject *arrays = PyDict_New();
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        Dimension::Id d = *di;
        const Dimension::Detail *dd = layout->dimDetail(d);
        void *data = malloc(dd->size() * view->size());
        char *p = (char *)data;
        for (PointId idx = 0; idx < view->size(); ++idx)
        {
            view->getFieldInternal(d, idx, (void *)p);
            p += dd->size();
        }
        std::string name = layout->dimName(*di);
        PyObject *array = addArray(name, (uint8_t *)data, dd->type(),
            view->size());
        PyDict_SetItemString(arrays, name.c_str(), array);

        m_pyInputArrays.push_back(array);  // Hold for de-referencing.
        m_numpyBuffers.push_back(data);    // Hold for deallocation
    }

    MetadataNode layoutMeta = view->layout()->toMetadata();
    MetadataNode srsMeta = view->spatialReference().toMetadata();

    addGlobalObject(m_module, plang::fromMetadata(m_inputMetadata), "metadata");
    addGlobalObject(m_module, getPyJSON(m_pdalargs), "pdalargs");
    addGlobalObject(m_module, getPyJSON(Utils::toJSON(layoutMeta)), "schema");
    addGlobalObject(m_module, getPyJSON(Utils::toJSON(srsMeta)),
        "spatialreference");

    return arrays;
}


PointViewPtr Invocation::maskData(PointViewPtr& view, PyObject *maskArray)
{
    PointViewPtr outView = view->makeNew();

    PyArrayObject* arr = (PyArrayObject*)maskArray;

    npy_intp zero = 0;
    PyArray_Descr *dtype = PyArray_DESCR(arr);

    npy_intp nDims = PyArray_NDIM(arr);
    if (nDims != 1 || dtype->kind != 'b')
        throw pdal_error("Mask array must be a vector of boolean values.");

    npy_intp* shape = PyArray_SHAPE(arr);
    point_count_t arraySize = (point_count_t)*shape;

    if (arraySize != view->size())
        throw pdal_error("Mask array much be the same length as the input "
            "data.");

    char *p = (char *)PyArray_GetPtr(arr, &zero);
    point_count_t idx = 0;
    while (idx < arraySize)
    {
        if (*p++)
            outView->appendPoint(*view, idx);
        idx++;
    }
    return outView;
}


void Invocation::extractData(PointViewPtr& view, PyObject *arrays)
{

    // for each entry in the script's outs dictionary,
    // look up that entry's name in the schema and then
    // copy the data into the right dimension spot in the
    // buffer

    StringList names = dictKeys(arrays);

    PointLayoutPtr layout(view->m_pointTable.layout());

    for (auto& name : names)
        if (layout->findDim(name) == Dimension::Id::Unknown)
            throw pdal_error("Can't set numpy array '" + name +
                "' as output.  Dimension not registered.");

    Dimension::IdList const& dims = layout->dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        Dimension::Id d = *di;
        const Dimension::Detail *dd = layout->dimDetail(d);
        std::string name = layout->dimName(*di);
        if (!Utils::contains(names, name))
            continue;

        size_t size = dd->size();
        size_t arrSize(0);
        PyObject* numpyArray = PyDict_GetItemString(arrays, name.c_str());
        void *data = extractArray(numpyArray, name, dd->type(), arrSize);
        char *p = (char *)data;
        for (PointId idx = 0; idx < arrSize; ++idx)
        {
            view->setField(d, dd->type(), idx, (void *)p);
            p += size;
        }
    }

    // Clean up the input buffers.
    for (void *b : m_numpyBuffers)
        free(b);
    m_numpyBuffers.clear();
}


void Invocation::extractMetadata(MetadataNode stageMetadata)
{
    PyObject *key = PyUnicode_FromString("metadata");
    PyObject *dictionary = PyModule_GetDict(m_module);
    PyObject *pyMeta = PyDict_GetItem(dictionary, key);
    if (pyMeta)
        addMetadata(pyMeta, stageMetadata);
    Py_DECREF(key);
}

} // namespace plang
} // namespace pdal
