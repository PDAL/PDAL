/******************************************************************************
* Copyright (c) 2018, Howard Butler, howard@hobu.co
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

#include "NumpyReader.hpp"

#include <pdal/PointView.hpp>
#include <pdal/pdal_features.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Algorithm.hpp>

#include "../plang/Environment.hpp"


std::string toString(PyObject *pname)
{
    std::stringstream mssg;
    PyObject* r = PyObject_Str(pname);
    if (!r)
        throw pdal::pdal_error("couldn't make string representation value");
#if PY_MAJOR_VERSION >= 3
    Py_ssize_t size;
    const char *d = PyUnicode_AsUTF8AndSize(r, &size);
#else
    const char *d = PyString_AsString(r);
#endif
    mssg << d;
    return mssg.str();
}


namespace pdal
{

static PluginInfo const s_info
{
    "readers.numpy",
    "Read data from .npy files.",
    ""
};

struct NumpyReader::Args
{
    std::string module;
    std::string function;
    std::string source;
    std::string fargs;
};

CREATE_SHARED_STAGE(NumpyReader, s_info)

std::string NumpyReader::getName() const { return s_info.name; }

std::ostream& operator << (std::ostream& out,
    const NumpyReader::Order& order)
{
    switch (order)
    {
    case NumpyReader::Order::Row:
        out << "row";
        break;
    case NumpyReader::Order::Column:
        out << "column";
        break;
    }
    return out;
}


std::istream& operator >> (std::istream& in, NumpyReader::Order& order)
{
    std::string s;
    in >> s;

    s = Utils::tolower(s);
    if (s == "row")
        order = NumpyReader::Order::Row;
    else if (s == "column")
        order = NumpyReader::Order::Column;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

NumpyReader::NumpyReader()
    : m_array(nullptr)
    , m_args(new NumpyReader::Args)
{}

NumpyReader::~NumpyReader()
{}

void NumpyReader::setArray(PyObject* array)
{
    plang::Environment::get();
    if (!PyArray_Check(array))
        throw pdal::pdal_error("object provided to setArray is not a python numpy array!");

    m_array = (PyArrayObject*)array;
    Py_XINCREF(m_array);
}

PyArrayObject* load_npy_file(std::string const& filename)
{

    PyObject *py_filename =  PyUnicode_FromString(filename.c_str());
    PyObject *numpy_module = PyImport_ImportModule("numpy");
    if (!numpy_module)
        throw pdal::pdal_error(plang::getTraceback());

    PyObject *numpy_mod_dict = PyModule_GetDict(numpy_module);
    if (!numpy_mod_dict)
        throw pdal::pdal_error(plang::getTraceback());

    PyObject *loads_func = PyDict_GetItemString(numpy_mod_dict, "load");
    if (!loads_func)
        throw pdal::pdal_error(plang::getTraceback());

    PyObject *numpy_args = PyTuple_New(1);
    if (!numpy_args)
        throw pdal::pdal_error(plang::getTraceback());

    if (PyTuple_SetItem(numpy_args, 0, py_filename))
        throw pdal::pdal_error(plang::getTraceback());

    PyObject* array = PyObject_CallObject(loads_func, numpy_args);
    if (!array)
        throw pdal::pdal_error(plang::getTraceback());

    return reinterpret_cast<PyArrayObject*>(array);
}

PyArrayObject* load_npy_script(std::string const& source,
                               std::string const& module,
                               std::string const& function,
                               std::string const& fargs)
{

    MetadataNode m;
    plang::Script script(source, module, function);
    plang::Invocation method(script, m, fargs);

    StringList args = Utils::split(fargs,',');

    PyObject *scriptArgs = PyTuple_New(args.size());
    for(size_t i=0; i < args.size(); ++i)
    {
        PyObject* arg = PyUnicode_FromString(args[i].c_str());
            if (!arg)
                throw pdal_error(plang::getTraceback());
        PyTuple_SetItem(scriptArgs, i, arg);

    }

    PyObject *array = PyObject_CallObject(method.m_function, scriptArgs);
    if (!array)
        throw pdal_error(plang::getTraceback());

    Py_XDECREF(scriptArgs);

    return reinterpret_cast<PyArrayObject*>(array);
}

void NumpyReader::initialize()
{
    plang::Environment::get();
    m_numPoints = 0;
    m_chunkCount = 0;
    m_ndims = 0;

    m_iter = NULL;

    p_data = NULL;
    m_dataptr = NULL;
    m_strideptr = NULL;
    m_innersizeptr = NULL;
    m_dtype = NULL;

    if (m_args->function.size() )
    {
        // Invoke the script and use the returned
        // numpy array
        //
        m_args->source = pdal::FileUtils::readFileIntoString(m_filename);
        m_array = load_npy_script(m_args->source,
                                  m_args->module,
                                  m_args->function,
                                  m_args->fargs);
        if (!PyArray_Check(m_array))
        {
            std::stringstream errMsg;
            errMsg << "Object returned from function '"
                   << m_args->function <<
                   "' in '" << m_filename <<
                   "' is not a Numpy array";
            throw pdal::pdal_error(errMsg.str());
        }
    }
    else if (m_filename.size())
    {
        m_array = load_npy_file(m_filename);
        if (!PyArray_Check(m_array))
            throw pdal::pdal_error("Object in file  '" + m_filename +
                "' is not a Numpy array");

    }
}


void NumpyReader::wakeUpNumpyArray()
{
    // TODO pivot whether we are a 1d, 2d, or named arrays

    if (PyArray_SIZE(m_array) == 0)
        throw pdal::pdal_error("Array cannot be empty!");

    m_iter = NpyIter_New(m_array,
        NPY_ITER_EXTERNAL_LOOP | NPY_ITER_READONLY| NPY_ITER_REFS_OK,
        NPY_KEEPORDER, NPY_NO_CASTING,
        NULL);
    if (!m_iter)
    {
        std::ostringstream oss;

        oss << "Unable to create iterator from array in '"
            << m_filename + "' with traceback: '"
            << plang::getTraceback() <<"'";
        throw pdal::pdal_error(oss.str());
    }

    char* itererr;
    m_iternext = NpyIter_GetIterNext(m_iter, &itererr);
    if (!m_iternext)
    {
        NpyIter_Deallocate(m_iter);
        throw pdal::pdal_error(itererr);
    }

    m_dtype = PyArray_DTYPE(m_array);
    if (!m_dtype)
        throw pdal_error(plang::getTraceback());

    m_ndims = PyArray_NDIM(m_array);
    m_shape = PyArray_SHAPE(m_array);
    if (!m_shape)
        throw pdal_error(plang::getTraceback());

    npy_intp* shape = PyArray_SHAPE(m_array);
    if (!shape)
        throw pdal_error(plang::getTraceback());
    m_numPoints = 1;
    for (int i = 0; i < m_ndims; ++i)
        m_numPoints *= m_shape[i];

    // If the order arg wasn't set, set order based on the internal order of
    // the array.
    if (!m_orderArg->set())
    {
        int flags = PyArray_FLAGS(m_array);
        if (flags & NPY_ARRAY_F_CONTIGUOUS)
            m_order = Order::Column;
        else
            m_order = Order::Row;
    }
}


void NumpyReader::addArgs(ProgramArgs& args)
{
    args.add("dimension", "In an unstructured array, the dimension name to "
        "map to values.", m_defaultDimension, "Intensity");
    m_orderArg = &args.add("order", "Order of dimension interpretation "
        "of the array.  Either 'row'-major (C) or 'column'-major (Fortran)",
        m_order);

    args.add("module", "Python module containing the function to run",
        m_args->module);
    args.add("function", "Function nameto call",
        m_args->function);
    args.add("fargs", "Args to call function with ", m_args->fargs);

}


// Try to map dimension names to existing PDAL dimension names by
// checking them with certain characters removed.
Dimension::Id NumpyReader::registerDim(PointLayoutPtr layout,
    const std::string& name, Dimension::Type pdalType)
{
    Dimension::Id id;

    auto registerName = [&layout, &pdalType, &id](std::string name, char elim)
    {
        if (elim != '\0')
            Utils::remove(name, elim);
        Dimension::Id tempId = Dimension::id(name);
        if (tempId != Dimension::Id::Unknown)
        {
            id = tempId;
            layout->registerDim(id, pdalType);
            return true;
        }
        return false;
    };

    // Try registering the name in various ways.  If that doesn't work,
    // just punt and use the name as is.
    if (!registerName(name, '\0') && !registerName(name, '-') &&
        !registerName(name, ' ') && !registerName(name, '_'))
        id = layout->registerOrAssignDim(name, pdalType);
    return id;
}

namespace
{


Dimension::Type getType(PyArray_Descr *dtype, const std::string& name)
{
    if (!dtype)
        throw pdal_error("Can't fetch data type for numpy field.");

    Dimension::Type pdalType =
        plang::Environment::getPDALDataType(dtype->type_num);
    if (pdalType == Dimension::Type::None)
    {
        std::ostringstream oss;
        oss << "Unable to map dimension '" << name << "' because its "
            "type '" << dtype->type_num <<"' is not mappable to PDAL";
        throw pdal_error(oss.str());
    }
    return pdalType;
}

} // unnamed namespace


void NumpyReader::createFields(PointLayoutPtr layout)
{
    Dimension::Id id;
    Dimension::Type type;
    int offset;

    m_numFields = 0;
    if (m_dtype->fields != Py_None)
        m_numFields = static_cast<int>(PyDict_Size(m_dtype->fields));

    // Array isn't structured - just a bunch of data.
    if (m_numFields <= 0)
    {
        type = getType(m_dtype, m_defaultDimension);
        id = registerDim(layout, m_defaultDimension, type);
        m_fields.push_back({id, type, 0});
    }
    else
    {
        PyObject* names_dict = m_dtype->fields;
        PyObject* names = PyDict_Keys(names_dict);
        PyObject* values = PyDict_Values(names_dict);
        if (!names || !values)
            throw pdal_error("Bad field specification for numpy array layout.");

        for (int i = 0; i < m_numFields; ++i)
        {
            std::string name = toString(PyList_GetItem(names, i));
            PyObject *tup = PyList_GetItem(values, i);
            if (!tup)
                throw pdal_error(plang::getTraceback());

            // Get offset.
            PyObject* offset_o = PySequence_Fast_GET_ITEM(tup, 1);
            if (!offset_o)
                throw pdal_error(plang::getTraceback());
            offset = PyLong_AsLong(offset_o);

            // Get type.
            type = getType((PyArray_Descr *)PySequence_Fast_GET_ITEM(tup, 0),
                name);
            id = registerDim(layout, name, type);
            m_fields.push_back({id, type, offset});
        }
    }
}


void NumpyReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;

    wakeUpNumpyArray();
    createFields(layout);

    m_storeXYZ = true;
    // If we already have an X dimension, we're done.
    for (const Field& field : m_fields)
        if (field.m_id == Id::X || field.m_id == Id::Y || field.m_id == Id::Z)
        {
            m_storeXYZ = false;
            return;
        }

    // We're storing a calculated XYZ, so register the dims.
    layout->registerDim(Id::X, Type::Signed32);
    if (m_ndims > 1)
    {
        layout->registerDim(Id::Y, Type::Signed32);
        if (m_ndims > 2)
            layout->registerDim(Id::Z, Type::Signed32);
    }
    if (m_order == Order::Row)
    {
        m_xIter = m_shape[m_ndims - 1];
        m_xDiv = 1;
        if (m_ndims > 1)
        {
            m_yDiv = 1;
            m_xDiv = m_xIter;
            m_xIter *= m_shape[m_ndims - 2];
            m_yIter = m_shape[m_ndims - 1];
            if (m_ndims > 2)
            {
                m_xDiv = m_xIter;
                m_yDiv = m_yIter;
                m_zDiv = 1;
                m_xIter *= m_shape[m_ndims - 3];
                m_yIter *= m_shape[m_ndims - 2];
                m_zIter = m_shape[m_ndims - 1];
            }
        }
    }
    else
    {
        m_xIter = m_shape[0];
        m_xDiv = 1;
        if (m_ndims > 1)
        {
            m_yIter = m_shape[0] * m_shape[1];
            m_yDiv = m_xIter;
            if (m_ndims > 2)
            {
                m_zIter = m_shape[0] * m_shape[1] * m_shape[2];
                m_zDiv = m_yIter;
            }
        }
    }
}


void NumpyReader::ready(PointTableRef table)
{
    plang::Environment::get()->set_stdout(log()->getLogStream());

    // Set our iterators
    // The location of the data pointer which the iterator may update
    m_dataptr = NpyIter_GetDataPtrArray(m_iter);

    // The location of the stride which the iterator may update
    m_strideptr = NpyIter_GetInnerStrideArray(m_iter);

    // The location of the inner loop size which the iterator may update
    m_innersizeptr = NpyIter_GetInnerLoopSizePtr(m_iter);

    p_data = *m_dataptr;
    m_chunkCount = *m_innersizeptr;
    m_index = 0;

    log()->get(LogLevel::Debug) << "Initializing Numpy array for file '" <<
        m_filename << "'" << std::endl;
    log()->get(LogLevel::Debug) << "numpy inner stride '" <<
        *m_strideptr << "'" << std::endl;
    log()->get(LogLevel::Debug) << "numpy inner stride size '" <<
        *m_innersizeptr << "'" << std::endl;
    log()->get(LogLevel::Debug) << "numpy number of points '" <<
        m_numPoints << "'" << std::endl;
    log()->get(LogLevel::Debug) << "numpy number of dimensions '" <<
        m_ndims << "'" << std::endl;
    for (npy_intp i = 0; i < m_ndims; ++i)
        log()->get(LogLevel::Debug) << "numpy shape dimension number '" <<
            i << "' is '" << m_shape[i] <<"'" << std::endl;
}

bool NumpyReader::nextPoint()
{
    // If we are at the end of this chunk, try to fetch another.  Otherwise,
    // just advance by the stride.
    if (--m_chunkCount == 0)
    {
        // If we can't fetch the next ite
        if (!m_iternext(m_iter))
            return false;
        m_chunkCount = *m_innersizeptr;
        p_data = *m_dataptr;
    }
    else
        p_data += *m_strideptr;
    return true;
}


bool NumpyReader::loadPoint(PointRef& point, point_count_t position)
{
    using namespace Dimension;

    for (const Field& f : m_fields)
        point.setField(f.m_id, f.m_type, (void*)(p_data + f.m_offset));

    if (m_storeXYZ)
    {
        point.setField(Dimension::Id::X, (position % m_xIter) / m_xDiv);
        if (m_ndims > 1)
        {
            point.setField(Dimension::Id::Y, (position % m_yIter) / m_yDiv);
            if (m_ndims > 2)
                point.setField(Dimension::Id::Z, (position % m_zIter) / m_zDiv);
        }
    }

    return (nextPoint());
}


bool NumpyReader::processOne(PointRef& point)
{
    if (m_index >= m_numPoints)
        return false;
    return loadPoint(point, m_index++);
}


point_count_t NumpyReader::read(PointViewPtr view, point_count_t numToRead)
{
    PointId idx = view->size();
    point_count_t numRead(0);

    while (numRead < numToRead)
    {
        PointRef point(*view, idx);
        if (!processOne(point))
            break;
        numRead++;
        idx++;
    }
    return numRead;
}


void NumpyReader::done(PointTableRef)
{
    // Dereference everything we're using

    if (m_iter)
        NpyIter_Deallocate(m_iter);

    Py_XDECREF(m_array);
}


} // namespace pdal


