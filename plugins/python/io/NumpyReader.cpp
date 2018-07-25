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
#include <pdal/util/Algorithm.hpp>

#include "../plang/Environment.hpp"

namespace pdal
{

static PluginInfo const s_info 
{
    "readers.numpy",
    "Read data from .npy files.",
    ""
};

CREATE_SHARED_STAGE(NumpyReader, s_info)

std::string NumpyReader::getName() const { return s_info.name; }


PyArrayObject* load_npy(std::string const& filename)
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


void NumpyReader::initialize()
{
    plang::Environment::get();
    m_numPoints = 0;
    m_chunkCount = 0;
    m_ndims = 0;

    m_iter = NULL;
    m_array = NULL;

    p_data = NULL;
    m_dataptr = NULL;
    m_strideptr = NULL;
    m_innersizeptr = NULL;

    m_array = load_npy(m_filename);
    if (!PyArray_Check(m_array))
        throw pdal::pdal_error("Object in file  '" + m_filename +
            "' is not a numpy array");
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
}


void NumpyReader::addArgs(ProgramArgs& args)
{
    args.add("dimension", "In a 2-D array, the dimension name to map to "
        "values.", m_defaultDimension, "Intensity");
    args.add("x", "In a 2-D array, the dimension number to map to X",
        m_xDimNum, size_t(0));
    args.add("assign_z", "Assign Z dimension to a single given value",
        m_assignZ);
}


// Try to map dimension names to existing PDAL dimension names by
// checking them with certain characters removed.
void NumpyReader::registerDim(PointLayoutPtr layout, const std::string& name,
    Dimension::Type pdalType)
{
    auto registerName = [this, &layout, &pdalType](std::string name, char elim)
    {
        if (elim != '\0')
            Utils::remove(name, elim);
        Dimension::Id id = Dimension::id(name);
        if (id != Dimension::Id::Unknown)
        {
            layout->registerDim(id, pdalType);
            m_ids.push_back(id);
            return true;
        }
        return false;
    };

    // Try registering the name in various ways.  If that doesn't work,
    // just punt and use the name as is.
    if (!registerName(name, '\0') && !registerName(name, '-') &&
        !registerName(name, ' ') && !registerName(name, '_'))
        m_ids.push_back(layout->registerOrAssignDim(name, pdalType));
}


// Ready an array of dimension 1.
void NumpyReader::prepareFieldsArray(PointLayoutPtr layout)
{
    m_numFields = PyDict_Size(m_dtype->fields);
    PyObject* names_dict = m_dtype->fields;
    PyObject* names = PyDict_Keys(names_dict);
    PyObject* values = PyDict_Values(names_dict);
    if (!names || !values)
        throw pdal::pdal_error(plang::getTraceback());

    for (int i = 0; i < m_numFields; ++i)
    {
        // Get the dimension name and dtype
        PyObject* pname = PyList_GetItem(names, i);
        if (!pname)
            throw pdal::pdal_error(plang::getTraceback());

        const char* cname(0);
#if PY_MAJOR_VERSION >= 3
        cname = PyBytes_AsString(PyUnicode_AsUTF8String(pname));
#else
        cname = PyString_AsString(pname);
#endif
        std::string name(cname);

        PyObject* tup = PyList_GetItem(values, i);
        if (!tup)
            throw pdal_error(plang::getTraceback());

        // Get offset.
        PyObject* offset_o = PySequence_Fast_GET_ITEM(tup, 1);
        if (!offset_o)
            throw pdal_error(plang::getTraceback());
        m_offsets.push_back(PyLong_AsLong(offset_o));

        // Get size.
        PyObject* dt = PySequence_Fast_GET_ITEM(tup, 0);
        if (!dt)
            throw pdal_error(plang::getTraceback());
        PyArray_Descr* dtype = (PyArray_Descr*)dt;
        m_sizes.push_back(dtype->elsize);

        // Get PDAL datatype.
        Dimension::Type p_type =
            plang::Environment::getPDALDataType(dtype->type_num);
        if (p_type == Dimension::Type::None)
        {
            std::ostringstream oss;
            oss << "Unable to map dimension '" << name << "' because its "
                "type '" << dtype->type_num <<"' is not mappable to PDAL";
            throw pdal_error(oss.str());
        }
        m_types.push_back(p_type);

        // Get dimension.
        registerDim(layout, name, p_type);
    }
}


void NumpyReader::prepareRasterArray(PointLayoutPtr layout)
{
    using namespace Dimension;

    Dimension::Type p_type =
        plang::Environment::getPDALDataType(m_dtype->type_num);
    if (p_type == Dimension::Type::None)
    {
        std::ostringstream oss;
        oss << "Unable to map raster dimension because its type '" <<
            m_dtype->type_num <<"' is not mappable to PDAL";
        throw pdal_error(oss.str());
    }
    m_types.push_back(p_type);

    // X is dimension 1
    // Y is dimension 2
    layout->registerDim(Id::X, Type::Signed32);
    layout->registerDim(Id::Y, Type::Signed32);

    if (m_assignZ != 0.0f)
        layout->registerDim(Id::Z, Type::Signed32);
    Dimension::Id id = layout->registerOrAssignDim(m_defaultDimension,
        m_types[0]);

    m_ids.push_back(id);
    m_sizes.push_back(layout->dimSize(id));
}


void NumpyReader::addDimensions(PointLayoutPtr layout)
{
    wakeUpNumpyArray();

    switch (m_ndims)
    {
    case 1:
        prepareFieldsArray(layout);
        break;
    case 2:
        prepareRasterArray(layout);
        break;
    default:
        throw pdal_error("Numpy arrays of dimension > 2 are currently "
            "unsupported.");
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


bool NumpyReader::loadPoint(PointRef& point, point_count_t position )
{
    using namespace Dimension;

    npy_intp stride = *m_strideptr;

    if (m_ndims == 1)
    {
        for (size_t dim = 0; dim < m_ids.size(); ++dim)
            point.setField(m_ids[dim], m_types[dim],
                (void*) (p_data + m_offsets[dim]));
    }
    else if (m_ndims == 2)
    {
        point.setField(pdal::Dimension::Id::X, position % m_shape[m_xDimNum]);
        point.setField(pdal::Dimension::Id::Y, position / m_shape[m_xDimNum]);
        if (m_assignZ != 0.0f)
            point.setField(Id::Z, m_assignZ);
        point.setField(m_ids[0], m_types[0], (void*) (p_data));
    }

    p_data += stride;
    m_chunkCount--;

    bool more = (m_chunkCount >= 0);
    if (!more)
    {
        more = (bool)m_iternext(m_iter);
        m_chunkCount = *m_innersizeptr;
        p_data = *m_dataptr;
    }
    return more;
}


bool NumpyReader::processOne(PointRef& point)
{
    if (m_index >= getNumPoints())
        return false;
    return loadPoint(point, m_index++);
}


point_count_t NumpyReader::getNumPoints() const
{
    if (m_array)
        return (point_count_t) m_numPoints;
    else
        throw pdal::pdal_error("Numpy array not initialized!");
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


