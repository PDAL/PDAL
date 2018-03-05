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

#include <Python.h>
#undef toupper
#undef tolower
#undef isspace

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#define NO_IMPORT_ARRAY
#define PY_ARRAY_UNIQUE_SYMBOL PDAL_ARRAY_API
#include <numpy/arrayobject.h>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.numpy",
    "Read data from .npy files.",
    "" );

CREATE_SHARED_PLUGIN(1, 0, NumpyReader, Reader, s_info)

std::string NumpyReader::getName() const { return s_info.name; }


PyObject* load_npy(std::string const& filename)
{

    PyObject* py_filename =  PyString_FromString(filename.c_str());
    PyObject* numpy_module = PyImport_ImportModule("numpy");
    if (!numpy_module)
        throw pdal::pdal_error(plang::getTraceback());

    PyObject* numpy_mod_dict = PyModule_GetDict(numpy_module);
    if (!numpy_mod_dict)
        throw pdal::pdal_error(plang::getTraceback());

    PyObject* loads_func = PyDict_GetItemString(numpy_mod_dict, "load");
    if (!loads_func)
        throw pdal::pdal_error(plang::getTraceback());

    PyObject* numpy_args = PyTuple_New(1);
    if (!numpy_args)
        throw pdal::pdal_error(plang::getTraceback());

    int success = PyTuple_SetItem(numpy_args, 0, py_filename);
    if (success != 0)
        throw pdal::pdal_error(plang::getTraceback());

    PyObject* array = PyObject_CallObject(loads_func, numpy_args);
    if (!array)
        throw pdal::pdal_error(plang::getTraceback());

    return array;
}


void NumpyReader::initialize()
{
    plang::Environment::get();
    m_array = load_npy(m_filename);
    if (!PyArray_Check(m_array))
        throw pdal::pdal_error("Object in file  '" + m_filename +
            "' is not a numpy array");
}



void NumpyReader::addArgs(ProgramArgs& args)
{
}



void NumpyReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;

    // TODO pivot whether we are a 1d, 2d, or named arrays

    PyArrayObject* arr = (PyArrayObject*)m_array;
    PyArray_Descr *dtype = PyArray_DTYPE(arr);
    if (!dtype)
        throw pdal::pdal_error(plang::getTraceback());

    // Get length of fields
    Py_ssize_t count = PyDict_Size(dtype->fields);
    log()->get(LogLevel::Debug) << "Adding " << count <<" dimensions" << std::endl;
    std::cerr<< "Adding " << count <<" dimensions" << std::endl;


//     PyObject* dtype = PyObject_GetAttrString(m_array, "dtype");
//     if (!dtype)
//         throw pdal::pdal_error(plang::getTraceback());
//     PyObject* names = PyObject_GetAttrString(dtype, "names");
//     if (!names)
//         throw pdal::pdal_error(plang::getTraceback());
//
//     // Get length of names list
//     Py_ssize_t count = PySequence_Size(names);
//     log()->get(LogLevel::Debug) << "Adding " << count <<" dimensions" << std::endl;
//     std::cerr<< "Adding " << count <<" dimensions" << std::endl;
//
//     for (int i=0; i < (int) count; i++)
//     {
//         char* name = PyString_AsString(PySequence_GetItem(names, i));
//         Id id = Dimension::id(name);
//         if (id != Id::Unknown)
//         {
//
//         }
//         log()->get(LogLevel::Debug) << "Adding dimension '" << name <<"'" << std::endl;
//
//     }
//     layout->registerDim(Id::X, Type::Double);


}


void NumpyReader::ready(PointTableRef table)
{
    // wake up python


    static_cast<plang::Environment*>(plang::Environment::get())->set_stdout(log()->getLogStream());
    log()->get(LogLevel::Debug) << "Initializing Numpy array '" << m_filename <<"'" << std::endl;
}


bool NumpyReader::processOne(PointRef& point)
{
//     if (m_index >= getNumPoints())
//         return false;
//
//     size_t pointLen = m_header.pointLen();

    return true;

}

point_count_t NumpyReader::read(PointViewPtr view, point_count_t count)
{
//     if (eof())
//         return 0;

//     log()->get(LogLevel::Debug4) << "read called with "
//         "PointView filled to " << view->size() << " points" <<
//         std::endl;

//     point_count_t totalNumRead = 0;
//     if (! b_doneQuery)
//     {
//         // read first patch
//         m_session->query(m_query);
//         validateQuery();
//         b_doneQuery = true;
//         totalNumRead = readPatch(view, count);
//     }
//
//     int patch_count(0);
//     while (totalNumRead < count)
//     {
//         if (m_patch->remaining == 0)
//         {
//             if (!nextBuffer())
//             {
//                 m_at_end = true;
//                 return totalNumRead;
//             }
//         }
//         PointId bufBegin = view->size();
//         point_count_t numRead = readPatch(view, count - totalNumRead);
//         PointId bufEnd = bufBegin + numRead;
//         totalNumRead += numRead;
//         patch_count++;
//     }
//     return totalNumRead;
return 0;
}

void NumpyReader::done(PointTableRef)
{
}


} // namespace pdal
