/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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

#include <pdal/plang/Array.hpp>
#include <pdal/plang/Environment.hpp>

#include <algorithm>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127) // conditional expression is constant
#endif

#include <Python.h>
#undef toupper
#undef tolower
#undef isspace


#define PY_ARRAY_UNIQUE_SYMBOL PDALARRAY_ARRAY_API
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

namespace pdal
{
namespace plang
{

Array::Array()
    : m_py_array(0)
{
    auto initNumpy = []()
    {
#undef NUMPY_IMPORT_ARRAY_RETVAL
#define NUMPY_IMPORT_ARRAY_RETVAL
        import_array();
    };
    initNumpy();
}


Array::~Array()
{
    cleanup();
}

void Array::cleanup()
{
    PyObject* p = (PyObject*)(m_py_array);
    Py_XDECREF(p);
    m_data_array.reset();
}
PyObject* Array::buildNumpyDescription(PointViewPtr view) const
{

    // Build up a numpy dtype dictionary
    //
    // {'formats': ['f8', 'f8', 'f8', 'u2', 'u1', 'u1', 'u1', 'u1', 'u1', 'f4', 'u1', 'u2', 'f8', 'u2', 'u2', 'u2'],
    // 'names': ['X', 'Y', 'Z', 'Intensity', 'ReturnNumber', 'NumberOfReturns',
    // 'ScanDirectionFlag', 'EdgeOfFlightLine', 'Classification',
    // 'ScanAngleRank', 'UserData', 'PointSourceId', 'GpsTime', 'Red', 'Green',
    // 'Blue']}
    //
    std::stringstream oss;
    Dimension::IdList dims = view->dims();

    PyObject* dict = PyDict_New();
    PyObject* sizes = PyList_New(dims.size());
    PyObject* formats = PyList_New(dims.size());
    PyObject* titles = PyList_New(dims.size());

    for (Dimension::IdList::size_type i=0; i < dims.size(); ++i)
    {
        Dimension::Id::Enum id = (dims[i]);
        Dimension::Type::Enum t = view->dimType(id);
        npy_intp stride = view->dimSize(id);

        std::string name = view->dimName(id);

        std::string kind("i");
        Dimension::BaseType::Enum b = Dimension::base(t);
        if (b == Dimension::BaseType::Unsigned)
            kind = "u";
        else if (b == Dimension::BaseType::Floating)
            kind = "f";
        else
        {
            std::stringstream o;
            oss << "unable to map kind '" << kind <<"' to PDAL dimension type";
            throw pdal::pdal_error(o.str());
        }

        oss << kind << stride;
        PyObject* pySize = PyLong_FromLong(stride);
        PyObject* pyTitle = PyUnicode_FromString(name.c_str());
        PyObject* pyFormat = PyUnicode_FromString(oss.str().c_str());

        PyList_SetItem(sizes, i, pySize);
        PyList_SetItem(titles, i, pyTitle);
        PyList_SetItem(formats, i, pyFormat);

        oss.str("");
    }

    PyDict_SetItemString(dict, "names", titles);
    PyDict_SetItemString(dict, "formats", formats);

//     PyObject* obj = PyUnicode_AsASCIIString(PyObject_Str(dict));
//     const char* s = PyBytes_AsString(obj);
//     std::string output(s);
//     std::cout << "array: " << output << std::endl;
    return dict;
}
void Array::update(PointViewPtr view)
{
    typedef std::unique_ptr<std::vector<uint8_t>> DataPtr;
    cleanup();
    int nd = 1;
    Dimension::IdList dims = view->dims();
    npy_intp mydims = view->size();
    npy_intp* ndims = &mydims;
    std::vector<npy_intp> strides(dims.size());


    DataPtr pdata( new std::vector<uint8_t>(view->pointSize()* view->size(), 0));

    PyArray_Descr *dtype(0);
    PyObject * dtype_dict = (PyObject*)buildNumpyDescription(view);
    if (!dtype_dict)
        throw pdal_error("Unable to build numpy dtype description dictionary");
    int did_convert = PyArray_DescrConverter(dtype_dict, &dtype);
    if (did_convert == NPY_FAIL)
        throw pdal_error("Unable to build numpy dtype");
    Py_XDECREF(dtype_dict);

#ifdef NPY_ARRAY_CARRAY
    int flags = NPY_ARRAY_CARRAY;
#else
    int flags = NPY_CARRAY;
#endif
    uint8_t* sp = pdata.get()->data();
    PyObject * pyArray = PyArray_NewFromDescr(&PyArray_Type,
                                              dtype,
                                              nd,
                                              ndims,
                                              0,
                                              sp,
                                              flags,
                                              NULL);

    // copy the data
    uint8_t* p(sp);
    DimTypeList types = view->dimTypes();
    for (PointId idx = 0; idx < view->size(); idx++)
    {
        p = sp + (view->pointSize() * idx);
        view->getPackedPoint(types, idx, (char*)p);
    }

    m_py_array = pyArray;
    m_data_array = std::move(pdata);

}


} // namespace plang
} // namespace pdal

