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

#pragma once

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define PY_ARRAY_UNIQUE_SYMBOL PDAL_ARRAY_API
#define NO_IMPORT_ARRAY
#include <numpy/arrayobject.h>

namespace pdal
{

class PDAL_DLL NumpyReader : public Reader, public Streamable
{
public:
    NumpyReader()
    {}

    std::string getName() const;
    point_count_t getNumPoints() const;

private:
    virtual void initialize();
    virtual void addArgs(ProgramArgs& args);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual bool processOne(PointRef& point);
    virtual void done(PointTableRef table);

    bool loadPoint(PointRef& point, point_count_t position );
    void wakeUpNumpyArray();
    void prepareFieldsArray(PointLayoutPtr layout);
    void prepareRasterArray(PointLayoutPtr layout);

    // Py_XDECREF these on the way out
    PyArrayObject* m_array;
    NpyIter* m_iter;
    NpyIter_IterNextFunc* m_iternext;
    PyArray_Descr* m_dtype;

    char** m_dataptr;
    char* p_data;
    npy_intp m_nonzero_count;
    npy_intp* m_strideptr, *m_innersizeptr;
    npy_intp* m_shape;
    npy_intp m_chunkCount;
    point_count_t m_numPoints;
    int m_numFields;

    int m_ndims;
    std::string m_defaultDimension;

    size_t m_xDimNum;
    size_t m_yDimNum;
    size_t m_zDimNum;
    double m_assignZ;

    std::vector<pdal::Dimension::Id> m_ids;
    std::vector<pdal::Dimension::Type> m_types;
    std::vector<int> m_sizes;
    std::vector<int> m_offsets;
    point_count_t m_index;

    NumpyReader& operator=(const NumpyReader&); // not implemented
    NumpyReader(const NumpyReader&); // not implemented
};

} // namespace pdal
