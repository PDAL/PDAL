/******************************************************************************
* Copyright (c) 2014, Pete Gadomski <pete.gadomski@gmail.com>
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

#include <array>
#include <string>

#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

class PDAL_DLL TransformationFilter : public Filter, public Streamable
{
public:
    class Transform;

    TransformationFilter();
    ~TransformationFilter();
    TransformationFilter& operator=(const TransformationFilter&) = delete;
    TransformationFilter(const TransformationFilter&) = delete;

    std::string getName() const override;
    void doFilter(PointView& view, const Transform& matrix);

private:
    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize() override;
    virtual bool processOne(PointRef& point) override;
    virtual void filter(PointView& view) override;
    virtual void spatialReferenceChanged(const SpatialReference& srs) override;

    std::unique_ptr<Transform> m_matrix;
    SpatialReference m_overrideSrs;
    bool m_invert;
};

class TransformationFilter::Transform
{
public:
    static const size_t RowSize = 4;
    static const size_t ColSize = 4;
    static const size_t Size = RowSize * ColSize;
    typedef double ValueType;
    typedef std::array<ValueType, Size> ArrayType;

    PDAL_DLL Transform();
    PDAL_DLL Transform(const ArrayType& arr);

    PDAL_DLL double operator[](size_t off) const
        { return m_vals[off]; }
    PDAL_DLL double& operator[](size_t off)
        { return m_vals[off]; }

private:
    ArrayType m_vals;

    PDAL_DLL friend std::istream& operator>>(std::istream& in,
        pdal::TransformationFilter::Transform& xform);
    PDAL_DLL friend std::ostream& operator<<(std::ostream& out,
        const pdal::TransformationFilter::Transform& xform);
};

} // namespace pdal
