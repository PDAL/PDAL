/******************************************************************************
* Copyright (c) 2014, Hobu Inc. (hobu@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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
#include <pdal/XMLSchema.hpp>

namespace pdal
{

class PDAL_DLL DbReader : public Reader
{
protected:
    DbReader() : m_orientation(Orientation::PointMajor), m_packedPointSize(0)
    {}

    DimTypeList dbDimTypes() const;
    void loadSchema(PointLayoutPtr layout, const std::string& schemaString);
    void loadSchema(PointLayoutPtr layout, const XMLSchema& schema);
    void updateSchema(const XMLSchema& schema);
    void writeField(PointView& view, const char *pos, const DimType& dim,
        PointId idx);
    void writePoint(PointView& view, PointId idx, const char *buf);
    size_t packedPointSize() const
        { return m_packedPointSize; }
    size_t dimOffset(Dimension::Id::Enum id) const;
    Orientation::Enum orientation() const
        { return m_orientation; }

private:
    PointLayoutPtr m_layout;
    XMLDimList m_dims;
    Orientation::Enum m_orientation;
    size_t m_packedPointSize;

    DbReader& operator=(const DbReader&); // not implemented
    DbReader(const DbReader&); // not implemented
};

} // namespace pdal

