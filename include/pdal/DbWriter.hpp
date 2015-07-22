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

#include <pdal/Writer.hpp>
#include <pdal/XMLSchema.hpp>

#include <string>
#include <unordered_map>

namespace pdal
{

class SQLiteWriter;
class PgWriter;
class OciWriter;

class PDAL_DLL DbWriter : public Writer
{
protected:
    DbWriter()
    {}

    virtual void setAutoXForm(const PointViewPtr view);
    XMLDimList dbDimTypes() const
        { return m_dbDims; }
    size_t readField(const PointView& view, char *pos, Dimension::Id::Enum id,
        PointId idx);
    size_t readPoint(const PointView& view, PointId idx, char *outbuf);
    size_t packedPointSize() const
        { return m_packedPointSize; }

    // Allows subclass access to ready() without the mess of friends.
    void doReady(PointTableRef table)
        { DbWriter::ready(table); }

private:
    virtual void prepared(PointTableRef table);
    virtual void ready(PointTableRef table);

    DimTypeList m_dimTypes;
    XMLDimList m_dbDims;
    std::unordered_map<int, DimType> m_dimMap;
    std::pair<int, int> m_xOffsets;
    std::pair<int, int> m_yOffsets;
    std::pair<int, int> m_zOffsets;

    // Size of point data as read from PointTable.
    size_t m_packedPointSize;
    // Size of point data as written to DB.
    size_t m_dbPointSize;
    bool m_locationScaling;

    DbWriter& operator=(const DbWriter&); // not implemented
    DbWriter(const DbWriter&); // not implemented
};

} // namespace pdal

