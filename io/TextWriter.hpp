/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/Streamable.hpp>
#include <pdal/Writer.hpp>

namespace pdal
{

typedef std::shared_ptr<std::ostream> FileStreamPtr;

class PDAL_EXPORT TextWriter : public Writer, public Streamable
{
    struct DimSpec
    {
        Dimension::Id id;
        size_t precision;
        std::string name;
    };

    enum class OutputType
    {
        CSV,
        GEOJSON
    };

    friend std::istream& operator >> (std::istream& in, OutputType& type);
    friend std::ostream& operator << (std::ostream& out,
        const OutputType& type);

public:
    TextWriter()
    {}
    TextWriter& operator=(const TextWriter&) = delete;
    TextWriter(const TextWriter&) = delete;

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);
    virtual bool processOne(PointRef& point);

    void writeHeader(PointTableRef table);
    void writeFooter();
    void writeGeoJSONHeader();
    void writeCSVHeader(PointTableRef table);
    void processOneCSV(PointRef& point);
    void processOneGeoJSON(PointRef& point);

    DimSpec extractDim(std::string dim, PointTableRef table);
    bool findDim(Dimension::Id id, DimSpec& ds);

    OutputType m_outputType;
    std::string m_callback;
    bool m_writeAllDims;
    std::string m_dimOrder;
    bool m_writeHeader;
    std::string m_newline;
    std::string m_delimiter;
    bool m_quoteHeader;
    int m_precision;
    PointId m_idx;

    FileStreamPtr m_stream;
    std::vector<DimSpec> m_dims;
    DimSpec m_xDim;
    DimSpec m_yDim;
    DimSpec m_zDim;
};

} // namespace pdal
