/******************************************************************************
* Copyright (c) 2023, Howard Butler (howard@hobu.co)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <pdal/pdal_features.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Streamable.hpp>


namespace arrow
{
    struct Array;
    struct MemoryPool;
    struct ArrayBuilder;
    struct Field;
    struct Table;
    struct Schema;
    namespace io
    {
        class FileOutputStream;
    };
};

namespace pdal
{

    typedef std::map<pdal::Dimension::Id, std::unique_ptr<arrow::ArrayBuilder> > DimBuilderMap;

class PDAL_DLL ArrowWriter  : public Writer, public Streamable
{
public:
    ArrowWriter();
    std::string getName() const;
    ~ArrowWriter();

private:

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void ready(PointTableRef table);
    virtual bool processOne(PointRef& point);
    virtual void done(PointTableRef table);
    virtual void write(const PointViewPtr view);

    void computeArrowSchema(pdal::PointTableRef table);

    std::string m_filename;
    std::string m_format;
    std::shared_ptr<arrow::Table> m_table;
    std::shared_ptr<arrow::Schema> m_schema;
    std::map<pdal::Dimension::Id, std::unique_ptr<arrow::ArrayBuilder> > m_builders;
    arrow::MemoryPool* m_pool;

    std::shared_ptr<arrow::io::FileOutputStream> m_file;

    ArrowWriter& operator=(const ArrowWriter&); // not implemented
    ArrowWriter(const ArrowWriter&); // not implemented
};

} // namespace pdal