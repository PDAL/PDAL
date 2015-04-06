/******************************************************************************
* Copyright (c) 2014, Hobu Inc., hobu.inc@gmail.com
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

#include <vector>

#include <pdal/DbReader.hpp>
#include <pdal/StageFactory.hpp>

#include "OciCommon.hpp"

namespace pdal
{

class PDAL_DLL OciReader : public DbReader
{
public:
    OciReader()
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

private:
    virtual void initialize();
    virtual void processOptions(const Options& options);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table)
        { m_atEnd = false; }
    virtual point_count_t read(PointViewPtr view, point_count_t);
    virtual bool eof()
        { return m_atEnd; }

    void validateQuery();
    void defineBlock(Statement statement, BlockPtr block) const;
    pdal::SpatialReference fetchSpatialReference(Statement statement,
        BlockPtr block) const;

    void readBlob(Statement stmt, BlockPtr block);
    point_count_t readDimMajor(PointView& view, BlockPtr block,
        point_count_t numPts);
    point_count_t readPointMajor(PointView& view, BlockPtr block,
        point_count_t numPts);
    char *seekDimMajor(const DimType& d, BlockPtr block);
    char *seekPointMajor(BlockPtr block);
    bool readOci(Statement stmt, BlockPtr block);
    XMLSchema *findSchema(Statement stmt, BlockPtr block);

    Connection m_connection;
    Statement m_stmt;
    BlockPtr m_block;
    std::string m_query;
    std::string m_schemaFile;
    std::string m_connSpec;
    bool m_updatePointSourceId;
    boost::optional<SpatialReference> m_spatialRef;
    bool m_atEnd;
    std::map<int32_t, XMLSchema> m_schemas;
    bool m_compression;

    OciReader& operator=(const OciReader&); // not implemented
    OciReader(const OciReader&); // not implemented
};

} // namespace pdal
