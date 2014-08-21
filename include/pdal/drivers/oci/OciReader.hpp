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

#include <pdal/Reader.hpp>
#include <pdal/drivers/oci/common.hpp>

namespace pdal
{

class PointContext;

namespace drivers
{
namespace oci
{

class PDAL_DLL OciReader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.oci.reader", "OCI Reader")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.oci.reader.html")
#ifdef PDAL_HAVE_ORACLE
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif
    
    OciReader(const Options& options) : pdal::Reader(options)
    {}

    static Options getDefaultOptions();
    StageSequentialIterator* createSequentialIterator() const;

private:
    virtual void initialize();
    virtual void processOptions(const Options& options);
    virtual void addDimensions(PointContext ctx);
    void validateQuery();
    void defineBlock(Statement statement, BlockPtr block) const;
    pdal::SpatialReference fetchSpatialReference(Statement statement,
        BlockPtr block) const;

    Connection m_connection;
    Statement m_stmt;
    BlockPtr m_block;
    std::string m_query;
    std::string m_schemaFile;
    std::string m_connSpec;
    boost::optional<SpatialReference> m_spatialRef;

    OciReader& operator=(const OciReader&); // not implemented
    OciReader(const OciReader&); // not implemented
};

} // namespace oci
} // namespace drivers
} // namespace pdal

