/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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
#include <pdal/ReaderIterator.hpp>
#include <pdal/PointBuffer.hpp>

#include <pdal/drivers/pgpointcloud/common.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <vector>

#include <pdal/drivers/pgpointcloud/PgIterator.hpp>

namespace pdal
{
namespace drivers
{
namespace pgpointcloud
{

class PDAL_DLL PgReader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.pgpointcloud.reader", "PostgreSQL Pointcloud Database Reader")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.pgpointcloud.reader.html")
#ifdef PDAL_HAVE_POSTGRESQL
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif
    
    PgReader(const Options&);
    ~PgReader();

    static Options getDefaultOptions();
    virtual boost::uint64_t getNumPoints() const;
    boost::uint64_t getMaxPoints() const;
    std::string getDataQuery() const;
    std::string connString() const
        { return m_connection; }
    void getSession() const;
    
    StageSequentialIterator* createSequentialIterator() const;

private:
    virtual void addDimensions(PointContext ctx);
    virtual void processOptions(const Options& options);
    virtual void ready(PointContext ctx);
    
    pdal::SpatialReference fetchSpatialReference() const;
    boost::uint32_t fetchPcid() const;

    PGconn* m_session;
    std::string m_connection;
    std::string m_table_name;
    std::string m_schema_name;
    std::string m_column_name;
    std::string m_where;
    mutable boost::uint32_t m_pcid;
    mutable boost::uint64_t m_cached_point_count;
    mutable boost::uint64_t m_cached_max_points;
    schema::XMLSchema m_schema;
    boost::optional<SpatialReference> m_spatialRef;
    PgReader& operator=(const PgReader&); // not implemented
    PgReader(const PgReader&); // not implemented
};

} // pgpointcloud
} // driver
} // pdal

