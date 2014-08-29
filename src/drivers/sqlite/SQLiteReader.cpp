/******************************************************************************
* Copyright (c) 2014, Howard Butler, howard@hobu.co
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

#include <pdal/drivers/sqlite/SQLiteReader.hpp>
#include <pdal/drivers/sqlite/SQLiteIterator.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Utils.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <map>
#include <string>

#ifdef USE_PDAL_PLUGIN_SOCI
MAKE_READER_CREATOR(sqliteReader, pdal::drivers::sqlite::Reader)
CREATE_READER_PLUGIN(sqlite, pdal::drivers::sqlite::Reader)
#endif


namespace pdal
{
namespace drivers
{
namespace sqlite
{


SQLiteReader::SQLiteReader(const Options& options)
    : pdal::Reader(options)
{}

void SQLiteReader::initialize()
{
    try
    {
        log()->get(LogLevel::Debug) << "Connection: '" << m_connection << "'" << std::endl;
        m_session = std::unique_ptr<SQLite>(new SQLite(m_connection, log()));
        m_session->connect(false); // don't connect in write mode
        log()->get(LogLevel::Debug) << "Connected to database" << std::endl;
        bool bHaveSpatialite = m_session->doesTableExist("geometry_columns");
        log()->get(LogLevel::Debug) << "Have spatialite?: " << bHaveSpatialite << std::endl;
        m_session->spatialite();

        if (!bHaveSpatialite)
        {
            std::ostringstream oss;
            oss << "no spatialite enabled!";
            throw sqlite_driver_error(oss.str());
        }
        
    }
    catch (sqlite::sqlite_driver_error const& e)
    {
        std::stringstream oss;
        oss << "Unable to connect to database with error '" << e.what() << "'";
        throw pdal_error(oss.str());
    }
    try
    {
        setSpatialReference(
            m_options.getValueOrThrow<pdal::SpatialReference>(
                "spatialreference"));
    }
    catch (pdal::option_not_found const&)
    {
        // If one wasn't set on the options, we'll ignore at this
        setSpatialReference(fetchSpatialReference(m_query));
    }

    m_patch = PatchPtr(new Patch());
    
}


Options SQLiteReader::getDefaultOptions()
{
    Options options;

    Option connection("connection", "",
        "Connection string to connect to database");
    Option query("query", "",
        "SELECT statement that returns point cloud");

    options.add(connection);
    options.add(query);

    return options;
}


pdal::SpatialReference
SQLiteReader::fetchSpatialReference(std::string const& query) const
{
    // Fetch the WKT for the SRID to set the coordinate system of this stage
    log()->get(LogLevel::Debug) << "Fetching schema object" << std::endl;

    // ::soci::row r;
    // ::soci::indicator ind = ::soci::i_null;
    boost::int64_t srid(0);
    // ::soci::statement clouds =
   //      (m_session->prepare << query, ::soci::into(r, ind));
   //  clouds.execute();
   //
   //  if (ind == ::soci::i_null)
   //      return pdal::SpatialReference();
   //
   //  bool bDidRead = clouds.fetch();
   //
   //  srid = (boost::int64_t)r.get<boost::int32_t>("srid");
   //
   //  if (!bDidRead)
   //      return pdal::SpatialReference();
   //
   //  log()->get(LogLevel::Debug) << "query returned " << srid << std::endl;
   //  std::ostringstream oss;
   //  oss <<"EPSG:" << srid;

    // if (srid >= 0)
    //     return pdal::SpatialReference(oss.str());
    // else
        return pdal::SpatialReference();
}

void SQLiteReader::processOptions(const Options& options)
{
    m_schemaFile = options.getValueOrDefault<std::string>(
        "xml_schema_dump", std::string());

    if (options.hasOption("spatialreference"))
        m_spatialRef = boost::optional<SpatialReference>(
            options.getValueOrThrow<pdal::SpatialReference>(
                "spatialreference"));
    m_query = options.getValueOrThrow<std::string>("query");
    m_connection = options.getValueOrDefault<std::string>("connection", "");
}



void SQLiteReader::validateQuery() const
{
    std::set<std::string> reqFields;
    reqFields.insert("POINTS");
    reqFields.insert("SCHEMA");
    reqFields.insert("NUM_POINTS");
    reqFields.insert("CLOUD");

    for (auto r = reqFields.begin(); r != reqFields.end(); ++r)
    {
        auto p = m_session->columns().find(*r);
        if (p == m_session->columns().end())
        {
            std::ostringstream oss;
            oss << "Unable to find required column name '" << *r << "'";
            throw pdal_error(oss.str());
        }
    }
}

void SQLiteReader::addDimensions(PointContext ctx)
{
    log()->get(LogLevel::Debug) << "Fetching schema object" << std::endl;

    std::ostringstream oss;
    oss << "SELECT SCHEMA FROM (" << m_query <<") as q LIMIT 1";
    std::string q(oss.str());

    m_session->query(q);
    const row* r = m_session->get(); // First result better have our schema
    if (!r)
        throw sqlite_driver_error("Unable to select schema from query!");
    
    column const& s = r->at(0); // First column is schema

    m_patch->m_schema = schema::Reader(s.data).schema(); 
    m_patch->m_ctx = ctx;
    
    schema::DimInfoList& dims = m_patch->m_schema.m_dims;
    for (auto di = dims.begin(); di != dims.end(); ++di)
        di->m_id = ctx.registerOrAssignDim(di->m_name, di->m_type);
}


pdal::StageSequentialIterator*
SQLiteReader::createSequentialIterator() const
{
    return new pdal::drivers::sqlite::iterators::sequential::SQLiteIterator(*this, m_patch);
}




}
}
} // namespace pdal::driver::sqlite
