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
#include <pdal/PointBuffer.hpp>

#ifdef USE_PDAL_PLUGIN_SOCI
//MAKE_READER_CREATOR(sqliteReader, pdal::drivers::sqlite::Reader)
CREATE_READER_PLUGIN(sqliteReader, pdal::drivers::sqlite::Reader)
#endif


namespace pdal
{
namespace drivers
{
namespace sqlite
{

SQLiteReader::SQLiteReader() : pdal::Reader()
{}

void SQLiteReader::initialize()
{
    try
    {
        log()->get(LogLevel::Debug) << "Connection: '" << m_connection << "'" <<
            std::endl;
        m_session = std::unique_ptr<SQLite>(new SQLite(m_connection, log()));
        m_session->connect(false); // don't connect in write mode
        log()->get(LogLevel::Debug) << "Connected to database" << std::endl;
        bool bHaveSpatialite = m_session->doesTableExist("geometry_columns");
        log()->get(LogLevel::Debug) << "Have spatialite?: " <<
            bHaveSpatialite << std::endl;
        m_session->spatialite(m_modulename);

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
    m_modulename = options.getValueOrDefault<std::string>("module", "");
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


void SQLiteReader::addDimensions(PointContextRef ctx)
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


void SQLiteReader::ready(PointContextRef ctx)
{
    m_at_end = false;
    b_doneQuery = false;

    m_session.reset(new SQLite(m_connection, log()));
    m_session->connect(false); // don't connect in write mode

    schema::DimInfoList dims = m_patch->m_schema.m_dims;
    m_point_size = 0;
    for (auto di = dims.begin(); di != dims.end(); ++di)
        m_point_size += Dimension::size(di->m_type);
}


bool SQLiteReader::NextBuffer()
{
    return m_session->next();
}


point_count_t SQLiteReader::readPatch(PointBuffer& buffer, point_count_t numPts)
{
    const row* r = m_session->get();
    if (!r)
        throw pdal_error("readPatch with no data in session!");
    std::map<std::string, int32_t> const& columns = m_session->columns();

    // Availability of positions already validated
    int32_t position = columns.find("POINTS")->second;
    auto bytes = (*r)[position].blobBuf;
    size_t size = (*r)[position].blobLen;
    position = columns.find("NUM_POINTS")->second;
    int32_t count = boost::lexical_cast<int32_t>((*r)[position].data);
    log()->get(LogLevel::Debug4) << "fetched patch with " << count <<
        " points and " << size << " bytes bytesize: " << size << std::endl;
    m_patch->remaining = count;
    m_patch->count = count;
    m_patch->bytes = bytes;
    m_patch->byte_size = size;

    point_count_t numRemaining = m_patch->remaining;
    PointId nextId = buffer.size();
    point_count_t numRead = 0;

    size_t offset = ((m_patch->count - m_patch->remaining) * m_point_size);
    uint8_t *pos = &(m_patch->bytes.front()) + offset;

    schema::DimInfoList& dims = m_patch->m_schema.m_dims;
    while (numRead < numPts && numRemaining > 0)
    {
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            schema::DimInfo& d = *di;
            buffer.setField(d.m_id, d.m_type, nextId, pos);
            pos += Dimension::size(d.m_type);
        }

        // Scale X, Y and Z
        // double v = buffer.getFieldAs<double>(Dimension::Id::X, nextId);
        // v = v * m_patch->xScale() + m_patch->xOffset();
        // buffer.setField(Dimension::Id::X, nextId, v);
        //
        // v = buffer.getFieldAs<double>(Dimension::Id::Y, nextId);
        // v = v * m_patch->yScale() + m_patch->yOffset();
        // buffer.setField(Dimension::Id::Y, nextId, v);
        //
        // v = buffer.getFieldAs<double>(Dimension::Id::Z, nextId);
        // v = v * m_patch->zScale() + m_patch->zOffset();
        // buffer.setField(Dimension::Id::Z, nextId, v);

        numRemaining--;
        nextId++;
        numRead++;
    }

    m_patch->remaining = numRemaining;

    return numRead;
}


point_count_t SQLiteReader::read(PointBuffer& buffer, point_count_t count)
{
    if (eof())
        return 0;

    log()->get(LogLevel::Debug4) << "readBufferImpl called with "
        "PointBuffer filled to " << buffer.size() << " points" <<
        std::endl;

    point_count_t totalNumRead = 0;
    if (! b_doneQuery)
    {
        // read first patch
        m_session->query(m_query);
        validateQuery();
        b_doneQuery = true;
        totalNumRead = readPatch(buffer, count);
    }

    int patch_count(0);
    while (totalNumRead < count)
    {
        if (m_patch->remaining == 0)
        {
            if (!NextBuffer())
            {
                m_at_end = true;
                return totalNumRead;
            }
        }
        PointId bufBegin = buffer.size();
        point_count_t numRead = readPatch(buffer, count - totalNumRead);
        PointId bufEnd = bufBegin + numRead;
        totalNumRead += numRead;
        patch_count++;
    }

    return totalNumRead;

}

} // namespace sqlite
} // namespace drivers
} // namespace pdal
