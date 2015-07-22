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

#include "SQLiteReader.hpp"
#include <pdal/PointView.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.sqlite",
    "Read data from SQLite3 database files.",
    "" );

CREATE_SHARED_PLUGIN(1, 0, SQLiteReader, Reader, s_info)

std::string SQLiteReader::getName() const { return s_info.name; }

void SQLiteReader::initialize()
{
    try
    {
        log()->get(LogLevel::Debug) << "Connection: '" << m_connection << "'" <<
            std::endl;
        m_session = std::unique_ptr<SQLite>(new SQLite(m_connection, log()));
        m_session->connect(false); // don't connect in write mode
        log()->get(LogLevel::Debug) << "Connected to database" << std::endl;
        
        bool bHaveSpatialite = m_session->haveSpatialite();
        log()->get(LogLevel::Debug) << "Have spatialite?: " <<
            bHaveSpatialite << std::endl;
        m_session->loadSpatialite(m_modulename);

        if (!bHaveSpatialite)
        {
            throw pdal_error("no spatialite enabled!");
        }

    }
    catch (pdal_error const& e)
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
    catch (Option::not_found)
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
    log()->get(LogLevel::Debug) << "Fetching srid object" << std::endl;

    // ::soci::row r;
    // ::soci::indicator ind = ::soci::i_null;
    int64_t srid(0);
    // ::soci::statement clouds =
   //      (m_session->prepare << query, ::soci::into(r, ind));
   //  clouds.execute();
   //
   //  if (ind == ::soci::i_null)
   //      return pdal::SpatialReference();
   //
   //  bool bDidRead = clouds.fetch();
   //
   //  srid = (int64_t)r.get<int32_t>("srid");
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


void SQLiteReader::addDimensions(PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "Fetching schema object" << std::endl;

    std::ostringstream oss;
    oss << "SELECT SCHEMA FROM (" << m_query <<") as q LIMIT 1";
    std::string q(oss.str());

    m_session->query(q);
    const row* r = m_session->get(); // First result better have our schema
    if (!r)
        throw pdal_error("Unable to select schema from query!");

    column const& s = r->at(0); // First column is schema

    if (m_schemaFile.size())
    {
        std::ostream* out = FileUtils::createFile(m_schemaFile);
        out->write(s.data.c_str(), s.data.size());
        FileUtils::closeFile(out);
    }

    XMLSchema schema(s.data);
    m_patch->m_metadata = schema.getMetadata();

    loadSchema(layout, schema);
}


void SQLiteReader::ready(PointTableRef table)
{
    m_at_end = false;
    b_doneQuery = false;

    m_session.reset(new SQLite(m_connection, log()));
    m_session->connect(false); // don't connect in write mode
}


bool SQLiteReader::nextBuffer()
{
    return m_session->next();
}


point_count_t SQLiteReader::readPatch(PointViewPtr view, point_count_t numPts)
{
    const row* r = m_session->get();
    if (!r)
        throw pdal_error("readPatch with no data in session!");
    std::map<std::string, int32_t> const& columns = m_session->columns();

    // Availability of positions already validated
    int32_t position = columns.find("POINTS")->second;

    MetadataNode comp = m_patch->m_metadata.findChild("compression");
    m_patch->m_isCompressed = boost::iequals(comp.value(), "lazperf");
    m_patch->m_compVersion = m_patch->m_metadata.findChild("version").value();

    position = columns.find("NUM_POINTS")->second;
    int32_t count = boost::lexical_cast<int32_t>((*r)[position].data);
    m_patch->remaining = count;
    m_patch->count = count;

    log()->get(LogLevel::Debug3) << "patch compression? "
                                 << m_patch->m_isCompressed << std::endl;

    if (m_patch->m_isCompressed)
        log()->get(LogLevel::Debug3) << "patch compression version: "
                                     << m_patch->m_compVersion << std::endl;

    position = columns.find("POINTS")->second;

    point_count_t numRead = 0;
    PointId nextId = view->size();
    if (m_patch->m_isCompressed)
    {
#ifdef PDAL_HAVE_LAZPERF
        LazPerfDecompressor<Patch> decompressor(*m_patch, dbDimTypes());

        // Set the data into the patch.
        m_patch->setBytes((*r)[position].blobBuf);
        std::vector<char> tmpBuf(decompressor.pointSize());
        while (numRead < numPts && count > 0)
        {
            decompressor.decompress(tmpBuf.data(), tmpBuf.size());
            writePoint(*view.get(), nextId, tmpBuf.data());

            if (m_cb)
                m_cb(*view, nextId);

            nextId++;
            numRead++;
            count--;
        }
#else
        throw pdal_error("Can't decompress without LAZperf.");
#endif

        log()->get(LogLevel::Debug3) << "Compressed byte size: " <<
            m_patch->byte_size() << std::endl;
        if (!m_patch->byte_size())
            throw pdal_error("Compressed patch size was 0!");
        log()->get(LogLevel::Debug3) << "Uncompressed byte size: " <<
            (m_patch->count * packedPointSize()) << std::endl;
    }
    else
    {
        const char *pos = (const char *)&((*r)[position].blobBuf[0]);
        while (numRead < numPts && count > 0)
        {
            writePoint(*view.get(), nextId, pos);

            pos += packedPointSize();
            if (m_cb)
                m_cb(*view, nextId);
            nextId++;
            numRead++;
            count--;
        }
    }
    m_patch->remaining -= numRead;
    return numRead;
}


point_count_t SQLiteReader::read(PointViewPtr view, point_count_t count)
{
    if (eof())
        return 0;

    log()->get(LogLevel::Debug4) << "read called with "
        "PointView filled to " << view->size() << " points" <<
        std::endl;

    point_count_t totalNumRead = 0;
    if (! b_doneQuery)
    {
        // read first patch
        m_session->query(m_query);
        validateQuery();
        b_doneQuery = true;
        totalNumRead = readPatch(view, count);
    }

    int patch_count(0);
    while (totalNumRead < count)
    {
        if (m_patch->remaining == 0)
        {
            if (!nextBuffer())
            {
                m_at_end = true;
                return totalNumRead;
            }
        }
        PointId bufBegin = view->size();
        point_count_t numRead = readPatch(view, count - totalNumRead);
        PointId bufEnd = bufBegin + numRead;
        totalNumRead += numRead;
        patch_count++;
    }
    return totalNumRead;
}

} // namespace pdal
