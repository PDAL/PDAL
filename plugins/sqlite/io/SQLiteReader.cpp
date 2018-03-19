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
#include <pdal/pdal_features.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/compression/LazPerfCompression.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "readers.sqlite",
    "Read data from SQLite3 database files.",
    ""
};

CREATE_SHARED_STAGE(SQLiteReader, s_info)

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
            throwError("Spatialite not enabled.");

    }
    catch (pdal_error const& e)
    {
        throwError("Unable to connect to database with error '" +
            std::string(e.what()));
    }

    if (m_spatialRef.empty())
        m_spatialRef = fetchSpatialReference(m_query);
    setSpatialReference(m_spatialRef);

    m_patch = PatchPtr(new Patch());
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


void SQLiteReader::addArgs(ProgramArgs& args)
{
    args.add("spatialreference", "Spatial reference to apply to points if "
       "one doesn't exist", m_spatialRef);
    args.add("query", "SELECT statement that returns point cloud", m_query);
    args.add("connection", "Database connection string", m_connection);
    args.add("module", "Spatialite module name", m_modulename);
    args.add("xml_schema_dump", "File to write point clould schema",
        m_schemaFile);
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
            throwError("Unable to find required column name '" + *r + "'");
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
        throwError("Unable to select schema from query.");

    column const& s = r->at(0); // First column is schema

    if (m_schemaFile.size())
    {
        std::ostream* out = Utils::createFile(m_schemaFile);
        out->write(s.data.c_str(), s.data.size());
        Utils::closeFile(out);
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
        throwError("readPatch with no data in session.");
    std::map<std::string, int32_t> const& columns = m_session->columns();

    // Availability of positions already validated
    int32_t position = columns.find("POINTS")->second;

    MetadataNode comp = m_patch->m_metadata.findChild("compression");
    m_patch->m_isCompressed = Utils::iequals(comp.value(), "lazperf");
    m_patch->m_compVersion = m_patch->m_metadata.findChild("version").value();

    position = columns.find("NUM_POINTS")->second;
    size_t count;
    Utils::fromString((*r)[position].data, count);
    m_patch->remaining = (int32_t)count;
    m_patch->count = (int32_t)count;

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
        size_t bufsize = 0;
#ifdef PDAL_HAVE_LAZPERF
        auto cb = [this, view, &nextId, &numRead, &count]
            (char *buf, size_t bufsize)
        {
            writePoint(*view.get(),  nextId, buf);
            if (m_cb)
                m_cb(*view, nextId);
            nextId++;
            numRead++;
            count--;
        };

        const char *buf = reinterpret_cast<const char *>(
            (*r)[position].blobBuf.data());
        bufsize = (*r)[position].blobBuf.size();
        count = std::min(count, size_t(numPts));
        LazPerfDecompressor(cb, dbDimTypes(), count).
            decompress(buf, bufsize);

        // Set the data into the patch.

#else
        throwError("Can't decompress without LAZperf.");
#endif

        log()->get(LogLevel::Debug3) << "Compressed byte size: " <<
            bufsize << std::endl;
        if (!bufsize)
            throwError("Compressed patch size was 0.");
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
