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

#include <boost/algorithm/string.hpp>

#include <pdal/Compression.hpp>
#include <pdal/GDALUtils.hpp>
#include <pdal/GlobalEnvironment.hpp>
#include "OciReader.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.oci",
    "Read point cloud data from Oracle SDO_POINTCLOUD.",
    "http://pdal.io/stages/readers.oci.html" );

CREATE_SHARED_PLUGIN(1, 0, OciReader, Reader, s_info)

std::string OciReader::getName() const { return s_info.name; }

void OciReader::processOptions(const Options& options)
{
    m_schemaFile = options.getValueOrDefault<std::string>(
        "xml_schema_dump", std::string());
    if (options.hasOption("spatialreference"))
        m_spatialRef = boost::optional<SpatialReference>(
            options.getValueOrThrow<pdal::SpatialReference>(
                "spatialreference"));
    m_query = options.getValueOrThrow<std::string>("query");
    m_connSpec = options.getValueOrDefault<std::string>("connection", "");

    m_updatePointSourceId =  options.getValueOrDefault<bool>(
        "populate_pointsourceid", false);
}

void OciReader::initialize()
{
    m_compression = false;
    GlobalEnvironment::get().initializeGDAL(log(), isDebug());
    m_connection = connect(m_connSpec);
    m_block = BlockPtr(new Block(m_connection));

    if (m_query.empty())
        throw pdal_error("'query' statement is empty. No data can be read "
            "from pdal::OciReader");

    m_stmt = Statement(m_connection->CreateStatement(m_query.c_str()));
    m_stmt->Execute(0);

    validateQuery();

    // Map the query to the block in which data is to be stored.
    defineBlock(m_stmt, m_block);

    // Fetch an initial row of data.
    if (!m_stmt->Fetch())
        throw pdal_error("Unable to fetch a point cloud entry entry!");
    m_block->setFetched();

    // Set the spatial reference from options or set the one from the block.
    if (m_spatialRef)
        setSpatialReference(*m_spatialRef);
    else
        setSpatialReference(fetchSpatialReference(m_stmt, m_block));
}


void OciReader::defineBlock(Statement stmt, BlockPtr block) const
{
    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];

    while (stmt->GetNextField(iCol, szFieldName, &hType, &nSize,
        &nPrecision, &nScale, szTypeName))
    {
        if (hType == SQLT_NTY && boost::iequals(szTypeName, "SDO_PC"))
            stmt->Define(&(block->pc));
        else if (boost::iequals(szFieldName, "OBJ_ID"))
            stmt->Define(&(block->obj_id));
        else if (boost::iequals(szFieldName, "BLK_ID"))
            stmt->Define(&(block->blk_id));
        else if (boost::iequals(szFieldName, "BLK_EXTENT"))
            stmt->Define(&(block->blk_extent));
        else if (boost::iequals(szFieldName, "BLK_DOMAIN"))
            stmt->Define(&(block->blk_domain));
        else if (boost::iequals(szFieldName, "PCBLK_MIN_RES"))
            stmt->Define(&(block->pcblk_min_res));
        else if (boost::iequals(szFieldName, "PCBLK_MAX_RES"))
            stmt->Define(&(block->pcblk_max_res));
        else if (boost::iequals(szFieldName, "NUM_POINTS"))
            stmt->Define(&(block->num_points));
        else if (boost::iequals(szFieldName, "NUM_UNSORTED_POINTS"))
            stmt->Define(&(block->num_unsorted_points));
        else if (boost::iequals(szFieldName, "PT_SORT_DIM"))
            stmt->Define(&(block->pt_sort_dim));
        else if (boost::iequals(szFieldName, "POINTS"))
            stmt->Define(&(block->locator));
        iCol++;
    }
}


Options OciReader::getDefaultOptions()
{
    Options options;

    Option connection("connection", std::string(), "Oracle connection "
        "string to connect to database");

    Option query("query", std::string(), "SELECT statement that returns "
        "an SDO_PC object as its first and only queried item.");

    Option xml_schema_dump("xml_schema_dump", std::string(),
        "Filename to dump the XML schema to.");

    options.add(connection);
    options.add(query);
    options.add(xml_schema_dump);

    return options;
}


// Throws an exception if a query is invalid.
void OciReader::validateQuery()
{
    int col = 0;
    char fieldName[OWNAME];
    int hType = 0;
    int size = 0;
    int precision = 0;
    signed short scale = 0;
    char typeName[OWNAME];
    bool typeCorrect = false;

    // We must have all of these field names present to be considered block
    // data.
    std::set<std::string> reqFields;
    reqFields.insert("OBJ_ID");
    reqFields.insert("BLK_ID");
    reqFields.insert("BLK_EXTENT");
    reqFields.insert("BLK_DOMAIN");
    reqFields.insert("PCBLK_MIN_RES");
    reqFields.insert("PCBLK_MAX_RES");
    reqFields.insert("NUM_POINTS");
    reqFields.insert("NUM_UNSORTED_POINTS");
    reqFields.insert("PT_SORT_DIM");
    reqFields.insert("POINTS");

    while (m_stmt->GetNextField(col, fieldName, &hType, &size,
        &precision, &scale, typeName))
    {
        reqFields.erase(fieldName);
        if (hType == SQLT_NTY)
        {
            if (strcmp(typeName,"SDO_PC") == 0)
                typeCorrect = true;
        }
        col++;
    }

    if (!typeCorrect)
    {
        std::ostringstream oss;
        oss << "Select statement '" << m_query <<
            "' does not fetch a SDO_PC object.";
        throw pdal_error(oss.str());
    }

    // If we found all the fields, the list of required fields will be empty.
    // If not, throw an exception.
    if (!reqFields.empty())
    {
        std::ostringstream oss;

        oss << "Query returns a block but is missing reqired fields: ";
        auto i = reqFields.begin();
        while (i != reqFields.end())
        {
            oss << *i;
            i++;
            if (i != reqFields.end())
               oss << ",";
        }
        throw pdal_error(oss.str());
    }
}


pdal::SpatialReference OciReader::fetchSpatialReference(Statement stmt,
    BlockPtr block) const
{
    // Fetch the WKT for the SRID to set the coordinate system of this stage
    int srid = stmt->GetInteger(&(block->pc->pc_geometry.sdo_srid));
    if (srid)
    {
        std::ostringstream oss;
        oss << "EPSG:" << srid;
        return pdal::SpatialReference(oss.str());
    }
    return pdal::SpatialReference();
}


void OciReader::addDimensions(PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "Fetching schema from SDO_PC object" <<
        std::endl;

    XMLSchema schema = fetchSchema(m_stmt, m_block);
    loadSchema(layout, schema);
    MetadataNode comp = schema.getMetadata().findChild("compression");
    m_compression = (comp.value() == "lazperf");

    if (m_schemaFile.size())
    {
        std::string pcSchema = schema.xml();
        std::ostream *out = FileUtils::createFile(m_schemaFile);
        out->write(pcSchema.c_str(), pcSchema.size());
        FileUtils::closeFile(out);
    }
}


point_count_t OciReader::read(PointViewPtr view, point_count_t count)
{
    if (eof())
        return 0;

    point_count_t totalNumRead = 0;
    while (totalNumRead < count)
    {
        if (m_block->numRemaining() == 0)
            if (!readOci(m_stmt, m_block))
                return totalNumRead;
        PointId bufBegin = view->size();

        point_count_t numRead = 0;
        if (orientation() == Orientation::DimensionMajor)
            numRead = readDimMajor(*view, m_block, count - totalNumRead);
        else if (orientation() == Orientation::PointMajor)
            numRead = readPointMajor(*view, m_block, count - totalNumRead);
        PointId bufEnd = bufBegin + numRead;
        totalNumRead += numRead;
    }
    return totalNumRead;
}


point_count_t OciReader::readDimMajor(PointView& view, BlockPtr block,
    point_count_t numPts)
{
    using namespace Dimension;

    point_count_t numRemaining = block->numRemaining();
    PointId startId = view.size();
    point_count_t blockRemaining = numRemaining;
    point_count_t numRead = 0;

    DimTypeList dims = dbDimTypes();
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        PointId nextId = startId;
        char *pos = seekDimMajor(*di, block);
        blockRemaining = numRemaining;
        numRead = 0;
        while (numRead < numPts && blockRemaining > 0)
        {
            writeField(view, pos, *di, nextId);
            pos += Dimension::size(di->m_type);

            if (di->m_id == Id::PointSourceId && m_updatePointSourceId)
                view.setField(Id::PointSourceId, nextId, block->obj_id);

            if (m_cb && di == dims.rbegin().base() - 1)
                m_cb(view, nextId);

            nextId++;
            numRead++;
            blockRemaining--;
        }
    }
    block->setNumRemaining(blockRemaining);
    return numRead;
}


point_count_t OciReader::readPointMajor(PointView& view,
    BlockPtr block, point_count_t numPts)
{
    size_t numRemaining = block->numRemaining();
    PointId nextId = view.size();
    point_count_t numRead = 0;

    if (m_compression)
    {
#ifdef PDAL_HAVE_LAZPERF
        LazPerfBuf buf(block->chunk);
        LazPerfDecompressor<LazPerfBuf> decompressor(buf, dbDimTypes());

        std::vector<char> ptBuf(decompressor.pointSize());
        while (numRead < numPts && numRemaining > 0)
        {
            point_count_t numWritten =
                decompressor.decompress(ptBuf.data(), ptBuf.size());
            writePoint(view, nextId, ptBuf.data());
            if (m_cb)
                m_cb(view, nextId);
            numRemaining--;
            nextId++;
            numRead++;
        }
#else
        throw pdal_error("Can't decompress without LAZperf.");
#endif
    }
    else
    {
        char *pos = seekPointMajor(block);
        while (numRead < numPts && numRemaining > 0)
        {
            writePoint(view, nextId, pos);

            if (m_cb)
                m_cb(view, nextId);

            pos += packedPointSize();
            numRemaining--;
            nextId++;
            numRead++;
        }
    }
    block->setNumRemaining(numRemaining);
    return numRead;
}


char *OciReader::seekDimMajor(const DimType& d, BlockPtr block)
{
    return block->data() +
        (dimOffset(d.m_id) * block->numPoints()) +
        (Dimension::size(d.m_type) * block->numRead());
}


char *OciReader::seekPointMajor(BlockPtr block)
{
    return block->data() + (block->numRead() * packedPointSize());
}


// Read a block (set of points) from the database.
bool OciReader::readOci(Statement stmt, BlockPtr block)
{
    if (!block->fetched())
    {
        if (!stmt->Fetch())
        {
            m_atEnd = true;
            return false;
        }
        block->setFetched();
    }
    // Read the points from the blob in the row.
    readBlob(stmt, block);
    XMLSchema *s = findSchema(stmt, block);
    updateSchema(*s);
    MetadataNode comp = s->getMetadata().findChild("compression");
    m_compression = (comp.value() == "lazperf");

    block->reset();
    block->clearFetched();
    return true;
}


void OciReader::readBlob(Statement stmt, BlockPtr block)
{
    uint32_t amountRead = 0;
    uint32_t blobLength = stmt->GetBlobLength(block->locator);

    if (block->chunk.size() < blobLength)
        block->chunk.resize(blobLength);

    if (!stmt->ReadBlob(block->locator, (void*)(block->chunk.data()),
                        block->chunk.size() , &amountRead))
        throw pdal_error("Did not read all blob data!");

    block->chunk.resize(amountRead);
}


// All of the schemas should be the same with regard to actual dimension
// name, order, etc, but each cloud may have its own scaling for X, Y and Z.
// Store it away so that it can be applied later if necessary.
// HOBU -- nope. Each Block/cloud combo could potentially have a different
// schema, with same names but different composition.
// ABELL -- the setup doesn't allow this.  Dimensions and order is all stored
// when the first block is read.  We don't have any facility for modifying
// dimensions as blocks are read.  If this were to actually happen, things
// would break.  Should we do something about it?
XMLSchema *OciReader::findSchema(Statement stmt, BlockPtr block)
{
    int32_t cloudId = stmt->GetInteger(&block->pc->pc_id);
    auto si = m_schemas.find(cloudId);
    if (si == m_schemas.end())
    {
        XMLSchema s = fetchSchema(stmt, block);
        auto i = m_schemas.insert(std::make_pair(cloudId, s));
        si = i.first;
    }
    return &(si->second);
}

} // namespace pdal
