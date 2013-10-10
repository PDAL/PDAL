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

#include <pdal/drivers/oci/Reader.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Utils.hpp>
#include <pdal/GlobalEnvironment.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <map>


#ifdef USE_PDAL_PLUGIN_OCI
MAKE_READER_CREATOR(ociReader, pdal::drivers::oci::Reader)
CREATE_READER_PLUGIN(oci, pdal::drivers::oci::Reader)
#endif


namespace pdal
{
namespace drivers
{
namespace oci
{


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , OracleDriver(getOptions())
    , m_querytype(QUERY_UNKNOWN)
    , m_capacity(0)
    , m_cachedPointCount(0)
{

}


boost::uint64_t Reader::getNumPoints() const
{
    if (m_cachedPointCount != 0) return m_cachedPointCount;

    {
        std::ostringstream query;
        query << "select sum(num_points) from (" << getQueryString() << ")";
        Statement statement = Statement(m_connection->CreateStatement(query.str().c_str()));
        statement->Execute(0);
        boost::int64_t count;
        statement->Define(&(count));
        bool bDidRead = statement->Fetch();
        if (!bDidRead)
            throw pdal_error("Unable to fetch a point count for view!");

        if (count < 0)
        {
            throw pdal_error("getNumPoints returned a count < 0!");
        }

        m_cachedPointCount = static_cast<boost::uint64_t>(count);
        return m_cachedPointCount;
    }
    return m_cachedPointCount;
}

void Reader::initialize()
{
    pdal::Reader::initialize();

    pdal::GlobalEnvironment::get().getGDALDebug()->addLog(log());
    m_connection = connect();
    m_block = BlockPtr(new Block(m_connection));

    if (getQueryString().size() == 0)
        throw pdal_error("'query' statement is empty. No data can be read from pdal::drivers::oci::Reader");

    m_initialQueryStatement = Statement(m_connection->CreateStatement(getQueryString().c_str()));

    m_initialQueryStatement->Execute(0);

    m_querytype = describeQueryType();
    if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
    {

        defineBlock(m_initialQueryStatement, m_block);
        bool bDidRead = m_initialQueryStatement->Fetch();

        if (!bDidRead) throw pdal_error("Unable to fetch a point cloud entry entry!");
        Schema& schema = getSchemaRef();

        try
        {
            setSpatialReference(getOptions().getValueOrThrow<pdal::SpatialReference>("spatialreference"));

        }
        catch (pdal_error const&)
        {
            // If one wasn't set on the options, we'll ignore at this
            setSpatialReference(fetchSpatialReference(m_initialQueryStatement, m_block->pc));

        }

        schema = fetchSchema(m_initialQueryStatement, m_block->pc, m_capacity);

    }

    else
        throw pdal_error("SQL statement does not define a SDO_PC or CLIP_PC block");

}



void Reader::defineBlock(Statement statement, BlockPtr block) const
{

    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];


    while (statement->GetNextField(iCol, szFieldName, &hType, &nSize, &nPrecision, &nScale, szTypeName))
    {
        std::string name = boost::to_upper_copy(std::string(szFieldName));

        if (hType == SQLT_NTY)
        {
            if (boost::iequals(szTypeName, "SDO_PC"))
            {
                statement->Define(&(block->pc));
            }
        }
        if (boost::iequals(szFieldName, "OBJ_ID"))
        {
            statement->Define(&(block->obj_id));
        }

        if (boost::iequals(szFieldName, "BLK_ID"))
        {
            statement->Define(&(block->blk_id));
        }

        if (boost::iequals(szFieldName, "BLK_EXTENT"))
        {
            statement->Define(&(block->blk_extent));
        }

        if (boost::iequals(szFieldName, "BLK_DOMAIN"))
        {
            statement->Define(&(block->blk_domain));
        }

        if (boost::iequals(szFieldName, "PCBLK_MIN_RES"))
        {
            statement->Define(&(block->pcblk_min_res));
        }

        if (boost::iequals(szFieldName, "PCBLK_MAX_RES"))
        {
            statement->Define(&(block->pcblk_max_res));
        }

        if (boost::iequals(szFieldName, "NUM_POINTS"))
        {
            statement->Define(&(block->num_points));
        }

        if (boost::iequals(szFieldName, "NUM_UNSORTED_POINTS"))
        {
            statement->Define(&(block->num_unsorted_points));
        }

        if (boost::iequals(szFieldName, "PT_SORT_DIM"))
        {
            statement->Define(&(block->pt_sort_dim));
        }

        if (boost::iequals(szFieldName, "POINTS"))
        {
            statement->Define(&(block->locator));
        }
        iCol++;
    }

}

Options Reader::getDefaultOptions()
{
    Options options;

    Option connection("connection",
                      "",
                      "Oracle connection string to connect to database");

    Option query("query",
                 "",
                 "SELECT statement that returns an SDO_PC object \
                 as its first and only queried item.");

    Option capacity("capacity",
                    0,
                    "Block capacity");

    Option xml_schema_dump("xml_schema_dump", std::string(""), "Filename to dump the XML schema to.");

    options.add(connection);
    options.add(query);
    options.add(capacity);
    options.add(xml_schema_dump);

    return options;
}



std::string Reader::getQueryString() const
{
    return getOptions().getValueOrThrow<std::string>("query");
}

Reader::~Reader()
{
    return;
}

QueryType Reader::describeQueryType()
{

    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];


    std::ostringstream oss;

    std::map<std::string, std::string> objects;
    while (m_initialQueryStatement->GetNextField(iCol, szFieldName, &hType, &nSize, &nPrecision, &nScale, szTypeName))
    {

        std::pair<std::string, int> p(szFieldName, hType);
        m_fields.insert(p);

        if (hType == SQLT_NTY)
        {
            std::pair<std::string, std::string> p(szTypeName, szFieldName);
            oss << "Typed Field " << std::string(szFieldName) << " with type " << hType << " and typename " << std::string(szTypeName) << std::endl;

            objects.insert(p);
        }
        else
        {
            oss << "Field " << std::string(szFieldName) << " with type " << hType << std::endl;

        }
        iCol++;
    }
    log()->get(logDEBUG)  << oss.str() << std::endl;

    bool bHaveSDO_PC(false);
    if (objects.find("SDO_PC") != objects.end()) bHaveSDO_PC = true;
    bool bHaveSDO_PC_BLK_TYPE(false);
    if (objects.find("SDO_PC_BLK_TYPE") != objects.end()) bHaveSDO_PC_BLK_TYPE = true;
    if (!bHaveSDO_PC && !bHaveSDO_PC_BLK_TYPE)
    {
        std::ostringstream oss;
        oss << "Select statement '" << getQueryString() << "' does not fetch a SDO_PC object"
            " or SDO_PC_BLK_TYPE";
        throw pdal_error(oss.str());

    }

    typedef std::pair<std::string, bool> SB;
    std::map<std::string, bool> block_fields;

    // We must have all of these field names present to be considered block
    // data.
    block_fields.insert(SB("OBJ_ID", false));
    block_fields.insert(SB("BLK_ID", false));
    block_fields.insert(SB("BLK_EXTENT", false));
    block_fields.insert(SB("BLK_DOMAIN", false));
    block_fields.insert(SB("PCBLK_MIN_RES", false));
    block_fields.insert(SB("PCBLK_MAX_RES", false));
    block_fields.insert(SB("NUM_POINTS", false));
    block_fields.insert(SB("NUM_UNSORTED_POINTS", false));
    block_fields.insert(SB("PT_SORT_DIM", false));
    block_fields.insert(SB("POINTS", false));

    std::map<std::string, int>::const_iterator i = m_fields.begin();

    bool bHasBlockFields(false);
    for (i = m_fields.begin(); i != m_fields.end(); ++i)
    {
        std::map<std::string, bool>::iterator t = block_fields.find(i->first);
        if (t != block_fields.end())
            t->second = true;
    }

    std::map<std::string, bool>::const_iterator q;

    for (q = block_fields.begin(); q != block_fields.end(); ++q)
    {
        if (q->second == false)
        {
            bHasBlockFields = false;
            break;
        }
        bHasBlockFields = true;
    }

    if (bHaveSDO_PC && bHasBlockFields)
    {
        log()->get(logDEBUG) << "Query type is QUERY_SDO_BLK_PC_VIEW" << std::endl;
        return QUERY_SDO_BLK_PC_VIEW;
    }
    if (bHaveSDO_PC)
    {
        log()->get(logDEBUG) << "Query type is QUERY_SDO_PC" << std::endl;
        return QUERY_SDO_PC;
    }

    log()->get(logDEBUG) << "Query type is QUERY_UNKNOWN" << std::endl;
    return QUERY_UNKNOWN;

}

pdal::SpatialReference Reader::fetchSpatialReference(Statement statement, sdo_pc* pc) const
{
    // Fetch the WKT for the SRID to set the coordinate system of this stage
    int srid = statement->GetInteger(&(pc->pc_geometry.sdo_srid));

    std::ostringstream oss;
    oss <<"EPSG:" << srid;

    if (srid)
        return pdal::SpatialReference(oss.str());
    else
        return pdal::SpatialReference();
}

pdal::Schema Reader::fetchSchema(Statement statement, sdo_pc* pc, boost::uint32_t& capacity, std::string ns_override) const
{
    log()->get(logDEBUG) << "Fetching schema from SDO_PC object" << std::endl;

    // Fetch the XML that defines the schema for this point cloud
    std::ostringstream select_schema;
    OCILobLocator* metadata = NULL;
    select_schema
            << "DECLARE" << std::endl
            << "PC_TABLE VARCHAR2(32) := '" << statement->GetString(pc->base_table) << "';" << std::endl
            << "PC_ID NUMBER := " << statement->GetInteger(&(pc->pc_id)) << ";" << std::endl
            << "PC_COLUMN VARCHAR2(32) := '" << statement->GetString(pc->base_column) << "';" << std::endl
            << "BEGIN" << std::endl
            << std::endl
            << "EXECUTE IMMEDIATE" << std::endl
            << " 'SELECT T.'||PC_COLUMN||'.PC_OTHER_ATTRS.getClobVal(), T.'||PC_COLUMN||'.PTN_PARAMS FROM '||pc_table||' T WHERE T.ID='||PC_ID INTO :metadata, :capacity;"
            << std::endl
            << "END;"
            << std::endl;
    Statement get_schema(m_connection->CreateStatement(select_schema.str().c_str()));
    get_schema->BindName(":metadata", &metadata);

    int ptn_params_length = 1024;
    char* ptn_params = (char*) malloc(sizeof(char*) * ptn_params_length);
    ptn_params[ptn_params_length-1] = '\0'; //added trailing null to fix ORA-01480
    get_schema->BindName(":capacity", ptn_params, ptn_params_length);
    get_schema->Execute();

    char* pc_schema = get_schema->ReadCLob(metadata);

    std::string write_schema_file = getOptions().getValueOrDefault<std::string>("xml_schema_dump", std::string(""));
    if (write_schema_file.size() > 0)
    {
        std::ostream* out = FileUtils::createFile(write_schema_file);
        out->write(pc_schema, strlen(pc_schema));
        FileUtils::closeFile(out);
    }


    // Fetch the block capacity from the point cloud object so we
    // can use it later.
    // PTN_PARAMS is like:
    // 'blk_capacity=1000,work_tablespace=my_work_ts'
    int block_capacity = 0;
    boost::char_separator<char> sep_space(" ");
    boost::char_separator<char> sep_equal("=");

    std::string s_cap(ptn_params);
    tokenizer parameters(s_cap, sep_space);
    for (tokenizer::iterator t = parameters.begin(); t != parameters.end(); ++t)
    {
        tokenizer parameter((*t), sep_equal);

        for (tokenizer::iterator c = parameter.begin(); c != parameter.end(); ++c)
        {
            if (boost::iequals(c->c_str(), "blk_capacity"))
            {
                tokenizer::iterator d = ++c;
                block_capacity = atoi(d->c_str());
            }
        }
    }

    if (block_capacity < 1)
    {
        std::ostringstream oss;
        oss << "Invalid block capacity for point cloud object in Oracle: " << block_capacity;
        throw pdal_error(oss.str());
    }

    capacity = block_capacity;

    std::string pc_schema_xml(pc_schema);
    if (pc_schema)
        CPLFree(pc_schema);

    if (ptn_params)
        free(ptn_params);

    Schema schema = Schema::from_xml(pc_schema_xml);

    schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();

    for (schema::index_by_index::const_iterator iter = dims.begin(); iter != dims.end(); ++iter)
    {
        // For dimensions that do not have namespaces, we'll set the namespace
        // to the namespace of the current stage

        if (iter->getNamespace().size() == 0)
        {
            log()->get(logDEBUG4) << "setting namespace for dimension " << iter->getName() << " to "  << getName() << std::endl;


            Dimension d(*iter);
            if (iter->getUUID().is_nil())
            {
                d.createUUID();
            }

            if (ns_override.size() > 0)
            {
                d.setNamespace(ns_override);
            }
            else
            {
                d.setNamespace(getName());
            }
            schema.setDimension(d);
        }
    }

    return schema;
}


pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::oci::iterators::sequential::Reader(*this, buffer);
}

namespace iterators
{
namespace sequential
{


IteratorBase::IteratorBase(const pdal::drivers::oci::Reader& reader)
    : m_initialQueryStatement(Statement())
    , m_at_end(false)
    , m_at_end_of_blocks(false)
    , m_at_end_of_clouds(false)
    , m_block(BlockPtr(new Block(reader.getConnection())))
    , m_active_cloud_id(0)
    , m_oracle_buffer(BufferPtr())
    , m_buffer_position(0)
    , m_reader(reader)

{

    m_querytype = reader.getQueryType();

    if (m_querytype == QUERY_SDO_PC)
    {
        m_initialQueryStatement = getNextCloud(m_block, m_active_cloud_id);
    }

    if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
    {
        m_initialQueryStatement = reader.getInitialQueryStatement();
        m_block = reader.getBlock();

        m_active_cloud_id = m_initialQueryStatement->GetInteger(&m_block->pc->pc_id);
    }

    return;
}

Statement IteratorBase::getNextCloud(BlockPtr block, boost::int32_t& cloud_id)
{

    std::ostringstream select_blocks;
    BlockPtr cloud_block = m_reader.getBlock();
    Statement cloud_statement = m_reader.getInitialQueryStatement();

    cloud_id = cloud_statement->GetInteger(&cloud_block->pc->pc_id);
    std::string cloud_table = std::string(cloud_statement->GetString(cloud_block->pc->blk_table));
    select_blocks
            << "select T.OBJ_ID, T.BLK_ID, T.BLK_EXTENT, T.NUM_POINTS, T.POINTS from "
            << cloud_table << " T WHERE T.OBJ_ID = "
            << cloud_id;


    Statement output = Statement(m_reader.getConnection()->CreateStatement(select_blocks.str().c_str()));

    output->Execute(0);
    m_reader.defineBlock(output, block);
    return output;

}

IteratorBase::~IteratorBase()
{
}



const pdal::drivers::oci::Reader& IteratorBase::getReader() const
{
    return m_reader;
}

void IteratorBase::readBlob(Statement statement,
                            BlockPtr block,
                            boost::uint32_t howMany)
{
    boost::uint32_t nAmountRead = 0;
    boost::uint32_t nBlobLength = statement->GetBlobLength(block->locator);

    if (block->chunk.size() < nBlobLength)
    {
        block->chunk.resize(nBlobLength);
    }

    getReader().log()->get(logDEBUG4) << "IteratorBase::readBlob expected nBlobLength: " << nBlobLength << std::endl;

    statement->OpenBlob(block->locator);
    bool read_all_data = statement->ReadBlob(block->locator,
                         (void*)(&(block->chunk)[0]),
                         block->chunk.size() ,
                         &nAmountRead);

    statement->CloseBlob(block->locator);
    if (!read_all_data) throw pdal_error("Did not read all blob data!");

    getReader().log()->get(logDEBUG4) << "IteratorBase::readBlob actual nAmountRead: " << nAmountRead  << std::endl;

    Schema const& oracle_schema = m_oracle_buffer->getSchema();

    boost::uint32_t howMuchToRead = howMany * oracle_schema.getByteSize();

    if (howMany > m_oracle_buffer->getCapacity())
    {
        std::ostringstream oss;
        oss << "blob size is larger than oracle buffer size! howMany: " << howMany << " m_oracle_buffer->getCapacity(): " << m_oracle_buffer->getCapacity();
        throw pdal_error(oss.str());
    }

    m_oracle_buffer->setDataStride(&(block->chunk)[0], 0, howMuchToRead);

    m_oracle_buffer->setNumPoints(howMany);
}

void IteratorBase::fillUserBuffer(PointBuffer& user_buffer)
{

    Schema const& user_schema = user_buffer.getSchema();
    schema::index_by_index const& idx = user_schema.getDimensions().get<schema::index>();

    boost::int32_t numOraclePoints = m_oracle_buffer->getNumPoints() - m_buffer_position;

    boost::int32_t numUserSpace = user_buffer.getCapacity() - user_buffer.getNumPoints();
    boost::int32_t howManyThisRead = (std::min)(numUserSpace, numOraclePoints);

    if (numUserSpace < 0)
    {
        std::ostringstream oss;
        oss << "numUserSpace < 0! : " << numUserSpace;
        throw pdal_error(oss.str());
    }

    if (numOraclePoints < 0)
    {
        std::ostringstream oss;
        oss << "numOraclePoints < 0! : " << numOraclePoints;
        throw pdal_error(oss.str());
    }


    schema::index_by_index::size_type i(0);
    for (i = 0; i < idx.size(); ++i)
    {
        copyOracleData(*m_oracle_buffer,
                       user_buffer,
                       idx[i],
                       m_buffer_position,
                       user_buffer.getNumPoints(),
                       howManyThisRead);

    }

    bool bSetPointSourceId = getReader().getOptions().getValueOrDefault<bool>("populate_pointsourceid", false);
    if (bSetPointSourceId)
    {
        Dimension const* point_source_field = user_buffer.getSchema().getDimensionPtr("PointSourceId");
        if (point_source_field)
        {
            for (boost::int32_t i = 0; i < howManyThisRead; ++i)
            {
                user_buffer.setField(*point_source_field,
                                     user_buffer.getNumPoints() + i,
                                     m_active_cloud_id);
            }
        }
    }

    if (numOraclePoints > numUserSpace)
        m_buffer_position = m_buffer_position + numUserSpace;
    else if (numOraclePoints < numUserSpace)
        m_buffer_position = 0;

    user_buffer.setNumPoints(howManyThisRead + user_buffer.getNumPoints());
}

void IteratorBase::copyOracleData(PointBuffer& source,
                                  PointBuffer& destination,
                                  Dimension const& dest_dim,
                                  boost::uint32_t source_starting_position,
                                  boost::uint32_t destination_starting_position,
                                  boost::uint32_t howMany)
{

    boost::optional<Dimension const&> source_dim = source.getSchema().getDimensionOptional(dest_dim.getName(), dest_dim.getNamespace());

    if (!source_dim)
    {
        source_dim = source.getSchema().getDimensionOptional(dest_dim.getName(), dest_dim.getNamespace()+".blocks");
        if (!source_dim)
            return;
    }

    for (boost::uint32_t i = 0; i < howMany; ++i)
    {
        if (dest_dim.getInterpretation() == source_dim->getInterpretation() &&
                dest_dim.getByteSize() == source_dim->getByteSize() &&
                pdal::Utils::compare_distance(dest_dim.getNumericScale(), source_dim->getNumericScale()) &&
                pdal::Utils::compare_distance(dest_dim.getNumericOffset(), source_dim->getNumericOffset()) &&
                dest_dim.getEndianness() == source_dim->getEndianness()
           )
        {
            // FIXME: This test could produce false positives
            boost::uint8_t* source_position = source.getData(source_starting_position+i) + source_dim->getByteOffset();
            boost::uint8_t* destination_position = destination.getData(destination_starting_position + i) + dest_dim.getByteOffset();
            memcpy(destination_position, source_position, source_dim->getByteSize());
        }
        else
        {
            PointBuffer::scaleData(source,
                                   destination,
                                   *source_dim,
                                   dest_dim,
                                   source_starting_position + i,
                                   destination_starting_position + i);
        }
    }

}

boost::uint32_t IteratorBase::myReadBuffer(PointBuffer& data)
{
    return myReadBlocks(data);
}


BufferPtr IteratorBase::fetchPointBuffer(Statement statement, sdo_pc* pc)
{
    boost::int32_t id = statement->GetInteger(&pc->pc_id);
    BufferMap::const_iterator i = m_buffers.find(id);

    if (i != m_buffers.end())
    {
        getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: found existing PointBuffer with id " << id << std::endl;
        return i->second;
    }
    else
    {
        boost::uint32_t block_capacity(0);
        Schema schema = m_reader.fetchSchema(statement, pc, block_capacity, getReader().getName()+".blocks");

        // if (block_capacity > capacity)
        // {
        //     std::ostringstream oss;
        //     oss << "Block capacity, " << block_capacity <<", is too large to fit in "
        //         << "buffer of size " << capacity<<". Increase buffer capacity with writer's \"chunk_size\" option "
        //         << "or increase the read buffer size";
        //     throw buffer_too_small(oss.str());
        // }

        BufferPtr output  = BufferPtr(new PointBuffer(schema, block_capacity));
        std::pair<int, BufferPtr> p(id, output);
        m_buffers.insert(p);
        getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: creating new PointBuffer with id " << id << std::endl;

        return p.second;
    }
}
boost::uint32_t IteratorBase::myReadBlocks(PointBuffer& user_buffer)
{
    boost::uint32_t numPointsRead = 0;

    user_buffer.setNumPoints(0);

    bool bDidRead = false;

    if (!m_oracle_buffer)
    {
        m_oracle_buffer = fetchPointBuffer(m_initialQueryStatement, m_block->pc);
        boost::int32_t current_cloud_id(0);
        current_cloud_id  = m_initialQueryStatement->GetInteger(&m_block->pc->pc_id);
        m_active_cloud_id = current_cloud_id;
    }

    // This shouldn't ever happen
    if (m_block->num_points > static_cast<boost::int32_t>(m_oracle_buffer->getCapacity()))
    {
        std::ostringstream oss;
        oss << "Block size, " << m_block->num_points <<", is too large to fit in "
            << "buffer of size " << user_buffer.getCapacity() <<". Increase buffer capacity with writer's \"chunk_size\" option "
            << "or increase the read buffer size";
        throw buffer_too_small(oss.str());
    }

    if (!m_block->num_points)
    {
        // We still have a block of data from the last readBuffer call
        // that was partially read.
        getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: fetching first block" << std::endl;
        bDidRead = m_initialQueryStatement->Fetch();
        if (!bDidRead)
        {
            m_at_end = true;
            return 0;
        }

        user_buffer.setSpatialBounds(getBounds(m_initialQueryStatement, m_block));

    }
    else
    {
        // Our read was already "done" last readBuffer call, but if we're done,
        // we're done
        if (m_at_end)
            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we are at end of the blocks;" << std::endl;
        else
            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we have points left to read on this block" << std::endl;

        if (m_at_end) return 0;
        bDidRead = true;

    }

    while (bDidRead)
    {
        boost::uint32_t numReadThisBlock = m_block->num_points;
        boost::uint32_t numSpaceLeftThisBuffer = user_buffer.getCapacity() - user_buffer.getNumPoints();

        getReader().log()->get(logDEBUG4) << "IteratorBase::myReadBlocks:" "numReadThisBlock: "
                                          << numReadThisBlock << " numSpaceLeftThisBlock: "
                                          << numSpaceLeftThisBuffer << " total numPointsRead: "
                                          << numPointsRead << std::endl;

        numPointsRead = numPointsRead + numReadThisBlock;

        readBlob(m_initialQueryStatement, m_block, m_block->num_points);
        fillUserBuffer(user_buffer);
        if (m_buffer_position != 0)
        {
            return user_buffer.getNumPoints();
        }
        else
        {
            bDidRead = m_initialQueryStatement->Fetch();
            if (!bDidRead)
            {
                getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: done reading block. Read " << numPointsRead << " points" << std::endl;
                m_at_end = true;
                return user_buffer.getNumPoints();
            }
        }

        boost::int32_t current_cloud_id(0);
        current_cloud_id  = m_initialQueryStatement->GetInteger(&m_block->pc->pc_id);

        getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: current_cloud_id: "
                                          << current_cloud_id << " m_active_cloud_id: "
                                          << m_active_cloud_id << std::endl;

        if (current_cloud_id != m_active_cloud_id)
        {
            m_oracle_buffer = fetchPointBuffer(m_initialQueryStatement, m_block->pc);

            m_active_cloud_id = current_cloud_id;
            return user_buffer.getNumPoints();
        }

    }


    return numPointsRead;
}

pdal::Bounds<double> IteratorBase::getBounds(Statement statement, BlockPtr block)
{
    pdal::Vector<double> mins;
    pdal::Vector<double> maxs;

    boost::int32_t bounds_length = statement->GetArrayLength(&(block->blk_extent->sdo_ordinates));

    getReader().log()->get(logDEBUG3) << "IteratorBase::getBounds: bounds length " << bounds_length << std::endl;

    double x(0.0);
    double y(0.0);

    statement->GetElement(&(block->blk_extent->sdo_ordinates), 0, &x);
    mins.add(x);
    statement->GetElement(&(block->blk_extent->sdo_ordinates), 1, &y);
    mins.add(y);
    statement->GetElement(&(block->blk_extent->sdo_ordinates), 2, &x);
    maxs.add(x);
    statement->GetElement(&(block->blk_extent->sdo_ordinates), 3, &y);
    maxs.add(y);

    pdal::Bounds<double> block_bounds(mins, maxs);

    getReader().log()->get(logDEBUG2) << "IteratorBase::getBounds: Fetched bounds of " << block_bounds << std::endl;
    return block_bounds;
}



//---------------------------------------------------------------------------
//
// SequentialIterator
//
//---------------------------------------------------------------------------

Reader::Reader(const pdal::drivers::oci::Reader& reader, PointBuffer& buffer)
    : IteratorBase(reader)
    , pdal::StageSequentialIterator(reader, buffer)
{
    return;
}


Reader::~Reader()
{
    return;
}


boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{

    return naiveSkipImpl(count);
}


bool Reader::atEndImpl() const
{
    return m_at_end;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    return myReadBuffer(data);
}



}
} // iterators::sequential::

}
}
} // namespace pdal::driver::oci
