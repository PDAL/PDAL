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

#include <pdal/GDALUtils.hpp>
#include <pdal/GlobalEnvironment.hpp>
#include <pdal/drivers/oci/OciReader.hpp>
#include <pdal/drivers/oci/OciSeqIterator.hpp>

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

void OciReader::processOptions(const Options& options)
{
    m_schemaFile = options.getValueOrDefault<std::string>(
        "xml_schema_dump", std::string());
    m_normalizeXYZ = options.getValueOrDefault<bool>("do_normalize_xyz", true);
    m_setPointSourceId = options.getValueOrDefault<bool>("populate_pointsourceid", false);
    if (options.hasOption("scale_x"))
        m_scaleX = boost::optional<double>(
            options.getValueOrThrow<double>("scale_x"));
    if (options.hasOption("scale_y"))
        m_scaleY = boost::optional<double>(
            options.getValueOrThrow<double>("scale_y"));
    if (options.hasOption("scale_z"))
        m_scaleZ= boost::optional<double>(
            options.getValueOrThrow<double>("scale_z"));

    if (options.hasOption("offset_x"))
        m_offsetX = boost::optional<double>(
            options.getValueOrThrow<double>("offset_x"));
    if (options.hasOption("offset_y"))
        m_offsetX = boost::optional<double>(
            options.getValueOrThrow<double>("offset_y"));
    if (options.hasOption("offset_z"))
        m_offsetX = boost::optional<double>(
            options.getValueOrThrow<double>("offset_z"));

    if (options.hasOption("spatialreference"))
        m_spatialRef = boost::optional<SpatialReference>(
            options.getValueOrThrow<pdal::SpatialReference>(
                "spatialreference"));
    m_query = options.getValueOrThrow<std::string>("query");
    m_connSpec = options.getValueOrDefault<std::string>("connection", "");
}

void OciReader::initialize()
{
    pdal::GlobalEnvironment::get().getGDALDebug()->addLog(log());
    m_connection = connect(m_connSpec);
    m_block = BlockPtr(new Block(m_connection));

    if (m_query.empty())
        throw pdal_error("'query' statement is empty. No data can be read "
            "from pdal::drivers::oci::Reader");

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

    Option do_normalize_xyz("do_normalize_xyz", true, "Normalize XYZ "
        "dimensions from selections that have varying scale/offsets");

    options.add(connection);
    options.add(query);
    options.add(xml_schema_dump);
    options.add(do_normalize_xyz);

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
        log()->get(logDEBUG) << "Fetched field '" << fieldName <<
            "' of type '" << typeName << "'"<< std::endl;

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


void OciReader::buildSchema(Schema *schema)
{
    log()->get(logDEBUG) << "Fetching schema from SDO_PC object" << std::endl;

    Schema storedSchema = fetchSchema(m_stmt, m_block);
    if (m_schemaFile.size())
    {
        std::string pcSchema = Schema::to_xml(storedSchema);
        std::ostream* out = FileUtils::createFile(m_schemaFile);
        out->write(pcSchema.c_str(), pcSchema.size());
        FileUtils::closeFile(out);
    }

    for (size_t i = 0; i < storedSchema.numDimensions(); ++i)
    {
        Dimension d = storedSchema.getDimension(i);

        // For dimensions that do not have namespaces, we'll set the namespace
        // to the namespace of the current stage
        if (d.getNamespace().empty())
        {
            log()->get(logDEBUG4) << "setting namespace for dimension " <<
                d.getName() << " to "  << getName() << std::endl;

            if (d.getUUID().is_nil())
                d.createUUID();
            d.setNamespace(getName());
        }
        m_dims.push_back(schema->appendDimension(d));
    }

    if (m_normalizeXYZ)
    {
        auto optionSetter = [](Schema *s, std::string dimName,
            boost::optional<double> scaleOp, boost::optional<double> offsetOp)
        {
            Dimension *d = s->getDimensionPtr(dimName);
            if (scaleOp)
                d->setNumericScale(*scaleOp);
            if (offsetOp)
                d->setNumericScale(*offsetOp);
        };

        optionSetter(schema, "drivers.oci.reader.X", m_scaleX, m_offsetX);
        optionSetter(schema, "drivers.oci.reader.Y", m_scaleY, m_offsetY);
        optionSetter(schema, "drivers.oci.reader.Z", m_scaleZ, m_offsetZ);
    }
}


StageSequentialIterator* OciReader::createSequentialIterator() const
{
    using namespace pdal::drivers::oci::iterators::sequential;

    return new OciSeqIterator(m_stmt, m_block, m_dims, m_normalizeXYZ, m_setPointSourceId);
}


} // namespace oci
} // namespace drivers
} // namespace pdal
