/******************************************************************************
* Copyright (c) 2018, RIEGL Laser Measurement Systems GmbH (support@riegl.com)
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
*     * Neither the name of Hobu, Inc., Flaxen Geo Consulting or RIEGL
*       Laser Measurement Systems GmbH nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <array>
#include <sstream>
#include <nlohmann/json.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/PDALUtils.hpp>
#include "RdbReader.hpp"

namespace pdal
{

//---< PDAL PLUGIN REGISTRATION >-----------------------------------------------


static PluginInfo const s_info
{
    "readers.rdb",
    "RIEGL RDB Reader",
    "http://pdal.io/stages/readers.rdb.html"
};

CREATE_SHARED_STAGE(RdbReader, s_info)


//---< RdbReader::PUBLIC >------------------------------------------------------


RdbReader::RdbReader():
    pdal::Reader(),
    pdal::Streamable(),
    m_pointcloud(),
    m_filter(),
    m_extras(false)
{
}


RdbReader::~RdbReader()
{
}


std::string RdbReader::getName() const
{
    return s_info.name;
}


//---< RdbReader::PRIVATE >------------------------------------------------------


QuickInfo RdbReader::inspect()
{
    using namespace riegl::rdb::pointcloud;

    if (pdal::Utils::isRemote(m_filename))
        m_filename = pdal::Utils::fetchRemote(m_filename);

    RdbPointcloud reader(m_filename, m_filter, m_extras);
    riegl::rdb::Pointcloud& rdb = reader.pointcloud();
    {
        // query index
        QuickInfo result;
        QueryStat stat = rdb.stat();
        GraphNode root = stat.index();
        result.m_pointCount = root.pointCountTotal;

        // query spatial reference system
        result.m_srs.set(getSpatialReferenceSystem(reader));

        // query dimensions
        PointLayout layout;
        reader.addDimensions(&layout);
        const auto dimensions = layout.dims();
        for (const auto& dimension: dimensions)
        {
            result.m_dimNames.push_back(layout.dimName(dimension));
        }

        // query XYZ bounds (if available)
        Eigen::Vector4d minimum, maximum;
        if (reader.getBoundingBox(minimum, maximum))
        {
            result.m_bounds = BOX3D(
                minimum[0], minimum[1], minimum[2],
                maximum[0], maximum[1], maximum[2]
            );
        }

        // finalize result
        result.m_valid = true;
        return result;
    }
}


void RdbReader::addArgs(ProgramArgs& args)
{
    args.add(
        "filter",
        "Optional point filter expression string "
        "(see RDB SDK documentation for details)",
        m_filter,
        ""
    );
    args.add(
        "extras",
        "Read all available dimensions (true) "
        "or known PDAL dimensions only (false)",
        m_extras,
        false
    );
}


void RdbReader::initialize()
{
    if (pdal::Utils::isRemote(m_filename))
        m_filename = pdal::Utils::fetchRemote(m_filename);

    m_pointcloud.reset(new RdbPointcloud(m_filename, m_filter, m_extras));
    // Set spatial reference form source if not overridden.
    if (getSpatialReference().empty())
        setSpatialReference(getSpatialReferenceSystem(*m_pointcloud));
    readMetadata(*m_pointcloud, getMetadata());
}


void RdbReader::addDimensions(PointLayoutPtr layout)
{
    m_pointcloud->addDimensions(layout);
}


bool RdbReader::processOne(PointRef& point)
{
    return m_pointcloud->read(point);
}


point_count_t RdbReader::read(PointViewPtr view, point_count_t count)
{
    return m_pointcloud->read(view, count);
}


void RdbReader::done(PointTableRef /*table*/)
{
    m_pointcloud.reset();
}


void RdbReader::readMetadata(RdbPointcloud &reader, MetadataNode root)
{
    struct Converter // simple JSON to MetadataNode converter
    {
        static void add(
            MetadataNode&      parent,
            const std::string& name,
            const std::string& value
        )
        {
            try
            {
                NL::json node = NL::json::parse(value);
                add(parent, name, node);
            }
            catch (const NL::json::parse_error&)
            {
                parent.add(name, value);
            }
        }

        static void add(
            MetadataNode&      parent,
            const std::string& name,
            const NL::json& node
        )
        {
            if (node.is_null())
                parent.add(name, "");
            else if (node.is_boolean())
                parent.add(name, node.get<bool>());
            else if (node.is_number_unsigned())
                parent.add(name, node.get<uint64_t>());
            else if (node.is_number_integer())
                parent.add(name, node.get<int64_t>());
            else if (node.is_number_float())
                parent.add(name, node.get<double>());
            else if (node.is_string())
                parent.add(name, node.get<std::string>());
            else if (node.is_object())
            {
                MetadataNode object = parent.add(name);
                for (auto it : node.items())
                    add(object, it.key(), it.value());
            }
            else if (node.is_array())
            {
                for (size_t i = 0; i < node.size(); ++i)
                    add(parent, name, node.at(i));
            }
        }
    };

    // get database object
    using namespace riegl::rdb::pointcloud;
    riegl::rdb::Pointcloud& rdb = reader.pointcloud();

    // basic database information
    {
        QueryStat stat = rdb.stat();
        GraphNode tree = stat.index();

        // database ID and number of points
        MetadataNode node = root.add("database");
        node.add("uuid",   rdb.getUUID());
        node.add("points", tree.pointCountTotal);

        // XYZ bounds (if available)
        Eigen::Vector4d minimum, maximum;
        if (reader.getBoundingBox(minimum, maximum))
        {
            MetadataNode bounds = node.add("bounds");
            MetadataNode min = bounds.add("minimum");
            MetadataNode max = bounds.add("maximum");
            min.add("X", minimum[0]);    max.add("X", maximum[0]);
            min.add("Y", minimum[1]);    max.add("Y", maximum[1]);
            min.add("Z", minimum[2]);    max.add("Z", maximum[2]);
        }
    }

    // database transactions
    {
        const auto list = rdb.transaction().list();
        for (const auto& item: list)
        {
            const auto details = rdb.transaction().details(item);
            MetadataNode node = root.add("transactions");
            node.add("id",       details.id);
            node.add("rdb",      details.rdb);
            node.add("title",    details.title);
            node.add("agent",    details.agent);
            node.add("comments", details.comments);
            node.add("start",    details.start);
            node.add("stop",     details.stop);
            Converter::add(node, "settings", details.settings);
        }
    }

    // point attributes
    {
        const auto list = rdb.pointAttribute().list();
        for (const auto& item: list)
        {
            const auto details = rdb.pointAttribute().get(item);
            Converter::add(root, "dimensions", details.save());
        }
    }

    // meta data
    {
        MetadataNode node = root.add("metadata");
        const auto list = rdb.metaData().list();
        for (const auto& item: list)
        {
            const auto details = rdb.metaData().get(item);
            Converter::add(node, item, details);
        }
    }
}


std::string RdbReader::getSpatialReferenceSystem(const RdbPointcloud &reader)
{
    std::stringstream srs;
    if (reader.crsEPSG() != 0)
    {
        srs << "EPSG:" << reader.crsEPSG();
    }
    else if (!reader.crsWKT().empty())
    {
        srs << reader.crsWKT();
    }
    return srs.str();
}

} // namespace pdal
