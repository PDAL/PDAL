/******************************************************************************
 * Copyright (c) 2019 TileDB, Inc.
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

#include <cctype>
#include <limits>
#include <string.h>

#include <nlohmann/json.hpp>

#include <pdal/util/FileUtils.hpp>

#include "TileDBUtils.hpp"
#include "TileDBWriter.hpp"

const char pathSeparator =
#ifdef _WIN32
    '\\';
#else
    '/';
#endif

namespace pdal
{

static PluginInfo const s_info{
    "writers.tiledb", "Write data using TileDB.",
    "http://pdal.io/stages/drivers.tiledb.writer.html"};

struct TileDBWriter::Args
{
    std::string m_arrayName;
    std::string m_cfgFileName;
    size_t m_tile_capacity;
    size_t m_x_tile_size;
    size_t m_y_tile_size;
    size_t m_z_tile_size;
    size_t m_time_tile_size;
    float m_x_domain_st;
    float m_x_domain_end;
    float m_y_domain_st;
    float m_y_domain_end;
    float m_z_domain_st;
    float m_z_domain_end;
    float m_time_domain_st;
    float m_time_domain_end;
    size_t m_cache_size;
    bool m_stats;
    std::string m_compressor;
    int m_compressionLevel;
    NL::json m_filters;
    bool m_append;
    uint64_t m_timeStamp = UINT64_MAX;
};

CREATE_SHARED_STAGE(TileDBWriter, s_info)

void writeAttributeValue(TileDBWriter::DimBuffer& dim, PointRef& point,
                         size_t idx)
{
    Everything e;

    switch (dim.m_type)
    {
    case Dimension::Type::Double:
        e.d = point.getFieldAs<double>(dim.m_id);
        break;
    case Dimension::Type::Float:
        e.f = point.getFieldAs<float>(dim.m_id);
        break;
    case Dimension::Type::Signed8:
        e.s8 = point.getFieldAs<int8_t>(dim.m_id);
        break;
    case Dimension::Type::Signed16:
        e.s16 = point.getFieldAs<int16_t>(dim.m_id);
        break;
    case Dimension::Type::Signed32:
        e.s32 = point.getFieldAs<int32_t>(dim.m_id);
        break;
    case Dimension::Type::Signed64:
        e.s64 = point.getFieldAs<int64_t>(dim.m_id);
        break;
    case Dimension::Type::Unsigned8:
        e.u8 = point.getFieldAs<uint8_t>(dim.m_id);
        break;
    case Dimension::Type::Unsigned16:
        e.u16 = point.getFieldAs<uint16_t>(dim.m_id);
        break;
    case Dimension::Type::Unsigned32:
        e.u32 = point.getFieldAs<uint32_t>(dim.m_id);
        break;
    case Dimension::Type::Unsigned64:
        e.u64 = point.getFieldAs<uint64_t>(dim.m_id);
        break;
    default:
        throw pdal_error("Unsupported attribute type for " + dim.m_name);
    }

    size_t size = Dimension::size(dim.m_type);
    memcpy(dim.m_buffer.data() + (idx * size), &e, size);
}

tiledb::Attribute createAttribute(const tiledb::Context& ctx,
                                  const std::string name, Dimension::Type t)
{
    switch (t)
    {
    case Dimension::Type::Double:
        return tiledb::Attribute::create<double>(ctx, name);
    case Dimension::Type::Float:
        return tiledb::Attribute::create<float>(ctx, name);
    case Dimension::Type::Signed8:
        return tiledb::Attribute::create<char>(ctx, name);
    case Dimension::Type::Signed16:
        return tiledb::Attribute::create<short>(ctx, name);
    case Dimension::Type::Signed32:
        return tiledb::Attribute::create<int>(ctx, name);
    case Dimension::Type::Signed64:
        return tiledb::Attribute::create<long>(ctx, name);
    case Dimension::Type::Unsigned8:
        return tiledb::Attribute::create<unsigned char>(ctx, name);
    case Dimension::Type::Unsigned16:
        return tiledb::Attribute::create<unsigned short>(ctx, name);
    case Dimension::Type::Unsigned32:
        return tiledb::Attribute::create<unsigned int>(ctx, name);
    case Dimension::Type::Unsigned64:
        return tiledb::Attribute::create<unsigned long>(ctx, name);
    case Dimension::Type::None:
    default:
        throw pdal_error("Unsupported attribute type for " + name);
    }
}

TileDBWriter::TileDBWriter() : m_args(new TileDBWriter::Args) {}

TileDBWriter::~TileDBWriter() {}

std::string TileDBWriter::getName() const
{
    return s_info.name;
}

void TileDBWriter::addArgs(ProgramArgs& args)
{
    args.add("array_name", "TileDB array name", m_args->m_arrayName)
        .setPositional();
    args.addSynonym("array_name", "filename");
    args.add("config_file", "TileDB configuration file location",
             m_args->m_cfgFileName);
    args.add("data_tile_capacity", "TileDB tile capacity",
             m_args->m_tile_capacity, size_t(100000));
    args.add("x_tile_size", "TileDB tile size", m_args->m_x_tile_size,
             size_t(0));
    args.add("y_tile_size", "TileDB tile size", m_args->m_y_tile_size,
             size_t(0));
    args.add("z_tile_size", "TileDB tile size", m_args->m_z_tile_size,
             size_t(0));
    args.add("time_tile_size", "TileDB tile size", m_args->m_time_tile_size,
             size_t(0));
    args.add("x_domain_st", "TileDB start of domain in X",
             m_args->m_x_domain_st, 0.f);
    args.add("x_domain_end", "TileDB end of domain in X",
             m_args->m_x_domain_end, 0.f);
    args.add("y_domain_st", "TileDB start of domain in Y",
             m_args->m_y_domain_st, 0.f);
    args.add("y_domain_end", "TileDB end of domain in Y",
             m_args->m_y_domain_end, 0.f);
    args.add("z_domain_st", "TileDB start of domain in Z",
             m_args->m_z_domain_st, 0.f);
    args.add("z_domain_end", "TileDB end of domain in Z",
             m_args->m_z_domain_end, 0.f);
    args.add("time_domain_st", "TileDB start of domain in GpsTime",
             m_args->m_time_domain_st, 0.f);
    args.add("time_domain_end", "TileDB end of domain in GpsTime",
             m_args->m_time_domain_end, 0.f);
    args.add("chunk_size", "Point cache size for chunked writes",
             m_args->m_cache_size, size_t(10000));
    args.add("stats", "Dump TileDB query stats to stdout", m_args->m_stats,
             false);
    args.add("compression", "TileDB compression type for attributes",
             m_args->m_compressor);
    args.add("compression_level", "TileDB compression level",
             m_args->m_compressionLevel, -1);
    args.add("filters", "Specify filter and level per dimension/attribute",
             m_args->m_filters, NL::json({}));
    args.add("append", "Append to existing TileDB array", m_args->m_append,
             false);
    args.add("use_time_dim", "Use GpsTime coordinate data as array dimension",
             m_use_time, false);
    args.addSynonym("use_time_dim", "use_time");
    args.add("time_first",
             "If writing 4D array with XYZ and Time, choose to put time dim "
             "first or last (default)",
             m_time_first, false);
    args.add<uint64_t>("timestamp", "TileDB array timestamp",
                       m_args->m_timeStamp, UINT64_MAX);
}

void TileDBWriter::initialize()
{
    try
    {
        if (!m_args->m_cfgFileName.empty())
        {
            tiledb::Config cfg(m_args->m_cfgFileName);
            m_ctx.reset(new tiledb::Context(cfg));
        }
        else
            m_ctx.reset(new tiledb::Context());
    }
    catch (const tiledb::TileDBError& err)
    {
        throwError(std::string("TileDB Error: ") + err.what());
    }
}

void TileDBWriter::ready(pdal::BasePointTable& table)
{
    // Create objects to store the buffer data.
    auto layout = table.layout();
    for (const auto& dimId : layout->dims())
    {
        // Get the name and type of the dim.
        std::string dimName = layout->dimName(dimId);
        Dimension::Type dimType = layout->dimType(dimId);

        // Allocate the buffer.
        m_buffers.emplace_back(dimName, dimId, dimType);
        m_buffers.back().m_buffer.resize(m_args->m_cache_size *
                                         Dimension::size(dimType));
    }

    // Enable TileDB stats (if requested).
    if (m_args->m_stats)
        tiledb::Stats::enable();

    // If not appending to an existing array, then create the TileDB array.
    if (!m_args->m_append)
    {
        // Create schema and set basic properties.
        tiledb::ArraySchema schema{*m_ctx, TILEDB_SPARSE};
        schema.set_allows_dups(true);
        schema.set_capacity(m_args->m_tile_capacity);

        // Check if using Hilbert order or row-major order. Use row-major if all
        // dimensions have positive tiles set. Otherwise, use Hilbert order.
        bool hasValidTiles =
            ((m_args->m_x_tile_size > 0) && (m_args->m_y_tile_size > 0) &&
             (m_args->m_z_tile_size > 0) &&
             (!m_use_time || m_args->m_time_tile_size > 0));
        if (!hasValidTiles)
            schema.set_cell_order(TILEDB_HILBERT);

        // Get filter factory class.
        FilterFactory filterFactory{m_args->m_filters, m_args->m_compressor,
                                    m_args->m_compressionLevel};

        // Check if the domain is set for all dimensions.
        bool hasValidDomain = (m_args->m_x_domain_end > m_args->m_x_domain_st &&
                               m_args->m_y_domain_end > m_args->m_y_domain_st &&
                               m_args->m_z_domain_end > m_args->m_z_domain_st &&
                               (!m_use_time || m_args->m_time_domain_end >
                                                   m_args->m_time_domain_st));

        tiledb::Domain domain(*m_ctx);

        // Set the domain values for the dimensions. Use the user provided
        // domain values, and update if they are the default domain or
        // otherwise not valid.
        std::array<std::array<double, 2>, 4> bbox{
            {{m_args->m_x_domain_st, m_args->m_x_domain_end},
             {m_args->m_y_domain_st, m_args->m_y_domain_end},
             {m_args->m_z_domain_st, m_args->m_z_domain_end},
             {m_args->m_time_domain_st, m_args->m_time_domain_end}}};
        if (!hasValidDomain)
        {
            // Get table metadata and check if it is valid.
            MetadataNode meta =
                table.metadata().findChild("filters.stats:bbox:native:bbox");
            bool hasMetadataStats = meta.valid();

            // Check the user set valid tile extents, valid domains, or ran
            // stats on the point table.
            if (!hasValidTiles && !hasMetadataStats)
                throwError("Must specify a tile extent for all dimensions, "
                           "specify a valid domain for all dimensions, or "
                           "execute a prior stats filter stage.");

            if (hasMetadataStats)
            {
                // Update any missing domains using table statistics.
                auto updateWithStats = [&](const std::string& minStr,
                                           const std::string& maxStr,
                                           std::array<double, 2>& range)
                {
                    if (range[1] <= range[0])
                        range = {meta.findChild(minStr).value<double>() - 1.0,
                                 meta.findChild(maxStr).value<double>() + 1.0};
                };
                updateWithStats("minx", "maxx", bbox[0]);
                updateWithStats("miny", "maxy", bbox[1]);
                updateWithStats("minz", "maxz", bbox[2]);
                if (m_use_time)
                    updateWithStats("mintm", "maxtm", bbox[3]);
            }
            else
            {
                // Update any missing domains to be the entire space.
                for (auto& range : bbox)
                    if (range[1] <= range[0])
                        range = {std::numeric_limits<double>::lowest(),
                                 std::numeric_limits<double>::max()};
            }
        }

        // Create and add dimensions to the TileDB domain.
        if (hasValidTiles)
        {
            auto addDimension = [&](const std::string& dimName,
                                    const std::array<double, 2>& range,
                                    double tile_size)
            {
                auto filters = filterFactory.filterList(*m_ctx, dimName);
                domain.add_dimension(tiledb::Dimension::create<double>(
                                         *m_ctx, dimName, range, tile_size)
                                         .set_filter_list(filters));
            };
            if (m_use_time && m_time_first)
                addDimension("GpsTime", bbox[3], m_args->m_time_tile_size);
            addDimension("X", bbox[0], m_args->m_x_tile_size);
            addDimension("Y", bbox[1], m_args->m_y_tile_size);
            addDimension("Z", bbox[2], m_args->m_z_tile_size);
            if (m_use_time && !m_time_first)
                addDimension("GpsTime", bbox[3], m_args->m_time_tile_size);
        }
        else
        {
            auto addDimension = [&](const std::string& dimName,
                                    const std::array<double, 2>& range)
            {
                auto filters = filterFactory.filterList(*m_ctx, dimName);
                domain.add_dimension(
                    tiledb::Dimension::create<double>(*m_ctx, dimName, range)
                        .set_filter_list(filters));
            };
            if (m_use_time && m_time_first)
                addDimension("GpsTime", bbox[3]);
            addDimension("X", bbox[0]);
            addDimension("Y", bbox[1]);
            addDimension("Z", bbox[2]);
            if (m_use_time && !m_time_first)
                addDimension("GpsTime", bbox[3]);
        }

        schema.set_domain(domain);

        // Set the attributes.
        for (const auto& dimBuffer : m_buffers)
        {
            // Create and add the attribute to the domain.
            if (!domain.has_dimension(dimBuffer.m_name))
            {
                tiledb::Attribute att =
                    createAttribute(*m_ctx, dimBuffer.m_name, dimBuffer.m_type);
                att.set_filter_list(
                    filterFactory.filterList(*m_ctx, dimBuffer.m_name));
                schema.add_attribute(att);
            }
        }

        // Create the TileDB array.
        tiledb::Array::create(m_args->m_arrayName, schema);
    }

    // Open the array at the requested timestamp range.
#if TILEDB_VERSION_MINOR < 15
    if (m_args->m_timeStamp != UINT64_MAX)
        m_array.reset(new tiledb::Array(*m_ctx, m_args->m_arrayName,
                                        TILEDB_WRITE, m_args->m_timeStamp));
    else
        m_array.reset(
            new tiledb::Array(*m_ctx, m_args->m_arrayName, TILEDB_WRITE));
#else
    m_array.reset(
        new tiledb::Array(*m_ctx, m_args->m_arrayName, TILEDB_WRITE,
                          {tiledb::TimeTravelMarker(), m_args->m_timeStamp}));
#endif
    const auto& schema = m_array->schema();

    // Check if GpsTime is a dimension.
    if (schema.domain().has_dimension("GpsTime"))
        m_use_time = true;

    for (const auto& dimBuffer : m_buffers)
    {
        // If appending, check the layout dim exists in the original schema.
        if (m_args->m_append && !schema.has_attribute(dimBuffer.m_name) &&
            !schema.domain().has_dimension(dimBuffer.m_name))
            throwError("Attribute/Dimension '" + dimBuffer.m_name +
                       "' does not exist in original array.");
    }

    m_current_idx = 0;
}

bool TileDBWriter::processOne(PointRef& point)
{

    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);
    double tm(0);
    if (m_use_time)
        tm = point.getFieldAs<double>(Dimension::Id::GpsTime);

    for (auto& a : m_buffers)
        writeAttributeValue(a, point, m_current_idx);

    m_xs.push_back(x);
    m_ys.push_back(y);
    m_zs.push_back(z);
    if (m_use_time)
        m_tms.push_back(tm);

    if (++m_current_idx == m_args->m_cache_size)
    {
        if (!flushCache(m_current_idx))
        {
            throwError("Unable to flush points to TileDB array");
        }
    }

    return true;
}

void TileDBWriter::write(const PointViewPtr view)
{
    PointRef point(*view, 0);
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}

void TileDBWriter::done(PointTableRef table)
{
    if (flushCache(m_current_idx))
    {
        if (!m_args->m_append)
        {
            if (!getSpatialReference().empty() &&
                table.spatialReferenceUnique())
            {
                // The point view takes on the spatial reference of that stage,
                // if it had one.
                getMetadata().add("spatialreference",
                                  Utils::toString(getSpatialReference()));
            }

            MetadataNode node = table.metadata();

            // serialize metadata
            std::string m = pdal::Utils::toJSON(node);
            m_array->put_metadata("_pdal", TILEDB_UINT8, m.length() + 1,
                                  m.c_str());
        }
        m_array->close();
    }
    else
    {
        throwError("Unable to flush points to TileDB array");
    }
}

bool TileDBWriter::flushCache(size_t size)
{
    tiledb::Query query(*m_ctx, *m_array);
    query.set_layout(TILEDB_UNORDERED);

    query.set_data_buffer("X", m_xs);
    query.set_data_buffer("Y", m_ys);
    query.set_data_buffer("Z", m_zs);

    if (m_use_time)
        query.set_data_buffer("GpsTime", m_tms);

    // set tiledb buffers
    for (const auto& a : m_buffers)
    {
        uint8_t* buf = const_cast<uint8_t*>(a.m_buffer.data());
        switch (a.m_type)
        {
        case Dimension::Type::Double:
            query.set_data_buffer(a.m_name, reinterpret_cast<double*>(buf),
                                  size);
            break;
        case Dimension::Type::Float:
            query.set_data_buffer(a.m_name, reinterpret_cast<float*>(buf),
                                  size);
            break;
        case Dimension::Type::Signed8:
            query.set_data_buffer(a.m_name, reinterpret_cast<int8_t*>(buf),
                                  size);
            break;
        case Dimension::Type::Signed16:
            query.set_data_buffer(a.m_name, reinterpret_cast<int16_t*>(buf),
                                  size);
            break;
        case Dimension::Type::Signed32:
            query.set_data_buffer(a.m_name, reinterpret_cast<int32_t*>(buf),
                                  size);
            break;
        case Dimension::Type::Signed64:
            query.set_data_buffer(a.m_name, reinterpret_cast<int64_t*>(buf),
                                  size);
            break;
        case Dimension::Type::Unsigned8:
            query.set_data_buffer(a.m_name, reinterpret_cast<uint8_t*>(buf),
                                  size);
            break;
        case Dimension::Type::Unsigned16:
            query.set_data_buffer(a.m_name, reinterpret_cast<uint16_t*>(buf),
                                  size);
            break;
        case Dimension::Type::Unsigned32:
            query.set_data_buffer(a.m_name, reinterpret_cast<uint32_t*>(buf),
                                  size);
            break;
        case Dimension::Type::Unsigned64:
            query.set_data_buffer(a.m_name, reinterpret_cast<uint64_t*>(buf),
                                  size);
            break;
        case Dimension::Type::None:
        default:
            throw pdal_error("Unsupported attribute type for " + a.m_name);
        }
    }

    tiledb::Query::Status status = query.submit();

    if (m_args->m_stats)
    {
        tiledb::Stats::dump(stdout);
        tiledb::Stats::reset();
    }

    m_current_idx = 0;
    m_xs.clear();
    m_ys.clear();
    m_zs.clear();
    m_tms.clear();

    if (status == tiledb::Query::Status::FAILED)
        return false;
    else
        return true;
}

} // namespace pdal
