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

#include <algorithm>
#include <cctype>
#include <limits>
#include <optional>
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
    std::string m_cell_order;
    std::string m_tile_order;
    size_t m_tile_capacity;
    double m_x_tile_size;
    double m_y_tile_size;
    double m_z_tile_size;
    double m_time_tile_size;
    double m_x_domain_st;
    double m_x_domain_end;
    double m_y_domain_st;
    double m_y_domain_end;
    double m_z_domain_st;
    double m_z_domain_end;
    double m_time_domain_st;
    double m_time_domain_end;
    std::array<double, 3> m_scale;
    std::array<double, 3> m_offset;
    size_t m_cache_size;
    bool m_stats;
    std::string m_filter_profile;
    std::string m_compressor;
    int m_compressionLevel;
    NL::json m_filters;
    bool m_append;
    bool m_use_time;
    bool m_time_first;
    bool m_combine_bit_fields;
    uint64_t m_timeStamp = UINT64_MAX;
};

CREATE_SHARED_STAGE(TileDBWriter, s_info)

TileDBWriter::TileDBWriter() : m_args(new TileDBWriter::Args) {}

TileDBWriter::~TileDBWriter() {}

std::string TileDBWriter::getName() const
{
    return s_info.name;
}

void TileDBWriter::addArgs(ProgramArgs& args)
{
    args.addSynonym("filename", "array_name");
    args.add("config_file", "TileDB configuration file location",
             m_args->m_cfgFileName);
    args.add("cell_order", "TileDB cell order", m_args->m_cell_order, "auto");
    args.add("tile_order", "TileDB tile order", m_args->m_tile_order,
             "row-major");
    args.add("data_tile_capacity", "TileDB tile capacity",
             m_args->m_tile_capacity, size_t(100000));
    args.add<double>("x_tile_size", "TileDB tile size", m_args->m_x_tile_size,
                     0.0);
    args.add<double>("y_tile_size", "TileDB tile size", m_args->m_y_tile_size,
                     0.0);
    args.add<double>("z_tile_size", "TileDB tile size", m_args->m_z_tile_size,
                     0.0);
    args.add<double>("time_tile_size", "TileDB tile size",
                     m_args->m_time_tile_size, 0.0);
    args.add<double>("x_domain_st", "TileDB start of domain in X",
                     m_args->m_x_domain_st, 0.0);
    args.add<double>("x_domain_end", "TileDB end of domain in X",
                     m_args->m_x_domain_end, 0.0);
    args.add<double>("y_domain_st", "TileDB start of domain in Y",
                     m_args->m_y_domain_st, 0.0);
    args.add<double>("y_domain_end", "TileDB end of domain in Y",
                     m_args->m_y_domain_end, 0.0);
    args.add<double>("z_domain_st", "TileDB start of domain in Z",
                     m_args->m_z_domain_st, 0.0);
    args.add<double>("z_domain_end", "TileDB end of domain in Z",
                     m_args->m_z_domain_end, 0.0);
    args.add<double>("time_domain_st", "TileDB start of domain in GpsTime",
                     m_args->m_time_domain_st, 0.0);
    args.add<double>("time_domain_end", "TileDB end of domain in GpsTime",
                     m_args->m_time_domain_end, 0.0);
    args.add<double>("scale_x",
                     "Scale factor to use for default x float-scale filter",
                     m_args->m_scale[0], 0.01);
    args.add<double>("scale_y",
                     "Scale factor to use for default y float-scale fitler",
                     m_args->m_scale[1], 0.01);
    args.add<double>("scale_z",
                     "Scale factor to use for default z float-scale filter",
                     m_args->m_scale[2], 0.01);
    args.add<double>("offset_x",
                     "Add offset to use for default x float-scale filter",
                     m_args->m_offset[0], 0.0);
    args.add<double>("offset_y",
                     "Add offset to use for default y float-scale filter",
                     m_args->m_offset[1], 0.0);
    args.add<double>("offset_z",
                     "Add offset to use fo default x float-scale filter",
                     m_args->m_offset[2], 0.0);
    args.add("chunk_size", "Point cache size for chunked writes",
             m_args->m_cache_size, size_t(1000000));
    args.add("stats", "Dump TileDB query stats to stdout", m_args->m_stats,
             false);
    args.add("filter_profile", "Filter profile to use for compression filters",
             m_args->m_filter_profile, "balanced");
    args.add("compression", "TileDB compression type for attributes",
             m_args->m_compressor);
    args.add("compression_level", "TileDB compression level",
             m_args->m_compressionLevel, -1);
    args.add("filters", "Specify filter and level per dimension/attribute",
             m_args->m_filters, NL::json({}));
    args.add("append", "Append to existing TileDB array", m_args->m_append,
             false);
    args.add("use_time_dim", "Use GpsTime coordinate data as array dimension",
             m_args->m_use_time, false);
    args.addSynonym("use_time_dim", "use_time");
    args.add("time_first",
             "If writing 4D array with XYZ and Time, choose to put time dim "
             "first or last (default)",
             m_args->m_time_first, false);
    args.add<uint64_t>("timestamp", "TileDB array timestamp",
                       m_args->m_timeStamp, UINT64_MAX);
    args.add("combine_bit_fields",
             "Combine all bit fields into a single 2 byte attribute",
             m_args->m_combine_bit_fields, true);
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
    bool hasBitFields = false;
    std::array<std::optional<Dimension::Id>, 6> bitFieldIds;
    for (const auto& dimId : layout->dims())
    {
        if (m_args->m_combine_bit_fields)
        {
            const auto& dimName = layout->dimName(dimId);
            auto bitIndex = BitFieldsBuffer::bitFieldIndex(dimName);
            if (bitIndex.has_value())
            {
                hasBitFields = true;
                bitFieldIds[bitIndex.value()] = dimId;
                continue;
            }
        }
        // Allocate the buffer.
        switch (layout->dimType(dimId))
        {
        case Dimension::Type::Double:
            m_buffers.emplace_back(new TypedDimBuffer<double>(layout, dimId));
            break;
        case Dimension::Type::Float:
            m_buffers.emplace_back(new TypedDimBuffer<float>(layout, dimId));
            break;
        case Dimension::Type::Signed8:
            m_buffers.emplace_back(new TypedDimBuffer<int8_t>(layout, dimId));
            break;
        case Dimension::Type::Signed16:
            m_buffers.emplace_back(new TypedDimBuffer<int16_t>(layout, dimId));
            break;
        case Dimension::Type::Signed32:
            m_buffers.emplace_back(new TypedDimBuffer<int32_t>(layout, dimId));
            break;
        case Dimension::Type::Signed64:
            m_buffers.emplace_back(new TypedDimBuffer<int64_t>(layout, dimId));
            break;
        case Dimension::Type::Unsigned8:
            m_buffers.emplace_back(new TypedDimBuffer<uint8_t>(layout, dimId));
            break;
        case Dimension::Type::Unsigned16:
            m_buffers.emplace_back(new TypedDimBuffer<uint16_t>(layout, dimId));
            break;
        case Dimension::Type::Unsigned32:
            m_buffers.emplace_back(new TypedDimBuffer<uint32_t>(layout, dimId));
            break;
        case Dimension::Type::Unsigned64:
            m_buffers.emplace_back(new TypedDimBuffer<uint64_t>(layout, dimId));
            break;
        case Dimension::Type::None:
        default:
            throw pdal_error("Unsupported attribute type for " +
                             layout->dimName(dimId));
        }

        m_buffers.back()->resizeBuffer(m_args->m_cache_size);
    }
    if (hasBitFields)
    {
        m_buffers.emplace_back(new BitFieldsBuffer("BitFields", bitFieldIds));

        m_buffers.back()->resizeBuffer(m_args->m_cache_size);
    }

    // Enable TileDB stats (if requested).
    if (m_args->m_stats)
        tiledb::Stats::enable();

    // If not appending to an existing array, then create the TileDB array.
    if (!m_args->m_append)
    {
        // Check if any or all of the tiles are set to valid sizes.
        bool hasValidTiles =
            ((m_args->m_x_tile_size > 0) && (m_args->m_y_tile_size > 0) &&
             (m_args->m_z_tile_size > 0) &&
             (!m_args->m_use_time || m_args->m_time_tile_size > 0));
        if (!hasValidTiles &&
            ((m_args->m_x_tile_size > 0) || (m_args->m_y_tile_size > 0) ||
             (m_args->m_z_tile_size > 0) ||
             (m_args->m_use_time && m_args->m_time_tile_size > 0)))
            std::cerr << "WARNING: Not all tile sizes are valid. Ignoring tile "
                         "sizes.";

        // Create schema and set basic properties.
        tiledb::ArraySchema schema{*m_ctx, TILEDB_SPARSE};
        schema.set_allows_dups(true);
        schema.set_capacity(m_args->m_tile_capacity);

        // Set tile order.
        if (m_args->m_tile_order == "row-major" || m_args->m_tile_order == "R")
            schema.set_tile_order(TILEDB_ROW_MAJOR);
        else if (m_args->m_tile_order == "col-major" ||
                 m_args->m_tile_order == "C")
            schema.set_tile_order(TILEDB_COL_MAJOR);
        else
            throwError("Invalid tile order option '" + m_args->m_tile_order +
                       "'.");

        // Set cell order.
        if (m_args->m_cell_order == "auto")
        {
            // Use Hilbert order if not all tiles are set, and row-major if they
            // are.
            if (!hasValidTiles)
            {
                schema.set_cell_order(TILEDB_HILBERT);
            }
            else
                schema.set_cell_order(TILEDB_ROW_MAJOR);
        }
        else if (m_args->m_cell_order == "row-major" ||
                 m_args->m_cell_order == "R")
        {
            if (!hasValidTiles)
                throwError("The tile size must be set for all dimensions when "
                           "using row major cell order.");
            schema.set_cell_order(TILEDB_ROW_MAJOR);
        }
        else if (m_args->m_cell_order == "col-major" ||
                 m_args->m_cell_order == "C")
        {
            if (!hasValidTiles)
                throwError("The tile size must be set for all dimensions when "
                           "using column major cell order.");
            schema.set_cell_order(TILEDB_COL_MAJOR);
        }
        else if (m_args->m_cell_order == "hilbert" ||
                 m_args->m_cell_order == "H")
            schema.set_cell_order(TILEDB_HILBERT);
        else
            throwError("Invalid cell order option '" + m_args->m_cell_order +
                       "'.");

        // Get filter factory class.
        FilterFactory filterFactory{
            m_args->m_filters,    m_args->m_filter_profile,
            m_args->m_scale,      m_args->m_offset,
            m_args->m_compressor, m_args->m_compressionLevel};

        // Check if the domain is set for all dimensions.
        bool hasValidDomain =
            (m_args->m_x_domain_end > m_args->m_x_domain_st &&
             m_args->m_y_domain_end > m_args->m_y_domain_st &&
             m_args->m_z_domain_end > m_args->m_z_domain_st &&
             (!m_args->m_use_time ||
              m_args->m_time_domain_end > m_args->m_time_domain_st));

        tiledb::Domain domain(*m_ctx);

        // Set the domain values for the dimensions. Use the user provided
        // domain values, and update if they are the default domain or
        // otherwise not valid.
        std::array<std::array<double, 2>, 4> bbox{
            {{m_args->m_x_domain_st, m_args->m_x_domain_end},
             {m_args->m_y_domain_st, m_args->m_y_domain_end},
             {m_args->m_z_domain_st, m_args->m_z_domain_end},
             {m_args->m_time_domain_st, m_args->m_time_domain_end}}};

        // If X, Y, or Z domain are not valid, then attempt to update with
        // the stats filter bbox.
        if (bbox[0][1] <= bbox[0][0] || bbox[1][1] <= bbox[1][0] ||
            bbox[2][1] <= bbox[2][0])
        {

            // Check if we can update with
            MetadataNode meta =
                table.metadata().findChild("filters.stats:bbox:native:bbox");
            if (meta.valid())
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
            }
        }

        // If using time as a dimension and the GPSTime domain is not valid,
        // attempt to update with the stats filter GPSTime dimension.
        if (m_args->m_use_time && bbox[3][1] <= bbox[3][0])
        {
            MetadataNode stats_meta =
                table.metadata().findChild("filters.stats");

            if (stats_meta.valid())
            {

                for (const auto& dim_summary : stats_meta.children("statistic"))
                {
                    // Iterate over the dimension summary and see if the name
                    // of this dimension is "GpsTime".
                    auto dim_details = dim_summary.children();
                    bool is_gps_time =
                        std::any_of(dim_details.cbegin(), dim_details.cend(),
                                    [](const auto& detail) {
                                        return detail.name() == "name" &&
                                               detail.value() == "GpsTime";
                                    });

                    // If this dimension is for GpsTime, then use the statistics
                    // to set the domain.
                    if (is_gps_time)
                    {
                        auto min_stat = dim_summary.findChild("minimum");
                        auto max_stat = dim_summary.findChild("maximum");
                        if (min_stat.valid() && max_stat.valid())
                        {
                            bbox[3] = {min_stat.value<double>() - 1.0,
                                       max_stat.value<double>() + 1.0};
                        }
                        break;
                    }
                }
            }
        }

        // Update any remaining invalid domains to be the whole valid domain.
        for (auto& range : bbox)
            if (range[1] <= range[0])
                range = {std::numeric_limits<double>::lowest(),
                         std::numeric_limits<double>::max()};

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
            if (m_args->m_use_time && m_args->m_time_first)
                addDimension("GpsTime", bbox[3], m_args->m_time_tile_size);
            addDimension("X", bbox[0], m_args->m_x_tile_size);
            addDimension("Y", bbox[1], m_args->m_y_tile_size);
            addDimension("Z", bbox[2], m_args->m_z_tile_size);
            if (m_args->m_use_time && !m_args->m_time_first)
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
            if (m_args->m_use_time && m_args->m_time_first)
                addDimension("GpsTime", bbox[3]);
            addDimension("X", bbox[0]);
            addDimension("Y", bbox[1]);
            addDimension("Z", bbox[2]);
            if (m_args->m_use_time && !m_args->m_time_first)
                addDimension("GpsTime", bbox[3]);
        }

        schema.set_domain(domain);

        // Set the attributes.
        for (const auto& dimBuffer : m_buffers)
        {
            // Create and add the attribute to the domain.
            const auto& dimName = dimBuffer->name();
            if (!domain.has_dimension(dimName))
            {
                auto att = dimBuffer->createAttribute(*m_ctx);
                att.set_filter_list(filterFactory.filterList(*m_ctx, dimName));
                schema.add_attribute(att);
            }
        }

        // Create the TileDB array.
        tiledb::Array::create(m_args->m_arrayName, schema);
    }

    // Open the array at the requested timestamp range.
#if TILEDB_VERSION_MAJOR == 2 && TILEDB_VERSION_MINOR < 15
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

    for (const auto& dimBuffer : m_buffers)
    {
        // If appending, check the layout dim exists in the original schema.
        const auto& dimName = dimBuffer->name();
        if (m_args->m_append && !schema.has_attribute(dimName) &&
            !schema.domain().has_dimension(dimName))
            throwError("Attribute/Dimension '" + dimName +
                       "' does not exist in original array.");
    }

    m_current_idx = 0;
}

bool TileDBWriter::processOne(PointRef& point)
{

    for (auto& dimBuffer : m_buffers)
    {
        dimBuffer->copyDataToBuffer(point, m_current_idx);
    }

    if (++m_current_idx == m_args->m_cache_size)
    {
        if (!flushCache())
            throwError("Unable to flush points to TileDB array");
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
    for (auto& dimBuffer : m_buffers)
    {
        dimBuffer->resizeBuffer(m_current_idx);
    }
    if (flushCache())
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
            // add tiledb pointcloud tag
            std::string datasetType = "pointcloud";
            m_array->put_metadata("dataset_type", TILEDB_STRING_UTF8, datasetType.size(), datasetType.data());
        }
        m_array->close();
    }
    else
    {
        throwError("Unable to flush points to TileDB array");
    }
}

bool TileDBWriter::flushCache()
{
    tiledb::Query query(*m_ctx, *m_array);
    query.set_layout(TILEDB_UNORDERED);

    // Set the query buffers.
    for (auto& dimBuffer : m_buffers)
        dimBuffer->setQueryBuffer(query);

    tiledb::Query::Status status = query.submit();

    if (m_args->m_stats)
    {
        tiledb::Stats::dump(stdout);
        tiledb::Stats::reset();
    }

    m_current_idx = 0;

    if (status == tiledb::Query::Status::FAILED)
        return false;
    else
        return true;
}

} // namespace pdal
