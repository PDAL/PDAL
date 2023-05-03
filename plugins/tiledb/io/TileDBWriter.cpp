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
    NL::json m_defaults;
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

std::unique_ptr<tiledb::Filter> createFilter(const tiledb::Context& ctx,
                                             const NL::json& opts)
{
    std::unique_ptr<tiledb::Filter> filter;

    if (!opts.empty())
    {
        std::string name = opts["compression"];

        if (name.empty())
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_NONE));
        else if (name == "gzip")
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_GZIP));
        else if (name == "zstd")
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_ZSTD));
        else if (name == "lz4")
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_LZ4));
        else if (name == "rle")
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_RLE));
        else if (name == "bzip2")
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_BZIP2));
        else if (name == "double-delta")
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_DOUBLE_DELTA));
        else if (name == "bit-width-reduction")
            filter.reset(
                new tiledb::Filter(ctx, TILEDB_FILTER_BIT_WIDTH_REDUCTION));
        else if (name == "bit-shuffle")
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_BITSHUFFLE));
        else if (name == "byte-shuffle")
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_BYTESHUFFLE));
        else if (name == "positive-delta")
            filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_POSITIVE_DELTA));
        else
            throw tiledb::TileDBError("Unable to parse compression type: " +
                                      name);

        if (opts.count("compression_level") > 0)
            filter->set_option(TILEDB_COMPRESSION_LEVEL,
                               opts["compression_level"].get<int>());

        if (opts.count("bit_width_max_window") > 0)
            filter->set_option(TILEDB_BIT_WIDTH_MAX_WINDOW,
                               opts["bit_width_max_window"].get<int>());

        if (opts.count("positive_delta_max_window") > 0)
            filter->set_option(TILEDB_POSITIVE_DELTA_MAX_WINDOW,
                               opts["positive_delta_max_window"].get<int>());
    }
    else
    {
        filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_NONE));
    }
    return filter;
}

std::unique_ptr<tiledb::FilterList> createFilterList(const tiledb::Context& ctx,
                                                     const NL::json& opts)
{
    std::unique_ptr<tiledb::FilterList> filterList(new tiledb::FilterList(ctx));
    if (!opts.empty())
    {
        if (opts.is_array())
        {
            for (auto& el : opts.items())
            {
                auto v = el.value();
                filterList->add_filter(*createFilter(ctx, v));
            }
        }
        else
        {
            filterList->add_filter(*createFilter(ctx, opts));
        }
    }
    return filterList;
}

std::unique_ptr<tiledb::FilterList>
getDimFilter(const tiledb::Context& ctx, const std::string& dimName,
             const NL::json& opts, const std::string& defaultCompressor,
             const int& defaultCompressorLevel)
{
    NL::json dimOpts;
    if (opts.count(dimName) > 0)
    {
        dimOpts = opts[dimName];
    }
    else if (!defaultCompressor.empty())
    {
        dimOpts["compression"] = defaultCompressor;
        dimOpts["compression_level"] = defaultCompressorLevel;
    }

    return createFilterList(ctx, dimOpts);
}

TileDBWriter::TileDBWriter() : m_args(new TileDBWriter::Args)
{
    std::string attributeDefaults(R"(
    {
        "X" : {"compression": "zstd", "compression_level": 7},
        "Y" : {"compression": "zstd", "compression_level": 7},
        "Z" : {"compression": "zstd", "compression_level": 7},
        "Intensity":{"compression": "bzip2", "compression_level": 5},
        "ReturnNumber": {"compression": "zstd", "compression_level": 7},
        "NumberOfReturns": {"compression": "zstd", "compression_level": 7},
        "ScanDirectionFlag": {"compression": "bzip2", "compression_level": 5},
        "EdgeOfFlightLine": {"compression": "bzip2", "compression_level": 5},
        "Classification": {"compression": "gzip", "compression_level": 9},
        "ScanAngleRank": {"compression": "bzip2", "compression_level": 5},
        "UserData": {"compression": "gzip", "compression_level": 9},
        "PointSourceId": {"compression": "bzip2"},
        "Red": {"compression": "zstd", "compression_level": 7},
        "Green": {"compression": "zstd", "compression_level": 7},
        "Blue": {"compression": "zstd", "compression_level": 7},
        "GpsTime": {"compression": "zstd", "compression_level": 7}
    })");

    m_args->m_defaults = NL::json::parse(attributeDefaults);
}

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

        if (!m_args->m_append)
        {
            m_args->m_defaults.update(m_args->m_filters);
            m_schema.reset(new tiledb::ArraySchema(*m_ctx, TILEDB_SPARSE));
            m_schema->set_allows_dups(true);
        }
    }
    catch (const tiledb::TileDBError& err)
    {
        throwError(std::string("TileDB Error: ") + err.what());
    }
}

void TileDBWriter::ready(pdal::BasePointTable& table)
{
    auto layout = table.layout();
    auto all = layout->dims();
    std::vector<tiledb::Dimension> dims;

    if (m_args->m_stats)
        tiledb::Stats::enable();

    // get a list of all the dimensions & their types and add to schema
    // x,y,z will be tiledb dimensions other pdal dimensions will be
    // tiledb attributes
    if (!m_args->m_append)
    {

        // Check if using Hilbert order or row-major order. Use row-major if all
        // dimensions have positive tiles set. Otherwise, use Hilbert order.
        bool hasValidTiles =
            ((m_args->m_x_tile_size > 0) && (m_args->m_y_tile_size > 0) &&
             (m_args->m_z_tile_size > 0) &&
             (!m_use_time || m_args->m_time_tile_size > 0));

        // Check if the domain is set for all dimensions.
        bool hasValidDomain = (m_args->m_x_domain_end > m_args->m_x_domain_st &&
                               m_args->m_y_domain_end > m_args->m_y_domain_st &&
                               m_args->m_z_domain_end > m_args->m_z_domain_st &&
                               (!m_use_time || m_args->m_time_domain_end >
                                                   m_args->m_time_domain_st));

        // Get table metadata and check if it is valid.
        MetadataNode meta =
            table.metadata().findChild("filters.stats:bbox:native:bbox");
        bool hasMetadataStats = meta.valid();

        // Check the user set valid tile extents, valid domains, or ran stats on
        // the point table.
        if (!hasValidTiles && !hasMetadataStats && !hasValidDomain)
            throwError("Must specify a tile extent for all dimensions, specify "
                       "a valid domain for all dimensions, or execute a prior "
                       "stats filter stage.");

        tiledb::Domain domain(*m_ctx);

        // Get filters for the dimensions.
        tiledb::FilterList xFltrs =
            *getDimFilter(*m_ctx, "X", m_args->m_defaults, m_args->m_compressor,
                          m_args->m_compressionLevel);
        tiledb::FilterList yFltrs =
            *getDimFilter(*m_ctx, "Y", m_args->m_defaults, m_args->m_compressor,
                          m_args->m_compressionLevel);
        tiledb::FilterList zFltrs =
            *getDimFilter(*m_ctx, "Z", m_args->m_defaults, m_args->m_compressor,
                          m_args->m_compressionLevel);
        tiledb::FilterList tFltrs =
            *getDimFilter(*m_ctx, "GpsTime", m_args->m_defaults,
                          m_args->m_compressor, m_args->m_compressionLevel);

        // Set the domain values for the dimensions. Use the user provided
        // domain values, and update if they are the default domain or otherwise
        // not valid.
        std::array<double, 2> xDomain{m_args->m_x_domain_st,
                                      m_args->m_x_domain_end};
        std::array<double, 2> yDomain{m_args->m_y_domain_st,
                                      m_args->m_y_domain_end};
        std::array<double, 2> zDomain{m_args->m_z_domain_st,
                                      m_args->m_z_domain_end};
        std::array<double, 2> gpsTimeDomain{m_args->m_time_domain_st,
                                            m_args->m_time_domain_end};
        if (!hasValidDomain)
        {
            if (hasMetadataStats)
            {
                // Use statistics from the point table to set the invalid
                // domains.
                if (xDomain[1] <= xDomain[0])
                {
                    xDomain[0] = meta.findChild("minx").value<double>() - 1.0;
                    xDomain[1] = meta.findChild("maxx").value<double>() + 1.0;
                }
                if (yDomain[1] <= yDomain[0])
                {
                    yDomain[0] = meta.findChild("miny").value<double>() - 1.0;
                    yDomain[1] = meta.findChild("maxy").value<double>() + 1.0;
                }
                if (zDomain[1] <= zDomain[0])
                {
                    zDomain[0] = meta.findChild("minz").value<double>() - 1.0;
                    zDomain[1] = meta.findChild("maxz").value<double>() + 1.0;
                }
                if (m_use_time && gpsTimeDomain[1] <= gpsTimeDomain[0])
                {
                    gpsTimeDomain[0] =
                        meta.findChild("mintm").value<double>() - 1.0;
                    gpsTimeDomain[1] =
                        meta.findChild("maxtm").value<double>() + 1.0;
                }
            }
            else
            {
                // Use the maximum possible domain to set the invalid domains.
                double dimMin = std::numeric_limits<double>::lowest();
                double dimMax = std::numeric_limits<double>::max();
                if (xDomain[1] <= xDomain[0])
                {
                    xDomain[0] = dimMin;
                    xDomain[1] = dimMax;
                }
                if (yDomain[1] <= yDomain[0])
                {
                    yDomain[0] = dimMin;
                    yDomain[1] = dimMax;
                }
                if (zDomain[1] <= zDomain[0])
                {
                    zDomain[0] = dimMin;
                    zDomain[1] = dimMax;
                }
                if (gpsTimeDomain[1] <= gpsTimeDomain[0])
                {
                    gpsTimeDomain[0] = dimMin;
                    gpsTimeDomain[1] = dimMax;
                }
            }
        }

        // Create and add dimensions to the TileDB domain.
        if (hasValidTiles)
        {
            if (m_use_time && m_time_first)
                domain.add_dimension(tiledb::Dimension::create<double>(
                                         *m_ctx, "GpsTime", gpsTimeDomain,
                                         m_args->m_time_tile_size)
                                         .set_filter_list(tFltrs));
            domain.add_dimension(
                tiledb::Dimension::create<double>(*m_ctx, "X", xDomain,
                                                  m_args->m_x_tile_size)
                    .set_filter_list(xFltrs));
            domain.add_dimension(
                tiledb::Dimension::create<double>(*m_ctx, "Y", yDomain,
                                                  m_args->m_y_tile_size)
                    .set_filter_list(yFltrs));
            domain.add_dimension(
                tiledb::Dimension::create<double>(*m_ctx, "Z", zDomain,
                                                  m_args->m_z_tile_size)
                    .set_filter_list(zFltrs));
            if (m_use_time && !m_time_first)
                domain.add_dimension(tiledb::Dimension::create<double>(
                                         *m_ctx, "GpsTime", gpsTimeDomain,
                                         m_args->m_time_tile_size)
                                         .set_filter_list(tFltrs));
        }
        else
        {
            if (m_use_time && m_time_first)
                domain.add_dimension(tiledb::Dimension::create<double>(
                                         *m_ctx, "GpsTime", gpsTimeDomain)
                                         .set_filter_list(tFltrs));
            domain.add_dimension(
                tiledb::Dimension::create<double>(*m_ctx, "X", xDomain)
                    .set_filter_list(xFltrs));
            domain.add_dimension(
                tiledb::Dimension::create<double>(*m_ctx, "Y", yDomain)
                    .set_filter_list(yFltrs));
            domain.add_dimension(
                tiledb::Dimension::create<double>(*m_ctx, "Z", zDomain)
                    .set_filter_list(zFltrs));
            if (m_use_time && !m_time_first)
                domain.add_dimension(tiledb::Dimension::create<double>(
                                         *m_ctx, "GpsTime", gpsTimeDomain)
                                         .set_filter_list(tFltrs));
        }

        m_schema->set_domain(domain);
        m_schema->set_capacity(m_args->m_tile_capacity);
        if (!hasValidTiles)
            m_schema->set_cell_order(TILEDB_HILBERT);
    }
    else
    {
#if TILEDB_VERSION_MINOR < 15
        if (m_args->m_timeStamp != UINT64_MAX)
            m_array.reset(new tiledb::Array(*m_ctx, m_args->m_arrayName,
                                            TILEDB_WRITE, m_args->m_timeStamp));
        else
            m_array.reset(
                new tiledb::Array(*m_ctx, m_args->m_arrayName, TILEDB_WRITE));
#else
        m_array.reset(new tiledb::Array(
            *m_ctx, m_args->m_arrayName, TILEDB_WRITE,
            {tiledb::TimeTravelMarker(), m_args->m_timeStamp}));
#endif
        if (m_array->schema().domain().has_dimension("GpsTime"))
            m_use_time = true;
    }

    for (const auto& d : all)
    {
        std::string dimName = layout->dimName(d);

        Dimension::Type type = layout->dimType(d);
        if (!m_args->m_append)
        {
            if (!m_schema->domain().has_dimension(dimName))
            {
                tiledb::Attribute att = createAttribute(*m_ctx, dimName, type);
                att.set_filter_list(*getDimFilter(
                    *m_ctx, dimName, m_args->m_defaults, m_args->m_compressor,
                    m_args->m_compressionLevel));

                m_schema->add_attribute(att);
            }
        }
        else
        {
            // check attribute and dimension exist in original tiledb array
            auto attrs = m_array->schema().attributes();
            auto it = attrs.find(dimName);
            if (it == attrs.end() &&
                (!m_array->schema().domain().has_dimension(dimName)))
                throwError("Attribute/Dimension " + dimName +
                           " does not exist in original array.");
        }

        m_attrs.emplace_back(dimName, d, type);
        // Size the buffers.
        m_attrs.back().m_buffer.resize(m_args->m_cache_size *
                                       Dimension::size(type));
    }

    if (!m_args->m_append)
    {
        tiledb::Array::create(m_args->m_arrayName, *m_schema);
#if TILEDB_VERSION_MINOR < 15
        if (m_args->m_timeStamp != UINT64_MAX)
            m_array.reset(new tiledb::Array(*m_ctx, m_args->m_arrayName,
                                            TILEDB_WRITE, m_args->m_timeStamp));
        else
            m_array.reset(
                new tiledb::Array(*m_ctx, m_args->m_arrayName, TILEDB_WRITE));
#else
        m_array.reset(new tiledb::Array(
            *m_ctx, m_args->m_arrayName, TILEDB_WRITE,
            {tiledb::TimeTravelMarker(), m_args->m_timeStamp}));
#endif
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

    for (auto& a : m_attrs)
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
    for (const auto& a : m_attrs)
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
