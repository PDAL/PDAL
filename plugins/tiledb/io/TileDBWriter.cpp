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

#include <string.h>
#include <cctype>
#include <limits>

#include <nlohmann/json.hpp>

#include <pdal/util/FileUtils.hpp>

#include "TileDBWriter.hpp"


const char pathSeparator =
#ifdef _WIN32
        '\\';
#else
        '/';
#endif

namespace pdal {

static PluginInfo const s_info
{
    "writers.tiledb",
    "Write data using TileDB.",
    "http://pdal.io/stages/drivers.tiledb.writer.html"
};

struct TileDBWriter::Args
{
    std::string m_arrayName;
    std::string m_cfgFileName;
    size_t m_tile_capacity;
    size_t m_x_tile_size;
    size_t m_y_tile_size;
    size_t m_z_tile_size;
    size_t m_cache_size;
    bool m_stats;
    std::string m_compressor;
    int m_compressionLevel;
    NL::json m_filters;
    bool m_append;
};


CREATE_SHARED_STAGE(TileDBWriter, s_info)

void writeAttributeValue(TileDBWriter::DimBuffer& dim,
    PointRef& point, size_t idx)
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
    switch(t)
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


std::unique_ptr<tiledb::Filter> createFilter(const tiledb::Context& ctx, const NL::json& opts)
{
    std::unique_ptr<tiledb::Filter> filter;
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
        filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_BIT_WIDTH_REDUCTION));
    else if (name == "bit-shuffle")
        filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_BITSHUFFLE));
    else if (name == "byte-shuffle")
        filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_BYTESHUFFLE));
    else if (name == "positive-delta")
        filter.reset(new tiledb::Filter(ctx, TILEDB_FILTER_POSITIVE_DELTA));
    else
        throw tiledb::TileDBError("Unable to parse compression type: " + name);

    if (opts.count("compression_level") > 0)
        filter->set_option(TILEDB_COMPRESSION_LEVEL, opts["compression_level"].get<int>());

    if (opts.count("bit_width_max_window") > 0)
        filter->set_option(TILEDB_BIT_WIDTH_MAX_WINDOW, opts["bit_width_max_window"].get<int>());

    if (opts.count("positive_delta_max_window") > 0)
        filter->set_option(TILEDB_POSITIVE_DELTA_MAX_WINDOW, opts["positive_delta_max_window"].get<int>());

    return filter;
}


std::unique_ptr<tiledb::FilterList> createFilterList(const tiledb::Context& ctx, const NL::json& opts)
{
    std::unique_ptr<tiledb::FilterList> filterList(new tiledb::FilterList(ctx));

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
    return filterList;
}

TileDBWriter::TileDBWriter(): m_args(new TileDBWriter::Args){}


TileDBWriter::~TileDBWriter(){}


std::string TileDBWriter::getName() const { return s_info.name; }


void TileDBWriter::addArgs(ProgramArgs& args)
{
    args.add("array_name", "TileDB array name",
        m_args->m_arrayName).setPositional();
    args.addSynonym("array_name", "filename");
    args.add("config_file", "TileDB configuration file location",
        m_args->m_cfgFileName);
    args.add("data_tile_capacity", "TileDB tile capacity",
        m_args->m_tile_capacity, size_t(100000));
    args.add("x_tile_size", "TileDB tile size", m_args->m_x_tile_size,
        size_t(1000));
    args.add("y_tile_size", "TileDB tile size", m_args->m_y_tile_size,
        size_t(1000));
    args.add("z_tile_size", "TileDB tile size", m_args->m_z_tile_size,
        size_t(1000));
    args.add("chunk_size", "Point cache size for chunked writes",
        m_args->m_cache_size, size_t(10000));
    args.add("stats", "Dump TileDB query stats to stdout", m_args->m_stats,
        false);
    args.add("compression", "TileDB compression type for attributes",
        m_args->m_compressor);
    args.add("compression_level", "TileDB compression level",
        m_args->m_compressionLevel, -1);
    args.add("filters", "Specify filter and level per dimension/attribute",
        m_args->m_filters);
    args.add("append", "Append to existing TileDB array",
        m_args->m_append, false);
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
            if (tiledb::Object::object(*m_ctx, m_args->m_arrayName).type() ==
                    tiledb::Object::Type::Array)
                throwError("Array already exists.");

            m_schema.reset(new tiledb::ArraySchema(*m_ctx, TILEDB_SPARSE));

            if (!m_args->m_compressor.empty() ||
                    m_args->m_filters.count("coords") > 0)
            {
                if (!m_args->m_compressor.empty())
                {
                    NL::json opts;
                    opts["compression"] = m_args->m_compressor;
                    opts["compression_level"] = m_args->m_compressionLevel;
                    m_schema->set_coords_filter_list(
                        *createFilterList(*m_ctx, opts));
                }
                else
                {
                    m_schema->set_coords_filter_list(
                        *createFilterList(*m_ctx, m_args->m_filters["coords"]));
                }
            }
        }
    }
    catch (const tiledb::TileDBError& err)
    {
        throwError(std::string("TileDB Error: ") + err.what());
    }
}


void TileDBWriter::ready(pdal::BasePointTable &table)
{
    auto layout = table.layout();
    auto all = layout->dims();

    // get a list of all the dimensions & their types and add to schema
    // x,y,z will be tiledb dimensions other pdal dimensions will be
    // tiledb attributes
    if (!m_args->m_append)
    {
        tiledb::Domain domain(*m_ctx);
        double dimMin = std::numeric_limits<double>::lowest();
        double dimMax = std::numeric_limits<double>::max();

        domain.add_dimension(tiledb::Dimension::create<double>(*m_ctx, "X",
            {{dimMin, dimMax}}, m_args->m_x_tile_size))
            .add_dimension(tiledb::Dimension::create<double>(*m_ctx, "Y",
            {{dimMin, dimMax}}, m_args->m_y_tile_size))
            .add_dimension(tiledb::Dimension::create<double>(*m_ctx, "Z",
            {{dimMin, dimMax}}, m_args->m_z_tile_size));

        m_schema->set_domain(domain).set_order(
            {{TILEDB_ROW_MAJOR, TILEDB_ROW_MAJOR}});
        m_schema->set_capacity(m_args->m_tile_capacity);
    }
    else
    {
        m_array.reset(new tiledb::Array(*m_ctx, m_args->m_arrayName,
            TILEDB_WRITE));
    }

    for (const auto& d : all)
    {
        std::string dimName = layout->dimName(d);

        if ((dimName != "X") && (dimName != "Y") && (dimName != "Z"))
        {
            Dimension::Type type = layout->dimType(d);
            if (!m_args->m_append)
            {
                tiledb::Attribute att = createAttribute(*m_ctx, dimName, type);
                if (!m_args->m_compressor.empty())
                {
                    NL::json opts;
                    opts["compression"] = m_args->m_compressor;
                    opts["compression_level"] = m_args->m_compressionLevel;
                    att.set_filter_list(*createFilterList(*m_ctx, opts));
                }
                else
                {
                    if (m_args->m_filters.count(dimName) > 0)
                        att.set_filter_list(
                            *createFilterList(*m_ctx, m_args->m_filters[dimName]));
                }

                m_schema->add_attribute(att);
            }
            else
            {
                // check attribute exists in original tiledb array
                auto attrs = m_schema->attributes();
                auto it = attrs.find(dimName);
                if (it == attrs.end())
                    throwError("Attribute " + dimName +
                        " does not exist in original array.");
            }
            
            m_attrs.emplace_back(dimName, d, type);
            // Size the buffers.
            m_attrs.back().m_buffer.resize(
                m_args->m_cache_size * Dimension::size(type));
        }
    }

    if (!m_args->m_append)
    {
        tiledb::Array::create(m_args->m_arrayName, *m_schema);
        m_array.reset(new tiledb::Array(*m_ctx, m_args->m_arrayName,
            TILEDB_WRITE));
    }

    m_query.reset(new tiledb::Query(*m_ctx, *m_array));
    m_query->set_layout(TILEDB_UNORDERED);
    m_current_idx = 0;
}


bool TileDBWriter::processOne(PointRef& point)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);

    for (auto& a : m_attrs)
        writeAttributeValue(a, point, m_current_idx);

    m_coords.push_back(x);
    m_coords.push_back(y);
    m_coords.push_back(z);
    m_bbox.grow(x, y, z);

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
        tiledb::VFS vfs(*m_ctx, m_ctx->config());

        // write pipeline metadata sidecar inside array
        MetadataNode anon;
        MetadataNode meta("pipeline");
        anon.addList(meta);
        // set output to tiledb reader
        meta.add("type", "readers.tiledb");
        if (!getSpatialReference().empty() && table.spatialReferenceUnique())
        {
            // The point view takes on the spatial reference of that stage,
            // if it had one.
            anon.add("spatialreference", 
                Utils::toString(getSpatialReference()));
        }
        anon.add("bounds", pdal::Utils::toString(m_bbox));

        // serialize metadata
        tiledb::VFS::filebuf fbuf(vfs);

        if (vfs.is_dir(m_args->m_arrayName))
            fbuf.open(m_args->m_arrayName + pathSeparator + "pdal.json",
                std::ios::out);
        else
        {
            std::string fname = m_args->m_arrayName + "/pdal.json";
            vfs.touch(fname);
            fbuf.open(fname, std::ios::out);
        }

        std::ostream os(&fbuf);

        if (!os.good())
            throwError("Unable to create sidecar file for " +
                m_args->m_arrayName);

        pdal::Utils::toJSON(anon, os);

        fbuf.close();
        m_array->close();
    }
    else{
        throwError("Unable to flush points to TileDB array");
    }
}


bool TileDBWriter::flushCache(size_t size)
{
    m_query->set_coordinates(m_coords);

    // set tiledb buffers
    for (const auto& a : m_attrs)
    {
        uint8_t *buf = const_cast<uint8_t *>(a.m_buffer.data());
        switch (a.m_type)
        {
        case Dimension::Type::Double:
            m_query->set_buffer(a.m_name, reinterpret_cast<double *>(buf),
                size);
            break;
        case Dimension::Type::Float:
            m_query->set_buffer(a.m_name, reinterpret_cast<float *>(buf),
                size);
            break;
        case Dimension::Type::Signed8:
            m_query->set_buffer(a.m_name, reinterpret_cast<int8_t *>(buf),
                size);
            break;
        case Dimension::Type::Signed16:
            m_query->set_buffer(a.m_name, reinterpret_cast<int16_t *>(buf),
                size);
            break;
        case Dimension::Type::Signed32:
            m_query->set_buffer(a.m_name, reinterpret_cast<int32_t *>(buf),
                size);
            break;
        case Dimension::Type::Signed64:
            m_query->set_buffer(a.m_name, reinterpret_cast<int64_t *>(buf),
                size);
            break;
        case Dimension::Type::Unsigned8:
            m_query->set_buffer(a.m_name, reinterpret_cast<uint8_t *>(buf),
                size);
            break;
        case Dimension::Type::Unsigned16:
            m_query->set_buffer(a.m_name, reinterpret_cast<uint16_t *>(buf),
                size);
            break;
        case Dimension::Type::Unsigned32:
            m_query->set_buffer(a.m_name, reinterpret_cast<uint32_t *>(buf),
                size);
            break;
        case Dimension::Type::Unsigned64:
            m_query->set_buffer(a.m_name, reinterpret_cast<uint64_t *>(buf),
                size);
            break;
        case Dimension::Type::None:
        default:
            throw pdal_error("Unsupported attribute type for " + a.m_name);
        }
    }

    if (m_args->m_stats)
        tiledb::Stats::enable();

    tiledb::Query::Status status = m_query->submit();

    if (m_args->m_stats)
    {
        tiledb::Stats::dump(stdout);
        tiledb::Stats::disable();
    }

    m_current_idx = 0;
    m_coords.clear();

    if (status == tiledb::Query::Status::FAILED)
        return false;
    else
        return true;
}

} // namespace pdal
