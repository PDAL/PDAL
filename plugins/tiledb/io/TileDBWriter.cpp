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

CREATE_SHARED_STAGE(TileDBWriter, s_info)

void writeAttributeValue(TileDBWriter::DimBuffer& dim,
    PointViewPtr view, PointId idx)
{
    Everything e;

    switch (dim.m_type)
    {
    case Dimension::Type::Double:
        e.d = view->getFieldAs<double>(dim.m_id, idx);
        break;
    case Dimension::Type::Float:
        e.f = view->getFieldAs<float>(dim.m_id, idx);
        break;
    case Dimension::Type::Signed8:
        e.s8 = view->getFieldAs<int8_t>(dim.m_id, idx);
        break;
    case Dimension::Type::Signed16:
        e.s16 = view->getFieldAs<int16_t>(dim.m_id, idx);
        break;
    case Dimension::Type::Signed32:
        e.s32 = view->getFieldAs<int32_t>(dim.m_id, idx);
        break;
    case Dimension::Type::Signed64:
        e.s64 = view->getFieldAs<int64_t>(dim.m_id, idx);
        break;
    case Dimension::Type::Unsigned8:
        e.u8 = view->getFieldAs<uint8_t>(dim.m_id, idx);
        break;
    case Dimension::Type::Unsigned16:
        e.u16 = view->getFieldAs<uint16_t>(dim.m_id, idx);
        break;
    case Dimension::Type::Unsigned32:
        e.u32 = view->getFieldAs<uint32_t>(dim.m_id, idx);
        break;
    case Dimension::Type::Unsigned64:
        e.u64 = view->getFieldAs<uint64_t>(dim.m_id, idx);
        break;
    default:
        throw pdal_error("Unsupported attribute type for " +
            view->dimName(dim.m_id));
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


tiledb::Filter createFilter(const tiledb::Context& ctx, const std::string& name)
{
    if (name.empty())
        return tiledb::Filter(ctx, TILEDB_FILTER_NONE);
    else if (name == "gzip")
        return tiledb::Filter(ctx, TILEDB_FILTER_GZIP);
    else if (name == "zstd")
        return tiledb::Filter(ctx, TILEDB_FILTER_ZSTD);
    else if (name == "lz4")
        return tiledb::Filter(ctx, TILEDB_FILTER_LZ4);
    else if (name == "rle")
        return tiledb::Filter(ctx, TILEDB_FILTER_RLE);
    else if (name == "bzip2")
        return tiledb::Filter(ctx, TILEDB_FILTER_BZIP2);
    else if (name == "double-delta")
        return tiledb::Filter(ctx, TILEDB_FILTER_DOUBLE_DELTA);
    else
        throw tiledb::TileDBError("Unable to parse compression type: " + name);
}

std::string TileDBWriter::getName() const { return s_info.name; }

void TileDBWriter::addArgs(ProgramArgs& args)
{
    args.add("array_name", "TileDB array name", m_arrayName).setPositional();
    args.add("config_file", "TileDB configuration file location",
        m_cfgFileName);
    args.add("data_tile_capacity", "TileDB tile capacity", m_tile_capacity,
        size_t(100000));
    args.add("x_tile_size", "TileDB tile size", m_x_tile_size, size_t(1000));
    args.add("y_tile_size", "TileDB tile size", m_y_tile_size, size_t(1000));
    args.add("z_tile_size", "TileDB tile size", m_z_tile_size, size_t(1000));
    args.add("stats", "Dump TileDB query stats to stdout", m_stats, false);
    args.add("compression", "TileDB compression type for attributes",
        m_compressor);
    args.add("compression_level", "TileDB compression level",
        m_compressionLevel, -1);
}


void TileDBWriter::initialize()
{
    if (!m_cfgFileName.empty())
    {
        tiledb::Config cfg(m_cfgFileName);
        m_ctx.reset(new tiledb::Context(cfg));
    }
    else
        m_ctx.reset(new tiledb::Context());

    // If the array already exists on disk, throw an error
    if (tiledb::Object::object(*m_ctx, m_arrayName).type() ==
        tiledb::Object::Type::Array)
        throwError("Array already exists.");

    m_schema.reset(new tiledb::ArraySchema(*m_ctx, TILEDB_SPARSE));

    if (!m_compressor.empty())
    {
        tiledb::Filter filter = createFilter(*m_ctx, m_compressor);
        filter.set_option(TILEDB_COMPRESSION_LEVEL, m_compressionLevel);
        m_filterList.reset(new tiledb::FilterList(*m_ctx));
        m_filterList->add_filter(filter);
        m_schema->set_coords_filter_list(*m_filterList);
    }
}


void TileDBWriter::ready(pdal::BasePointTable &table)
{
    // get a list of all the dimensions & their types and add to schema
    // x,y,z will be tiledb dimensions other pdal dimensions will be
    // tiledb attributes
    tiledb::Domain domain(*m_ctx);
    auto layout = table.layout();
    auto all = layout->dims();
    double dimMin = std::numeric_limits<double>::lowest();
    double dimMax = std::numeric_limits<double>::max();

    domain.add_dimension(tiledb::Dimension::create<double>(*m_ctx, "X",
        {{dimMin, dimMax}}, m_x_tile_size))
        .add_dimension(tiledb::Dimension::create<double>(*m_ctx, "Y",
        {{dimMin, dimMax}}, m_y_tile_size))
        .add_dimension(tiledb::Dimension::create<double>(*m_ctx, "Z",
        {{dimMin, dimMax}}, m_z_tile_size));

    for (const auto& d : all)
    {
        std::string dimName = layout->dimName(d);
        if ((dimName != "X") && (dimName != "Y") && (dimName != "Z"))
        {
            Dimension::Type type = layout->dimType(d);
            tiledb::Attribute att = createAttribute(*m_ctx, dimName, type);
            if (!m_compressor.empty())
                att.set_filter_list(*m_filterList);
            m_schema->add_attribute(att);
            m_attrs.emplace_back(dimName, d, type);
        }
    }

    m_schema->set_domain(domain).set_order(
        {{TILEDB_ROW_MAJOR, TILEDB_ROW_MAJOR}});
    m_schema->set_capacity(m_tile_capacity);

    tiledb::Array::create(m_arrayName, *m_schema);
    m_array.reset(new tiledb::Array(*m_ctx, m_arrayName, TILEDB_WRITE));
    m_query.reset(new tiledb::Query(*m_ctx, *m_array));
    m_query->set_layout(TILEDB_UNORDERED);
}


void TileDBWriter::write(const PointViewPtr view)
{
    std::vector<double> coords;

    auto attrs = m_schema->attributes();

    // Size the buffers.
    for (auto& a : m_attrs)
        a.m_buffer.resize(view->size() * Dimension::size(a.m_type));

    PointRef point(*view, 0);
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        double x = view->getFieldAs<double>(Dimension::Id::X, idx);
        double y = view->getFieldAs<double>(Dimension::Id::Y, idx);
        double z = view->getFieldAs<double>(Dimension::Id::Z, idx);

        coords.push_back(x);
        coords.push_back(y);
        coords.push_back(z);

        m_bbox.grow(x, y, z);

        for (auto& a : m_attrs)
            writeAttributeValue(a, view, idx);
    }
    m_query->set_coordinates(coords);

    // set tiledb buffers
    size_t size = view->size();
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

    if (m_stats)
        tiledb::Stats::enable();

    m_query->submit();

    if (m_stats)
    {
        tiledb::Stats::dump(stdout);
        tiledb::Stats::disable();
    }
}


void TileDBWriter::done(PointTableRef table)
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
        anon.add("spatialreference", Utils::toString(getSpatialReference()));
    }
    anon.add("bounds", pdal::Utils::toString(m_bbox));

    // serialize metadata
    tiledb::VFS::filebuf fbuf(vfs);

    if (vfs.is_dir(m_arrayName))
        fbuf.open(m_arrayName + pathSeparator + "pdal.json", std::ios::out);
    else
    {
        std::string fname = m_arrayName + "/pdal.json";
        vfs.touch(fname);
        fbuf.open(fname, std::ios::out);
    }

    std::ostream os(&fbuf);

    if (!os.good())
        throwError("Unable to create sidecar file for " + m_arrayName);

    pdal::Utils::toJSON(anon, os);

    fbuf.close();
    m_array->close();
}

} // namespace pdal
