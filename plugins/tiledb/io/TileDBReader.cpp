/******************************************************************************
 * Copyright (c) 2019 TileDB, Inc
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

#include "TileDBReader.hpp"

#include "TileDBUtils.hpp"

#include <algorithm>
#include <iostream>

#include <nlohmann/json.hpp>

const char pathSeparator =
#ifdef _WIN32
    '\\';
#else
    '/';
#endif
namespace pdal
{

static PluginInfo const s_info{"readers.tiledb",
                               "Read data from a TileDB array.",
                               "http://pdal.io/stages/readers.tiledb.html"};

struct TileDBReader::Args
{
    std::string m_cfgFileName;
    point_count_t m_chunkSize;
    bool m_stats;
    DomainBounds m_bbox;
    uint64_t m_startTimeStamp;
    uint64_t m_endTimeStamp;
    bool m_strict;
};

CREATE_SHARED_STAGE(TileDBReader, s_info)
std::string TileDBReader::getName() const
{
    return s_info.name;
}

Dimension::Type getPdalType(tiledb_datatype_t t)
{
    switch (t)
    {
    case TILEDB_INT8:
        return Dimension::Type::Signed8;
    case TILEDB_UINT8:
        return Dimension::Type::Unsigned8;
    case TILEDB_INT16:
        return Dimension::Type::Signed16;
    case TILEDB_UINT16:
        return Dimension::Type::Unsigned16;
    case TILEDB_INT32:
        return Dimension::Type::Signed32;
    case TILEDB_UINT32:
        return Dimension::Type::Unsigned32;
    case TILEDB_INT64:
        return Dimension::Type::Signed64;
    case TILEDB_UINT64:
        return Dimension::Type::Unsigned64;
    case TILEDB_FLOAT32:
        return Dimension::Type::Float;
    case TILEDB_FLOAT64:
        return Dimension::Type::Double;
    case TILEDB_DATETIME_AS:
    case TILEDB_DATETIME_DAY:
    case TILEDB_DATETIME_FS:
    case TILEDB_DATETIME_HR:
    case TILEDB_DATETIME_MIN:
    case TILEDB_DATETIME_MONTH:
    case TILEDB_DATETIME_MS:
    case TILEDB_DATETIME_NS:
    case TILEDB_DATETIME_PS:
    case TILEDB_DATETIME_SEC:
    case TILEDB_DATETIME_US:
    case TILEDB_DATETIME_WEEK:
    case TILEDB_DATETIME_YEAR:
    case TILEDB_TIME_HR:
    case TILEDB_TIME_MIN:
    case TILEDB_TIME_SEC:
    case TILEDB_TIME_MS:
    case TILEDB_TIME_US:
    case TILEDB_TIME_NS:
    case TILEDB_TIME_PS:
    case TILEDB_TIME_FS:
    case TILEDB_TIME_AS:
        return Dimension::Type::Signed64;
    default:
        // Not supported tiledb domain types, return as None
        return Dimension::Type::None;
    }
}

TileDBReader::TileDBReader() : m_args(new TileDBReader::Args) {}

TileDBReader::~TileDBReader() {}

void TileDBReader::addArgs(ProgramArgs& args)
{
    args.addSynonym("filename", "array_name");
    args.add("config_file", "TileDB configuration file location",
             m_args->m_cfgFileName);
    args.add("chunk_size", "TileDB read chunk size", m_args->m_chunkSize,
             point_count_t(1000000));
    args.add("stats", "Dump TileDB query stats to stdout", m_args->m_stats,
             false);
    args.add("bbox3d",
             "Bounding box subarray to read from TileDB in format"
             "([minx, maxx], [miny, maxy], [minz, maxz])",
             m_args->m_bbox);
    args.add("bbox4d",
             "Bounding box subarray to read from TileDB in format"
             "([minx, maxx], [miny, maxy], [minz, maxz], [min_gpstime, "
             "max_gpstime] )",
             m_args->m_bbox);
    args.add("end_timestamp", "TileDB array timestamp", m_args->m_endTimeStamp,
             UINT64_MAX);
    args.addSynonym("end_timestamp", "timestamp");
    args.add<uint64_t>("start_timestamp", "TileDB array timestamp",
                       m_args->m_startTimeStamp, 0);
    args.add("strict", "Raise an error for unsupported attributes",
             m_args->m_strict, true);
}

void TileDBReader::prepared(PointTableRef table)
{
    if (m_filename.empty())
        throwError("Required argument 'filename' (TileDB array name) "
                   "not provided.");
}

void TileDBReader::initialize()
{
    if (!m_args->m_cfgFileName.empty())
    {
        tiledb::Config cfg(m_args->m_cfgFileName);
        m_ctx.reset(new tiledb::Context(cfg));
    }
    else
        m_ctx.reset(new tiledb::Context());

    try
    {
        if (m_args->m_stats)
            tiledb::Stats::enable();

#if TILEDB_VERSION_MINOR < 15
        m_array.reset(new tiledb::Array(*m_ctx, m_filename, TILEDB_READ));
        if (m_args->m_startTimeStamp != 0)
            m_array->set_open_timestamp_start(m_args->m_startTimeStamp);
        if (m_args->m_endTimeStamp != UINT64_MAX)
            m_array->set_open_timestamp_end(m_endTimeStamp);
        m_array->reopen();
#else
        m_array.reset(new tiledb::Array(*m_ctx, m_filename, TILEDB_READ,
                                        {tiledb::TimestampStartEndMarker(),
                                         m_args->m_startTimeStamp,
                                         m_args->m_endTimeStamp}));
#endif
    }
    catch (const tiledb::TileDBError& err)
    {
        throwError(std::string("TileDB Error: ") + err.what());
    }
}

void TileDBReader::addDimensions(PointLayoutPtr layout)
{
    // Dimensions are X/Y and maybe Z
    std::vector<tiledb::Dimension> dims =
        m_array->schema().domain().dimensions();

    Dimension::Id id;
    for (size_t i = 0; i < dims.size(); ++i)
    {
        tiledb::Dimension& dim = dims[i];

        DimInfo di;

        di.m_name = dim.name();

        di.m_offset = 0;
        di.m_span = 1;
        di.m_tileType = dim.type();
        di.m_type = getPdalType(di.m_tileType);

        if (di.m_type == pdal::Dimension::Type::None)
            throwError("Invalid Dim type from TileDB");

        di.m_id = layout->registerOrAssignDim(dim.name(), di.m_type);
        m_dims.push_back(di);
    }

    auto attrs = m_array->schema().attributes();
    for (const auto& a : attrs)
    {
        DimInfo di;

        di.m_name = a.first;
        di.m_offset = 0;
        di.m_span = 1;
        di.m_tileType = a.second.type();
        di.m_type = getPdalType(di.m_tileType);
        if (di.m_type != pdal::Dimension::Type::None)
        {
            di.m_id = layout->registerOrAssignDim(a.first, di.m_type);
            m_dims.push_back(di);
        }
        else
        {
            if (!m_args->m_strict)
                std::cerr << "Skipping over unsupported attribute type - "
                          << di.m_name << "!\n";
            else
                throwError("TileDB dimension '" + di.m_name +
                           "' can't be mapped "
                           "to trivial type.");
        }
    }

    // ABELL
    //  Should we check that X Y and Z exist and remap the primary/secondary
    //  dimensions to X Y and Z if necessary?
}

template <typename T> void TileDBReader::setQueryBuffer(const DimInfo& di)
{
    m_query->set_data_buffer(di.m_name, di.m_buffer->get<T>(),
                             di.m_buffer->count());
}

void TileDBReader::setQueryBuffer(const DimInfo& di)
{
    switch (di.m_tileType)
    {
    case TILEDB_INT8:
        setQueryBuffer<int8_t>(di);
        break;
    case TILEDB_UINT8:
        setQueryBuffer<uint8_t>(di);
        break;
    case TILEDB_INT16:
        setQueryBuffer<int16_t>(di);
        break;
    case TILEDB_UINT16:
        setQueryBuffer<uint16_t>(di);
        break;
    case TILEDB_INT32:
        setQueryBuffer<int32_t>(di);
        break;
    case TILEDB_UINT32:
        setQueryBuffer<uint32_t>(di);
        break;
    case TILEDB_UINT64:
        setQueryBuffer<uint64_t>(di);
        break;
    case TILEDB_FLOAT32:
        setQueryBuffer<float>(di);
        break;
    case TILEDB_FLOAT64:
        setQueryBuffer<double>(di);
        break;
    case TILEDB_DATETIME_AS:
    case TILEDB_DATETIME_DAY:
    case TILEDB_DATETIME_FS:
    case TILEDB_DATETIME_HR:
    case TILEDB_DATETIME_MIN:
    case TILEDB_DATETIME_MONTH:
    case TILEDB_DATETIME_MS:
    case TILEDB_DATETIME_NS:
    case TILEDB_DATETIME_PS:
    case TILEDB_DATETIME_SEC:
    case TILEDB_DATETIME_US:
    case TILEDB_DATETIME_WEEK:
    case TILEDB_DATETIME_YEAR:
    case TILEDB_TIME_HR:
    case TILEDB_TIME_MIN:
    case TILEDB_TIME_SEC:
    case TILEDB_TIME_MS:
    case TILEDB_TIME_US:
    case TILEDB_TIME_NS:
    case TILEDB_TIME_PS:
    case TILEDB_TIME_FS:
    case TILEDB_TIME_AS:
    case TILEDB_INT64:
        setQueryBuffer<int64_t>(di);
        break;
    default:
        throwError("TileDB dimension '" + di.m_name +
                   "' can't be mapped "
                   "to trivial type.");
    }
}

void TileDBReader::ready(PointTableRef)
{
    try
    {
        localReady();
    }
    catch (const tiledb::TileDBError& err)
    {
        throwError(std::string("TileDB Error: ") + err.what());
    }
}

void TileDBReader::localReady()
{

    m_query.reset(new tiledb::Query(*m_ctx, *m_array));
    m_query->set_layout(TILEDB_UNORDERED);

    for (DimInfo& di : m_dims)
    {
        // All dimensions use the same buffer.
        std::unique_ptr<Buffer> dimBuf(
            new Buffer(di.m_tileType, m_args->m_chunkSize));
        di.m_buffer = dimBuf.get();
        m_buffers.push_back(std::move(dimBuf));
        setQueryBuffer(di);
    }

    // Set the subarray to query. The default for each dimension is to query
    // the entire dimension domain unless a range is explicitly set on it.
    tiledb::Subarray subarray(*m_ctx, *m_array);
    const auto ndim_bbox = m_args->m_bbox.ndim();
    const auto domain = m_array->schema().domain();
    switch (m_args->m_bbox.ndim())
    {
    case 4:
        if (domain.has_dimension("GpsTime"))
            subarray.add_range("GpsTime", m_args->m_bbox.minGpsTime(),
                               m_args->m_bbox.maxGpsTime());
        [[fallthrough]];
    case 3:
        if (domain.has_dimension("Z"))
            subarray.add_range("Z", m_args->m_bbox.minZ(),
                               m_args->m_bbox.maxZ());
        [[fallthrough]];
    case 2:
        if (domain.has_dimension("Y"))
            subarray.add_range("Y", m_args->m_bbox.minY(),
                               m_args->m_bbox.maxY());
        [[fallthrough]];
    case 1:
        if (domain.has_dimension("X"))
            subarray.add_range("X", m_args->m_bbox.minX(),
                               m_args->m_bbox.maxX());
    }
    m_query->set_subarray(subarray);

    // read spatial reference
    NL::json meta = nullptr;

    tiledb_datatype_t v_type = TILEDB_UINT8;
    const void* v_r;
    uint32_t v_num;
    m_array->get_metadata("_pdal", &v_type, &v_num, &v_r);

    if (v_r != NULL)
        meta = NL::json::parse(static_cast<const char*>(v_r));

    if ((meta != nullptr) && (meta.count("root") > 0) &&
        (meta["root"].count("writers.tiledb") > 0) &&
        (meta["root"]["writers.tiledb"].count("spatialreference") > 0))
    {
        SpatialReference ref(
            meta["root"]["writers.tiledb"]["spatialreference"]);
        setSpatialReference(ref);
    }

    // initialize read buffer variables
    m_offset = 0;
    m_resultSize = 0;
    m_complete = false;
}

namespace
{

bool setField(PointRef& point, TileDBReader::DimInfo di, size_t bufOffset)
{
    // Span is a count of the number of elements in each set of data, so
    // offset is a count of item types.  We're doing pointer arithmetic
    // below, so the size of the type is accounted for.
    bufOffset = bufOffset * di.m_span + di.m_offset;
    TileDBReader::Buffer& buf = *di.m_buffer;
    switch (di.m_type)
    {
    case Dimension::Type::Signed8:
        point.setField(di.m_id, *(buf.get<int8_t>() + bufOffset));
        break;
    case Dimension::Type::Unsigned8:
        point.setField(di.m_id, *(buf.get<uint8_t>() + bufOffset));
        break;
    case Dimension::Type::Signed16:
        point.setField(di.m_id, *(buf.get<int16_t>() + bufOffset));
        break;
    case Dimension::Type::Unsigned16:
        point.setField(di.m_id, *(buf.get<uint16_t>() + bufOffset));
        break;
    case Dimension::Type::Signed32:
        point.setField(di.m_id, *(buf.get<int32_t>() + bufOffset));
        break;
    case Dimension::Type::Unsigned32:
        point.setField(di.m_id, *(buf.get<uint32_t>() + bufOffset));
        break;
    case Dimension::Type::Signed64:
        point.setField(di.m_id, *(buf.get<int64_t>() + bufOffset));
        break;
    case Dimension::Type::Unsigned64:
        point.setField(di.m_id, *(buf.get<uint64_t>() + bufOffset));
        break;
    case Dimension::Type::Float:
        point.setField(di.m_id, *(buf.get<float>() + bufOffset));
        break;
    case Dimension::Type::Double:
        point.setField(di.m_id, *(buf.get<double>() + bufOffset));
        break;
    default:
        return false;
    }
    return true;
}

} // unnamed namespace

bool TileDBReader::processOne(PointRef& point)
{
    try
    {
        return processPoint(point);
    }
    catch (const tiledb::TileDBError& err)
    {
        throwError(std::string("TileDB Error: ") + err.what());
    }
    return false;
}

bool TileDBReader::processPoint(PointRef& point)
{
    if (m_offset == m_resultSize)
    {
        if (m_complete)
        {
            return false;
        }
        else
        {
            tiledb::Query::Status status;

            m_query->submit();

            if (m_args->m_stats)
            {
                tiledb::Stats::dump(stdout);
                tiledb::Stats::reset();
            }

            status = m_query->query_status();

            // The result buffer count represents the total number of items
            // returned by the query for dimensions.  So if there are three
            // dimensions, the number of points returned is the buffer count
            // divided by the number of dimensions.
            m_resultSize = (int)m_query->result_buffer_elements()["X"].second;

            if (status == tiledb::Query::Status::INCOMPLETE &&
                m_resultSize == 0)
                throwError("Need to increase chunk_size for reader.");

            if (status == tiledb::Query::Status::COMPLETE)
                m_complete = true;

            m_offset = 0;
        }
    }

    if (m_resultSize > 0)
    {
        for (DimInfo& dim : m_dims)
            if (!setField(point, dim, m_offset))
                throwError("Invalid dimension type when setting data.");

        ++m_offset;
        return true;
    }
    else
    {
        return false;
    }
}

point_count_t TileDBReader::read(PointViewPtr view, point_count_t count)
{
    PointRef point = view->point(0);
    PointId id;
    for (id = 0; id < count; ++id)
    {
        point.setPointId(id);
        if (!processOne(point))
            break;
    }
    return id;
}

void TileDBReader::done(pdal::BasePointTable& table)
{
    m_array->close();
}

} // namespace pdal
