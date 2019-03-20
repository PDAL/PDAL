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

#include <algorithm>

#include "TileDBReader.hpp"

namespace pdal {

static PluginInfo const s_info
{
        "readers.tiledb",
        "Read data from a TileDB array.",
        "http://pdal.io/stages/readers.tiledb.html"
};

CREATE_SHARED_STAGE(TileDBReader, s_info)
std::string TileDBReader::getName() const { return s_info.name; }

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
        case TILEDB_CHAR:
        case TILEDB_STRING_ASCII:
        case TILEDB_STRING_UTF8:
        case TILEDB_STRING_UTF16:
        case TILEDB_STRING_UTF32:
        case TILEDB_STRING_UCS2:
        case TILEDB_STRING_UCS4:
        case TILEDB_ANY:
        default:
            // Not supported tiledb domain types
            throw pdal_error("Invalid Dim type from TileDB");
    }
}

void TileDBReader::addArgs(ProgramArgs& args)
{
    args.add("array_name", "TileDB array name", m_arrayName).setPositional();
    args.add("config_file", "TileDB configuration file location",
        m_cfgFileName);
    args.add("chunk_size", "TileDB read chunk size", m_chunkSize,
        point_count_t(1000000));
    args.add("stats", "Dump TileDB query stats to stdout", m_stats, false);
    args.add("bbox3d", "Bounding box subarray to read from TileDB in format "
        "([minx, maxx], [miny, maxy], [minz, maxz])", m_bbox);
}

void TileDBReader::initialize()
{
    if (!m_cfgFileName.empty())
    {
        tiledb::Config cfg(m_cfgFileName);
        m_ctx.reset(new tiledb::Context(cfg));
    }
    else
        m_ctx.reset(new tiledb::Context());

    m_array.reset(new tiledb::Array(*m_ctx, m_arrayName, TILEDB_READ));
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
        di.m_offset = i;
        di.m_span = dims.size();
        di.m_dimCategory = DimCategory::Dimension;
        di.m_tileType = dim.type();
        di.m_type = getPdalType(di.m_tileType);
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
        di.m_dimCategory = DimCategory::Attribute;
        di.m_tileType = a.second.type();
        di.m_type = getPdalType(di.m_tileType);
        di.m_id = layout->registerOrAssignDim(a.first, di.m_type);

        m_dims.push_back(di);
    }

    //ABELL
    // Should we check that X Y and Z exist and remap the primary/secondary
    // dimensions to X Y and Z if necessary?
}

template <typename T>
void TileDBReader::setQueryBuffer(const DimInfo& di)
{
    m_query->set_buffer(di.m_name, di.m_buffer->get<T>(), di.m_buffer->count());
}

void TileDBReader::setQueryBuffer(const DimInfo& di)
{
    switch(di.m_tileType)
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
    case TILEDB_INT64:
        setQueryBuffer<int64_t>(di);
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
    default:
        throwError("TileDB dimension '" + di.m_name + "' can't be mapped "
            "to trivial type.");
    }
}

void TileDBReader::ready(PointTableRef)
{
    int numDims = m_array->schema().domain().dimensions().size();

    m_query.reset(new tiledb::Query(*m_ctx, *m_array));

    // Build the buffer for the dimensions.
    auto it = std::find_if(m_dims.begin(), m_dims.end(),
        [](DimInfo& di){ return di.m_dimCategory == DimCategory::Dimension; });

    DimInfo& di = *it;
    Buffer *dimBuf = new Buffer(di.m_tileType, m_chunkSize * numDims);
    m_query->set_coordinates(dimBuf->get<double>(), dimBuf->count());
    m_buffers.push_back(std::unique_ptr<Buffer>(dimBuf));

    for (DimInfo& di : m_dims)
    {
        // All dimensions use the same buffer.
        if (di.m_dimCategory == DimCategory::Dimension)
            di.m_buffer = dimBuf;
        else
        {
            std::unique_ptr<Buffer> dimBuf(
                new Buffer(di.m_tileType, m_chunkSize));
            di.m_buffer = dimBuf.get();
            m_buffers.push_back(std::move(dimBuf));
            setQueryBuffer(di);
        }
    }

// Set the extent of the query.
    if (!m_bbox.empty())
    {
        if (numDims == 2)
            m_query->set_subarray({m_bbox.minx, m_bbox.minx,
                m_bbox.miny, m_bbox.maxy});
        else
            m_query->set_subarray({m_bbox.minx, m_bbox.minx,
                m_bbox.miny, m_bbox.maxy, m_bbox.minz, m_bbox.maxz});
    }
    else
    {
        // get extents
        std::vector<double> subarray;
        auto domain = m_array->non_empty_domain<double>();
        for (const auto& kv : domain)
        {
            subarray.push_back(kv.second.first);
            subarray.push_back(kv.second.second);
        }
        m_query->set_subarray(subarray);
    }
}

namespace
{

bool setField(PointViewPtr view, TileDBReader::DimInfo di, PointId idx,
    size_t bufOffset)
{
    // Span is a count of the number of elements in each set of data, so
    // offset is a count of item types.  We're doing pointer arithmetic
    // below, so the size of the type is accounted for.
    bufOffset = bufOffset * di.m_span + di.m_offset;
    TileDBReader::Buffer& buf = *di.m_buffer;
    switch (di.m_type)
    {
    case Dimension::Type::Signed8:
        view->setField(di.m_id, idx, *(buf.get<int8_t>() + bufOffset));
        break;
    case Dimension::Type::Unsigned8:
        view->setField(di.m_id, idx, *(buf.get<uint8_t>() + bufOffset));
        break;
    case Dimension::Type::Signed16:
        view->setField(di.m_id, idx, *(buf.get<int16_t>() + bufOffset));
        break;
    case Dimension::Type::Unsigned16:
        view->setField(di.m_id, idx, *(buf.get<uint16_t>() + bufOffset));
        break;
    case Dimension::Type::Signed32:
        view->setField(di.m_id, idx, *(buf.get<int32_t>() + bufOffset));
        break;
    case Dimension::Type::Unsigned32:
        view->setField(di.m_id, idx, *(buf.get<uint32_t>() + bufOffset));
        break;
    case Dimension::Type::Signed64:
        view->setField(di.m_id, idx, *(buf.get<int64_t>() + bufOffset));
        break;
    case Dimension::Type::Unsigned64:
        view->setField(di.m_id, idx, *(buf.get<uint64_t>() + bufOffset));
        break;
    case Dimension::Type::Float:
        view->setField(di.m_id, idx, *(buf.get<float>() + bufOffset));
        break;
    case Dimension::Type::Double:
        view->setField(di.m_id, idx, *(buf.get<double>() + bufOffset));
        break;
    default:
        return false;
    }
    return true;
}

} // unnamed namespace

point_count_t TileDBReader::read(PointViewPtr view, point_count_t count)
{
    PointId idx = view->size();
    point_count_t numRead = 0;

    tiledb::Query::Status status;
    do
    {
        if (m_stats)
            tiledb::Stats::enable();
        m_query->submit();
        if (m_stats)
        {
            tiledb::Stats::dump(stdout);
            tiledb::Stats::disable();
        }

        status = m_query->query_status();

        // The result buffer count represents the total number of items
        // returned by the query for dimensions.  So if there are three
        // dimensions, the number of points returned is the buffer count
        // divided by the number of dimensions.
        size_t result_num =
            (int)m_query->result_buffer_elements()[TILEDB_COORDS].second /
            m_array->schema().domain().dimensions().size();

        if (status == tiledb::Query::Status::INCOMPLETE && result_num == 0)
            throwError("Need to increase chunk_size for reader.");

        for (size_t i = 0; i < result_num; ++i)
        {
            for (DimInfo& dim : m_dims)
                if (!setField(view, dim, idx, i))
                    throwError("Invalid dimension type when setting data.");

            // progess callback
            if (m_cb)
                m_cb(*view, idx);

            idx++;
            numRead++;

            if (numRead == count)
                break;
        }
    }
    while(status == tiledb::Query::Status::INCOMPLETE);

    if (status == tiledb::Query::Status::FAILED)
        throwError("Unable to read from " + m_arrayName);
    return numRead;
}

void TileDBReader::done(pdal::BasePointTable &table)
{
    m_array->close();
}

} // namespace pdal
