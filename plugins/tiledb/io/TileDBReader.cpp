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

#include <boost/any.hpp>

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

template <typename T>
void makeBuffer(tiledb::Query& query, std::map<std::string, pdalboost::any>& mp,  const std::string& key,
        const unsigned bufSize)
{
    auto sp = std::make_shared<std::vector<T>>(bufSize);
    mp[key] = sp;

    if (key != TILEDB_COORDS)
        query.set_buffer(key, *sp);
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
        case TILEDB_FLOAT64:
            return Dimension::Type::Float;
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
    args.add("config_file", "TileDB configuration file location", m_cfgFileName);
    args.add("chunk_size", "TileDB read chunk size", m_chunkSize, point_count_t(1000000));
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
    std::vector<tiledb::Dimension> dims = m_array->schema().domain().dimensions();
    auto attrs = m_array->schema().attributes();

    for (const auto& dim: dims)
    {
        layout->registerOrAssignDim(dim.name(), getPdalType(dim.type()));
    }

    for (const auto& a : attrs) {
        layout->registerOrAssignDim(a.first, getPdalType(a.second.type()));
    }

}

point_count_t TileDBReader::read(PointViewPtr view, point_count_t count)
{
    PointId idx = view->size();
    PointRef point(*view, idx);
    point_count_t numRead = 0;
    tiledb::Query query(*m_ctx, *m_array);
    std::vector<tiledb::Dimension> dims = m_array->schema().domain().dimensions();
    point_count_t chunkSize = std::min(m_chunkSize, count);
    int nDims = dims.size();

    std::map<std::string, pdalboost::any> buffers;

    auto attrs = m_array->schema().attributes();

    makeBuffer<double>(query, buffers, TILEDB_COORDS, chunkSize * dims.size());
    auto spCoords = pdalboost::any_cast<std::shared_ptr<std::vector<double>>>(buffers[TILEDB_COORDS]);
    query.set_coordinates(*spCoords);

    if (!m_bbox.empty())
    {
        if (nDims == 2)
            query.set_subarray({m_bbox.minx, m_bbox.minx, m_bbox.miny, m_bbox.maxy});
        else
            query.set_subarray({m_bbox.minx, m_bbox.minx, m_bbox.miny, m_bbox.maxy, m_bbox.minz, m_bbox.maxz});
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
        query.set_subarray(subarray);
    }

    for (const auto& a : attrs) {
        std::string attrName = a.first;
        Dimension::Id id = view->layout()->findDim(a.first);
        Dimension::Type t = view->dimType(id);

        switch (t)
        {
            case Dimension::Type::Double:
            {
                makeBuffer<double>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::Float:
            {
                makeBuffer<float>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::Signed8:
            {
                makeBuffer<char>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::Signed16:
            {
                makeBuffer<short>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::Signed32:
            {
                makeBuffer<int>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::Signed64:
            {
                makeBuffer<long>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::Unsigned8:
            {
                makeBuffer<unsigned char>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::Unsigned16:
            {
                makeBuffer<unsigned short>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::Unsigned32:
            {
                makeBuffer<unsigned int>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::Unsigned64:
            {
                makeBuffer<unsigned long>(query, buffers, attrName, chunkSize);
                break;
            }
            case Dimension::Type::None:
            default:
                throw pdal_error("Unsupported attribute type for " + attrName);
        }
    }

    tiledb::Query::Status status;

    do
    {
        if (m_stats)
            tiledb::Stats::enable();

        query.submit();

        if (m_stats)
        {
            tiledb::Stats::dump(stdout);
            tiledb::Stats::disable();
        }

        status = query.query_status();

        auto result_num = (int)query.result_buffer_elements()[TILEDB_COORDS].second;
        if (status == tiledb::Query::Status::INCOMPLETE &&
            result_num == 0)
        {
            throwError("Need to increase chunk_size for reader.");
        }
        else
        {
            point.setPointId(idx);
            for (int i = 0; i < result_num; i += nDims) {
                view->setField(Dimension::id("X"), idx, (*spCoords)[i]);
                view->setField(Dimension::id("Y"), idx, (*spCoords)[i + 1]);

                if (nDims == 3)
                    view->setField(Dimension::id("Z"), idx, (*spCoords)[i + 2]);

                // read tiledb attributes, this has to be done point by point for future streaming support
                for (const auto &kv : buffers) {
                    if (kv.first != TILEDB_COORDS) {
                        Dimension::Id id = view->layout()->findDim(kv.first);
                        Dimension::Type t = view->dimType(id);

                        int p = i / nDims;
                        switch (t) {
                            case Dimension::Type::Double: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<double>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::Float: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<float>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::Signed8: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<char>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::Signed16: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<short>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::Signed32: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<int>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::Signed64: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<long>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::Unsigned8: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<unsigned char>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::Unsigned16: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<unsigned short>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::Unsigned32: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<unsigned int>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::Unsigned64: {
                                auto sp = pdalboost::any_cast<std::shared_ptr<std::vector<unsigned long>>>(kv.second);
                                view->setField(id, idx, (*sp)[p]);
                                break;
                            }
                            case Dimension::Type::None:
                            default:
                                throw pdal_error("Unsupported attribute type for " + kv.first);
                        }
                    }
                }
                // progess callback
                if (m_cb)
                    m_cb(*view, idx);

                idx++;
                numRead++;

                if (numRead == count)
                    break;
            } // end for loop
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
