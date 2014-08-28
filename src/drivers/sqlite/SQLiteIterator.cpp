/******************************************************************************
* Copyright (c) 2014, Howard Butler, howard@hobu.co
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

#include <pdal/drivers/sqlite/SQLiteIterator.hpp>
#include <pdal/drivers/sqlite/SQLiteReader.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Utils.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <map>
#include <string>

namespace pdal
{
namespace drivers
{
namespace sqlite
{


namespace iterators
{
namespace sequential
{

SQLiteIterator::SQLiteIterator(const pdal::drivers::sqlite::SQLiteReader& reader, PatchPtr patch )
    : m_reader(reader)
    , m_at_end(false)
    , b_doneQuery(false)
    , m_patch(patch)
{

    pdal::Options const& options = reader.getOptions();
    std::string const& connection = options.getValueOrThrow<std::string>("connection");

    m_reader.log()->get(LogLevel::Debug) << "Connection: '" << connection << "'" << std::endl;
    m_session = std::unique_ptr<SQLite>(new SQLite(connection, m_reader.log()));
    m_session->connect(false); // don't connect in write mode


    schema::DimInfoList dims = patch->m_schema.m_dims;
    for (auto di = dims.begin(); di != dims.end(); ++di)
        m_point_size += Dimension::size(di->m_type);    
}

bool SQLiteIterator::doQuery()
{
    pdal::Options const& options = m_reader.getOptions();
    std::string const& query = options.getValueOrThrow<std::string>("query");
    m_session->query(query);
    return true;
}

void SQLiteIterator::validateQuery() const
{
    std::set<std::string> reqFields;
    reqFields.insert("POINTS");
    reqFields.insert("SCHEMA");
    reqFields.insert("NUM_POINTS");
    reqFields.insert("CLOUD");

    for (auto r = reqFields.begin(); r != reqFields.end(); ++r)
    {
        auto p = m_session->columns().find(*r);
        if (p == m_session->columns().end())
        {
            std::ostringstream oss;
            oss << "Unable to find required column name '" << *r << "'";
            throw pdal_error(oss.str());
        }
    }
}


bool SQLiteIterator::NextBuffer()
{
    return m_session->next();
}

point_count_t SQLiteIterator::readPatch(PointBuffer& buffer, point_count_t numPts)
{

    const row* r = m_session->get();
    if (!r)
        throw pdal_error("readPatch with no data in session!");
    std::map<std::string, int32_t> const& columns = m_session->columns();

    // Availability of positions already validated
    int32_t position = columns.find("POINTS")->second;
    auto bytes = (*r)[position].blobBuf;
    size_t size = (*r)[position].blobLen;
    position = columns.find("NUM_POINTS")->second;
    int32_t count = boost::lexical_cast<int32_t>((*r)[position].data);
    m_reader.log()->get(LogLevel::Debug4) << "fetched patch with " << count 
         << " points and " << size << " bytes bytesize: " << size << std::endl;    
    m_patch->remaining = count;
    m_patch->count = count;
    m_patch->bytes = bytes;
    m_patch->byte_size = size;
    
    point_count_t numRemaining = m_patch->remaining;
    PointId nextId = buffer.size();
    point_count_t numRead = 0;

    size_t offset = ((m_patch->count - m_patch->remaining) * m_point_size);
    uint8_t *pos = &(m_patch->bytes.front()) + offset;

    schema::DimInfoList& dims = m_patch->m_schema.m_dims;
    while (numRead < numPts && numRemaining > 0)
    {
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            schema::DimInfo& d = *di;
            buffer.setField(d.m_id, d.m_type, nextId, pos);
            pos += Dimension::size(d.m_type);
        }

        // Scale X, Y and Z
        // double v = buffer.getFieldAs<double>(Dimension::Id::X, nextId);
        // v = v * m_patch->xScale() + m_patch->xOffset();
        // buffer.setField(Dimension::Id::X, nextId, v);
        //
        // v = buffer.getFieldAs<double>(Dimension::Id::Y, nextId);
        // v = v * m_patch->yScale() + m_patch->yOffset();
        // buffer.setField(Dimension::Id::Y, nextId, v);
        //
        // v = buffer.getFieldAs<double>(Dimension::Id::Z, nextId);
        // v = v * m_patch->zScale() + m_patch->zOffset();
        // buffer.setField(Dimension::Id::Z, nextId, v);

        numRemaining--;
        nextId++;
        numRead++;
    }
    
    m_patch->remaining = numRemaining;
    
    
    return numRead;
}


point_count_t SQLiteIterator::readImpl(PointBuffer& buffer, point_count_t count)
{
    if (atEndImpl())
        return 0;
    
    m_reader.log()->get(LogLevel::Debug4) << "readBufferImpl called with "
        "PointBuffer filled to " << buffer.size() << " points" <<
        std::endl;

    point_count_t totalNumRead = 0;

    if (! b_doneQuery)
    {
        // read first patch
        doQuery();
        validateQuery();
        b_doneQuery = true;
        totalNumRead = readPatch(buffer, count); 
    }
    int patch_count(0);
    while (totalNumRead < count)
    {
        if (m_patch->remaining == 0)
        {
            if (!NextBuffer())
            {
                return totalNumRead;
            }
        }
        PointId bufBegin = buffer.size();
        point_count_t numRead = readPatch(buffer, count - totalNumRead);
        PointId bufEnd = bufBegin + numRead;
        totalNumRead += numRead;
        patch_count++;

    }

    return totalNumRead;

}

boost::uint64_t SQLiteIterator::skipImpl(boost::uint64_t count)
{
    boost::uint64_t initialCount = count;
    while (count)
    {
        point_count_t numRem = m_patch->remaining;
        point_count_t blockCount = std::min((point_count_t)count, numRem);
        m_patch->remaining = (numRem - blockCount);
        count -= blockCount;
        if (count > 0)
            if (! NextBuffer())
                break;
    }
    return initialCount - count;
}


bool SQLiteIterator::atEndImpl() const
{
    return m_at_end;
}


}
} // iterators::sequential::

}
}
} // namespace pdal::driver::sqlite
