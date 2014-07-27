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


// SQLiteIterator::SQLiteIterator(const pdal::drivers::sqlite::SQLiteReader& reader)
//     : m_at_end(false),  m_reader(reader)
// {
//     std::string const& connection =
//         m_reader.getOptions().getValueOrThrow<std::string>("connection");
//     std::string const& query =
//         m_reader.getOptions().getValueOrThrow<std::string>("query");
// }
//
//
// const pdal::drivers::sqlite::SQLiteReader& SQLiteIterator::getReader() const
// {
//     return m_reader;
// }
//
// // void IteratorBase::readBlob(::soci::row& block, boost::uint32_t howMany)
// // {
// //     boost::uint32_t nAmountRead = 0;
// //
// //     std::stringstream hex_data;
// //     hex_data << block.get<std::string>("points");
// //
// //     std::size_t trim = 2;
// //     std::string trimmed = hex_data.str().substr(trim, hex_data.str().size()-trim);
// //     std::vector<boost::uint8_t> binary_data = Utils::hex_string_to_binary(trimmed);
// //
// //
// //     unsigned char* data = (unsigned char*) &(binary_data.front());
// //
// //     Schema const& oracle_schema = m_active_buffer->getSchema();
// //
// //     boost::uint32_t howMuchWeRead = binary_data.size();
// //     boost::uint32_t howMuchTheBlobShouldBe =
// //         block.get<int>("num_points") * oracle_schema.getByteSize();
// //     if (howMuchWeRead != howMuchTheBlobShouldBe)
// //     {
// //         std::stringstream oss;
// //         oss << "Did not read the amount of binary data as expected "
// //             "-- read: " << howMuchWeRead << " should read: " <<
// //             howMuchTheBlobShouldBe;
// //         throw sqlite_driver_error(oss.str());
// //     }
// //     boost::uint32_t howMuchToRead = howMany * oracle_schema.getByteSize();
// // //ABELL
// // //    m_active_buffer->setDataStride(data, 0, howMuchToRead);
// // }
//
//
//
// //
// // void IteratorBase::copyDatabaseData(PointBuffer& source,
// //     PointBuffer& destination, Dimension const& dest_dim,
// //     uint32_t source_starting_position, uint32_t destination_starting_position,
// //     uint32_t howMany)
// // {
// //
// //     boost::optional<Dimension const&> source_dim =
// //         source.getSchema().getDimensionOptional(dest_dim.getName());
// //
// //     if (!source_dim)
// //         return;
// //
// //     for (uint32_t i = 0; i < howMany; ++i)
// //     {
// //         if (dest_dim.getInterpretation() == source_dim->getInterpretation() &&
// //             dest_dim.getByteSize() == source_dim->getByteSize() &&
// //             pdal::Utils::compare_distance(dest_dim.getNumericScale(),
// //                 source_dim->getNumericScale()) &&
// //             pdal::Utils::compare_distance(dest_dim.getNumericOffset(),
// //                 source_dim->getNumericOffset()) )
// //         {
// //             // FIXME: This test could produce false positives
// // //ABELL
// // uint8_t *source_position = NULL;
// // //            uint8_t *source_position =
// // //                source.getData(source_starting_position+i) +
// // //                source_dim->getByteOffset();
// // //ABELL
// // uint8_t *destination_position = NULL;
// // //            uint8_t *destination_position =
// // //                destination.getData(destination_starting_position + i) +
// // //                dest_dim.getByteOffset();
// //             memcpy(destination_position, source_position,
// //                 source_dim->getByteSize());
// //         }
// //         else
// //         {
// //             //ABELL - No scaling.
// //             ;
// // /**
// //             PointBuffer::scaleData(source, destination, *source_dim, dest_dim,
// //                 source_starting_position + i,
// //                 destination_starting_position + i);
// // **/
// //         }
// //     }
// // }
//
//
// //
// // PointBufferPtr IteratorBase::fetchPointBuffer(int32_t const& cloud_id,
// //     std::string const& schema_xml, uint32_t capacity)
// // {
// //     BufferMap::const_iterator i = m_buffers.find(cloud_id);
// //     if (i != m_buffers.end())
// //     {
// //         getReader().log()->get(logDEBUG2) <<
// //             "IteratorBase::fetchPointBuffer: found existing PointBuffer "
// //             "with id " << cloud_id << std::endl;
// //         return i->second;
// //     }
// //     else
// //     {
// //         std::stringstream query;
// //
// //         Schema schema = Schema::from_xml(schema_xml);
// // //ABELL
// // //        PointBufferPtr output  = BufferPtr(new PointBuffer(schema, capacity));
// // PointBufferPtr output;
// //         std::pair<int, PointBufferPtr> p(cloud_id, output);
// //         m_buffers.insert(p);
// //         getReader().log()->get(logDEBUG2) <<
// //             "IteratorBase::fetchPointBuffer: creating new PointBuffer "
// //             "with id " << cloud_id << std::endl;
// //         return p.second;
// //     }
// // }
//
//
// uint32_t SQLiteIterator::myReadBlocks(PointBuffer& user_buffer)
// {
//     uint32_t numPointsRead = 0;
//
//     std::string const& query =
//         getReader().getOptions().getValueOrThrow<std::string>("query");
//
//     // ::soci::row block;
//    //  ::soci::indicator ind = ::soci::i_null;
//    //  ::soci::statement blocks =
//    //      (m_session->prepare << query, ::soci::into(block, ind));
//    //  blocks.execute();
//    //
//    //  bool bDidRead = blocks.fetch();
//    //  if (!m_active_buffer)
//    //  {
//    //      //ABELL
//    //      m_active_buffer = fetchPointBuffer(block.get<int>("cloud_id"),
//    //          block.get<std::string>("schema"), user_buffer.size());
//    //      m_active_cloud_id = block.get<int>("cloud_id");
//    //  }
//    //
//    //  // This shouldn't ever happen
//    //  int num_points = block.get<int>("num_points");
//    //  if (num_points > static_cast<int32_t>(m_active_buffer->size()))
//    //  {
//    //      std::ostringstream oss;
//    //      oss << "Block size, " << num_points << ", is too large to fit in " <<
//    //          "buffer of size " << user_buffer.size() <<
//    //          ". Increase buffer capacity with writer's \"chunk_size\" option " <<
//    //          "or increase the read buffer size";
//    //      throw buffer_too_small(oss.str());
//    //  }
//    //
//    //  while (bDidRead)
//    //  {
//    //      uint32_t numReadThisBlock =
//    //          static_cast<uint32_t>(block.get<int>("num_points"));
//    //
//    //      //ABELL - Borken and broken.
//    //      uint32_t numSpaceLeftThisBuffer =
//    //          user_buffer.size() - user_buffer.size();
//    //
//    //      getReader().log()->get(logDEBUG4) << "IteratorBase::myReadBlocks:"
//    //          "numReadThisBlock: " << numReadThisBlock <<
//    //          " numSpaceLeftThisBlock: " << numSpaceLeftThisBuffer <<
//    //          " total numPointsRead: " << numPointsRead << std::endl;
//    //
//    //      numPointsRead = numPointsRead + numReadThisBlock;
//    //      readBlob(block, std::min(numReadThisBlock, numSpaceLeftThisBuffer));
//    //      fillUserBuffer(user_buffer);
//    //      bDidRead = blocks.fetch();
//    //
//    //      int32_t const& current_cloud_id = block.get<int>("cloud_id");
//    //      if (current_cloud_id != m_active_cloud_id)
//    //      {
//    //          getReader().log()->get(logDEBUG3) <<
//    //              "IteratorBase::myReadBlocks: current_cloud_id: " <<
//    //              current_cloud_id << " m_active_cloud_id: " <<
//    //              m_active_cloud_id << std::endl;
//    //          m_active_buffer = fetchPointBuffer(current_cloud_id,
//    //              block.get<std::string>("schema"), user_buffer.size());
//    //          m_active_cloud_id = current_cloud_id;
//    //          return user_buffer.size();
//    //      }
//    //  }
//     return numPointsRead;
// }


//---------------------------------------------------------------------------
//
// SequentialIterator
//
//---------------------------------------------------------------------------

SQLiteIterator::SQLiteIterator(const pdal::drivers::sqlite::SQLiteReader& reader,
                               std::vector<Dimension *> const& dims)
    : m_reader(reader)
    , m_at_end(false)
    , m_dims(dims)
    , b_doneQuery(false)
    , m_point_size(0)
{
    for (size_t i = 0; i < m_dims.size(); ++i)
    {
        m_point_size += m_dims[i]->getByteSize();
    }

    pdal::Options const& options = reader.getOptions();
    std::string const& connection = options.getValueOrThrow<std::string>("connection");

    m_reader.log()->get(logDEBUG) << "Connection: '" << connection << "'" << std::endl;
    m_session = std::unique_ptr<SQLite>(new SQLite(connection, m_reader.log()));
    m_session->connect(false); // don't connect in write mode

    m_patch = std::unique_ptr<Patch>(new Patch());
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
    m_reader.log()->get(logDEBUG) << "fetched patch with " << count 
         << " points and " << size << " bytes bytesize: " << size << std::endl;    
    m_patch->remaining = count;
    m_patch->count = count;
    m_patch->bytes = bytes;
    m_patch->byte_size = size;
    
    point_count_t numRemaining = m_patch->remaining;
    PointId nextId = buffer.size();
    point_count_t numRead = 0;

    size_t offset = ((m_patch->count - m_patch->remaining) * m_point_size);
    uint8_t *pos = (uint8_t*)m_patch->bytes + offset;
    assert(offset <= m_patch->byte_size);
    while (numRead < numPts && numRemaining > 0)
    {
        for (size_t d = 0; d < m_dims.size(); ++d)
        {
            buffer.setRawField(*m_dims[d], nextId, pos);
            pos += m_dims[d]->getByteSize();
        }
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
    
    m_reader.log()->get(logDEBUG) << "readBufferImpl called with "
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
        if (patch_count >= 4 && patch_count < 7)
            std::cout << buffer << std::endl;
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
