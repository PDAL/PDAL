/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#pragma once

#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>


#include <pdal/drivers/sqlite/SQLiteCommon.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <vector>


namespace pdal
{
namespace drivers
{
namespace sqlite
{

    class SQLiteReader;

namespace iterators
{

namespace sequential
{
    

// class IteratorBase
// {
// public:
//     IteratorBase(const pdal::drivers::sqlite::SQLiteReader& reader);
//
// protected:
//     const pdal::drivers::sqlite::SQLiteReader& getReader() const;
//
//     boost::uint32_t myReadBuffer(PointBuffer& data);
//
//     boost::uint32_t myReadBlocks(PointBuffer& data);
//     //
//     // PointBufferPtr fetchPointBuffer( boost::int32_t const& cloud_id,
//     //                             std::string const& schema_xml,
//     //                             boost::uint32_t capacity);
//
//     bool m_at_end;
//     boost::int32_t m_active_cloud_id;
//     PointBufferPtr m_active_buffer;
//     BufferMap m_buffers;
//
//
// private:
//     const pdal::drivers::sqlite::SQLiteReader& m_reader;
//     sqlite3* m_session;
//
//
//     // ::soci::statement getNextCloud(std::string const& cloud_table_name,
//     //     boost::int32_t& cloud_id, ::soci::row& r);
//     // void readBlob(::soci::row& block, boost::uint32_t howMany);
//     // void fillUserBuffer(PointBuffer& user_buffer);
//     //
//     // pdal::Bounds<double> getBounds(Statement statement, BlockPtr block);
//     IteratorBase& operator=(const IteratorBase&); // not implemented
//     IteratorBase(const IteratorBase&); // not implemented;
// };


class SQLiteIterator : public pdal::StageSequentialIterator
{
public:
    SQLiteIterator(const pdal::drivers::sqlite::SQLiteReader& reader, PatchPtr patch);

protected:
    // Skip count points, return number of points skipped
    boost::uint64_t skipImpl(boost::uint64_t count);

    // Fill the provided pointbuffer, return the number of points written
    point_count_t readImpl(PointBuffer& user_buffer, point_count_t count);

    point_count_t readBufferImpl(PointBuffer& buffer)
    {
        return readImpl(buffer, (std::numeric_limits<point_count_t>::max)());
    }
    // True when there are no more points to read
    bool atEndImpl() const;
           
private:
    const pdal::drivers::sqlite::SQLiteReader& m_reader;
    point_count_t readPatch(PointBuffer& buffer, 
                              point_count_t numPts);
    bool NextBuffer();
    bool doQuery();
    void validateQuery() const;
    bool m_at_end;
    std::unique_ptr<SQLite> m_session;
    bool b_doneQuery;
    PatchPtr m_patch;
    int32_t m_point_size;    
};



} // sequential

} // iterators
}
}
} // namespace pdal::driver::sqlite

