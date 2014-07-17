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
#include <pdal/PointBuffer.hpp>

#include <pdal/drivers/pgpointcloud/common.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <vector>
#include <memory>


namespace pdal
{
namespace drivers
{
namespace pgpointcloud
{

    class PgReader;

namespace iterators
{
namespace sequential
{

    class Patch;


class PgIterator : public pdal::StageSequentialIterator
{
public:
    PgIterator(const pdal::drivers::pgpointcloud::PgReader& reader, std::vector<Dimension *> const& dims);
    ~PgIterator();

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
    //
    // Methods
    //
    const pdal::drivers::pgpointcloud::PgReader& getReader() const;

    point_count_t readPgPatch(PointBuffer& buffer, 
                              point_count_t numPts);
    // Internal functions for managing scroll cursor
    bool CursorSetup();
    bool CursorTeardown();
    bool NextBuffer();

    //
    // Members
    //
    const pdal::drivers::pgpointcloud::PgReader& m_reader;
    bool m_at_end;
    // boost::uint64_t m_buffer_position;

    bool m_cursor;
    PGconn* m_session;

    std::vector<Dimension *> m_dims;
    schema::size_type m_point_size;
    boost::uint32_t m_cur_row;
    boost::uint32_t m_cur_nrows;
    PGresult* m_cur_result;
    std::unique_ptr<Patch> m_patch;

}; // pdal.drivers.pgpointcloud.sequential.iterators.Iterator

class Patch
{
public:
    Patch() : count(0), remaining(0)
    {
    };
    
    point_count_t count;
    point_count_t remaining;
    std::string hex;
    
    std::vector<uint8_t> binary;
    static const boost::uint32_t trim = 26;

#define _base(x) ((x >= '0' && x <= '9') ? '0' : \
         (x >= 'a' && x <= 'f') ? 'a' - 10 : \
         (x >= 'A' && x <= 'F') ? 'A' - 10 : \
            '\255')
#define HEXOF(x) (x - _base(x))

    inline void update_binary()
    {
        // http://stackoverflow.com/questions/8197838/convert-a-long-hex-string-in-to-int-array-with-sscanf
        binary.resize((hex.size() - trim)/2);
        
        char const* source = hex.c_str() + trim;
        char const* p = 0;

        for (p = source; p && *p; p+=2 ) {
                 binary[(p - source) >> 1] =
                 ((HEXOF(*p)) << 4) + HEXOF(*(p+1));
         }
     }
 };

} // sequential
} // iterators


} // pgpointcloud
} // driver
} // pdal


