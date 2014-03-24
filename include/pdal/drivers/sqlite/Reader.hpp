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

#ifndef INCLUDED_PDAL_DRIVER_SQLITE_READER_HPP
#define INCLUDED_PDAL_DRIVER_SQLITE_READER_HPP

#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>


#include <pdal/drivers/sqlite/common.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <vector>


namespace pdal
{
namespace drivers
{
namespace sqlite
{



class PDAL_DLL Reader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.sqlite.reader", "SOCI Reader")

    Reader(const Options&);
    ~Reader();

    virtual void initialize();
    static Options getDefaultOptions();
    virtual boost::uint64_t getNumPoints() const;
    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;
    pdal::Schema fetchSchema(std::string const& query) const;
    pdal::SpatialReference
        fetchSpatialReference(std::string const& query) const;

private:

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented

    mutable boost::uint64_t m_cachedPointCount;


    ::soci::session* m_session;

};

namespace iterators
{

namespace sequential
{


    typedef boost::shared_ptr<PointBuffer> BufferPtr;
    typedef std::map<int, BufferPtr> BufferMap;

class IteratorBase
{
public:
    IteratorBase(const pdal::drivers::sqlite::Reader& reader);
    ~IteratorBase();

protected:
    const pdal::drivers::sqlite::Reader& getReader() const;

    boost::uint32_t myReadBuffer(PointBuffer& data);

    boost::uint32_t myReadBlocks(PointBuffer& data);
    // 
    BufferPtr fetchPointBuffer( boost::int32_t const& cloud_id,
                                std::string const& schema_xml,
                                boost::uint32_t capacity);

    bool m_at_end;


    boost::int32_t m_active_cloud_id;
    BufferPtr m_active_buffer;
    BufferMap m_buffers;
    boost::uint32_t m_buffer_position;



private:
    const pdal::drivers::sqlite::Reader& m_reader;

    ::soci::session* m_session;

    
    ::soci::statement getNextCloud(   std::string const& cloud_table_name, 
                                      boost::int32_t& cloud_id,
                                      ::soci::row& r);
    void readBlob(::soci::row& block,
                  boost::uint32_t howMany);
    void fillUserBuffer(PointBuffer& user_buffer);
    // 
    void copyDatabaseData(PointBuffer& source, 
                          PointBuffer& destination, 
                          Dimension const& dest_dim, 
                          boost::uint32_t source_starting_position, 
                          boost::uint32_t destination_starting_position,
                          boost::uint32_t howMany);
    // pdal::Bounds<double> getBounds(Statement statement, BlockPtr block);
    IteratorBase& operator=(const IteratorBase&); // not implemented
    IteratorBase(const IteratorBase&); // not implemented;


};


class Reader : public IteratorBase, public pdal::StageSequentialIterator
{
public:
    Reader(const pdal::drivers::sqlite::Reader& reader, PointBuffer& buffer);
    ~Reader();

private:
    boost::uint64_t skipImpl(boost::uint64_t count);
    boost::uint32_t readBufferImpl(PointBuffer& data);
    bool atEndImpl() const;
};


} // sequential

} // iterators
}
}
} // namespace pdal::driver::oci


#endif // INCLUDED_PDAL_DRIVER_OCI_READER_HPP
