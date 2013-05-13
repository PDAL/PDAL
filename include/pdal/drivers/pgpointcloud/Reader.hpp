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

#ifndef INCLUDED_PDAL_DRIVER_PGPOINTCLOUD_READER_HPP
#define INCLUDED_PDAL_DRIVER_PGPOINTCLOUD_READER_HPP

#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>
#include <pdal/PointBuffer.hpp>

#include <pdal/drivers/pgpointcloud/common.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <vector>


namespace pdal
{
namespace drivers
{
namespace pgpointcloud
{


class PDAL_DLL Reader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.pgpointcloud.reader", "PGPointcloud Reader")

    Reader(const Options&);
    ~Reader();

    virtual void initialize();
    static Options getDefaultOptions();

    bool supportsIterator(StageIteratorType t) const
    {
        if (t == StageIterator_Sequential) return true;
        return false;
    }
    
    virtual boost::uint64_t getNumPoints() const;
    boost::uint64_t getMaxPoints() const;
    std::string getDataQuery() const;
    void getSession() const;

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;


private:

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented

    pdal::SpatialReference fetchSpatialReference() const;
    boost::uint32_t fetchPcid() const;
    pdal::Schema fetchSchema() const;

    ::soci::session* m_session;
    std::string m_connection;
    std::string m_table_name;
    std::string m_schema_name;
    std::string m_column_name;
    std::string m_where;
    mutable boost::uint32_t m_pcid;
    mutable boost::uint64_t m_cached_point_count;
    mutable boost::uint64_t m_cached_max_points;

}; // pdal.drivers.pgpointcloud.Reader


namespace iterators
{
namespace sequential
{

typedef boost::shared_ptr<PointBuffer> BufferPtr;


class Iterator : public pdal::StageSequentialIterator
{
public:
    Iterator(const pdal::drivers::pgpointcloud::Reader& reader, PointBuffer& buffer);
    ~Iterator();

protected:


private:
    //
    // Methods
    //
    const pdal::drivers::pgpointcloud::Reader& getReader() const;

    void setupDatabaseQuery();

    // Skip count points, return number of points skipped
    boost::uint64_t skipImpl(boost::uint64_t count);

    // Fill the provided pointbuffer, return the number of points written
    boost::uint32_t readBufferImpl(PointBuffer& data);

    // True when there are no more points to read
    bool atEndImpl() const;

    //
    // Members
    //
    const pdal::drivers::pgpointcloud::Reader& m_reader;
    bool m_at_end;
    pdal::PointBuffer* m_buffer;
    boost::uint64_t m_buffer_position;

    ::soci::statement* m_statement;
    std::string m_patch_hex;
    boost::uint32_t m_patch_npoints;
    ::soci::session* m_session;
    pdal::DimensionMap* m_dimension_map;


}; // pdal.drivers.pgpointcloud.sequential.iterators.Iterator


} // sequential
} // iterators


} // pgpointcloud
} // driver
} // pdal


#endif // INCLUDED_PDAL_DRIVER_PGPOINTCLOUD_READER_HPP
