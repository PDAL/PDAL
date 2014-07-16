/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
* Copyright (c) 2013, Paul Ramsey, pramsey@cleverelephant.ca
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

#include <pdal/drivers/pgpointcloud/PgReader.hpp>
#include <pdal/PointBuffer.hpp>
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





namespace pdal
{
namespace drivers
{
namespace pgpointcloud
{

//*********************************************************************************
//  pdal.drivers.pgpointcloud.Reader
//
//  The iterator downbelow controls the actual reading, the Reader just does some
//  basic setup and returning of metadata to the Writer at the other end of
//  the chain.
//
//  The core of PDAL only calls the following methods:
//
//  Options Reader::getDefaultOptions()
//  void Reader::initialize()
//  boost::uint64_t Reader::getNumPoints() const
//  pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
//
//*********************************************************************************


namespace iterators
{
namespace sequential
{

PgIterator::PgIterator(const pdal::drivers::pgpointcloud::PgReader& reader,
                       std::vector<Dimension *> const& dims)
    : m_reader(reader)
    , m_at_end(false)
    , m_cursor(false)
    , m_session(NULL)
    , m_dims(dims)
    , m_point_size(0)
    , m_cur_row(0)
    , m_cur_nrows(0)
    , m_cur_result(NULL)
    , m_patch(std::unique_ptr<Patch>(new Patch))
{
    pdal::Options const& options = reader.getOptions();
    std::string const& connection = options.getValueOrThrow<std::string>("connection");
    m_session = pg_connect(connection);

    m_point_size = 0;
    for (size_t i = 0; i < m_dims.size(); ++i)
    {
        m_point_size += m_dims[i]->getByteSize();
    }
    return;
}

PgIterator::~PgIterator()
{
    if (m_session)
    {
        PQfinish(m_session);
        m_session = NULL;
    }

    if (m_cur_result)
    {
        PQclear(m_cur_result);
        m_cur_result = NULL;
    }

}


const pdal::drivers::pgpointcloud::PgReader& PgIterator::getReader() const
{
    return m_reader;
}


boost::uint64_t PgIterator::skipImpl(boost::uint64_t count)
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


bool PgIterator::atEndImpl() const
{
    getReader().log()->get(logDEBUG) << "atEndImpl called" << std::endl;
    return m_at_end;
}

bool PgIterator::CursorSetup()
{
    std::ostringstream oss;
    oss << "DECLARE cur CURSOR FOR " << getReader().getDataQuery();
    pg_begin(m_session);
    pg_execute(m_session, oss.str());
    m_cursor = true;

    getReader().log()->get(logDEBUG) << "SQL cursor prepared: " << oss.str() << std::endl;
    return true;
}

bool PgIterator::CursorTeardown()
{
    pg_execute(m_session, "CLOSE cur");
    pg_commit(m_session);
    m_cursor = false;
    getReader().log()->get(logDEBUG) << "SQL cursor closed." << std::endl;
    return true;
}

point_count_t PgIterator::readPgPatch(PointBuffer& buffer, point_count_t numPts)
{
    point_count_t numRemaining = m_patch->remaining;
    PointId nextId = buffer.size();
    point_count_t numRead = 0;

    size_t offset = ((m_patch->count - m_patch->remaining) * m_point_size);
    uint8_t *pos = &(m_patch->binary.front()) + offset;
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

bool PgIterator::NextBuffer()
{
    if (! m_cursor)
        CursorSetup();

    if (m_cur_row >= m_cur_nrows || ! m_cur_result)
    {
        static std::string fetch = "FETCH 2 FROM cur";
        if (m_cur_result) PQclear(m_cur_result);
        m_cur_result = pg_query_result(m_session, fetch);
        bool logOutput = getReader().log()->getLevel() > logDEBUG3;
        if (logOutput)
            getReader().log()->get(logDEBUG3) << "SQL: " << fetch << std::endl;
        if (PQresultStatus(m_cur_result) != PGRES_TUPLES_OK || PQntuples(m_cur_result) == 0)
        {
            PQclear(m_cur_result);
            m_cur_result = NULL;
            CursorTeardown();
            return false;
        }

        m_cur_row = 0;
        m_cur_nrows = PQntuples(m_cur_result);
    }

    
    m_patch->hex = PQgetvalue(m_cur_result, m_cur_row, 0);
    m_patch->count = atoi(PQgetvalue(m_cur_result, m_cur_row, 1));
    m_patch->remaining = m_patch->count;
    m_patch->update_binary();

    m_cur_row++;
    return true;
}

point_count_t PgIterator::readImpl(PointBuffer& buffer, point_count_t count)
{
    if (atEndImpl())
        return 0;
    
    getReader().log()->get(logDEBUG) << "readBufferImpl called with "
        "PointBuffer filled to " << buffer.size() << " points" <<
        std::endl;

    // First time through, create the SQL statement, allocate holding pens
    // and fire it off!
    if (! m_cursor)
    {
        CursorSetup();
    }

    point_count_t totalNumRead = 0;
    while (totalNumRead < count)
    {
        if (m_patch->remaining == 0)
            if (!NextBuffer())
                return totalNumRead;
        PointId bufBegin = buffer.size();
        point_count_t numRead = readPgPatch(buffer, count - totalNumRead);
        PointId bufEnd = bufBegin + numRead;
        totalNumRead += numRead;

    }
    return totalNumRead;

}



}
} // pdal.drivers.pgpointcloud.iterators.sequential.Iterator

} // pgpointcloud
} // drivers
} // pdal
