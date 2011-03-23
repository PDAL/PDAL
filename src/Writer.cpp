/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <cassert>

#include <boost/scoped_ptr.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/Writer.hpp>
#include <libpc/Iterator.hpp>

namespace libpc
{

const boost::uint32_t Writer::s_defaultChunkSize = 1024 * 32;


Writer::Writer(Stage& prevStage)
    : m_actualNumPointsWritten(0)
    , m_targetNumPointsToWrite(0)
    , m_prevStage(prevStage)
    , m_chunkSize(s_defaultChunkSize)
{
    return;
}


Stage& Writer::getPrevStage()
{
    return m_prevStage;
}


void Writer::setChunkSize(boost::uint32_t chunkSize)
{
    m_chunkSize = chunkSize;
}


boost::uint32_t Writer::getChunkSize() const
{
    return m_chunkSize;
}


boost::uint64_t Writer::write(std::size_t targetNumPointsToWrite)
{
    m_targetNumPointsToWrite = targetNumPointsToWrite;
    m_actualNumPointsWritten = 0;

    boost::scoped_ptr<SequentialIterator> iter(m_prevStage.createSequentialIterator());

    writeBegin();

    while (m_actualNumPointsWritten < m_targetNumPointsToWrite)
    {
        boost::uint64_t numRemainingPointsToRead = m_targetNumPointsToWrite - m_actualNumPointsWritten;
        boost::uint32_t numPointsToReadThisChunk = static_cast<boost::uint32_t>(std::min<boost::uint64_t>(numRemainingPointsToRead, m_chunkSize));

        PointBuffer buffer(m_prevStage.getHeader().getSchema(), numPointsToReadThisChunk);


        boost::uint32_t numPointsReadThisChunk = iter->read(buffer);
        assert(numPointsReadThisChunk <= numPointsToReadThisChunk);

        boost::uint32_t numPointsWrittenThisChunk = writeBuffer(buffer);
        assert(numPointsWrittenThisChunk == numPointsReadThisChunk);

        m_actualNumPointsWritten += numPointsWrittenThisChunk;

        if (iter->atEnd())
        {
            break;
        }
    }

    writeEnd();

    assert(m_actualNumPointsWritten <= m_targetNumPointsToWrite);

    return m_actualNumPointsWritten;
}


} // namespace libpc
