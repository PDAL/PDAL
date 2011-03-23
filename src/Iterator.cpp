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

#include <libpc/Iterator.hpp>
#include <libpc/Stage.hpp>
#include <libpc/Filter.hpp>

namespace libpc
{

static boost::uint32_t s_defaultChunkSize = 1024;

Iterator::Iterator(const Stage& stage)
    : m_index(0)
    , m_stage(stage)
    , m_chunkSize(s_defaultChunkSize)
{
    return;
}


Iterator::~Iterator()
{
    return;
}


const Stage& Iterator::getStage() const
{
    return m_stage;
}


boost::uint64_t Iterator::getIndex() const
{
    return m_index;
}


void Iterator::setChunkSize(boost::uint32_t size)
{
    m_chunkSize = size;
}


boost::uint32_t Iterator::getChunkSize() const
{
    return m_chunkSize;
}


boost::uint32_t Iterator::read(PointBuffer& data)
{
    const boost::uint32_t numRead = readImpl(data);

    m_index += numRead;

    return numRead;
}




SequentialIterator::SequentialIterator(const Stage& stage)
    : Iterator(stage)
{
    return;
}


SequentialIterator::~SequentialIterator()
{
    return;
}


boost::uint64_t SequentialIterator::skip(boost::uint64_t count)
{
    const boost::uint64_t numSkipped = skipImpl(count);

    m_index += numSkipped;

    return numSkipped;
}


bool SequentialIterator::atEnd() const
{
    return atEndImpl();
}



RandomIterator::RandomIterator(const Stage& stage)
    : Iterator(stage)
{
    return;
}


RandomIterator::~RandomIterator()
{
    return;
}


boost::uint64_t RandomIterator::seek(boost::uint64_t position)
{
    const boost::uint64_t newPos = seekImpl(position);

    m_index = newPos;

    return newPos;
}


} // namespace libpc
