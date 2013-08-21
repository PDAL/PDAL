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

#include <pdal/FilterIterator.hpp>
#include <pdal/Filter.hpp>

namespace pdal
{


FilterSequentialIterator::FilterSequentialIterator(const Filter& filter, PointBuffer& buffer)
    : StageSequentialIterator(filter, buffer)
    , m_filter(filter)
    , m_prevIterator(NULL)
{
    m_prevIterator = m_filter.getPrevStage().createSequentialIterator(buffer);

    return;
}


FilterSequentialIterator::~FilterSequentialIterator()
{
    if (m_prevIterator)
        delete m_prevIterator;
}


StageSequentialIterator& FilterSequentialIterator::getPrevIterator()
{
    if (!m_prevIterator)
    {
        std::ostringstream oss;
        oss << "Non-const sequential iterator for stage of type '" <<  m_filter.getPrevStage().getName() << "' is not available";
        throw iterator_not_found(oss.str());
    }
    return *m_prevIterator;
}


const StageSequentialIterator& FilterSequentialIterator::getPrevIterator() const
{
    if (!m_prevIterator)
    {
        std::ostringstream oss;
        oss << "Const sequential iterator for stage of type '" <<  m_filter.getPrevStage().getName() << "' is not available";
        throw iterator_not_found(oss.str());
    }
    return *m_prevIterator;
}



FilterRandomIterator::FilterRandomIterator(const Filter& filter, PointBuffer& buffer)
    : StageRandomIterator(filter, buffer)
    , m_filter(filter)
    , m_prevIterator(NULL)
{
    m_prevIterator = m_filter.getPrevStage().createRandomIterator(buffer);

    return;
}


FilterRandomIterator::~FilterRandomIterator()
{
    if (m_prevIterator)
        delete m_prevIterator;
}


StageRandomIterator& FilterRandomIterator::getPrevIterator()
{
    if (!m_prevIterator)
    {
        std::ostringstream oss;
        oss << "Non-const random iterator for stage of type '" <<  m_filter.getPrevStage().getName() << "' is not available";
        throw iterator_not_found(oss.str());
    }
    return *m_prevIterator;
}


const StageRandomIterator& FilterRandomIterator::getPrevIterator() const
{
    if (!m_prevIterator)
    {
        std::ostringstream oss;
        oss << "Const random terator for stage of type '" <<  m_filter.getPrevStage().getName() << "' is not available";
        throw iterator_not_found(oss.str());
    }
    return *m_prevIterator;
}



} // namespace pdal
