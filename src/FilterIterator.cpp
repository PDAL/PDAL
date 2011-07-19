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


FilterSequentialIterator::FilterSequentialIterator(const Filter& filter)
    : StageSequentialIterator(filter)
    , m_filter(filter)
{
    m_prevIterator = m_filter.getPrevStage()->createSequentialIterator();

    return;
}


FilterSequentialIterator::~FilterSequentialIterator()
{
    return;
}


StageSequentialIteratorPtr FilterSequentialIterator::getPrevIterator()
{
    return m_prevIterator;
}


const StageSequentialIteratorPtr FilterSequentialIterator::getPrevIterator() const
{
    return m_prevIterator;
}



FilterRandomIterator::FilterRandomIterator(const Filter& filter)
    : StageRandomIterator(filter)
    , m_filter(filter)
{
    m_prevIterator = m_filter.getPrevStage()->createRandomIterator();

    return;
}


FilterRandomIterator::~FilterRandomIterator()
{
    return;
}


StageRandomIteratorPtr FilterRandomIterator::getPrevIterator()
{
    return m_prevIterator;
}


const StageRandomIteratorPtr FilterRandomIterator::getPrevIterator() const
{
    return m_prevIterator;
}

FilterBlockIterator::FilterBlockIterator(const Filter& filter)
    : StageBlockIterator(filter)
    , m_filter(filter)
{
    m_prevIterator = m_filter.getPrevStage()->createBlockIterator();

    return;
}


FilterBlockIterator::~FilterBlockIterator()
{
    return;
}


StageBlockIteratorPtr FilterBlockIterator::getPrevIterator()
{
    return m_prevIterator;
}


const StageBlockIteratorPtr FilterBlockIterator::getPrevIterator() const
{
    return m_prevIterator;
}


} // namespace pdal
