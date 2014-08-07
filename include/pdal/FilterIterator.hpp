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

#ifndef INCLUDED_FILTERITERATOR_HPP
#define INCLUDED_FILTERITERATOR_HPP

#include <pdal/pdal_internal.hpp>

#include <pdal/StageIterator.hpp>

namespace pdal
{
class Filter;

class PDAL_DLL FilterSequentialIterator : public StageSequentialIterator
{
public:
    FilterSequentialIterator(const Filter&, PointBuffer& buffer);
    virtual ~FilterSequentialIterator();

protected:
    // from StageSequentialIterator
    virtual point_count_t readBufferImpl(PointBuffer&) = 0;
    virtual boost::uint64_t skipImpl(boost::uint64_t pointNum) = 0;
    virtual bool atEndImpl() const = 0;

    StageSequentialIterator& getPrevIterator();
    const StageSequentialIterator& getPrevIterator() const;

private:
    const Filter& m_filter;
    StageSequentialIterator* m_prevIterator;
};


class PDAL_DLL FilterRandomIterator : public StageRandomIterator
{
public:
    FilterRandomIterator(const Filter&, PointBuffer& buffer);
    virtual ~FilterRandomIterator();

protected:
    // from StageRandomIterator
    virtual point_count_t readBufferImpl(PointBuffer&) = 0;
    virtual boost::uint64_t seekImpl(boost::uint64_t pointNum) = 0;

    StageRandomIterator& getPrevIterator();
    const StageRandomIterator& getPrevIterator() const;

private:
    const Filter& m_filter;
    StageRandomIterator* m_prevIterator;
};


} // namespace pdal

#endif
