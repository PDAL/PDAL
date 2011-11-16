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

#ifndef INCLUDED_DRIVERS_LAS_ITERATOR_HPP
#define INCLUDED_DRIVERS_LAS_ITERATOR_HPP

#include <pdal/ReaderIterator.hpp>
#include <boost/scoped_ptr.hpp>

class LASzip;
class LASunzipper;

namespace pdal { namespace drivers { namespace las {

class Reader;
class ZipPoint;


class IteratorBase
{
public:
    IteratorBase(const Reader& reader);
    ~IteratorBase();

private:
    void initializeZip();

protected:
    const Reader& m_reader;
    std::istream* m_istream;

public:
#ifdef PDAL_HAVE_LASZIP
    boost::scoped_ptr<ZipPoint> m_zipPoint;
    boost::scoped_ptr<LASunzipper> m_unzipper;
    std::streampos m_zipReadStartPosition;

#endif

private:
    IteratorBase& operator=(const IteratorBase&); // not implemented
    IteratorBase(const IteratorBase&); // not implemented
};


class SequentialIterator : public IteratorBase, public pdal::ReaderSequentialIterator
{
public:
    SequentialIterator(const Reader& reader);
    ~SequentialIterator();

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;
};


class RandomIterator : public IteratorBase, public pdal::ReaderRandomIterator
{
public:
    RandomIterator(const Reader& reader);
    ~RandomIterator();

private:
    boost::uint64_t seekImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
};

} } } // namespaces

#endif
