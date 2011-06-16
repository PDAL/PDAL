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

#ifndef INCLUDED_METADATARECORD_HPP
#define INCLUDED_METADATARECORD_HPP

#include <pdal/pdal.hpp>

#include <boost/shared_array.hpp>

namespace pdal
{

class PDAL_DLL MetadataRecord
{
public:
    // makes a local copy of the buffer, which is a shared ptr among by all copes of the metadata record
    MetadataRecord(const boost::uint8_t* bytes, std::size_t len);

    MetadataRecord(const MetadataRecord&);

    virtual ~MetadataRecord();

    MetadataRecord& operator=(MetadataRecord const& rhs);

    bool operator==(MetadataRecord const& rhs) const;

    const boost::shared_array<boost::uint8_t> getBytes() const;
    std::size_t getLength() const;

private:
    boost::shared_array<boost::uint8_t> m_bytes;
    std::size_t m_length;
};


std::ostream& operator<<(std::ostream& ostr, const MetadataRecord& srs);


} // namespace pdal

#endif
