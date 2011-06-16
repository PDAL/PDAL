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

#include <pdal/MetadataRecord.hpp>

#include <sstream>

namespace pdal
{


MetadataRecord::MetadataRecord(const boost::uint8_t* bytes, std::size_t length)
    : m_bytes(new boost::uint8_t[length])
    , m_length(length)
{
    memcpy(m_bytes.get(), bytes, m_length);
    return;
}


MetadataRecord::MetadataRecord(const MetadataRecord& other)
    : m_bytes(other.m_bytes)
    , m_length(other.m_length)
{
    return;
}


MetadataRecord::~MetadataRecord()
{
    m_length = 0;
}


MetadataRecord& MetadataRecord::operator=(MetadataRecord const& rhs)
{
    if (&rhs != this)
    {
        m_length = rhs.m_length;
        m_bytes = rhs.m_bytes;
    }
    return *this;
}


bool MetadataRecord::operator==(MetadataRecord const& rhs) const
{
    if (m_length == rhs.m_length)
    {
        for (std::size_t i=0; i<m_length; i++)
        {
            if (m_bytes[i] != rhs.m_bytes[i]) return false;
        }
        return true;
    }

    return false;
}


const boost::shared_array<boost::uint8_t> MetadataRecord::getBytes() const
{
    return m_bytes;
}


std::size_t MetadataRecord::getLength() const
{
    return m_length;
}


std::ostream& operator<<(std::ostream& ostr, const MetadataRecord& metadata)
{
    ostr << "MetadataRecord: ";
    ostr << "  len=" << metadata.getLength();
    ostr << std::endl;
    return ostr;
}


} // namespace pdal
