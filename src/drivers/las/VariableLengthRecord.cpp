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

#include <libpc/drivers/las/VariableLengthRecord.hpp>

#include <libpc/exceptions.hpp>


namespace libpc { namespace drivers { namespace las {

VariableLengthRecord::VariableLengthRecord(boost::uint16_t reserved,
                                           boost::uint8_t* userId, 
                                           boost::uint16_t recordId,
                                           boost::uint8_t* description,
                                           const boost::uint8_t* bytes, std::size_t len)
    : MetadataRecord(bytes, len)
    , m_reserved(reserved)
    , m_recordId(recordId)
{
    m_userId = new boost::uint8_t[16];
    memcpy(m_userId, userId, 16);
    m_description = new boost::uint8_t[32];
    memcpy(m_description, description, 32);
    return;
}


VariableLengthRecord::VariableLengthRecord(const VariableLengthRecord& vlr)
    : MetadataRecord(vlr)
    , m_reserved(vlr.m_reserved)
    , m_recordId(vlr.m_recordId)
{
    m_userId = new boost::uint8_t[16];
    memcpy(m_userId, vlr.m_userId, 16);
    m_description = new boost::uint8_t[32];
    memcpy(m_description, vlr.m_description, 32);
    return;
}


VariableLengthRecord::~VariableLengthRecord()
{
    delete[] m_userId;
    delete[] m_description;
    return;
}


VariableLengthRecord& VariableLengthRecord::operator=(const VariableLengthRecord& vlr)
{
    (MetadataRecord&)vlr = *(MetadataRecord*)this;

    m_reserved = vlr.m_reserved;
    m_recordId = vlr.m_recordId;

    memcpy(m_userId, vlr.m_userId, 16);
    memcpy(m_description, vlr.m_description, 32);

    return *this;
}


bool VariableLengthRecord::operator==(const VariableLengthRecord& vlr) const
{
    if (m_reserved != vlr.m_reserved) return false;
    if (m_recordId != vlr.m_recordId) return false;

    for (int i=0; i<16; i++)
        if (m_userId[i] != vlr.m_userId[i]) return false;
    for (int i=0; i<32; i++)
        if (m_description[i] != vlr.m_description[i]) return false;

    if (!((MetadataRecord&)vlr == *(MetadataRecord*)this)) return false;

    return true;
}


} } } // namespaces
