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

#ifndef INCLUDED_DRIVERS_LAS_VARIABLELENGTHRECORD_HPP
#define INCLUDED_DRIVERS_LAS_VARIABLELENGTHRECORD_HPP

#include <libpc/libpc.hpp>

#include <libpc/MetadataRecord.hpp>

#include <iostream>

namespace libpc { namespace drivers { namespace las {
    

class LIBPC_DLL VariableLengthRecord : public MetadataRecord
{
public:
    // makes a local copy of the bytes buffer, which is a shared ptr among by all copes of the metadata record
    VariableLengthRecord(boost::uint8_t userId[16], 
                         boost::uint16_t recordId,
                         boost::uint16_t recordLenAfterHeader,
                         boost::uint8_t description[32],
                         const boost::uint8_t* bytes, std::size_t len);
    VariableLengthRecord(const VariableLengthRecord&);

    ~VariableLengthRecord();

    bool operator==(const VariableLengthRecord&) const;
    VariableLengthRecord& operator=(const VariableLengthRecord&);

    static const int s_headerLength = 54;

private:
    boost::uint16_t m_reserved;
    boost::uint8_t m_userId[16];
    boost::uint16_t m_recordId;
    boost::uint16_t m_recordLenAfterHeader;
    boost::uint8_t m_description[32];
};


} } } // namespace

#endif
