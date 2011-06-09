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

#include <pdal/pdal.hpp>

#include <string>
#include <vector>

#include <pdal/SpatialReference.hpp>


namespace pdal { namespace drivers { namespace las {
    

class LIBPC_DLL VariableLengthRecord
{
public:
    // makes a local copy of the bytes buffer, which is a shared ptr among by all copes of the metadata record
    VariableLengthRecord(boost::uint16_t reserved,
                         std::string userId,
                         boost::uint16_t recordId,
                         std::string description,
                         const boost::uint8_t* bytes,
                         std::size_t len);
    VariableLengthRecord(const VariableLengthRecord&);
    ~VariableLengthRecord();

    boost::uint16_t getReserved() const { return m_reserved; }
    std::string getUserId() const { return m_userId; }
    boost::uint16_t getRecordId() const { return m_recordId; }
    std::string getDescription() const { return m_description; }

    bool isGeoVLR() const;
    enum GeoVLRType
    {
        eGeoTIFF = 1,
        eOGRWKT = 2
    };
    static void clearVLRs(GeoVLRType eType, std::vector<VariableLengthRecord>& vlrs);

    bool isMatch(const std::string& userId) const;
    bool isMatch(const std::string& userId, boost::uint16_t recordId) const;

    bool operator==(const VariableLengthRecord&) const;
    VariableLengthRecord& operator=(const VariableLengthRecord&);

    const boost::uint8_t* getBytes() const;
    std::size_t getLength() const;

    static const int s_headerLength = 54;

    static void setSRSFromVLRs(const std::vector<VariableLengthRecord>& vlrs, SpatialReference& srs);
    static void setVLRsFromSRS(const SpatialReference& srs, std::vector<VariableLengthRecord>& vlrs, SpatialReference::WKTModeFlag modeFlag);

    static std::string bytes2string(const boost::uint8_t* bytes, boost::uint32_t len);

    // bytes array is return, user responsible for deleting
    // len is the size of the array he wants, it will be padded with zeros if str.length() < len
    static boost::uint8_t* string2bytes(boost::uint32_t len, const std::string& str);

private:
    boost::uint16_t m_reserved;
    std::string m_userId; // always stored as 16 bytes (padded with 0's)
    boost::uint16_t m_recordId;
    std::string m_description; // always stored as 16 bytes (padded with 0's)
    
    boost::uint8_t* m_bytes;
    std::size_t m_length;
};


class LIBPC_DLL VLRList
{
public:
    void add(const VariableLengthRecord& v);
    
    const VariableLengthRecord& get(boost::uint32_t index) const;
    VariableLengthRecord& get(boost::uint32_t index);
    
    const std::vector<VariableLengthRecord>& getAll() const;
    std::vector<VariableLengthRecord>& getAll();

    void remove(boost::uint32_t index);
    void remove(std::string const& userId, boost::uint16_t recordId);

    boost::uint32_t count() const;

    void constructSRS(SpatialReference&) const;
    void addVLRsFromSRS(const SpatialReference& srs, SpatialReference::WKTModeFlag modeFlag);

private:
   std::vector<VariableLengthRecord> m_list;
};


} } } // namespace

#endif
