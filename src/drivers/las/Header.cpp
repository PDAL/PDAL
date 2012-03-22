/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS header class 
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 ******************************************************************************
 * Copyright (c) 2008, Mateusz Loskot
 * Copyright (c) 2008, Phil Vachon
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
 *     * Neither the name of the Martin Isenburg or Iowa Department 
 *       of Natural Resources nor the names of its contributors may be 
 *       used to endorse or promote products derived from this software 
 *       without specific prior written permission.
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

#include <pdal/drivers/las/Header.hpp>

#include <boost/uuid/uuid_io.hpp>

#include <pdal/Stage.hpp>

namespace pdal { namespace drivers { namespace las {

    
// BUG: should be std::string
char const* const LasHeader::FileSignature = "LASF";
char const* const LasHeader::SystemIdentifier = "PDAL";
char const* const LasHeader::SoftwareIdentifier = "PDAL 0.1.0";

LasHeader::LasHeader()
    : m_scales(0.01,0.01,0.01)
{
    // BUG: set default here -- m_schema(LasSchema::ePointFormat3)
    initialize();

    return;
}



std::string LasHeader::GetFileSignature() const
{
    return std::string(m_signature, eFileSignatureSize);
}

void LasHeader::SetFileSignature(std::string const& v)
{
    if (0 != v.compare(0, eFileSignatureSize, FileSignature))
        throw std::invalid_argument("invalid file signature");

    std::strncpy(m_signature, v.c_str(), eFileSignatureSize);
}

boost::uint16_t LasHeader::GetFileSourceId() const
{
    return m_sourceId;
}

void LasHeader::SetFileSourceId(boost::uint16_t v)
{
    // TODO: Should we warn or throw about type overflow occuring when
    //       user passes 65535 + 1 = 0
    m_sourceId = v;
}

boost::uint16_t LasHeader::GetReserved() const
{
    return m_reserved;
}

void LasHeader::SetReserved(boost::uint16_t v)
{
    // TODO: Should we warn or throw about type overflow occuring when
    //       user passes 65535 + 1 = 0
    m_reserved = v;
}

boost::uuids::uuid LasHeader::GetProjectId() const
{
    return m_projectGuid;
}

void LasHeader::SetProjectId(boost::uuids::uuid const& v)
{
    m_projectGuid = v;
}

boost::uint8_t LasHeader::GetVersionMajor() const
{
    return m_versionMajor;
}

void LasHeader::SetVersionMajor(boost::uint8_t v)
{
    if (eVersionMajorMin > v || v > eVersionMajorMax)
        throw std::out_of_range("version major out of range");

    m_versionMajor = v;
}

boost::uint8_t LasHeader::GetVersionMinor() const
{
    return m_versionMinor;
}

void LasHeader::SetVersionMinor(boost::uint8_t v)
{
    if (v > eVersionMinorMax)
        throw std::out_of_range("version minor out of range");
    
    m_versionMinor = v;


}

std::string LasHeader::GetSystemId(bool pad /*= false*/) const
{
    // copy array of chars and trim zeros if smaller than 32 bytes
    std::string tmp(std::string(m_systemId, eSystemIdSize).c_str());

    // pad right side with spaces
    if (pad && tmp.size() < eSystemIdSize)
    {
        tmp.resize(eSystemIdSize, 0);
        assert(tmp.size() == eSystemIdSize);
    }

    assert(tmp.size() <= eSystemIdSize);
    return tmp;
}

void LasHeader::SetSystemId(std::string const& v)
{
    if (v.size() > eSystemIdSize)
        throw std::invalid_argument("system id too long");

    std::fill(m_systemId, m_systemId + eSystemIdSize, 0);
    std::strncpy(m_systemId, v.c_str(), eSystemIdSize);
}

std::string LasHeader::GetSoftwareId(bool pad /*= false*/) const
{
    std::string tmp(std::string(m_softwareId, eSoftwareIdSize).c_str());

    // pad right side with spaces
    if (pad && tmp.size() < eSoftwareIdSize)
    {
        tmp.resize(eSoftwareIdSize, 0);
        assert(tmp.size() == eSoftwareIdSize);
    }

    assert(tmp.size() <= eSoftwareIdSize);
    return tmp;
}

void LasHeader::SetSoftwareId(std::string const& v)
{
    if (v.size() > eSoftwareIdSize)
        throw std::invalid_argument("generating software id too long");
    
//    m_softwareId = v;
    std::fill(m_softwareId, m_softwareId + eSoftwareIdSize, 0);
    std::strncpy(m_softwareId, v.c_str(), eSoftwareIdSize);
}

boost::uint16_t LasHeader::GetCreationDOY() const
{
    return m_createDOY;
}

void LasHeader::SetCreationDOY(boost::uint16_t v)
{
    if (v > 366)
        throw std::out_of_range("day of year out of range");

    m_createDOY = v;
}

boost::uint16_t LasHeader::GetCreationYear() const
{
    return m_createYear;
}

void LasHeader::SetCreationYear(boost::uint16_t v)
{
    // mloskot: I've taken these values arbitrarily
    if (v > 9999)
        throw std::out_of_range("year out of range");

    m_createYear = v;
}

boost::uint16_t LasHeader::GetHeaderSize() const
{
    return m_headerSize;
}

void LasHeader::SetHeaderSize(boost::uint16_t v)
{

    m_headerSize = v;
}

boost::uint32_t LasHeader::GetDataOffset() const
{
    return m_dataOffset;
}

void LasHeader::SetDataOffset(boost::uint32_t v)
{
    // boost::uint32_t const dataSignatureSize = 2;
    // boost::uint16_t const hsize = GetHeaderSize();
    // 
    // if ( (m_versionMinor == 0 && v < hsize + dataSignatureSize) ||
    //      (m_versionMinor == 1 && v < hsize) ||
    //      (m_versionMinor == 2 && v < hsize) )
    // {
    //     throw std::out_of_range("data offset out of range");
    // }
    
    m_dataOffset = v;
    
}

pdal::drivers::las::PointFormat LasHeader::getPointFormat() const
{
    return m_pointFormat;
}

void LasHeader::setPointFormat(pdal::drivers::las::PointFormat v)
{
    m_pointFormat = v;
}

boost::uint16_t LasHeader::GetDataRecordLength() const
{
    // No matter what the schema says, this must be a short in size.
    return pdal::drivers::las::Support::getPointDataSize(m_pointFormat);
}

boost::uint32_t LasHeader::GetPointRecordsCount() const
{
    return m_pointRecordsCount;
}

void LasHeader::SetPointRecordsCount(boost::uint32_t v)
{
    m_pointRecordsCount = v;
}

LasHeader::RecordsByReturnArray const& LasHeader::GetPointRecordsByReturnCount() const
{
    return m_pointRecordsByReturn;
}

void LasHeader::SetPointRecordsByReturnCount(std::size_t index, boost::uint32_t v)
{
    assert(m_pointRecordsByReturn.size() == LasHeader::ePointsByReturnSize);

    boost::uint32_t& t = m_pointRecordsByReturn.at(index);
    t = v;
}


double LasHeader::GetScaleX() const
{
    return m_scales[0];
}

double LasHeader::GetScaleY() const
{
    return m_scales[1];
}

double LasHeader::GetScaleZ() const
{
    return m_scales[2];
}

void LasHeader::SetScale(double x, double y, double z)
{

    double const minscale = 0.01;
    m_scales[0] = Utils::compare_distance(0.0, x) ? minscale : x;
    m_scales[1] = Utils::compare_distance(0.0, y) ? minscale : y;
    m_scales[2] = Utils::compare_distance(0.0, z) ? minscale : z;
}

double LasHeader::GetOffsetX() const
{
    return m_offsets[0];
}

double LasHeader::GetOffsetY() const
{
    return m_offsets[1];
}

double LasHeader::GetOffsetZ() const
{
    return m_offsets[2];
}

void LasHeader::SetOffset(double x, double y, double z)
{
    m_offsets = PointOffsets(x, y, z);
}

double LasHeader::GetMaxX() const
{
    return m_bounds.getMaximum(0);
}

double LasHeader::GetMinX() const
{
    return m_bounds.getMinimum(0);
}

double LasHeader::GetMaxY() const
{
    return m_bounds.getMaximum(1);
}

double LasHeader::GetMinY() const
{
    return m_bounds.getMinimum(1);
}

double LasHeader::GetMaxZ() const
{
    return m_bounds.getMaximum(2);
}

double LasHeader::GetMinZ() const
{
    return m_bounds.getMinimum(2);
}

boost::uint32_t LasHeader::GetHeaderPadding() const
{
    return m_headerPadding;
}

void LasHeader::SetHeaderPadding( boost::uint32_t v) 
{
    m_headerPadding = v;
}

void LasHeader::initialize()
{
    // Initialize public header block with default
    // values according to LAS 1.2
    m_pointFormat = pdal::drivers::las::PointFormat0;

    m_versionMajor = 1;
    m_versionMinor = 2;
    
    m_createDOY = m_createYear = 0;
    std::time_t now;
    std::time(&now);
    std::tm* ptm = std::gmtime(&now);
    if (0 != ptm)
    {
        m_createDOY = static_cast<boost::uint16_t>(ptm->tm_yday);
        m_createYear = static_cast<boost::uint16_t>(ptm->tm_year + 1900);
    }

    m_headerSize = eHeaderSize;

    m_sourceId = m_reserved = boost::uint16_t();
    memset(m_projectGuid.data, 0, 16);

    m_dataOffset = eHeaderSize; // excluding 2 bytes of Point Data Start Signature
    m_pointRecordsCount = 0;

    std::memset(m_signature, 0, eFileSignatureSize);
    std::strncpy(m_signature, FileSignature, eFileSignatureSize);
//    m_signature = Header::FileSignature;

    std::memset(m_systemId, 0, eSystemIdSize);
    std::strncpy(m_systemId, SystemIdentifier, eSystemIdSize);
//    m_systemId = Header::SystemIdentifier;

    std::memset(m_softwareId, 0, eSoftwareIdSize);
    std::strncpy(m_softwareId, SoftwareIdentifier, eSoftwareIdSize);
//    m_softwareId = Header::SoftwareIdentifier;

    m_pointRecordsByReturn.resize(ePointsByReturnSize);

    // Zero scale value is useless, so we need to use a small value.
    SetScale(0.01, 0.01, 0.01);

    m_isCompressed = false;
    m_headerPadding = 0;
}


VLRList& LasHeader::getVLRs()
{
    return m_vlrList;
}


const VLRList& LasHeader::getVLRs() const
{
    return m_vlrList;
}

std::size_t LasHeader::getVLRBlockSize() const
{
    std::size_t vlr_total_size = 0;

    VLRList const& vlrs = m_vlrList;
    
    for (boost::uint32_t i = 0; i < m_vlrList.count(); ++i)
    {
        VariableLengthRecord const & vlr = vlrs.get(i);
        vlr_total_size += static_cast<boost::uint32_t>(vlr.getTotalSize());
    }

    return vlr_total_size;
}



void LasHeader::setSpatialReference(const SpatialReference& srs)
{
    m_spatialReference = srs;
    return;
}


const SpatialReference& LasHeader::getSpatialReference() const
{
    return m_spatialReference;
}


void LasHeader::SetCompressed(bool b)
{
    m_isCompressed = b;
}

bool LasHeader::Compressed() const
{
    return m_isCompressed;
}

boost::property_tree::ptree LasHeader::GetPTree( ) const
{
    using boost::property_tree::ptree;
    ptree pt;
    
    pt.put("filesignature", GetFileSignature());
    pt.put("projectdid", GetProjectId());
    pt.put("systemid", GetSystemId());
    pt.put("softwareid", GetSoftwareId());
    
    
    std::ostringstream version;
    version << static_cast<int>(GetVersionMajor());
    version <<".";
    version << static_cast<int>(GetVersionMinor());
    pt.put("version", version.str());
    
    pt.put("filesourceid", GetFileSourceId());
    pt.put("reserved", GetReserved());

    //////ptree srs = GetSRS().GetPTree();
    //////pt.add_child("srs", srs);
    
    std::ostringstream date;
    date << GetCreationDOY() << "/" << GetCreationYear();
    pt.put("date", date.str());
    
    pt.put("size", GetHeaderSize());
    pt.put("dataoffset", GetDataOffset());

    pt.put("count", GetPointRecordsCount());
    pt.put("pointformat", getPointFormat());
    pt.put("datarecordlength", GetDataRecordLength());
    pt.put("compressed", Compressed());

    ptree return_count;
    LasHeader::RecordsByReturnArray returns = GetPointRecordsByReturnCount();
    for (boost::uint32_t i=0; i< 5; i++){
        ptree r;
        r.put("id", i);
        r.put("count", returns[i]);
        return_count.add_child("return", r);
    }
    pt.add_child("returns", return_count);
    
    pt.put("scale.x", GetScaleX());
    pt.put("scale.y", GetScaleY());
    pt.put("scale.z", GetScaleZ());
    
    pt.put("offset.x", GetOffsetX());
    pt.put("offset.y", GetOffsetY());
    pt.put("offset.z", GetOffsetZ());
    
    pt.put("minimum.x", GetMinX());
    pt.put("minimum.y", GetMinY());
    pt.put("minimum.z", GetMinZ());
    
    pt.put("maximum.x", GetMaxX());
    pt.put("maximum.y", GetMaxY());
    pt.put("maximum.z", GetMaxZ());

    
    ////for (boost::uint32_t i=0; i< GetRecordsCount(); i++) {
    ////    pt.add_child("vlrs.vlr", GetVLR(i).GetPTree());
    ////}    

    //////Schema const& schema = getSchema(); 
    //////ptree t = schema.GetPTree(); 
    //////pt.add_child("schema",  t);
    
    return pt;
}

std::ostream& operator<<(std::ostream& ostr, const LasHeader& header)
{
    ostr << "  LasHeader" << std::endl;
    ostr << "    Header size: " << header.GetHeaderSize() << std::endl;
    ostr << "    Point records count: " << header.GetPointRecordsCount() << std::endl;
    ostr << "    VLR count: " << header.getVLRs().count() << std::endl;

    return ostr;
}



} } } // namespaces
