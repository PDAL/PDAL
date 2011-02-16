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

#include "libpc/LasHeader.hpp"

#include <boost/concept_check.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace libpc
{
    
// BUG: should be std::string
char const* const LasHeader::FileSignature = "LASF";
char const* const LasHeader::SystemIdentifier = "libLAS";
char const* const LasHeader::SoftwareIdentifier = "libLAS 1.6.0";

LasHeader::LasHeader()
    : Header()
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

uint16_t LasHeader::GetFileSourceId() const
{
    return m_sourceId;
}

void LasHeader::SetFileSourceId(uint16_t v)
{
    // TODO: Should we warn or throw about type overflow occuring when
    //       user passes 65535 + 1 = 0
    m_sourceId = v;
}

uint16_t LasHeader::GetReserved() const
{
    return m_reserved;
}

void LasHeader::SetReserved(uint16_t v)
{
    // TODO: Should we warn or throw about type overflow occuring when
    //       user passes 65535 + 1 = 0
    m_reserved = v;
}

LasHeader::uuid LasHeader::GetProjectId() const
{
    return m_projectGuid;
}

void LasHeader::SetProjectId(uuid const& v)
{
    m_projectGuid = v;
}

uint8_t LasHeader::GetVersionMajor() const
{
    return m_versionMajor;
}

void LasHeader::SetVersionMajor(uint8_t v)
{
    if (eVersionMajorMin > v || v > eVersionMajorMax)
        throw std::out_of_range("version major out of range");

    m_versionMajor = v;
}

uint8_t LasHeader::GetVersionMinor() const
{
    return m_versionMinor;
}

void LasHeader::SetVersionMinor(uint8_t v)
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

uint16_t LasHeader::GetCreationDOY() const
{
    return m_createDOY;
}

void LasHeader::SetCreationDOY(uint16_t v)
{
    if (v > 366)
        throw std::out_of_range("day of year out of range");

    m_createDOY = v;
}

uint16_t LasHeader::GetCreationYear() const
{
    return m_createYear;
}

void LasHeader::SetCreationYear(uint16_t v)
{
    // mloskot: I've taken these values arbitrarily
    if (v > 9999)
        throw std::out_of_range("year out of range");

    m_createYear = v;
}

uint16_t LasHeader::GetHeaderSize() const
{
    return m_headerSize;
}

void LasHeader::SetHeaderSize(uint16_t v)
{

    m_headerSize = v;
}

uint32_t LasHeader::GetDataOffset() const
{
    return m_dataOffset;
}

void LasHeader::SetDataOffset(uint32_t v)
{
    // uint32_t const dataSignatureSize = 2;
    // uint16_t const hsize = GetHeaderSize();
    // 
    // if ( (m_versionMinor == 0 && v < hsize + dataSignatureSize) ||
    //      (m_versionMinor == 1 && v < hsize) ||
    //      (m_versionMinor == 2 && v < hsize) )
    // {
    //     throw std::out_of_range("data offset out of range");
    // }
    
    m_dataOffset = v;
    
}

uint32_t LasHeader::GetRecordsCount() const
{
    return m_recordsCount;
}

void LasHeader::SetRecordsCount(uint32_t v)
{
    m_recordsCount = v;
}

LasHeader::PointFormatId LasHeader::getDataFormatId() const
{
    return m_data_format_id;
}

void LasHeader::setDataFormatId(PointFormatId v)
{
    m_data_format_id = v;
}

uint16_t LasHeader::GetDataRecordLength() const
{
    // No matter what the schema says, this must be a short in size.
    return static_cast<boost::uint16_t>(getSchema().getByteSize());
}

uint32_t LasHeader::GetPointRecordsCount() const
{
    return m_pointRecordsCount;
}

void LasHeader::SetPointRecordsCount(uint32_t v)
{
    m_pointRecordsCount = v;
    this->setNumPoints(v);
}

LasHeader::RecordsByReturnArray const& LasHeader::GetPointRecordsByReturnCount() const
{
    return m_pointRecordsByReturn;
}

void LasHeader::SetPointRecordsByReturnCount(std::size_t index, uint32_t v)
{
    assert(m_pointRecordsByReturn.size() == LasHeader::ePointsByReturnSize);

    uint32_t& t = m_pointRecordsByReturn.at(index);
    t = v;
}


double LasHeader::GetScaleX() const
{
    return m_scales.get0();
}

double LasHeader::GetScaleY() const
{
    return m_scales.get1();
}

double LasHeader::GetScaleZ() const
{
    return m_scales.get2();
}

void LasHeader::SetScale(double x, double y, double z)
{

    double const minscale = 0.01;
    m_scales.set( (Utils::compare_distance(0.0, x)) ? minscale : x,
                  (Utils::compare_distance(0.0, y)) ? minscale : y,
                  (Utils::compare_distance(0.0, z)) ? minscale : z );
}

double LasHeader::GetOffsetX() const
{
    return m_offsets.get0();
}

double LasHeader::GetOffsetY() const
{
    return m_offsets.get1();
}

double LasHeader::GetOffsetZ() const
{
    return m_offsets.get2();
}

void LasHeader::SetOffset(double x, double y, double z)
{
    m_offsets = PointOffsets(x, y, z);
}

double LasHeader::GetMaxX() const
{
    return getBounds().maximum(0);
}

double LasHeader::GetMinX() const
{
    return getBounds().minimum(0);
}

double LasHeader::GetMaxY() const
{
    return getBounds().maximum(1);
}

double LasHeader::GetMinY() const
{
    return getBounds().minimum(1);
}

double LasHeader::GetMaxZ() const
{
    return getBounds().maximum(2);
}

double LasHeader::GetMinZ() const
{
    return getBounds().minimum(2);
}

void LasHeader::SetMax(double x, double y, double z)
{
    // m_extent = Bounds(m_extent.min(0), m_extent.min(1), m_extent.max(0), m_extent.max(1), m_extent.min(2), m_extent.max(2));
    // Bounds(minx, miny, minz, maxx, maxy, maxz)
    setBounds( Bounds<double>(getBounds().minimum(0), getBounds().minimum(1), getBounds().minimum(2), x, y, z) );
}

void LasHeader::SetMin(double x, double y, double z)
{
    setBounds( Bounds<double>(x, y, z, getBounds().maximum(0), getBounds().maximum(1), getBounds().maximum(2)) );
}

//void Header::AddVLR(VariableRecord const& v) 
//{
//    m_vlrs.push_back(v);
//    m_recordsCount += 1;
//}
//
//VariableRecord const& Header::GetVLR(uint32_t index) const 
//{
//    return m_vlrs[index];
//}
//
//const std::vector<VariableRecord>& Header::GetVLRs() const
//{
//    return m_vlrs;
//}
//
//void Header::DeleteVLR(uint32_t index) 
//{    
//    if (index >= m_vlrs.size())
//        throw std::out_of_range("index is out of range");
//
//    std::vector<VariableRecord>::iterator i = m_vlrs.begin() + index;
//
//    m_vlrs.erase(i);
//    m_recordsCount = static_cast<uint32_t>(m_vlrs.size());
//
//}


void LasHeader::initialize()
{
    // Initialize public header block with default
    // values according to LAS 1.2
    m_data_format_id = ePointFormat0;

    m_versionMajor = 1;
    m_versionMinor = 2;
    // m_dataFormatId = ePointFormat0;
    // m_dataRecordLen = ePointSize0;
    
    m_createDOY = m_createYear = 0;
    std::time_t now;
    std::time(&now);
    std::tm* ptm = std::gmtime(&now);
    if (0 != ptm)
    {
        m_createDOY = static_cast<uint16_t>(ptm->tm_yday);
        m_createYear = static_cast<uint16_t>(ptm->tm_year + 1900);
    }

    m_headerSize = eHeaderSize;

    m_sourceId = m_reserved = uint16_t();
    memset(m_projectGuid.data, 0, 16);

    m_dataOffset = eHeaderSize; // excluding 2 bytes of Point Data Start Signature
    m_recordsCount = 0;
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
}

//bool SameVLRs(std::string const& name, boost::uint16_t id, liblas::VariableRecord const& record)
//{
//    if (record.GetUserId(false) == name) {
//        if (record.GetRecordId() == id) {
//            return true;
//        }
//    }
//    return false;
//}
//
//
//void Header::DeleteVLRs(std::string const& name, boost::uint16_t id)
//{
//
//    m_vlrs.erase( std::remove_if( m_vlrs.begin(), 
//                                  m_vlrs.end(),
//                                  boost::bind( &SameVLRs, name, id, _1 ) ),
//                  m_vlrs.end());
//
//    m_recordsCount = static_cast<uint32_t>(m_vlrs.size());        
//
//}



//void LasHeader::SetGeoreference() 
//{    
//    std::vector<VariableRecord> vlrs = m_srs.GetVLRs();
//
//    // Wipe the GeoTIFF-related VLR records off of the Header
//    DeleteVLRs("LASF_Projection", 34735);
//    DeleteVLRs("LASF_Projection", 34736);
//    DeleteVLRs("LASF_Projection", 34737);
//
//    std::vector<VariableRecord>::const_iterator i;
//
//    for (i = vlrs.begin(); i != vlrs.end(); ++i) 
//    {
//        AddVLR(*i);
//    }
//}
//
//SpatialReference Header::GetSRS() const
//{
//    return m_srs;
//}
//
//void Header::SetSRS(SpatialReference& srs)
//{
//    m_srs = srs;
//}



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
    pt.put("dataformatid", getDataFormatId());
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

//void Header::to_rst(std::ostream& os) const
//{
//
//    using liblas::property_tree::ptree;
//    ptree tree = GetPTree();
//
//    os << "---------------------------------------------------------" << std::endl;
//    os << "  Header Summary" << std::endl;
//    os << "---------------------------------------------------------" << std::endl;
//    os << std::endl;
//
//    os << "  Version:                     " << tree.get<std::string>("version") << std::endl;
//    os << "  Source ID:                   " << tree.get<boost::uint32_t>("filesourceid") << std::endl;
//    os << "  Reserved:                    " << tree.get<std::string>("reserved") << std::endl;
//    os << "  Project ID/GUID:             '" << tree.get<std::string>("projectdid") << "'" << std::endl;
//    os << "  System ID:                   '" << tree.get<std::string>("systemid") << "'" << std::endl;
//    os << "  Generating Software:         '" << tree.get<std::string>("softwareid") << "'" << std::endl;
//    os << "  File Creation Day/Year:      " << tree.get<std::string>("date") << std::endl;
//    os << "  Header Byte Size             " << tree.get<boost::uint32_t>("size") << std::endl;
//    os << "  Data Offset:                 " << tree.get<boost::uint32_t>("dataoffset") << std::endl;
//
//    os << "  Number Var. Length Records:  ";
//    try {
//      os << tree.get_child("vlrs").size();
//    }
//    catch (liblas::property_tree::ptree_bad_path const& e) {
//      ::boost::ignore_unused_variable_warning(e);
//      os << "None";
//    }
//    os << std::endl;
//
//    os << "  Point Data Format:           " << tree.get<boost::uint32_t>("dataformatid") << std::endl;
//    os << "  Number of Point Records:     " << tree.get<boost::uint32_t>("count") << std::endl;
//    os << "  Compressed:                  " << (tree.get<bool>("compressed")?"True":"False") << std::endl;
//    os << "  Number of Points by Return:  " ;
//    BOOST_FOREACH(ptree::value_type &v,
//          tree.get_child("returns"))
//    {
//          os << v.second.get<boost::uint32_t>("count")<< " ";
//
//    }      
//    os << std::endl;
//
//    os.setf(std::ios_base::fixed, std::ios_base::floatfield);
//    double x_scale = tree.get<double>("scale.x");
//    double y_scale = tree.get<double>("scale.y");
//    double z_scale = tree.get<double>("scale.z");
//
//    boost::uint32_t x_precision = 6;
//    boost::uint32_t y_precision = 6;
//    boost::uint32_t z_precision = 6;
//    
//    x_precision = GetStreamPrecision(x_scale);
//    y_precision = GetStreamPrecision(y_scale);
//    z_precision = GetStreamPrecision(z_scale);
//
//    os << "  Scale Factor X Y Z:          ";
//    os.precision(x_precision);
//    os << tree.get<double>("scale.x") << " "; 
//    os.precision(y_precision);
//    os << tree.get<double>("scale.y") << " "; 
//    os.precision(z_precision);
//    os << tree.get<double>("scale.z") << std::endl;
//
//    os << "  Offset X Y Z:                ";
//    os.precision(x_precision);
//    os << tree.get<double>("offset.x") << " ";
//    os.precision(y_precision);
//    os << tree.get<double>("offset.y") << " ";
//    os.precision(z_precision);
//    os << tree.get<double>("offset.z") << std::endl;
//
//    os << "  Min X Y Z:                   ";
//    os.precision(x_precision);
//    os << tree.get<double>("minimum.x") << " "; 
//    os.precision(y_precision);
//    os << tree.get<double>("minimum.y") << " ";
//    os.precision(z_precision);
//    os << tree.get<double>("minimum.z") << std::endl;
//
//    os << "  Max X Y Z:                   ";
//    os.precision(x_precision);
//    os << tree.get<double>("maximum.x") << " ";
//    os.precision(y_precision);
//    os << tree.get<double>("maximum.y") << " ";
//    os.precision(z_precision);
//    os << tree.get<double>("maximum.z") << std::endl;         
//
//    
//    os << "  Spatial Reference:           ";
//#ifdef HAVE_GDAL
//    if (tree.get<std::string>("srs.prettywkt").size() > 0)
//#else
//    if (tree.get<std::string>("srs.gtiff").size() > 0)
//#endif
//    {
//        os << std::endl << tree.get<std::string>("srs.prettywkt") << std::endl;
//        os << std::endl << tree.get<std::string>("srs.gtiff") << std::endl; 
//    } else 
//    {
//        os << "None" << std::endl;
//    }
//
//
//}


void LasHeader::read(std::istream& ifs) 
{
    // Helper variables
    uint8_t n1 = 0;
    uint16_t n2 = 0;
    uint32_t n4 = 0;
    double x1 = 0;
    double y1 = 0;
    double z1 = 0;
    double x2 = 0;
    double y2 = 0;
    double z2 = 0;

    // BUG: these two were std::string, but the read_n() failed until I made them char[]
    char buff[32];
    char fsig[4];

    ifs.seekg(0);

    // 1. File Signature
    Utils::read_n(fsig, ifs, 4);
    SetFileSignature(fsig);

    // 2. File Source ID
    Utils::read_n(n2, ifs, sizeof(n2));
    SetFileSourceId(n2);

    // 3. Reserved
    Utils::read_n(n2, ifs, sizeof(n2));
    SetReserved(n2);

    // 4-7. Project ID
    uint8_t d16[16];
    Utils::read_n(d16, ifs, sizeof(d16));
    uuid u;
    memcpy(&u, d16, 16);
    SetProjectId(u);

    // 8. Version major
    Utils::read_n(n1, ifs, sizeof(n1));
    SetVersionMajor(n1);

    // 9. Version minor
    Utils::read_n(n1, ifs, sizeof(n1));
    SetVersionMinor(n1);

    // 10. System ID
    Utils::read_n(buff, ifs, 32);
    SetSystemId(buff);

    // 11. Generating Software ID
    Utils::read_n(buff, ifs, 32);
    SetSoftwareId(buff);

    // 12. File Creation Day of Year
    Utils::read_n(n2, ifs, sizeof(n2));
    SetCreationDOY(n2);

    // 13. File Creation Year
    Utils::read_n(n2, ifs, sizeof(n2));
    SetCreationYear(n2);

    // 14. Header Size
    // NOTE: Size of the stanard header block must always be 227 bytes
    Utils::read_n(n2, ifs, sizeof(n2));
    SetHeaderSize(n2);

    // 15. Offset to data
    Utils::read_n(n4, ifs, sizeof(n4));
    
    if (n4 < GetHeaderSize())
    {
        std::ostringstream msg; 
        msg <<  "The offset to the start of point data, "
            << n4 << ", is smaller than the header size, "
            << GetHeaderSize() << ".  This is "
            "an invalid condition and incorrectly written "
            "file.  We cannot ignore this error because we "
            "do not know where to begin seeking to read the "
            "file.  Please report whomever's software who "
            "wrote this file to the proper authorities.  They "
            "will be dealt with swiftly and humanely.";
        throw std::runtime_error(msg.str());
    }
    SetDataOffset(n4);

    // 16. Number of variable length records
    Utils::read_n(n4, ifs, sizeof(n4));
    SetRecordsCount(n4);

    // 17. Point Data Format ID
    Utils::read_n(n1, ifs, sizeof(n1));

    // the high two bits are reserved for laszip compression type
    uint8_t compression_bit_7 = (n1 & 0x80) >> 7;
    uint8_t compression_bit_6 = (n1 & 0x40) >> 6;
    if (!compression_bit_7 && !compression_bit_6)
    {
        SetCompressed(false);
    }
    else if (compression_bit_7 && !compression_bit_6)
    {
        SetCompressed(true);
    }
    else if (compression_bit_7 && compression_bit_6)
    {
        throw std::domain_error("This file was compressed with an earlier, experimental version of laszip; please contact 'martin.isenburg@gmail.com' for assistance.");
    }
    else
    {
        assert(!compression_bit_7 && compression_bit_6);
        throw std::domain_error("invalid point compression format");
    }

    // strip the high bits, to determine point type
    n1 &= 0x3f;
    if (n1 <= 5)
    {
        setDataFormatId((PointFormatId)n1);
        update_required_dimensions((PointFormatId)n1, getSchema());
    } 
    else
    {
        throw std::domain_error("invalid point data format");
    }
    
    // 18. Point Data Record Length
    Utils::read_n(n2, ifs, sizeof(n2));
    // FIXME: We currently only use the DataFormatId, this needs to 
    // adjust the schema based on the difference between the DataRecordLength
    // and the base size of the pointformat.  If we have an XML schema in the 
    // form of a VLR in the file, we'll use that to apportion the liblas::Schema.
    // Otherwise, all bytes after the liblas::Schema::GetBaseByteSize will be 
    // a simple uninterpreted byte field. 
    // SetDataRecordLength(n2);

    // 19. Number of point records
    Utils::read_n(n4, ifs, sizeof(n4));
    SetPointRecordsCount(n4);

    // 20. Number of points by return
    // A few versions of the spec had this as 7, but 
    // https://lidarbb.cr.usgs.gov/index.php?showtopic=11388 says 
    // it is supposed to always be 5
    std::size_t const return_count_length = 5; 
    for (std::size_t i = 0; i < return_count_length; ++i)
    {
        uint32_t count = 0;
        Utils::read_n(count, ifs, sizeof(uint32_t));
        SetPointRecordsByReturnCount(i, count);
    }  

    // 21-23. Scale factors
    Utils::read_n(x1, ifs, sizeof(x1));
    Utils::read_n(y1, ifs, sizeof(y1));
    Utils::read_n(z1, ifs, sizeof(z1));
    SetScale(x1, y1, z1);

    // 24-26. Offsets
    Utils::read_n(x1, ifs, sizeof(x1));
    Utils::read_n(y1, ifs, sizeof(y1));
    Utils::read_n(z1, ifs, sizeof(z1));
    SetOffset(x1, y1, z1);

    // 27-28. Max/Min X
    Utils::read_n(x1, ifs, sizeof(x1));
    Utils::read_n(x2, ifs, sizeof(x2));

    // 29-30. Max/Min Y
    Utils::read_n(y1, ifs, sizeof(y1));
    Utils::read_n(y2, ifs, sizeof(y2));

    // 31-32. Max/Min Z
    Utils::read_n(z1, ifs, sizeof(z1));
    Utils::read_n(z2, ifs, sizeof(z2));

    SetMax(x1, y1, z1);
    SetMin(x2, y2, z2);
    
    

    // We're going to check the two bytes off the end of the header to 
    // see if they're pad bytes anyway.  Some softwares, notably older QTModeler, 
    // write 1.0-style pad bytes off the end of their header but state that the
    // offset is actually 2 bytes back.  We need to set the dataoffset 
    // appropriately in those cases anyway. 
    ifs.seekg(GetDataOffset());
    

    if (HasLAS10PadSignature(ifs)) {
        std::streamsize const current_pos = ifs.tellg();
        ifs.seekg(current_pos + 2);
        SetDataOffset(GetDataOffset() + 2);
    }
     
    // only go read VLRs if we have them.
    if (GetRecordsCount() > 0)
        ReadVLRs(ifs);



    // If we're eof, we need to reset the state
    if (ifs.eof())
        ifs.clear();

    // Seek to the data offset so we can start reading points
    ifs.seekg(GetDataOffset());

}

bool LasHeader::HasLAS10PadSignature(std::istream& ifs) 
{
    uint8_t const sgn1 = 0xCC;
    uint8_t const sgn2 = 0xDD;
    uint8_t pad1 = 0x0; 
    uint8_t pad2 = 0x0;

    std::streamsize const current_pos = ifs.tellg();
    
    // If our little test reads off the end of the file (in the case 
    // of a file with just a header and no points), we'll try to put the 
    // borken dishes back up in the cabinet
    try
    {
        Utils::read_n(pad1, ifs, sizeof(uint8_t));
        Utils::read_n(pad2, ifs, sizeof(uint8_t));
    }
    catch (std::out_of_range& e) 
    {
        boost::ignore_unused_variable_warning(e);
        ifs.seekg(current_pos, std::ios::beg);
        return false;
    }
    catch (std::runtime_error& e)
    {
        boost::ignore_unused_variable_warning(e);
        ifs.seekg(current_pos, std::ios::beg);
        return false;        
    }

    // BUG: endianness
    //LIBLAS_SWAP_BYTES(pad1);
    //LIBLAS_SWAP_BYTES(pad2);
    
    // Put the stream back where we found it
    ifs.seekg(current_pos, std::ios::beg);
    
    // Let's check both ways in case people were 
    // careless with their swapping.  This will do no good 
    // when we go to read point data though.
    bool found = false;
    if (sgn1 == pad2 && sgn2 == pad1) found = true;
    if (sgn1 == pad1 && sgn2 == pad2) found = true;
    
    return found;
}

void LasHeader::ReadVLRs(std::istream& )
{
#if 0
    VLRHeader vlrh = { 0 };

    if (ifs.eof()) {
        // if we hit the end of the file already, it's because 
        // we don't have any points.  We still want to read the VLRs 
        // in that case.
        ifs.clear();  
    }

    // seek to the start of the VLRs
    ifs.seekg(GetHeaderSize(), std::ios::beg);

    uint32_t count = GetRecordsCount();
    
    // We set the VLR records count to 0 because AddVLR 
    // will ++ it each time we add a VLR instance to the 
    // header.
    SetRecordsCount(0);
    for (uint32_t i = 0; i < count; ++i)
    {
        Utils::read_n(vlrh, ifs, sizeof(VLRHeader));

        uint16_t length = vlrh.recordLengthAfterHeader;

        std::vector<uint8_t> data(length);

        Utils::read_n(data.front(), ifs, length);
         
        VariableRecord vlr;
        vlr.SetReserved(vlrh.reserved);
        vlr.SetUserId(std::string(vlrh.userId));
        vlr.SetDescription(std::string(vlrh.description));
        vlr.SetRecordLength(vlrh.recordLengthAfterHeader);
        vlr.SetRecordId(vlrh.recordId);
        vlr.SetData(data);

        AddVLR(vlr);
    }

    liblas::SpatialReference srs(GetVLRs());    
    SetSRS(srs);
    
    // Go fetch the schema from the VLRs if we've got one.
    try {
        liblas::Schema schema(GetVLRs());
        SetSchema(schema);

    } catch (std::runtime_error const& e) 
    {
        // Create one from the PointFormat if we don't have
        // one in the VLRs.  Create a custom dimension on the schema 
        // That comprises the rest of the bytes after the end of the 
        // required dimensions.
        liblas::Schema schema(GetDataFormatId());
        
        // FIXME: handle custom bytes here.
        SetSchema(schema);
        boost::ignore_unused_variable_warning(e);
    }

#ifdef HAVE_LASZIP
    if (Compressed())
    {
         ZipPoint zpd(GetDataFormatId());
         bool ok = zpd.ValidateVLR(GetVLRs());
         if (!ok)
         {
            throw configuration_error("LASzip compression format does not match the LAS header format.");
         }
    }
#endif
#endif
}


void LasHeader::Validate(std::istream& ifs)
{
    // Check that the point count actually describes the number of points 
    // in the file.  If it doesn't, we're going to throw an error telling 
    // the user why.  It may also be a problem that the dataoffset is 
    // really what is wrong, but there's no real way to know that unless 
    // you go start mucking around in the bytes with hexdump or od
        
    // LAS 1.3 specification no longer mandates that the end of the file is the
    // end of the points. See http://trac.liblas.org/ticket/147 for more details
    // on this issue and why the seek can be trouble in the windows case.  
    // If you are having trouble properly seeking to the end of the stream on 
    // windows, use boost's iostreams or similar, which do not have an overflow 
    // problem.
    
    if (GetVersionMinor() < 3 && !Compressed() ) 
    {
        // Seek to the beginning 
        ifs.seekg(0, std::ios::beg);
        std::ios::pos_type beginning = ifs.tellg();
    
        // Seek to the end
        ifs.seekg(0, std::ios::end);
        std::ios::pos_type end = ifs.tellg();
        std::ios::off_type size = end - beginning;
        std::ios::off_type offset = static_cast<std::ios::off_type>(GetDataOffset());
        std::ios::off_type length = static_cast<std::ios::off_type>(GetDataRecordLength());
        std::ios::off_type point_bytes = end - offset;

        // Figure out how many points we have and whether or not we have 
        // extra slop in there.
        std::ios::off_type count = point_bytes / length;
        std::ios::off_type remainder = point_bytes % length;
        

        if ( GetPointRecordsCount() != static_cast<uint32_t>(count)) {
  
                std::ostringstream msg; 
                msg <<  "The number of points in the header that was set "
                        "by the software '" << GetSoftwareId() <<
                        "' does not match the actual number of points in the file "
                        "as determined by subtracting the data offset (" 
                        <<GetDataOffset() << ") from the file length (" 
                        << size <<  ") and dividing by the point record length (" 
                        << GetDataRecordLength() << ")."
                        " It also does not perfectly contain an exact number of"
                        " point data and we cannot infer a point count."
                        " Calculated number of points: " << count << 
                        " Header-specified number of points: " 
                        << GetPointRecordsCount() <<
                        " Point data remainder: " << remainder;
                throw std::runtime_error(msg.str());                

        }
    }
}


void LasHeader::add_record0_dimensions(Schema& schema)
{
    std::ostringstream text;

    Dimension x("X", Dimension::uint32_t);
    text << "x coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    x.setDescription(text.str());
    schema.addDimension(x);
    text.str("");

    Dimension y("Y", Dimension::uint32_t);
    text << "y coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    y.setDescription(text.str());
    schema.addDimension(y);
    text.str("");

    Dimension z("Z", Dimension::uint32_t);
    text << "z coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    z.setDescription(text.str());
    schema.addDimension(z);
    text.str("");

    Dimension intensity("Intensity", Dimension::int16_t);
    text << "The intensity value is the integer representation of the pulse "
         "return magnitude. This value is optional and system specific. "
         "However, it should always be included if available.";
    intensity.setDescription(text.str());
    schema.addDimension(intensity);
    text.str("");

    Dimension return_no("Return Number", Dimension::uint8_t); // 3 bits only
    text << "Return Number: The Return Number is the pulse return number for "
         "a given output pulse. A given output laser pulse can have many "
         "returns, and they must be marked in sequence of return. The first "
         "return will have a Return Number of one, the second a Return "
         "Number of two, and so on up to five returns.";
    return_no.setDescription(text.str());
    schema.addDimension(return_no);
    text.str("");

    Dimension no_returns("Number of Returns", Dimension::uint8_t); // 3 bits only
    text << "Number of Returns (for this emitted pulse): The Number of Returns "
         "is the total number of returns for a given pulse. For example, "
         "a laser data point may be return two (Return Number) within a "
         "total number of five returns.";
    no_returns.setDescription(text.str());
    schema.addDimension(no_returns);
    text.str("");

    Dimension scan_dir("Scan Direction", Dimension::uint8_t); // 1 bit only
    text << "The Scan Direction Flag denotes the direction at which the "
         "scanner mirror was traveling at the time of the output pulse. "
         "A bit value of 1 is a positive scan direction, and a bit value "
         "of 0 is a negative scan direction (where positive scan direction "
         "is a scan moving from the left side of the in-track direction to "
         "the right side and negative the opposite). ";
    scan_dir.setDescription(text.str());
    schema.addDimension(scan_dir);
    text.str("");

    Dimension edge("Flightline Edge", Dimension::uint8_t); // 1 bit only
    text << "The Edge of Flight Line data bit has a value of 1 only when "
         "the point is at the end of a scan. It is the last point on "
         "a given scan line before it changes direction.";
    edge.setDescription(text.str());
    schema.addDimension(edge);
    text.str("");

    Dimension classification("Classification", Dimension::uint8_t);
    text << "Classification in LAS 1.0 was essentially user defined and optional. "
         "LAS 1.1 defines a standard set of ASPRS classifications. In addition, "
         "the field is now mandatory. If a point has never been classified, this "
         "byte must be set to zero. There are no user defined classes since "
         "both point format 0 and point format 1 supply 8 bits per point for "
         "user defined operations. Note that the format for classification is a "
         "bit encoded field with the lower five bits used for class and the "
         "three high bits used for flags.";
    classification.setDescription(text.str());
    schema.addDimension(classification);
    text.str("");

    Dimension scan_angle("Scan Angle Rank", Dimension::int8_t);
    text << "The Scan Angle Rank is a signed one-byte number with a "
         "valid range from -90 to +90. The Scan Angle Rank is the "
         "angle (rounded to the nearest integer in the absolute "
         "value sense) at which the laser point was output from the "
         "laser system including the roll of the aircraft. The scan "
         "angle is within 1 degree of accuracy from +90 to –90 degrees. "
         "The scan angle is an angle based on 0 degrees being nadir, "
         "and –90 degrees to the left side of the aircraft in the "
         "direction of flight.";
    scan_angle.setDescription(text.str());
    schema.addDimension(scan_angle);
    text.str("");

    Dimension user_data("User Data", Dimension::uint8_t);
    text << "This field may be used at the user’s discretion";
    user_data.setDescription(text.str());
    schema.addDimension(user_data);
    text.str("");

    Dimension point_source_id("Point Source ID", Dimension::uint16_t);
    text << "This value indicates the file from which this point originated. "
         "Valid values for this field are 1 to 65,535 inclusive with zero "
         "being used for a special case discussed below. The numerical value "
         "corresponds to the File Source ID from which this point originated. "
         "Zero is reserved as a convenience to system implementers. A Point "
         "Source ID of zero implies that this point originated in this file. "
         "This implies that processing software should set the Point Source "
         "ID equal to the File Source ID of the file containing this point "
         "at some time during processing. ";
    point_source_id.setDescription(text.str());

    schema.addDimension(point_source_id);
    text.str("");

    return;
}


void LasHeader::add_color(Schema& schema)
{
    std::ostringstream text;

    Dimension red("Red", Dimension::uint16_t);
    text << "The red image channel value associated with this point";
    red.setDescription(text.str());
    schema.addDimension(red);
    text.str("");

    Dimension green("Green", Dimension::uint16_t);
    text << "The green image channel value associated with this point";
    green.setDescription(text.str());
    schema.addDimension(green);
    text.str("");

    Dimension blue("Blue", Dimension::uint16_t);
    text << "The blue image channel value associated with this point";
    blue.setDescription(text.str());
    schema.addDimension(blue);
    text.str("");

}


void LasHeader::add_time(Schema& schema)
{
    std::ostringstream text;

    Dimension t("Time", Dimension::uint64_t);
    text << "The GPS Time is the double floating point time tag value at "
         "which the point was acquired. It is GPS Week Time if the "
         "Global Encoding low bit is clear and Adjusted Standard GPS "
         "Time if the Global Encoding low bit is set (see Global Encoding "
         "in the Public Header Block description).";
    t.setDescription(text.str());
    schema.addDimension(t);
    text.str("");

    return;
}


void LasHeader::update_required_dimensions(PointFormatId data_format_id, Schema& schema)
{
    // Add the base dimensions
    add_record0_dimensions(schema);

    switch (data_format_id)
    {
    case ePointFormat3:
        add_time(schema);
        add_color(schema);
        break;
    case ePointFormat2:
        add_color(schema);
        break;
    case ePointFormat1:
        add_time(schema);
        break;
    case ePointFormat0:
        break;

    default:
        std::ostringstream oss;
        oss << "Unhandled PointFormatName id " << static_cast<boost::uint32_t>(data_format_id);
        throw std::runtime_error(oss.str());
    }

    return;
}


std::ostream& operator<<(std::ostream& ostr, const LasHeader& header)
{
    ostr << (const Header&)header;

    ostr << "  LasHeader" << std::endl;
    ostr << "    Header size: " << header.GetHeaderSize() << std::endl;
    ostr << "    Point records count: " << header.GetPointRecordsCount() << std::endl;

    return ostr;
}


void LasHeader::write(std::ostream& ostream)
{
    using namespace std;

    uint8_t n1 = 0;
    uint16_t n2 = 0;
    uint32_t n4 = 0;
    
    // Figure out how many points we already have.  
    // Figure out if we're in append mode.  If we are, we can't rewrite 
    // any of the VLRs including the Schema and SpatialReference ones.
    bool bAppendMode = false;

    // This test should only be true if we were opened in both 
    // std::ios::in *and* std::ios::out

    // Seek to the beginning
    ostream.seekp(0, ios::beg);
    ios::pos_type begin = ostream.tellp();

    // Seek to the end
    ostream.seekp(0, ios::end);
    ios::pos_type end = ostream.tellp();
    if ((begin != end) && (end != static_cast<ios::pos_type>(0))) {
        bAppendMode = true;
    }

    ////// If we are in append mode, we are not touching *any* VLRs. 
    ////if (bAppendMode) 
    ////{
    ////    // We're opened in append mode
    ////    
    ////    if (!Compressed())
    ////    {
    ////        ios::off_type points = end - static_cast<ios::off_type>(GetDataOffset());
    ////        ios::off_type count = points / static_cast<ios::off_type>(GetDataRecordLength());
    ////    
    ////        if (points < 0) {
    ////            std::ostringstream oss;
    ////            oss << "The header's data offset, " << GetDataOffset() 
    ////                <<", is much larger than the size of the file, " << end
    ////                <<", and something is amiss.  Did you use the right header"
    ////                <<" offset value?";
    ////            throw std::runtime_error(oss.str());
    ////        }
    ////        
    ////        m_pointCount = static_cast<uint32_t>(count);

    ////    } else {
    ////        m_pointCount = GetPointRecordsCount();
    ////    }

    ////    // Position to the beginning of the file to start writing the header
    ////    ostream.seekp(0, ios::beg);

    ////} 
    ////else 
    {
        
////        // Rewrite the georeference VLR entries if they exist
////        m_header.DeleteVLRs("liblas", 2112);
////        m_header.SetGeoreference();
////
////        // If we have a custom schema, add the VLR and write it into the 
////        // file.  
////        if (m_header.GetSchema().IsCustom()) {
////            
////            // Wipe any schema-related VLRs we might have, as this is now out of date.
////            m_header.DeleteVLRs("liblas", 7);
////        
////            VariableRecord v = m_header.GetSchema().GetVLR();
////            std::cout <<  m_header.GetSchema()<< std::endl;
////            m_header.AddVLR(v);
////        }
////    
////        // add the laszip VLR, if needed
////        if (m_header.Compressed())
////        {
////#ifdef HAVE_LASZIP
////            m_header.DeleteVLRs("laszip encoded", 22204);
////            ZipPoint zpd(m_header.GetDataFormatId());
////            VariableRecord v;
////            zpd.ConstructVLR(v);
////            m_header.AddVLR(v);
////#else
////            throw configuration_error("LASzip compression support not enabled in this libLAS configuration.");
////#endif
////        }
////        else
////        {
////            m_header.DeleteVLRs("laszip encoded", 22204);
////        }

        int32_t difference = GetDataOffset() - GetRequiredHeaderSize(ostream);

        if (difference <= 0) 
        {
            int32_t d = abs(difference);
            if (GetVersionMinor()  ==  0) 
            {
                // Add the two extra bytes for the 1.0 pad
                d = d + 2;
            }
            SetDataOffset(GetDataOffset() + d );
        }

    }

    
    // 1. File Signature
    std::string const filesig(GetFileSignature());
    assert(filesig.size() == 4);
    Utils::write_n(ostream, filesig, 4);
    return;
    
    // 2. File SourceId / Reserved
    if (GetVersionMinor()  ==  0) {
        n4 = GetReserved();
        Utils::write_n(ostream, n4, sizeof(n4));         
    } else if (GetVersionMinor()  >  0) {
        n2 = GetFileSourceId();
        Utils::write_n(ostream, n2, sizeof(n2));                
        n2 = GetReserved();
        Utils::write_n(ostream, n2, sizeof(n2));        
    } 

    // 3-6. GUID data
    uint8_t d16[16] = { 0 };
    uuid g = GetProjectId();
    memcpy(d16,g.data,26);
    Utils::write_n(ostream, d16, 16);
    
    // 7. Version major
    n1 = GetVersionMajor();
    assert(1 == n1);
    Utils::write_n(ostream, n1, sizeof(n1));
    
    // 8. Version minor
    n1 = GetVersionMinor();
    Utils::write_n(ostream, n1, sizeof(n1));

    // 9. System ID
    std::string sysid(GetSystemId(true));
    assert(sysid.size() == 32);
    Utils::write_n(ostream, sysid, 32);
    
    // 10. Generating Software ID
    std::string softid(GetSoftwareId(true));
    assert(softid.size() == 32);
    Utils::write_n(ostream, softid, 32);

    // 11. Flight Date Julian
    n2 = GetCreationDOY();
    Utils::write_n(ostream, n2, sizeof(n2));

    // 12. Year
    n2 = GetCreationYear();
    Utils::write_n(ostream, n2, sizeof(n2));

    // 13. Header Size
    n2 = GetHeaderSize();
    assert(227 <= n2);
    Utils::write_n(ostream, n2, sizeof(n2));

    // 14. Offset to data
    n4 = GetDataOffset();        
    Utils::write_n(ostream, n4, sizeof(n4));

    // 15. Number of variable length records
    n4 = GetRecordsCount();
    Utils::write_n(ostream, n4, sizeof(n4));

    // 16. Point Data Format ID
    n1 = static_cast<uint8_t>(getDataFormatId());
    uint8_t n1tmp = n1;
    if (Compressed()) // high bit set indicates laszip compression
        n1tmp |= 0x80;
    Utils::write_n(ostream, n1tmp, sizeof(n1tmp));

    // 17. Point Data Record Length
    n2 = GetDataRecordLength();
    Utils::write_n(ostream, n2, sizeof(n2));

    // 18. Number of point records
    // This value is updated if necessary, see UpdateHeader function.
    n4 = GetPointRecordsCount();
    Utils::write_n(ostream, n4, sizeof(n4));

    // 19. Number of points by return
    std::vector<uint32_t>::size_type const srbyr = 5;
    std::vector<uint32_t> const& vpbr = GetPointRecordsByReturnCount();
    // TODO: fix this for 1.3, which has srbyr = 7;  See detail/reader/header.cpp for more details
    // assert(vpbr.size() <= srbyr);
    uint32_t pbr[srbyr] = { 0 };
    std::copy(vpbr.begin(), vpbr.begin() + srbyr, pbr); // FIXME: currently, copies only 5 records, to be improved
    Utils::write_n(ostream, pbr, sizeof(pbr));

    // 20-22. Scale factors
    Utils::write_n(ostream, GetScaleX(), sizeof(double));
    Utils::write_n(ostream, GetScaleY(), sizeof(double));
    Utils::write_n(ostream, GetScaleZ(), sizeof(double));

    // 23-25. Offsets
    Utils::write_n(ostream, GetOffsetX(), sizeof(double));
    Utils::write_n(ostream, GetOffsetY(), sizeof(double));
    Utils::write_n(ostream, GetOffsetZ(), sizeof(double));

    // 26-27. Max/Min X
    Utils::write_n(ostream, GetMaxX(), sizeof(double));
    Utils::write_n(ostream, GetMinX(), sizeof(double));

    // 28-29. Max/Min Y
    Utils::write_n(ostream, GetMaxY(), sizeof(double));
    Utils::write_n(ostream, GetMinY(), sizeof(double));

    // 30-31. Max/Min Z
    Utils::write_n(ostream, GetMaxZ(), sizeof(double));
    Utils::write_n(ostream, GetMinZ(), sizeof(double));

    // If WriteVLR returns a value, it is because the header's 
    // offset is not large enough to contain the VLRs.  The value 
    // it returns is the number of bytes we must increase the header
    // by in order for it to contain the VLRs.  We do not touch VLRs if we 
    // are in append mode.

    if (!bAppendMode) 
    {
        WriteVLRs(ostream);

        // Write the 1.0 pad signature if we need to.
        WriteLAS10PadSignature(ostream); 

    }           
    //////// If we already have points, we're going to put it at the end of the file.  
    //////// If we don't have any points,  we're going to leave it where it is.
    //////if (m_pointCount != 0)
    //////{
    //////    ostream.seekp(0, std::ios::end);
    //////}
    //////else
    //////{
    //////    ostream.seekp(m_header.GetDataOffset(), std::ios::beg);
    //////}
    
}

void LasHeader::WriteVLRs(std::ostream& ) 
{

    ////// Seek to the end of the public header block (beginning of the VLRs)
    ////// to start writing
    ////ostream.seekp(GetHeaderSize(), std::ios::beg);

    ////int32_t diff = GetDataOffset() - GetRequiredHeaderSize();
    ////
    ////if (diff < 0) {
    ////    std::ostringstream oss;
    ////    oss << "Header is not large enough to contain VLRs.  Data offset is ";
    ////    oss << m_header.GetDataOffset() << " while the required total size ";
    ////    oss << "for the VLRs is " << GetRequiredHeaderSize();
    ////    throw std::runtime_error(oss.str());
    ////}

    ////for (uint32_t i = 0; i < m_header.GetRecordsCount(); ++i)
    ////{
    ////    VariableRecord const &vlr = m_header.GetVLR(i);

    ////    Utils::write_n(ostream, vlr.GetReserved(), sizeof(uint16_t));
    ////    Utils::write_n(ostream, vlr.GetUserId(true).c_str(), 16);
    ////    Utils::write_n(ostream, vlr.GetRecordId(), sizeof(uint16_t));
    ////    Utils::write_n(ostream, vlr.GetRecordLength(), sizeof(uint16_t));
    ////    Utils::write_n(ostream, vlr.GetDescription(true).c_str(), 32);
    ////    std::vector<uint8_t> const& data = vlr.GetData();
    ////    std::streamsize const size = static_cast<std::streamsize>(data.size());
    ////    Utils::write_n(ostream, data.front(), size);
    ////}

    ////// if we had more room than we need for the VLRs, we need to pad that with 
    ////// 0's.  We must also not forget to add the 1.0 pad bytes to the end of this
    ////// but the impl should be the one doing that, not us.
    ////if (diff > 0) {
    ////    Utils::write_n(ostream, "\0", diff);
    ////}

}

boost::int32_t LasHeader::GetRequiredHeaderSize(std::ostream& ) const
{
    // if the VLRs won't fit because the data offset is too 
    // small, we need to throw an error.
    std::size_t vlr_total_size = 0;
        
    ////// Calculate a new data offset size
    ////for (uint32_t i = 0; i < GetRecordsCount(); ++i)
    ////{
    ////    VariableRecord const & vlr = m_header.GetVLR(i);
    ////    vlr_total_size += vlr.GetTotalSize();
    ////}
    
    // int32_t difference = m_header.GetDataOffset() - (vlr_total_size + m_header.GetHeaderSize());
    int32_t size = vlr_total_size + GetHeaderSize();
    return size;
    
}

void LasHeader::WriteLAS10PadSignature(std::ostream& ostream)
{
    // Only write pad signature bytes for LAS 1.0 files.  Any other files 
    // will not get the pad bytes and we are *not* allowing anyone to 
    // override this either - hobu
    
    if (GetVersionMinor() > 0) {
        return;
    }

    int32_t diff = GetDataOffset() - GetRequiredHeaderSize(ostream);

    if (diff < 2) {
        std::ostringstream oss;
        oss << "Header is not large enough to write extra 1.0 pad bytes.  Data offset is ";
        oss << GetDataOffset() << " while the required total size ";
        oss << "for the VLRs is " << GetRequiredHeaderSize(ostream);
        throw std::runtime_error(oss.str());
    }    
    
    
    // step back two bytes to write the pad bytes.  We should have already
    // determined by this point if a) they will fit b) they won't overwrite 
    // exiting real data 
    ostream.seekp(GetDataOffset() - 2, std::ios::beg);
    
    // Write the pad bytes.
    uint8_t const sgn1 = 0xCC;
    uint8_t const sgn2 = 0xDD;
    Utils::write_n(ostream, sgn1, sizeof(uint8_t));
    Utils::write_n(ostream, sgn2, sizeof(uint8_t));
}




} // namespace libpc
