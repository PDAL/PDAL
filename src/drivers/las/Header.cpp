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

#include <libpc/drivers/las/Header.hpp>

#include <boost/uuid/uuid_io.hpp>

#include <libpc/Stage.hpp>

namespace libpc { namespace drivers { namespace las {

    
// BUG: should be std::string
char const* const LasHeader::FileSignature = "LASF";
char const* const LasHeader::SystemIdentifier = "libLAS";
char const* const LasHeader::SoftwareIdentifier = "libLAS 1.6.0";

LasHeader::LasHeader()
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

boost::uuids::uuid LasHeader::GetProjectId() const
{
    return m_projectGuid;
}

void LasHeader::SetProjectId(boost::uuids::uuid const& v)
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
 
    return LasHeader::ePointFormat3; // BUG BUG BUG
    //return static_cast<boost::uint16_t>(getSchema().getByteSize());
}

uint32_t LasHeader::GetPointRecordsCount() const
{
    return m_pointRecordsCount;
}

void LasHeader::SetPointRecordsCount(uint32_t v)
{
    m_pointRecordsCount = v;
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
    m_scales[0] = Utils::compare_distance(0.0, y) ? minscale : y;
    m_scales[0] = Utils::compare_distance(0.0, z) ? minscale : z;
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

void LasHeader::SetMax(double x, double y, double z)
{
    // m_extent = Bounds(m_extent.min(0), m_extent.min(1), m_extent.max(0), m_extent.max(1), m_extent.min(2), m_extent.max(2));
    // Bounds(minx, miny, minz, maxx, maxy, maxz)
    Bounds<double> temp(m_bounds.getMinimum(0), m_bounds.getMinimum(1), m_bounds.getMinimum(2), x, y, z);
    m_bounds = temp;
}

void LasHeader::SetMin(double x, double y, double z)
{
    Bounds<double> temp(x, y, z, m_bounds.getMaximum(0), m_bounds.getMaximum(1), m_bounds.getMaximum(2));
    m_bounds = temp;
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



void LasHeader::add_record0_dimensions(Schema& schema)
{
    std::ostringstream text;

    Dimension x(Dimension::Field_X, Dimension::Int32);
    text << "x coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    x.setDescription(text.str());
    schema.addDimension(x);
    text.str("");

    Dimension y(Dimension::Field_Y, Dimension::Int32);
    text << "y coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    y.setDescription(text.str());
    schema.addDimension(y);
    text.str("");

    Dimension z(Dimension::Field_Z, Dimension::Int32);
    text << "z coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    z.setDescription(text.str());
    schema.addDimension(z);
    text.str("");

    Dimension intensity(Dimension::Field_Intensity, Dimension::Int16);
    text << "The intensity value is the integer representation of the pulse "
         "return magnitude. This value is optional and system specific. "
         "However, it should always be included if available.";
    intensity.setDescription(text.str());
    schema.addDimension(intensity);
    text.str("");

    Dimension return_no(Dimension::Field_ReturnNumber, Dimension::Uint8); // 3 bits only
    text << "Return Number: The Return Number is the pulse return number for "
         "a given output pulse. A given output laser pulse can have many "
         "returns, and they must be marked in sequence of return. The first "
         "return will have a Return Number of one, the second a Return "
         "Number of two, and so on up to five returns.";
    return_no.setDescription(text.str());
    schema.addDimension(return_no);
    text.str("");

    Dimension no_returns(Dimension::Field_NumberOfReturns, Dimension::Uint8); // 3 bits only
    text << "Number of Returns (for this emitted pulse): The Number of Returns "
         "is the total number of returns for a given pulse. For example, "
         "a laser data point may be return two (Return Number) within a "
         "total number of five returns.";
    no_returns.setDescription(text.str());
    schema.addDimension(no_returns);
    text.str("");

    Dimension scan_dir(Dimension::Field_ScanDirectionFlag, Dimension::Uint8); // 1 bit only
    text << "The Scan Direction Flag denotes the direction at which the "
         "scanner mirror was traveling at the time of the output pulse. "
         "A bit value of 1 is a positive scan direction, and a bit value "
         "of 0 is a negative scan direction (where positive scan direction "
         "is a scan moving from the left side of the in-track direction to "
         "the right side and negative the opposite). ";
    scan_dir.setDescription(text.str());
    schema.addDimension(scan_dir);
    text.str("");

    Dimension edge(Dimension::Field_EdgeOfFlightLine, Dimension::Uint8); // 1 bit only
    text << "The Edge of Flight Line data bit has a value of 1 only when "
         "the point is at the end of a scan. It is the last point on "
         "a given scan line before it changes direction.";
    edge.setDescription(text.str());
    schema.addDimension(edge);
    text.str("");

    Dimension classification(Dimension::Field_Classification, Dimension::Uint8);
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

    Dimension scan_angle(Dimension::Field_ScanAngleRank, Dimension::Int8);
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

    Dimension user_data(Dimension::Field_UserData, Dimension::Uint8);
    text << "This field may be used at the user’s discretion";
    user_data.setDescription(text.str());
    schema.addDimension(user_data);
    text.str("");

    Dimension point_source_id(Dimension::Field_PointSourceId, Dimension::Uint16);
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

    Dimension red(Dimension::Field_Red, Dimension::Uint16);
    text << "The red image channel value associated with this point";
    red.setDescription(text.str());
    schema.addDimension(red);
    text.str("");

    Dimension green(Dimension::Field_Green, Dimension::Uint16);
    text << "The green image channel value associated with this point";
    green.setDescription(text.str());
    schema.addDimension(green);
    text.str("");

    Dimension blue(Dimension::Field_Blue, Dimension::Uint16);
    text << "The blue image channel value associated with this point";
    blue.setDescription(text.str());
    schema.addDimension(blue);
    text.str("");

}


void LasHeader::add_time(Schema& schema)
{
    std::ostringstream text;

    Dimension t(Dimension::Field_Time, Dimension::Double);
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
    ostr << "  LasHeader" << std::endl;
    ostr << "    Header size: " << header.GetHeaderSize() << std::endl;
    ostr << "    Point records count: " << header.GetPointRecordsCount() << std::endl;

    return ostr;
}



} } } // namespaces
