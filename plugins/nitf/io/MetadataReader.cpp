/******************************************************************************
* Copyright (c) 2014, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include "MetadataReader.hpp"

#include <pdal/Metadata.hpp>

#include <boost/algorithm/string.hpp>

#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic ignored "-Wenum-compare"
#endif


namespace pdal
{


MetadataReader::MetadataReader(::nitf::Record& record,
                               MetadataNode& node,
                               bool showEmptyFields) :
    m_record(record),
    m_node(node),
    m_showEmptyFields(showEmptyFields)
{
    return;
}


MetadataReader::~MetadataReader()
{
    return;
}


void MetadataReader::read()
{
    //
    // dump the file header
    //
    ::nitf::FileHeader header = m_record.getHeader();
    ::nitf::FileSecurity security = header.getSecurityGroup();        
    
    doFileHeader("FH", header);
    doSecurity("FH", "F", security);
    
    ::nitf::Extensions ext = header.getExtendedSection();
    doExtensions("FH", ext);

    ::nitf::ListIterator iter;
    ::nitf::Uint32 num;
    ::nitf::Uint32 i;

    //
    // dump the image info, for each image
    //
    iter = m_record.getImages().begin();
    num = m_record.getNumImages();
    for (i=0; i<num; i++)
    {
        const std::string key = "IM:" + std::to_string(i);
        
        ::nitf::ImageSegment segment = *iter;
        ::nitf::ImageSubheader header = segment.getSubheader();
        
        doImageSubheader(key, header);
        
        ::nitf::Extensions ext = header.getExtendedSection();
        doExtensions(key, ext);
        
        ::nitf::Extensions ext2 = header.getUserDefinedSection();
        doExtensions(key, ext2);
    }
    
    //
    // dump the DE info, for each DE
    //
    iter = m_record.getDataExtensions().begin();
    num = m_record.getNumDataExtensions();
    for (i=0; i<num; i++)
    {
        const std::string key = "DE:" + std::to_string(i);
        
        ::nitf::DESegment segment = *iter;
        ::nitf::DESubheader header = segment.getSubheader();
        ::nitf::FileSecurity security = header.getSecurityGroup();
        
        doDESubheader(key, header);
        
        ::nitf::Extensions ext = header.getUserDefinedSection();
        doExtensions(key, ext);
    }
}


void MetadataReader::writeField(const std::string& parentkey,
                                const std::string& key,
                                ::nitf::Field field)
{
    std::string v;
  
    if (field.getType() == (::nitf::Field::FieldType)NITF_BCS_A)
    {
        v = field.toString();
    }
    else if (field.getType() == (::nitf::Field::FieldType)NITF_BCS_N)
    {
        v = field.toString();
    }
    else if (field.getType() == (::nitf::Field::FieldType)NITF_BINARY)
    {
	if (key == "FBKGC")
	{
            // special case: field is three distinct bytes
            std::string t = field.toString();
            v = std::to_string((unsigned int)t[0]) + ", " +
                std::to_string((unsigned int)t[1]) + ", " +
                std::to_string((unsigned int)t[2]);
        }
        else
        {
            v = "(binary)";
        }
    }
    else
    {
        throw pdal_error("error reading nitf (2)");
    }    
   
    boost::algorithm::trim(v);
    const bool blank = (v.length() == 0);    
    if (!blank || (blank && m_showEmptyFields))
    {
        m_node.add<std::string>(parentkey + "." + key, v);
    }
    
    return;
}


void MetadataReader::writeInt(const std::string& parentkey,
                              const std::string& key,
                              int thevalue)
{   
    m_node.add<std::string>(parentkey + "." + key, std::to_string(thevalue));
}


void MetadataReader::writeString(const std::string& parentkey,
                                 const std::string& key,
                                 const std::string& thevalue)
{   
    m_node.add<std::string>(parentkey + "." + key, thevalue);
}
    
   
void MetadataReader::doFileHeader(const std::string& parentkey,
                                  ::nitf::FileHeader& header)
{
    writeField("FH", "FHDR", header.getFileHeader());
    writeField("FH", "FVER", header.getFileVersion());
    writeField("FH", "CLEVEL", header.getComplianceLevel());
    writeField("FH", "STYPE", header.getSystemType());
    writeField("FH", "OSTAID", header.getOriginStationID());
    writeField("FH", "FDT", header.getFileDateTime());
    writeField("FH", "FTITLE", header.getFileTitle());
    writeField("FH", "FSCLAS", header.getClassification());
    writeField("FH", "FSCOP", header.getMessageCopyNum());
    writeField("FH", "FSCPYS", header.getMessageNumCopies());
    writeField("FH", "ENCRYP", header.getEncrypted());
    writeField("FH", "FBKGC", header.getBackgroundColor());
    writeField("FH", "ONAME", header.getOriginatorName());
    writeField("FH", "OPHONE", header.getOriginatorPhone());
    writeField("FH", "FL", header.getFileLength());
    writeField("FH", "HL", header.getHeaderLength());
    writeField("FH", "NUMI", header.getNumImages());
    writeField("FH", "NUMS", header.getNumGraphics());
    //writeField("FH", "???", header.getNumLabels()); //unsupported in 2500c spec
    writeField("FH", "NUMT", header.getNumTexts());
    writeField("FH", "NUMDES", header.getNumDataExtensions());
    writeField("FH", "NUMRES", header.getNumReservedExtensions());
}
    

void MetadataReader::doSecurity(const std::string& parentkey,
                                const std::string& prefix,
                                ::nitf::FileSecurity& security)
{
    writeField(parentkey, prefix + "SCLSY", security.getClassificationSystem());
    writeField(parentkey, prefix + "SCODE", security.getCodewords());
    writeField(parentkey, prefix + "SCTLH", security.getControlAndHandling());
    writeField(parentkey, prefix + "SREL", security.getReleasingInstructions());
    writeField(parentkey, prefix + "SDCTP", security.getDeclassificationType());
    writeField(parentkey, prefix + "SDCDT", security.getDeclassificationDate());
    writeField(parentkey, prefix + "SDCXM", security.getDeclassificationExemption());
    writeField(parentkey, prefix + "SDG", security.getDowngrade());
    writeField(parentkey, prefix + "SDGDT", security.getDowngradeDateTime());
    writeField(parentkey, prefix + "SCLTX", security.getClassificationText());
    writeField(parentkey, prefix + "SCATP", security.getClassificationAuthorityType());
    writeField(parentkey, prefix + "SCAUT", security.getClassificationAuthority());
    writeField(parentkey, prefix + "SCRSN", security.getClassificationReason());
    writeField(parentkey, prefix + "SSRDT", security.getSecuritySourceDate());
    writeField(parentkey, prefix + "SCTLN", security.getSecurityControlNumber());
}

    
void MetadataReader::doBands(const std::string& key,
                             ::nitf::ImageSubheader& header)
{
    const int nbands = (int)header.getNumImageBands();
    for (int i=0; i<nbands; i++)
    {
        ::nitf::BandInfo bandinfo = header.getBandInfo(i);
        std::string subkey = key + ".BAND:" + std::to_string(i);
        doBand(subkey, bandinfo);
    }
}
    

void MetadataReader::doBand(const std::string& key,
                            ::nitf::BandInfo& band)
{
    writeField(key, "IREPBAND", band.getRepresentation());    
    writeField(key, "ISUBCAT", band.getSubcategory());
    writeField(key, "IFC", band.getImageFilterCondition());
    writeField(key, "IMFLT", band.getImageFilterCode());
    writeField(key, "NLUTS", band.getNumLUTs());
    writeField(key, "NELUT", band.getBandEntriesPerLUT());

    ::nitf::LookupTable lut = band.getLookupTable();
    writeInt(key, "num_lookup_tables", lut.getTables());
    writeInt(key, "num_lookup_entries", lut.getEntries());
}


void MetadataReader::doImageSubheader(const std::string& key,
                                      ::nitf::ImageSubheader& subheader)
{
    writeField(key, "IID1", subheader.getImageId());
    writeField(key, "IDATIM", subheader.getImageDateAndTime());
    writeField(key, "TGTID", subheader.getTargetId());
    writeField(key, "IID2", subheader.getImageTitle());
    writeField(key, "ISCLAS", subheader.getImageSecurityClass());
    
    ::nitf::FileSecurity security = subheader.getSecurityGroup();
    doSecurity(key, "I", security);

    writeField(key, "ENCRYP", subheader.getEncrypted());
    writeField(key, "ISORCE", subheader.getImageSource());
    writeField(key, "NROWS", subheader.getNumRows());
    writeField(key, "NCOLS", subheader.getNumCols());
    writeField(key, "PVTYPE", subheader.getPixelValueType());
    writeField(key, "IREP", subheader.getImageRepresentation());
    writeField(key, "ICAT", subheader.getImageCategory());
    writeField(key, "ABPP", subheader.getActualBitsPerPixel());
    writeField(key, "PJUST", subheader.getPixelJustification());
    writeField(key, "ICORDS", subheader.getImageCoordinateSystem());
    writeField(key, "IGEOLO", subheader.getCornerCoordinates());
    writeField(key, "NICOM", subheader.getNumImageComments());

    ::nitf::List list = subheader.getImageComments();
    doComments(key, list);
    
    writeField(key, "IC", subheader.getImageCompression());
    writeField(key, "COMRAT", subheader.getCompressionRate());
    writeField(key, "NBANDS", subheader.getNumImageBands());
    writeField(key, "XBANDS", subheader.getNumMultispectralImageBands());

    doBands(key, subheader);

    writeField(key, "ISYNC", subheader.getImageSyncCode());
    writeField(key, "IMODE", subheader.getImageMode());
    writeField(key, "NBPR", subheader.getNumBlocksPerRow());
    writeField(key, "NBPC", subheader.getNumBlocksPerCol());
    writeField(key, "NPPBH", subheader.getNumPixelsPerHorizBlock());
    writeField(key, "NPPVB", subheader.getNumPixelsPerVertBlock());
    writeField(key, "NBPP", subheader.getNumBitsPerPixel());
    writeField(key, "IDLVL", subheader.getImageDisplayLevel());
    writeField(key, "IALVL", subheader.getImageAttachmentLevel());
    writeField(key, "ILOC", subheader.getImageLocation());
    writeField(key, "IMAG", subheader.getImageMagnification());
}
    

void MetadataReader::doDESubheader(const std::string& key,
                                   ::nitf::DESubheader& subheader)
{
    writeField(key, "DESID", subheader.getTypeID());
    writeField(key, "DESVER", subheader.getVersion());
    writeField(key, "DECLAS", subheader.getSecurityClass());

    ::nitf::FileSecurity security = subheader.getSecurityGroup();
    doSecurity(key, "DE", security);

    // this is not an internal field, not interesting to users
    //writeField(key, "DESSHL", subheader.getDataLength());
}


void MetadataReader::doTRE(const std::string& key,
                           ::nitf::TRE& tre)
{
    const std::string& tag = key + "." + tre.getTag();

    // The C interface to nitro has a TREDescription object which
    // (I think) allows you to enumerate the keys in the
    // TRE. However, there is no C++ interface for it, so instead
    // we will use the raw iterator to access each (Key,Value)
    // pair. we won't use the Value from pair.second() directly,
    // however: instead, we'll call getField(key) and get the
    // value as represented by a Field object (which will tell us
    // the formatting, etc).
    
    ::nitf::TREFieldIterator iter = tre.begin();
    while (iter != tre.end())
    {
        try
        {
            // nitro will explicitly throw when dereferencing iter
            // if there's no pair object set (would be nice if
            // there was a is_valid() function or something...)
            ::nitf::Pair pair = *iter;
            
            const char* key = pair.first();
            
            // only put into metadata things that look like legit
            // stringy things
            if (strcmp(key, "raw_data") != 0)
            {
                ::nitf::Field field = tre.getField(key);
                writeField(tag, key, field);
            }
        }            
        catch (::except::NullPointerReference&)
        {
            // oh, well - skip this one, go to the next iteration
        }
        
        ++iter;
    }

    return;
}

    
void MetadataReader::doExtensions(const std::string& key,
                                  ::nitf::Extensions& ext)
{
    ::nitf::ExtensionsIterator iter = ext.begin();
    while (iter != ext.end())
    {
        ::nitf::TRE tre = *iter;

        doTRE(key, tre);

        ++iter;
    }

    return;
}
    

void MetadataReader::doComments(const std::string& key,
                                ::nitf::List& list)
{
    int i = 0;
    ::nitf::ListIterator iter = list.begin();
    while (iter != list.end())
    {
        ::nitf::Field field = *iter;

        const std::string subkey = "ICOM:" + std::to_string(i);
        
        writeField(key, subkey, field);

        ++i;
        ++iter;
    }

    return;
}
    

} // namespaces
