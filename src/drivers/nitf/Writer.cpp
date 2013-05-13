/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/drivers/nitf/Writer.hpp>
#include <pdal/drivers/las/Writer.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/GlobalEnvironment.hpp>


#ifdef PDAL_HAVE_NITRO
#include <nitro/c++/import/nitf.hpp>
#include <nitro/c++/except/Trace.h>
#endif
// NOTES
//
// is it legal to write a LAZ file?
// syntactically, how do we name all the LAS writer options that we will pass to the las writer?
//

namespace pdal
{
namespace drivers
{
namespace nitf
{

Writer::Writer(Stage& prevStage, const Options& options)
    :  pdal::drivers::las::Writer(prevStage, static_cast<std::ostream*>(&m_oss))


{
    Options&  opts = getOptions();
    opts = options;
    
    m_oss.str("");
    return;
}

Writer::~Writer()
{
    return;
}


void Writer::initialize()
{
    // call super class
    pdal::drivers::las::Writer::initialize();

    m_filename = getOptions().getValueOrThrow<std::string>("filename");

    return;
}


Options Writer::getDefaultOptions()
{
    Options options;
    return options;
}


void Writer::writeBegin(boost::uint64_t targetNumPointsToWrite)
{
    // call super class
    pdal::drivers::las::Writer::writeBegin(targetNumPointsToWrite);

    return;
}


void Writer::writeBufferBegin(PointBuffer const& buffer)
{
    // call super class
    pdal::drivers::las::Writer::writeBufferBegin(buffer);
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& buffer)
{
    // call super class
    return pdal::drivers::las::Writer::writeBuffer(buffer);
}


void Writer::writeBufferEnd(PointBuffer const& buffer)
{
    // call super class
    pdal::drivers::las::Writer::writeBufferEnd(buffer);
}


void Writer::writeEnd(boost::uint64_t actualNumPointsWritten)
{
    // call super class
    pdal::drivers::las::Writer::writeEnd(actualNumPointsWritten);

    m_oss.flush();

#ifdef PDAL_HAVE_NITRO

    try
    {

    ::nitf::Record record(NITF_VER_21);
    ::nitf::FileHeader header = record.getHeader();
    header.getFileHeader().set("NITF");
    header.getComplianceLevel().set(getOptions().getValueOrDefault<std::string>("CLEVEL","03"));
    header.getSystemType().set(getOptions().getValueOrDefault<std::string>("STYPE","BF01"));
    header.getOriginStationID().set(getOptions().getValueOrDefault<std::string>("OSTAID","PDAL"));
    header.getFileTitle().set(getOptions().getValueOrDefault<std::string>("FTITLE","FTITLE"));
    header.getClassification().set(getOptions().getValueOrDefault<std::string>("FSCLAS","U"));
    header.getMessageCopyNum().set("00000");
    header.getMessageNumCopies().set("00000");
    header.getEncrypted().set("0");
    header.getBackgroundColor().setRawData((char*)"000", 3);
    header.getOriginatorName().set(getOptions().getValueOrDefault<std::string>("ONAME",""));
    header.getOriginatorPhone().set(getOptions().getValueOrDefault<std::string>("OPHONE",""));

    ::nitf::DESegment des = record.newDataExtensionSegment();

    des.getSubheader().getFilePartType().set("DE");

    des.getSubheader().getTypeID().set("LIDARA DES");
    des.getSubheader().getVersion().set("01");
    des.getSubheader().getSecurityClass().set(getOptions().getValueOrDefault<std::string>("FSCLAS","U"));

    ::nitf::FileSecurity security =
        record.getHeader().getSecurityGroup();
    des.getSubheader().setSecurityGroup(security.clone());


    ::nitf::TRE usrHdr("LIDARA DES", "raw_data");
    
    usrHdr.setField("raw_data", "not");
    ::nitf::Field fld = usrHdr.getField("raw_data");
    fld.setType(::nitf::Field::BINARY);


    std::streambuf *buf = m_oss.rdbuf();


    long size = buf->pubseekoff(0, m_oss.end);
    buf->pubseekoff(0, m_oss.beg);
        
    char* bytes = new char[size];
    buf->sgetn(bytes, size);

    des.getSubheader().setSubheaderFields(usrHdr);

    ::nitf::ImageSegment image = record.newImageSegment();
    ::nitf::ImageSubheader subheader = image.getSubheader();

    subheader.getImageSecurityClass().set(getOptions().getValueOrDefault<std::string>("FSCLAS","U"));
    
    std::string fdate = getOptions().getValueOrDefault<std::string>("IDATIM", "");
    if (fdate.size())
        subheader.getImageDateAndTime().set(fdate);
    
    ::nitf::BandInfo info;    
    ::nitf::LookupTable lt(0,0);
    info.init( "G",   /* The band representation, Nth band */
               " ",      /* The band subcategory */
               "N",      /* The band filter condition */
               "   ",    /* The band standard image filter code */
               0,        /* The number of look-up tables */
               0,        /* The number of entries/LUT */
               lt);     /* The look-up tables */
    
    std::vector< ::nitf::BandInfo> bands;
    bands.push_back(info);
    subheader.setPixelInformation( "INT",     /* Pixel value type */
                                     8,         /* Number of bits/pixel */
                                     8,         /* Actual number of bits/pixel */
                                     "G",       /* Pixel justification */
                                     "G",     /* Image representation */
                                     "VIS",     /* Image category */
                                     1,         /* Number of bands */
                                     bands);
    
    subheader.setBlocking(  8, /*!< The number of rows */
                            8,  /*!< The number of columns */
                            8, /*!< The number of rows/block */
                            8,  /*!< The number of columns/block */
                            "P"                /*!< Image mode */
                                         );
    subheader.getImageId().set("None");
    // 64 char string
    char* buffer = "0000000000000000000000000000000000000000000000000000000000000000";
    
    ::nitf::BandSource* band =
        new ::nitf::MemorySource( buffer, 
                                strlen(buffer) /* memory size */, 
                                0 /* starting offset */, 
                                1 /* bytes per pixel */, 
                                0 /*skip*/);
    ::nitf::ImageSource iSource;
    iSource.addBand(*band);
    
    ::nitf::Writer writer;
    ::nitf::IOHandle output_io(m_filename.c_str(), NITF_ACCESS_WRITEONLY, NITF_CREATE);
    writer.prepare(output_io, record);
    
    ::nitf::SegmentWriter sWriter = writer.newDEWriter(0);

    ::nitf::SegmentMemorySource sSource(bytes, size, 0, 0, false);
    sWriter.attachSource(sSource);

    ::nitf::ImageWriter iWriter = writer.newImageWriter(0);
    iWriter.attachSource(iSource);


    writer.write();
    output_io.close();
    }

    catch (except::Throwable & t)
    {
        std::ostringstream oss;
        // std::cout << t.getTrace();
        throw pdal_error( t.getMessage());
    }
#endif
    return;
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

}
}
} // namespaces

