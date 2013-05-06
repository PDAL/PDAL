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

#ifdef PDAL_HAVE_GDAL

#include "gdal.h"
#include "cpl_vsi.h"
#include "cpl_conv.h"
#include "cpl_string.h"

#if ((GDAL_VERSION_MAJOR == 1 && GDAL_VERSION_MINOR < 10) || (GDAL_VERSION_MAJOR < 1))
// #error "NITF support requires GDAL 1.10 or GDAL 2.0+"
#endif

#include <import/nitf.hpp>
#include <except/Trace.h>

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
    : pdal::drivers::las::Writer(prevStage, static_cast<std::ostream*>(&m_oss))


{
    Options&  opts = getOptions();
    opts = options;
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

    size_t size = m_oss.str().size();
    log()->get(logDEBUG) <<  " lasfile size: " << size <<  std::endl;                    
    
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& buffer)
{
    // call super class
    return pdal::drivers::las::Writer::writeBuffer(buffer);

    size_t size = m_oss.str().size();
    log()->get(logDEBUG) <<  " lasfile size: " << size <<  std::endl;                    
    
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
    try
    {
        
    //     ::nitf::IOHandle in("/Users/hobu//dev/git/nitro/c/nitf/tests/test_blank.ntf"); /* Input I/O handle */
    // ::nitf::Reader reader;
    // ::nitf::Record record = reader.read(in);

    ::nitf::Record record(NITF_VER_21);
    ::nitf::FileHeader header = record.getHeader();
    header.getFileHeader().set("NITF");
    header.getComplianceLevel().set("03");
    header.getSystemType().set("BF01");
    header.getOriginStationID().set("PDAL");
    header.getFileTitle().set("FTITLE");
    header.getClassification().set("U");
    header.getMessageCopyNum().set("00000");
    header.getMessageNumCopies().set("00000");
    header.getEncrypted().set("0");
    header.getBackgroundColor().setRawData((char*)"000", 3);
    header.getOriginatorName().set("");
    header.getOriginatorPhone().set("");

    ::nitf::DESegment des = record.newDataExtensionSegment();

    des.getSubheader().getFilePartType().set("DE");

    des.getSubheader().getTypeID().set("LIDARA DES");
    des.getSubheader().getVersion().set("01");
    des.getSubheader().getSecurityClass().set("U");

    ::nitf::FileSecurity security =
        record.getHeader().getSecurityGroup();
    des.getSubheader().setSecurityGroup(security.clone());
    const char lasdata[] = "123456dfdfadsfasdfasdfa789ABCDEF0\0";    

    ::nitf::TRE usrHdr("LIDARA DES", "raw_data");
    
    usrHdr.setField("raw_data", "not");
    ::nitf::Field fld = usrHdr.getField("raw_data");
    fld.setType(::nitf::Field::BINARY);
    // fld.resize(strlen(lasdata));
    // fld.setRawData((char*)&lasdata, strlen(lasdata));
    
                 


    std::streambuf *buf = m_oss.rdbuf();
    size_t size = m_oss.str().size();
    // std::streamsize size = buf->pubseekpos(0,m_oss.end);
    buf->pubseekoff(0, m_oss.beg);

    log()->get(logDEBUG) <<  " lasfile size: " << size <<  std::endl;   
        
    char* bytes = new char[size];
    buf->sgetn(bytes, size);

    // fld.resize( size );
    // fld.setRawData(bytes,  size );
    
    des.getSubheader().setSubheaderFields(usrHdr);
    
    ::nitf::Writer writer;
    ::nitf::IOHandle output_io("written.ntf", NITF_ACCESS_WRITEONLY, NITF_CREATE);
    writer.prepare(output_io, record);
    
    ::nitf::SegmentWriter sWriter = writer.newDEWriter(0);

    // ::nitf::SegmentMemorySource sSource(lasdata, strlen(lasdata), 0, 0, false);
    ::nitf::SegmentMemorySource sSource(bytes, size, 0, 0, false);
    
    sWriter.attachSource(sSource);
    writer.write();
    }

    catch (except::Throwable & t)
    {
        std::ostringstream oss;
        // std::cout << t.getTrace();
        throw pdal_error( t.getMessage());
    }

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

#endif
