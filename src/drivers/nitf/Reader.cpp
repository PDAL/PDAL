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

#include <pdal/drivers/nitf/Reader.hpp>
#ifdef PDAL_HAVE_GDAL

#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/las/Reader.hpp>

#include "nitflib.h"

// gdal
#include "gdal.h"
#include "cpl_vsi.h"
#include "cpl_conv.h"
#include "cpl_string.h"


namespace pdal { namespace drivers { namespace nitf {

static int extractLASFromNITF(NITFDES* psDES, const char* pszLASName)
{
    NITFSegmentInfo* psSegInfo;

    if ( CSLFetchNameValue(psDES->papszMetadata, "NITF_DESID") == NULL )
        return FALSE;

    psSegInfo = psDES->psFile->pasSegmentInfo + psDES->iSegment;

    GByte* pabyBuffer;

    pabyBuffer = (GByte*) VSIMalloc((size_t)psSegInfo->nSegmentSize);
    if (pabyBuffer == NULL)
    {
        throw pdal_error("Unable to allocate buffer large enough to extract NITF data!");
    }

    VSIFSeekL(psDES->psFile->fp, psSegInfo->nSegmentStart, SEEK_SET);
    if (VSIFReadL(pabyBuffer, 1, (size_t)psSegInfo->nSegmentSize, psDES->psFile->fp) != psSegInfo->nSegmentSize)
    {
        VSIFree(pabyBuffer);
        throw pdal_error("Unable to allocate extract NITF data!");
    }

    VSILFILE* fp = NULL;
    fp = VSIFOpenL(pszLASName, "wb");
    if (fp == NULL)
    {
        VSIFree(pabyBuffer);
        throw pdal_error("Unable to open filename to write!");
    }

    VSIFWriteL(pabyBuffer, 1, (size_t)psSegInfo->nSegmentSize, fp);
    VSIFCloseL(fp);
    VSIFree(pabyBuffer);

    return TRUE;
}

// References:
//   - NITF 2.1 standard: MIL-STD-2500C (01 May 2006)
//   - Lidar implementation profile v1.0 (2010-09-07)
//
// There must be at least one Image segment ("IM").
// There must be at least one DES segment ("DE") named LIDARA.
//
// You could have multiple image segments and LIDARA segments, but
// the standard doesn't seem to say anything about how you associate which
// image segment(s) with which LIDARA segment(s). We will assume only
// one image segment and only one LIDARA segment.
//

// return the IID1 field as a string
static std::string getSegmentIdentifier(NITFFile* psFile, NITFSegmentInfo* psSegInfo)
{
    vsi_l_offset curr = VSIFTellL(psFile->fp);

    VSIFSeekL(psFile->fp, psSegInfo->nSegmentHeaderStart + 2, SEEK_SET);
    char p[11];
    if (VSIFReadL(p, 1, 10, psFile->fp) != 10)
    {
        throw pdal_error("error reading nitf");
    }
    p[10] = 0;
    std::string s = p;
    
    VSIFSeekL(psFile->fp, curr, SEEK_SET);
    return s;
}


static int findIMSegment(NITFFile* psFile)
{
    // as per 3.2.3 (pag 19) and 3.2.4 (page 39)

    int iSegment(0);
    NITFSegmentInfo *psSegInfo = NULL;
    for(iSegment = 0;  iSegment < psFile->nSegmentCount; iSegment++ )
    {
        psSegInfo = psFile->pasSegmentInfo + iSegment;

        if (strncmp(psSegInfo->szSegmentType,"IM",2)==0)
        {
            const std::string iid1 = getSegmentIdentifier(psFile, psSegInfo);
            // BUG: shouldn't allow "None" here!
            if (iid1 == "INTENSITY " || iid1 == "ELEVATION " || iid1 == "None      ")
            {
                return iSegment;
            }
        }
    }    

    throw pdal_error("Unable to find Image segment from NITF file");
}


static int findLIDARASegment(NITFFile* psFile)
{
    // as per 3.2.5, page 59

    int iSegment(0);
    NITFSegmentInfo *psSegInfo = NULL;
    for(iSegment = 0;  iSegment < psFile->nSegmentCount; iSegment++ )
    {
        psSegInfo = psFile->pasSegmentInfo + iSegment;
        if (strncmp(psSegInfo->szSegmentType,"DE",2)==0)
        {
            const std::string iid1 = getSegmentIdentifier(psFile, psSegInfo);
            if (iid1 == "LIDARA DES")
            {
                return iSegment;
            }
        }
    }    

    throw pdal_error("Unable to find LIDARA data extension segment from NITF file");
}


static void extractNITF(const std::string& nitf_filename, std::string& las_filename) 
{
    NITFFile    *psFile;
    psFile = NITFOpen( nitf_filename.c_str(), FALSE );
    if (psFile == NULL)
    {
        throw pdal_error("unable to open NITF file");
        //log()->get(logDEBUG) << "Unable to open " << getOptions().getValueOrThrow<std::string>("filename") << " for NITF access" << std::endl;
    }
    
    int imageSegmentNumber = findIMSegment(psFile);
    int lidarSegmentNumber = findLIDARASegment(psFile);

    //log()->get(logDEBUG) << "NITF Segment DataStart: " << psSegInfo->nSegmentStart 
    //                     << " DataSize: " << psSegInfo->nSegmentSize << std::endl;


    NITFDES *psDES = NULL;
    psDES = NITFDESAccess( psFile, lidarSegmentNumber );
    if( psDES == NULL )
    {
        std::string msg("NITFDESAccess(%d) failed!");
        throw pdal_error(msg);
    }
    
    std::string tempfile = Utils::generate_tempfile();
    //log()->get(logDEBUG) << "Using " << tempfile << " for NITF->LAS filename" << std::endl;
    
    if (extractLASFromNITF(psDES, tempfile.c_str()))
    {
        las_filename = std::string(tempfile);
    }

    NITFDESDeaccess(psDES);
    
    NITFClose(psFile);

    return;
}


// ==========================================================================


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_nitfFilename(options.getValueOrThrow<std::string>("filename"))
    , m_lasFilename("")
    , m_lasReader(NULL)
{
    addDefaultDimensions();
    return;
}


Reader::~Reader()
{
    delete m_lasReader;
}


void Reader::addDefaultDimensions()
{
    //Dimension x("X", dimension::Float, 8);
    //x.setUUID("9b6a21e7-6ace-45a9-8c66-d9031d07576a");
    //Dimension y("Y", dimension::Float, 8);
    //y.setUUID("2f820b5d-9ad4-46e9-8be8-3cc15c8f9778");
    //Dimension z("Z", dimension::Float, 8);
    //z.setUUID("0de362b2-a039-42f2-9287-85964394a22e");
    //Dimension t("Time", dimension::UnsignedInteger, 8);
    //t.setUUID("9b705441-3de6-4b16-b706-ebae22bedeb5");

    //addDefaultDimension(x, getName());
    //addDefaultDimension(y, getName());
    //addDefaultDimension(z, getName());
    //addDefaultDimension(t, getName());
}


void Reader::initialize()
{
    pdal::Reader::initialize();

    extractNITF(m_nitfFilename, m_lasFilename);

    Options opts;
    Option opt("filename",m_lasFilename);
    opts.add(opt);

    m_lasReader = new pdal::drivers::las::Reader(opts);
    m_lasReader->initialize();

    setCoreProperties(*m_lasReader);

    return;
}


const Options Reader::getDefaultOptions() const
{
    Options options;
    return options;
}


pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    pdal::StageSequentialIterator* lasIter = m_lasReader->createSequentialIterator(buffer);
    return lasIter;
}


boost::uint32_t Reader::processBuffer(PointBuffer& data, boost::uint64_t index) const
{
    return 0;
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    // add stuff here specific to this stage type

    return tree;
}



// == Iterators =============================================================

namespace iterators { namespace sequential {


Reader::Reader(const pdal::drivers::nitf::Reader& reader, PointBuffer& buffer)
    : pdal::ReaderSequentialIterator(reader, buffer)
    , m_reader(reader)
{
    return;
}


boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{
     return 0;
}


bool Reader::atEndImpl() const
{
     return false;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    return 0;
}


} } // iterators::sequential



} } } // namespaces
#endif
