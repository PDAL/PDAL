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
#include <pdal/StreamFactory.hpp>
#include <pdal/drivers/las/Reader.hpp>

// local
#include "NitfFile.hpp"
#include "nitflib.h"

// gdal
#include "gdal.h"
#include "cpl_vsi.h"
#include "cpl_conv.h"
#include "cpl_string.h"


namespace pdal
{
namespace drivers
{
namespace nitf
{


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
// We don't support LIDARA segments that are split into multiple DES segments
// via the DES INDEX mechanism.
//
// Metadata: we store...
//    - the file header fields    - namespace <root>.FH.fieldname
//    - the file header TREs      - namespace <root>.FH.TRE.TREname
//    - the IM segment fields     - namespace <root>.IM.1.fieldname
//    - the IM segment TREs       - namespace <root>.IM.1.TRE.TREname
//    - the DES fields            - namespace <root>.DE.1.fieldname
//    - the DES TREs              - namespace <root>.DE.1.fieldname
// Note we use a number to indicate which segment is being used,
// so there is no ambiuity with multisegment NITFs
//
// We also store some basic info for the IM segment: pixel width,
// pixel height, and number of bands.
//    BUG: this is commented out right now (see processImageInfo()),
//         because NITFImageDeaccess() leaks memory?
//
// We do not parse out the TRE fields; we leave the data as a byte stream
// (This would be easy enough to do later, at least for those TREs we have documentation on).
//
// The dimensions we write out are (precisely) the LAS dimensions; we use the same
// names, so as not to require upstream stgaes to understand both LAS dimension
// names and NITF dimension names.
//
// BUG: we should provide an option to set the SRS of the Stage using the IGEOLO
// field, but the GDAL C API doesn't provide an easy way to get the SRS. (When we
// add this option, the default will be to use NITF.)
//
// Need to test on all the other NITF LAS files
//
// BUG: need to implement addDefaultDimensions() so it does what LAS does.
//
// BUG: findIMSegment() allows "None" as an image type (for now, just to
// support the autzen test input)
//
// BUG: fix the reader test that asserts 80 pieces of metadata coming out

// ==========================================================================


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_filename(options.getValueOrThrow<std::string>("filename"))
    , m_streamFactory(NULL)
    , m_lasReader(NULL)
{
    addDefaultDimensions();
    return;
}


Reader::~Reader()
{
    delete m_streamFactory;
    delete m_lasReader;
}


void Reader::addDefaultDimensions()
{
    // BUG: implement this to inherit from LAS

    return;
}


void Reader::initialize()
{
    pdal::Reader::initialize();

    boost::uint64_t offset, length;

    Metadata nitf_metadata(getName());
    {
        NitfFile nitf(m_filename);
        nitf.open();

        nitf.getLasPosition(offset, length);

        nitf.extractMetadata(nitf_metadata);

        nitf.close();
    }

    m_streamFactory = new FilenameSubsetStreamFactory(m_filename, offset, length);

    m_lasReader = new pdal::drivers::las::Reader(m_streamFactory);
    m_lasReader->initialize();


    Metadata LAS = m_lasReader->getMetadata();

    nitf_metadata.addMetadata(LAS);
    Metadata& ref = getMetadataRef();
    ref = nitf_metadata;

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
    return m_lasReader->createSequentialIterator(buffer);
}

pdal::StageRandomIterator* Reader::createRandomIterator(PointBuffer& buffer) const
{
    return m_lasReader->createRandomIterator(buffer);
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    // add stuff here specific to this stage type

    return tree;
}


}
}
} // namespaces
#endif
