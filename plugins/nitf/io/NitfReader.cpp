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

#include "NitfFile.hpp"
#include "NitfReader.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.nitf",
    "NITF Reader",
    "http://pdal.io/stages/readers.nitf.html" );

CREATE_SHARED_PLUGIN(1, 0, NitfReader, Reader, s_info)

std::string NitfReader::getName() const { return s_info.name; }

//
// References:
//   - NITF 2.1 standard: MIL-STD-2500C (01 May 2006)
//   - Lidar implementation profile v1.0 (2010-09-07)
//
// To be a proper lidar NITF file, the file must:
//   - have at least one Image segment ("IM")
//   - have at least one DES segment ("DE") named LIDARA
//
// You could have multiple image segments and LIDARA segments, but the
// standard doesn't seem to say anything about how you associate which
// image segment(s) with which LIDARA segment(s). We will assume only
// one image segment and only one LIDARA segment.
//
// We don't support LIDARA segments that are split into multiple DES
// segments via the DES INDEX mechanism. We also don't support wierd
// things that Nitro doesn't support: NITF 1.0, streaming segments,
// and so on.
//
// For the metadata, we store:
//    - the file header fields    --> FH.field_name
//    - the file header TREs      --> FH.TRE.tre_name
//    - the IM segment fields     --> IM:0.field_name
//    - the IM segment TREs       --> IM:0.tre_name.field_name
//    - the DES fields            --> DE:0.field_name
//    - the DES TREs              --> DE:0.field_name
//
// Note we use a ":N" syntax to indicate which segment is being used,
// so there is no ambiuity with multisegment NITFs
//
// We parse out the TRE fields for those TREs that Nitro recognizes
// (see tre_plugins.cpp).
//
// The dimensions we write out are (precisely) the LAS dimensions; we
// use the same names, so as not to require upstream stgaes to
// understand both LAS dimension names and NITF dimension names.
//

//
// BUG: we should provide an option to set the SRS of the Stage using
// the IGEOLO field.
//
// BUG: need to implement addDefaultDimensions() so it does what LAS
// does.
//
// BUG: findIMSegment() allows "None" as an image type (for now, just
// to support the autzen test input)
//


void NitfReader::initialize()
{
    NitfFile nitf(m_filename);
    nitf.open();
    nitf.getLasOffset(m_offset, m_length);
    nitf.extractMetadata(m_metadata);
    m_metadata.add("DESDATA_OFFSET", m_offset);
    m_metadata.add("DESDATA_LENGTH", m_length);

    nitf.close();
    LasReader::initialize();
}


void NitfReader::ready(PointTableRef table)
{
    // Initialize the LAS stuff with its own metadata node.
    MetadataNode lasNode = m_metadata.add(LasReader::getName());
    LasReader::ready(table, lasNode);
}

} // namespace pdal
