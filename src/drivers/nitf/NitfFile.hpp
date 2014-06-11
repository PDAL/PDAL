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

#ifndef INCLUDED_DRIVERS_NITF_FILE_HPP
#define INCLUDED_DRIVERS_NITF_FILE_HPP

#include <pdal/pdal_internal.hpp>

#ifdef PDAL_HAVE_GDAL

#include <vector>

#include "nitflib.h"

namespace pdal
{
    class MetadataNode;
}


namespace pdal { namespace drivers { namespace nitf {

//
// all the processing that is NITF-file specific goes in here
//
class PDAL_DLL NitfFile
{
public:
    NitfFile(const std::string& filename);
    ~NitfFile();

    void open();
    void close();

    void getLasPosition(boost::uint64_t& offset, boost::uint64_t& length) const;

    void extractMetadata(MetadataNode& metadata);

private:
    std::string getSegmentIdentifier(NITFSegmentInfo* psSegInfo);
    std::string getDESVER(NITFSegmentInfo* psSegInfo);
    int findIMSegment();
    int findLIDARASegment();

    static void processTREs(int nTREBytes, const char *pszTREData,
        MetadataNode& m, const std::string& parentkey);
    static void processTREs_DES(NITFDES*, MetadataNode& m,
        const std::string& parentkey);
    static void processMetadata(char** papszMetadata, MetadataNode& m,
        const std::string& parentkey);
    void processImageInfo(MetadataNode& m, const std::string& parentkey);

    const std::string m_filename;
    NITFFile* m_file;
    int m_imageSegmentNumber;
    int m_lidarSegmentNumber;

    NitfFile(const NitfFile&); // nope
    NitfFile& operator=(const NitfFile&); // nope
};


} } } // namespaces

#endif

#endif
