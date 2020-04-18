/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
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

#pragma once

#include <pdal/pdal_export.hpp>
#include <pdal/pdal_features.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

#ifdef PDAL_HAVE_LASZIP
#include <laszip/laszip_api.h>
#else
using laszip_POINTER = void *;
using laszip_point_struct = void *;
struct laszip_point;
#endif

#include "LasError.hpp"
#include "LasHeader.hpp"
#include "LasUtils.hpp"

namespace pdal
{

class NitfReader;
class LasHeader;
class LeExtractor;
class PointDimensions;
class LazPerfVlrDecompressor;

class PDAL_DLL LasReader : public Reader, public Streamable
{
protected:
    class LasStreamIf
    {
    protected:
        LasStreamIf()
        {}

    public:
        LasStreamIf(const std::string& filename)
            { m_istream = Utils::openFile(filename); }

        virtual ~LasStreamIf()
        {
            if (m_istream)
                Utils::closeFile(m_istream);
        }

        std::istream *m_istream;
    };

    friend class NitfReader;
public:
    LasReader();
    ~LasReader();

    std::string getName() const;

    const LasHeader& header() const
        { return m_header; }
    point_count_t getNumPoints() const
        { return m_header.pointCount(); }

protected:
    virtual void createStream()
    {
        if (m_streamIf)
            std::cerr << "Attempt to create stream twice!\n";
        m_streamIf.reset(new LasStreamIf(m_filename));
        if (!m_streamIf->m_istream)
        {
            std::ostringstream oss;
            oss << "Unable to open stream for '"
                << m_filename <<"' with error '" << strerror(errno) <<"'";
            throw pdal_error(oss.str());
        }
    }

    std::unique_ptr<LasStreamIf> m_streamIf;

private:
    typedef std::vector<LasUtils::IgnoreVLR> IgnoreVLRList;

    LasHeader m_header;
    laszip_POINTER m_laszip;
    laszip_point_struct *m_laszipPoint;

    LazPerfVlrDecompressor *m_decompressor;
    std::vector<char> m_decompressorBuf;
    point_count_t m_index;
    StringList m_extraDimSpec;
    std::vector<ExtraDim> m_extraDims;
    IgnoreVLRList m_ignoreVLRs;
    std::string m_compression;
    StringList m_ignoreVLROption;
    bool m_useEbVlr;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table)
        { initializeLocal(table, m_metadata); }
    virtual void initializeLocal(PointTableRef table, MetadataNode& m);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual QuickInfo inspect();
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual bool processOne(PointRef& point);
    virtual void done(PointTableRef table);
    virtual bool eof()
        { return m_index >= getNumPoints(); }

    void handleCompressionOption();
    void setSrs(MetadataNode& m);
    void readExtraBytesVlr();
    void extractHeaderMetadata(MetadataNode& forward, MetadataNode& m);
    void extractVlrMetadata(MetadataNode& forward, MetadataNode& m);
    void loadPoint(PointRef& point, laszip_point& p);
    void loadPointV10(PointRef& point, laszip_point& p);
    void loadPointV14(PointRef& point, laszip_point& p);
    void loadPoint(PointRef& point, char *buf, size_t bufsize);
    void loadPointV10(PointRef& point, char *buf, size_t bufsize);
    void loadPointV14(PointRef& point, char *buf, size_t bufsize);
    void loadExtraDims(LeExtractor& istream, PointRef& data);
    point_count_t readFileBlock(std::vector<char>& buf,
        point_count_t maxPoints);
    void handleLaszip(int result);

    LasReader& operator=(const LasReader&); // not implemented
    LasReader(const LasReader&); // not implemented
};

} // namespace pdal
