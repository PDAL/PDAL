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

#include <memory>

#include <pdal/pdal_export.hpp>
#include <pdal/pdal_features.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

namespace pdal
{

namespace las
{
    struct Header;
    struct Vlr;
    using VlrList = std::vector<Vlr>;
};

class NitfReader;
class LeExtractor;
class PointDimensions;
class LazPerfVlrDecompressor;
class LasHeader;

class PDAL_DLL LasReader : public Reader, public Streamable
{
    friend class LasTester;

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
    LasReader& operator=(const LasReader&) = delete;
    LasReader(const LasReader&) = delete;

    std::string getName() const;

    const LasHeader& header() const;
    uint64_t vlrData(const std::string& userId, uint16_t recordId, char const * & data);
    point_count_t getNumPoints() const;

protected:
    virtual void createStream();

    std::unique_ptr<LasStreamIf> m_streamIf;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table);
    virtual void initializeLocal(PointTableRef table, MetadataNode& m);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual QuickInfo inspect();
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual bool processOne(PointRef& point);
    virtual void done(PointTableRef table);
    virtual bool eof();

    void handleCompressionOption();
    void setSrs(MetadataNode& m);
    void readExtraBytesVlr();
    void extractHeaderMetadata(MetadataNode& forward, MetadataNode& m);
    void extractVlrMetadata(MetadataNode& forward, MetadataNode& m);
    void loadPoint(PointRef& point, char *buf, size_t bufsize);
    void loadPointV10(PointRef& point, char *buf, size_t bufsize);
    void loadPointV14(PointRef& point, char *buf, size_t bufsize);
    void loadExtraDims(LeExtractor& istream, PointRef& data);
    point_count_t readFileBlock(std::vector<char>& buf, point_count_t maxPoints);

    const las::Header& lasHeader() const;

    struct Options;
    struct Private;
    std::unique_ptr<Private> d;
};

} // namespace pdal
