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
#include <pdal/plugin.h>
#include <pdal/Reader.hpp>
#include <pdal/StreamFactory.hpp>

#include "LasError.hpp"
#include "LasHeader.hpp"
#include "LasUtils.hpp"
#include "ZipPoint.hpp"

extern "C" int32_t LasReader_ExitFunc();
extern "C" PF_ExitFunc LasReader_InitPlugin();

namespace pdal
{

class NitfReader;
class LasHeader;
class LeExtractor;
class PointDimensions;

class PDAL_DLL LasReader : public pdal::Reader
{
    friend class NitfReader;
public:
    LasReader() : pdal::Reader(), m_index(0),
            m_istream(NULL)
        {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

    const LasHeader& header() const
        { return m_lasHeader; }
    point_count_t getNumPoints() const
        { return m_lasHeader.pointCount(); }

private:
    LasError m_error;
    typedef std::unique_ptr<StreamFactory> StreamFactoryPtr;
    StreamFactoryPtr m_streamFactory;
    LasHeader m_lasHeader;
    std::unique_ptr<ZipPoint> m_zipPoint;
    std::unique_ptr<LASunzipper> m_unzipper;
    point_count_t m_index;
    std::istream* m_istream;
    VlrList m_vlrs;
    std::vector<ExtraDim> m_extraDims;

    virtual StreamFactoryPtr createFactory() const
        { return StreamFactoryPtr(new FilenameStreamFactory(m_filename)); }
    virtual void processOptions(const Options& options);
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    void fixupVlrs();
    VariableLengthRecord *findVlr(const std::string& userId, uint16_t recordId);
    void setSrsFromVlrs(MetadataNode& m);
    void readExtraBytesVlr();
    SpatialReference getSrsFromVlrs();
    SpatialReference getSrsFromWktVlr();
    SpatialReference getSrsFromGeotiffVlr();
    void extractHeaderMetadata(MetadataNode& m);
    void extractVlrMetadata(MetadataNode& m);
    virtual QuickInfo inspect();
    virtual void ready(PointTableRef table)
        { ready(table, m_metadata); }
    virtual void ready(PointTableRef table, MetadataNode& m);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);
    virtual bool eof()
        { return m_index >= getNumPoints(); }
    void loadPoint(PointView& data, char *buf, size_t bufsize);
    void loadPointV10(PointView& data, char *buf, size_t bufsize);
    void loadPointV14(PointView& data, char *buf, size_t bufsize);
    void loadExtraDims(LeExtractor& istream, PointView& data, PointId nextId);
    point_count_t readFileBlock(
            std::vector<char>& buf,
            point_count_t maxPoints);

    LasReader& operator=(const LasReader&); // not implemented
    LasReader(const LasReader&); // not implemented
};

} // namespace pdal
