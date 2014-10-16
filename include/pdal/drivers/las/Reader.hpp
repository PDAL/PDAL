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

#include <pdal/Reader.hpp>

#include <pdal/StreamFactory.hpp>
#include <pdal/drivers/las/Header.hpp>
#include <pdal/drivers/las/ZipPoint.hpp>

namespace pdal
{
namespace drivers
{

namespace nitf
{
    class NitfReader;
}

namespace las
{

class LasHeader;
class PointDimensions;

class PDAL_DLL Reader : public pdal::Reader
{
    friend class nitf::NitfReader;
public:
    SET_STAGE_NAME("drivers.las.reader", "Las Reader")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.las.reader.html")
    SET_STAGE_ENABLED(true)

    Reader() : pdal::Reader(), m_index(0),
            m_istream(NULL)
        {}

    static Options getDefaultOptions();

    const LasHeader& header() const
        { return m_lasHeader; }
    point_count_t getNumPoints() const
        { return m_lasHeader.pointCount(); }

private:
    typedef std::unique_ptr<StreamFactory> StreamFactoryPtr;
    StreamFactoryPtr m_streamFactory;
    LasHeader m_lasHeader;
    std::unique_ptr<ZipPoint> m_zipPoint;
    std::unique_ptr<LASunzipper> m_unzipper;
    point_count_t m_index;
    std::istream* m_istream;
    VlrList m_vlrs;

    virtual StreamFactoryPtr createFactory() const
        { return StreamFactoryPtr(new FilenameStreamFactory(m_filename)); }
    virtual void initialize();
    virtual void addDimensions(PointContextRef ctx);
    void fixupVlrs();
    VariableLengthRecord *findVlr(const std::string& userId, uint16_t recordId);
    void setSrsFromVlrs(MetadataNode& m);
    bool setSrsFromWktVlr(MetadataNode& m);
    bool setSrsFromGeotiffVlr(MetadataNode& m);
    void extractHeaderMetadata(MetadataNode& m);
    void extractVlrMetadata(MetadataNode& m);
    virtual void ready(PointContextRef ctx)
        { ready(ctx, m_metadata); }
    virtual void ready(PointContextRef ctx, MetadataNode& m);
    virtual point_count_t read(PointBuffer& buf, point_count_t count);
    virtual void done(PointContextRef ctx);
    virtual bool eof()
        { return m_index >= getNumPoints(); }
    void loadPoint(PointBuffer& data, char *buf, size_t bufsize);
    void loadPointV10(PointBuffer& data, char *buf, size_t bufsize);
    void loadPointV14(PointBuffer& data, char *buf, size_t bufsize);

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented
};

} // namespace las
} // namespace drivers
} // namespace pdal

