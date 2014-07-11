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
#include <pdal/ReaderIterator.hpp>

#include <pdal/StreamFactory.hpp>

#include <pdal/drivers/las/Support.hpp>
#include <pdal/drivers/las/Header.hpp>

#include <boost/scoped_ptr.hpp>

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

    Reader(const Options&);
    Reader(const std::string&);

    static Options getDefaultOptions();
    StreamFactory& getStreamFactory() const;

    pdal::StageSequentialIterator* createSequentialIterator() const;

    const LasHeader& getLasHeader() const
        { return m_lasHeader; }
    point_count_t getNumPoints() const
        { return m_lasHeader.GetPointRecordsCount(); }

protected:
    typedef std::unique_ptr<StreamFactory> StreamFactoryPtr;
    LasHeader& getLasHeaderRef()
    {
        return m_lasHeader;
    }

private:
    StreamFactoryPtr m_streamFactory;
    LasHeader m_lasHeader;
    std::string m_filename;

    virtual void initialize();
    virtual void initialize(MetadataNode& m);
    virtual void buildSchema(Schema *schema);
    void extractMetadata(MetadataNode& m);
    virtual void processOptions(const Options& options);
    virtual StreamFactoryPtr createFactory() const
        { return StreamFactoryPtr(new FilenameStreamFactory(m_filename)); }

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented
};


namespace iterators
{

class Base
{
public:
    Base(pdal::drivers::las::Reader const& reader);
    ~Base();

protected:
    point_count_t processBuffer(PointBuffer& PointBuffer,
        std::istream& stream, point_count_t count, LASunzipper* unzipper,
        ZipPoint* zipPoint, PointDimensions* dimensions);

    Bounds<double> m_bounds;
    const pdal::drivers::las::Reader& m_reader;
    std::istream& m_istream;
#ifdef PDAL_HAVE_LASZIP
    boost::scoped_ptr<ZipPoint> m_zipPoint;
    boost::scoped_ptr<LASunzipper> m_unzipper;
#else
    void* m_zipPoint;
    void* m_unzipper;
#endif

private:
    Base& operator=(Base const&);
    Base(Base const&); // not implemented

    void initialize();
    void loadPoint(PointBuffer& data, PointDimensions *dimensions,
        char *buf, size_t bufsize);
};

namespace sequential
{

class Reader : public Base, public pdal::ReaderSequentialIterator
{
public:
    Reader(const pdal::drivers::las::Reader& reader);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    point_count_t readImpl(PointBuffer&, point_count_t count);
    bool atEndImpl() const
        { return getIndex() >= m_reader.getNumPoints(); }
};

} // sequential

} // iterators

} // namespace las
} // namespace drivers
} // namespace pdal

