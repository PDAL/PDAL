/******************************************************************************
* Copyright (c) 2014, Connor Manning, connor@hobu.co
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
#include <pdal/Options.hpp>
#include <pdal/Hdf5Handler.hpp>

#include <vector>

namespace pdal
{
namespace drivers
{
namespace icebridge
{

class icebridge_error : public pdal_error
{
public:
    icebridge_error(std::string const& msg) : pdal_error(msg)
    { }
};

class PDAL_DLL Reader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.icebridge.reader", "Icebridge Reader")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.icebridge.reader.html")
    SET_STAGE_ENABLED(true)

    Reader(const Options& options) : pdal::Reader(options)
        {}

    static Options getDefaultOptions();
    static Dimension::IdList getDefaultDimensions();

    StageSequentialIterator* createSequentialIterator() const;

private:
    std::string m_filename;
    Hdf5Handler m_hdf5Handler;

    virtual void processOptions(const Options& options);
    virtual void addDimensions(PointContext ctx);
    virtual void ready(PointContext ctx);
    virtual void done(PointContext ctx);

    Reader& operator=(const Reader&);   // Not implemented.
    Reader(const Reader&);              // Not implemented.
};

namespace iterators
{
namespace sequential
{

class PDAL_DLL IcebridgeSeqIter : public ReaderSequentialIterator
{
public:
    IcebridgeSeqIter(Hdf5Handler *hdf5Handler) : m_hdf5Handler(hdf5Handler)
    {}

private:
    Hdf5Handler *m_hdf5Handler;

    virtual point_count_t readImpl(PointBuffer& data, point_count_t count);
    virtual uint64_t skipImpl(boost::uint64_t);
    virtual point_count_t readBufferImpl(PointBuffer& pointBuffer);
    virtual bool atEndImpl() const;
};

} // namespace sequential
} // namespace iterators

} // namespace icebridge
} // namespace drivers
} // namespace pdal

