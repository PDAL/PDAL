/******************************************************************************
* Copyright (c) 2014, Connor Manning (connor@hobu.co)
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

#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <pdal/Dimension.hpp>
#include <pdal/Compression.hpp>

#include "WebSocketClient.hpp"
#include "GreyhoundReader.hpp"
#include "CompressionStream.hpp"

namespace pdal
{
namespace exchanges
{

struct DimData
{
    DimData(Dimension::Id::Enum id, Dimension::Type::Enum type)
        : id(id)
        , type(type)
    { }

    const Dimension::Id::Enum id;
    const Dimension::Type::Enum type;
};

class Exchange : public WebSocketExchange
{
protected:
    Exchange(const std::string& command);

    bool checkStatus() const;
};

class CreateSession : public Exchange
{
public:
    CreateSession(const std::string& pipelineId);

    virtual bool check();

    std::string getSession() const;

private:
    std::string m_session;
};

class GetNumPoints : public Exchange
{
public:
    GetNumPoints(const std::string& sessionId);

    virtual bool check();

    std::size_t count() const;

private:
    std::size_t m_count;
};

class GetSchema : public Exchange
{
public:
    GetSchema(const std::string& sessionId);

    virtual bool check();


    std::vector<DimData> schema() const;

private:
    std::vector<DimData> m_dimData;
};

class Read : public Exchange
{
public:
    Read(
            PointViewPtr view,
            const PointLayoutPtr layout,
            const std::string& sessionId,
            bool compress,
            int offset,
            int count);

    virtual bool check();
    virtual bool done() = 0;
    virtual void handleRx(const message_ptr message) = 0;

    std::size_t numRead() const;

protected:
    PointViewPtr m_view;
    const PointLayoutPtr m_layout;

    bool m_initialized;
    bool m_error;
    std::size_t m_pointsToRead;
    std::size_t m_numBytes;
    std::size_t m_numBytesReceived;
    std::vector<char> m_data;
};

class ReadUncompressed : public Read
{
public:
    ReadUncompressed(
            PointViewPtr view,
            const PointLayoutPtr,
            const std::string& sessionId,
            int offset = 0,
            int count = -1);

    virtual bool done();
    virtual void handleRx(const message_ptr message);
};

#ifdef PDAL_HAVE_LAZPERF
class ReadCompressed : public Read
{
public:
    ReadCompressed(
            PointViewPtr view,
            const PointLayoutPtr,
            const std::string& sessionId,
            int offset = 0,
            int count = -1);

    virtual bool done();
    virtual void handleRx(const message_ptr message);

private:
    std::thread m_decompressionThread;

    CompressionStream m_compressionStream;

    LazPerfDecompressor<CompressionStream> m_decompressor;

    bool m_done;
    std::condition_variable m_doneCv;
    std::mutex m_doneMutex;

    std::mutex m_mutex;
};
#endif

class Destroy: public Exchange
{
public:
    Destroy(const std::string& sessionId);

    virtual bool check();
};

} // namespace exchanges
} // namespace pdal

