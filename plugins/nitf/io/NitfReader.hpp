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

#pragma once

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/restrict.hpp>

#include <io/LasReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{


class PDAL_DLL NitfReader : public LasReader
{
typedef pdalboost::iostreams::restriction<std::istream> RDevice;
typedef pdalboost::iostreams::stream<RDevice> RStream;

    class NitfStreamIf : public LasStreamIf
    {
    public:
        NitfStreamIf(const std::string& filename, uint64_t offset, uint64_t len)
        {
            m_baseStream = FileUtils::openFile(filename);
            if (m_baseStream)
            {
                m_rdevice.reset(new RDevice(*m_baseStream, offset, len));
                m_rstream.reset(new RStream(*m_rdevice));
                m_istream = m_rstream.get();
            }
        }

        ~NitfStreamIf()
        {
            m_rstream.reset();
            m_rdevice.reset();
            if (m_baseStream)
                FileUtils::closeFile(m_baseStream);
            m_baseStream = NULL;
            m_istream = NULL;
        }

    private:
        std::istream *m_baseStream;
        std::unique_ptr<RDevice> m_rdevice;
        std::unique_ptr<RStream> m_rstream;
    };

public:
    NitfReader() : LasReader(), m_offset(0), m_length(0)
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

protected:
    virtual void createStream()
    {
        if (m_streamIf)
            std::cerr << "Attempt to create stream twice!\n";
        m_streamIf.reset(new NitfStreamIf(m_filename, m_offset, m_length));
    }

private:
    uint64_t m_offset;
    uint64_t m_length;

    virtual void initialize(PointTableRef table);
    NitfReader& operator=(const NitfReader&); // not implemented
    NitfReader(const NitfReader&); // not implemented
};

} // namespace pdal
