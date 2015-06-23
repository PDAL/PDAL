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

#include <las/LasReader.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{


class PDAL_DLL NitfReader : public LasReader
{
typedef boost::iostreams::restriction<std::istream> RDevice;
typedef boost::iostreams::stream<RDevice> RStream;

public:
    NitfReader() : LasReader(), m_offset(0), m_length(0), m_istream(NULL)
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

protected:
    virtual std::istream *createStream()
    {
        m_istream = FileUtils::openFile(m_filename);
        if (!m_istream)
            return NULL;
        m_rdevice.reset(new RDevice(*m_istream, m_offset, m_length));
        m_rstream.reset(new RStream(*m_rdevice));
        return m_rstream.get();
    }

    virtual void destroyStream()
    {
        m_rstream.reset();
        m_rdevice.reset();
        FileUtils::closeFile(m_istream);
        m_istream = NULL;
    }

private:
    uint64_t m_offset;
    uint64_t m_length;

    std::istream *m_istream;
    std::unique_ptr<RDevice> m_rdevice;
    std::unique_ptr<RStream> m_rstream;

    virtual void initialize();
    virtual void ready(PointTableRef table);
    NitfReader& operator=(const NitfReader&); // not implemented
    NitfReader(const NitfReader&); // not implemented
};

} // namespace pdal
