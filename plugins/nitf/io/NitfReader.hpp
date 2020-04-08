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

#include <fstream>

#include <io/LasReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{


class PDAL_DLL NitfReader : public LasReader
{
    template <class CharT, class Traits = std::char_traits<CharT>>
    class Shiftbuf : public std::basic_filebuf<CharT, Traits>
    {
    public:
        typedef typename Traits::off_type off_type;
        typedef typename Traits::pos_type pos_type;
        typedef typename std::basic_filebuf<CharT, Traits> Base;

        Shiftbuf(Shiftbuf::off_type offset) : m_offset(offset)
        {}

    protected:
        virtual pos_type seekpos(Shiftbuf::pos_type sp,
            std::ios_base::openmode which =
                std::ios_base::in | std::ios_base::binary) override
        {
            pos_type p = Base::seekpos(sp + m_offset, which);
            if (p >= 0)
                p -= m_offset;
            return p;
        }

        virtual pos_type seekoff(off_type off, std::ios_base::seekdir dir,
            std::ios_base::openmode which =
                std::ios_base::in | std::ios_base::binary) override
        {
            if (dir == std::ios_base::beg)
            {
                off += m_offset;
                pos_type p = Base::seekoff(off, dir, which);
                if (p >= 0)
                    p -= m_offset;
                return p;
            }
            else if ((dir == std::ios_base::cur) ||
                     (dir == std::ios_base::end))
            {
                pos_type p = Base::seekoff(off, dir, which);
                if (p >= 0)
                    p -= m_offset;
                return p;
            }
            else
                return 0;
        }

    private:
        off_type m_offset;
    };

    template <class CharT, class Traits = std::char_traits<CharT>>
    class basic_ShiftStream : public std::basic_istream<CharT, Traits>
    {
    public:
        typedef typename Traits::off_type off_type;
        typedef typename std::basic_istream<CharT, Traits> Base;

        basic_ShiftStream(const std::string& filename, off_type offset) :
            Base(&m_buf), m_buf(offset)
        {
            Base::init(&m_buf);
            if (!m_buf.open(filename,
                    std::ios_base::in | std::ios_base::binary))
                Base::setstate(std::ios_base::failbit);
        }

        ~basic_ShiftStream()
        { m_buf.close(); }

    private:
        Shiftbuf<CharT, Traits> m_buf;
    };
    using ShiftStream = basic_ShiftStream<char>;

    class NitfStreamIf : public LasStreamIf
    {
    public:
        NitfStreamIf(const std::string& filename, ShiftStream::off_type off)
        {
            m_istream = new ShiftStream(filename, off);
            // This makes sure that the stream is positioned at the beginning
            // of the embedded (LAS/LAZ) data.
            m_istream->seekg(0);
        }

        virtual ~NitfStreamIf()
        {
            delete m_istream;

            // Important - Otherwise the base class will attempt to use in dtor.
            m_istream = nullptr;
        }
    };

public:
    NitfReader() : LasReader(), m_offset(0), m_length(0)
    {}
    NitfReader& operator=(const NitfReader&) = delete;
    NitfReader(const NitfReader&) = delete;

    std::string getName() const;

protected:
    virtual void createStream()
    {
        if (m_streamIf)
            std::cerr << "Attempt to create stream twice!\n";
        m_streamIf.reset(new NitfStreamIf(m_filename, m_offset));
    }

private:
    uint64_t m_offset;
    uint64_t m_length;

    virtual void initialize(PointTableRef table);
};

} // namespace pdal
