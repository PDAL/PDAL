/******************************************************************************
* Copyright (c) 2025, Norman Barker (norman.barker@gmail.com)
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

#include <memory>
#include <sstream>
#include <streambuf>

#include <cpl_vsi.h>
#include <cpl_vsi_virtual.h>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/VSIIO.hpp>
#include <pdal/pdal_types.hpp>


namespace pdal
{

namespace VSI
{

// In GDAL 3.11
// https://github.com/OSGeo/gdal/blob/64e4d4bf2c972098160b2af0b34b90196e2144ff/port/cpl_vsi_virtual.h#L147C1-L158C3
struct PDALVirtualHandleCloser
{
    /** Operator () that closes and deletes the file handle. */
    void operator()(VSIVirtualHandle* poHandle)
    {
        if (poHandle)
        {
            poHandle->Close();
            delete poHandle;
        }
    }
};

typedef std::unique_ptr<VSIVirtualHandle, PDALVirtualHandleCloser>
    PDALVirtualHandleUniquePtr;

typedef std::streambuf::char_type char_type;
typedef std::streambuf::int_type int_type;
typedef std::streambuf::pos_type pos_type;
typedef std::streambuf::off_type off_type;
typedef std::streambuf::traits_type traits_type;


class VSIStreamBuffer : public std::streambuf
{
public:
    static const int defaultBufferSize = 65536;

    VSIStreamBuffer(std::string filename, std::ios_base::openmode mode,
                    std::size_t bufferSize = 0);
    ~VSIStreamBuffer();

    virtual std::streamsize showmanyc();
    virtual int_type underflow();
    virtual int_type overflow(int_type c = traits_type::eof());
    virtual int sync();
    virtual pos_type seekoff(off_type off, std::ios_base::seekdir way,
                             std::ios_base::openmode which = std::ios_base::in |
                                                             std::ios_base::out);
    virtual pos_type seekpos(pos_type sp,
                             std::ios_base::openmode which = std::ios_base::in |
                                                             std::ios_base::out);
private:
    PDALVirtualHandleUniquePtr fp{};
    std::string fname;
    std::size_t nBufferSize;
    std::streampos nBufferStart;
    std::vector<char> buffer;
};


VSIStreamBuffer::VSIStreamBuffer(std::string filename, std::ios_base::openmode mode,
    std::size_t bufferSize)
    : fname(filename),
        nBufferSize(bufferSize != 0 ? bufferSize : defaultBufferSize),
        nBufferStart(0)
{
    std::string fm;
    buffer.resize(nBufferSize);

    if ((mode & std::ios_base::in) && (mode & std::ios_base::out))
    {
        fm = "r+";
        // FileUtils::OpenExisting returns an ostream
        setp(buffer.data(), buffer.data() + nBufferSize);
    }
    else
    {
        if (mode & std::ios_base::in)
        {
            fm = "r";
            setg(nullptr, nullptr, nullptr);
        }

        if (mode & std::ios_base::out)
        {
            fm = "w";
            setp(buffer.data(), buffer.data() + nBufferSize);
        }
    }

    if (mode & std::ios::binary)
        fm += "b";

    fp.reset(reinterpret_cast<VSIVirtualHandle*>(
        VSIFOpenL(filename.c_str(), fm.c_str())));

    // can either set a bad bit in underflow/overflow or throw a PDAL error here
    if (fp == nullptr)
        throw pdal_error("Can't open file '" + filename + "'.");
}


VSIStreamBuffer::~VSIStreamBuffer()
{
    // if data to write then sync
    if (pptr() - pbase())
        sync();
}


std::streamsize VSIStreamBuffer::showmanyc()
{
    if (underflow() == traits_type::eof())
        return -1;
    return egptr() - gptr();
}


int_type VSIStreamBuffer::underflow()
{
    // update nBufferStart to current file position
    nBufferStart = (fp->Tell() / nBufferSize) * nBufferSize;
    size_t nRead = fp->Read(buffer.data(), 1, nBufferSize);
    if (nRead == 0)
        return traits_type::eof();

    setg(buffer.data(), buffer.data(), buffer.data() + nRead);
    return traits_type::not_eof(buffer.data()[0]);
}


int_type VSIStreamBuffer::overflow(int_type c)
{
    if (!traits_type::eq_int_type(c, traits_type::eof()))
    {
        // write and reset buffer
        size_t nWritten = fp->Write(buffer.data(), 1, pptr() - pbase());
        if (nWritten > 0)
        {
            nBufferStart += nWritten;
            setp(buffer.data(), buffer.data() + nBufferSize);
            *pptr() = c;
            pbump(1);
            return traits_type::not_eof(c);
        }
    }
    return traits_type::eof();
}


int VSIStreamBuffer::sync()
{
    if (pptr())
    {
        // write current buffer
        size_t nWritten = fp->Write(buffer.data(), 1, pptr() - pbase());
        if (nWritten > 0)
        {
            nBufferStart += nWritten;
            setp(buffer.data(), buffer.data() + nBufferSize);
        }
    }
    else
    {
        // refresh buffer
        size_t nRead = fp->Read(buffer.data(), 1, nBufferSize);
        if (nRead > 0)
        {
            nBufferStart += nRead;
            setg(buffer.data(), buffer.data(), buffer.data() + nRead);
        }
    }
    return 0;
}


pos_type VSIStreamBuffer::seekoff(off_type off,
    std::ios_base::seekdir way,
    std::ios_base::openmode which)
{
    if (way == std::ios_base::cur)
    {
        if (off == 0)
        {
            // used by tellg / tellp / seekg / seekp so exit early
            if (which & std::ios_base::out)
                return nBufferStart + std::streampos(pptr() - pbase());
            else
                return nBufferStart + std::streampos(gptr() - eback());
        }
        else
        {
            if (which & std::ios_base::out)
                off = nBufferStart + std::streampos(pptr() - pbase()) + off;
            else
                off = nBufferStart + std::streampos(gptr() - eback()) + off;
        }
    }
    else if (way == std::ios_base::end)
    {
        // delegate to VSI for backends that know the file size
        vsi_l_offset currOffset = fp->Tell();
        fp->Seek(off, SEEK_END);
        off = fp->Tell();
        fp->Seek(currOffset, SEEK_SET);
    }

    std::streampos nChunkPos = (off / nBufferSize) * nBufferSize;
    std::streampos nOffsetInBuffer = off % nBufferSize;

    if (which & std::ios_base::in)
    {
        if (nChunkPos != nBufferStart)
        {
            // reset buffers
            nBufferStart = nChunkPos;
            setg(nullptr, nullptr, nullptr);
            if ((fp->Seek(nBufferStart, SEEK_SET) == 0) && (underflow() != -1))
                gbump((int)nOffsetInBuffer);
            else
                return -1;
            return static_cast<off_type>(nBufferStart + nOffsetInBuffer);
        }
        else
        {
            if ((gptr() == nullptr) && (underflow() == -1))
                return -1;

            if (nOffsetInBuffer > (egptr() - eback()))
                return -1;
            else
                gbump((int)(nOffsetInBuffer - std::streampos(gptr() - eback())));

            return static_cast<off_type>(nBufferStart + std::streampos(gptr() - eback()));
        }
    }
    else
    {
        if (sync() == 0)
        {
            fp->Seek(off, SEEK_SET);
            nBufferStart = off;
            setp(buffer.data(), buffer.data() + nBufferSize);
            return off;
        }
        else
            return -1;
    }
}


pos_type VSIStreamBuffer::seekpos(pos_type sp,
    std::ios_base::openmode which)
{
    return VSIStreamBuffer::seekoff(sp, std::ios_base::beg, which);
}


VSIOStream::VSIOStream(const std::string& filename, std::ios::openmode mode, std::size_t bufferSize) :
    std::ofstream()
{
    pVsiBuf.reset(new VSIStreamBuffer(filename, mode, bufferSize));
    basic_ios<char>::rdbuf(pVsiBuf.get());
}


VSIOStream::~VSIOStream(){}


VSIIStream::VSIIStream(const std::string& filename, std::ios::openmode mode, std::size_t bufferSize) :
    std::ifstream()
{
    pVsiBuf.reset(new VSIStreamBuffer(filename, mode, bufferSize));
    basic_ios<char>::rdbuf(pVsiBuf.get());
}


VSIIStream::~VSIIStream(){}


} // namespace VSI

} // namespace pdal
