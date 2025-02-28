/*
Based on https://gist.github.com/asford/544323a5da7dddad2c9174490eb5ed06

Original license text
---------------------

This component utilizes components derived from cctbx, available at
http://cci.lbl.gov/cctbx_sources/boost_adaptbx/python_streambuf.h

*** License agreement ***

cctbx Copyright (c) 2006, The Regents of the University of
California, through Lawrence Berkeley National Laboratory (subject to
receipt of any required approvals from the U.S. Dept. of Energy).  All
rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

(1) Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

(2) Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

(3) Neither the name of the University of California, Lawrence Berkeley
National Laboratory, U.S. Dept. of Energy nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

You are under no obligation whatsoever to provide any bug fixes,
patches, or upgrades to the features, functionality or performance of
the source code ("Enhancements") to anyone; however, if you choose to
make your Enhancements available either publicly, or directly to
Lawrence Berkeley National Laboratory, without imposing a separate
written license agreement for such Enhancements, then you hereby grant
the following license: a  non-exclusive, royalty-free perpetual license
to install, use, modify, prepare derivative works, incorporate into
other computer software, distribute, and sublicense such enhancements or
derivative works thereof, in binary and source code form.

*/

#pragma once

#include <iostream>
#include <streambuf>

#include <cpl_vsi.h>
#include <cpl_vsi_virtual.h>

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

// Based on https://github.com/CadQuery/OCP/blob/master/pystreambuf.h with the
// license as above
class VSIStreamBuffer : public std::streambuf
{
public:
    static const int defaultBufferSize = 4096;

    typedef std::streambuf::char_type char_type;
    typedef std::streambuf::int_type int_type;
    typedef std::streambuf::pos_type pos_type;
    typedef std::streambuf::off_type off_type;
    typedef std::streambuf::traits_type traits_type;

    VSIStreamBuffer(std::string filename, std::ios_base::openmode mode,
                    std::size_t siz = 0)
        : fname(filename), nBufferSize(siz != 0 ? siz : defaultBufferSize),
          nReadBufferEndInVSIFile(0), nWriteBufferEndInVSIFile(nBufferSize),
          pFarthestPptr(0)
    {
        std::stringstream ss;

        if ((mode & std::ios_base::in) == std::ios::in)
        {
            ss << "r";
            readBuffer.resize(nBufferSize);
            setg(nullptr, nullptr, nullptr);
        }

        if ((mode & std::ios_base::out) == std::ios::out)
        {
            ss << "w";
            writeBuffer.resize(nBufferSize);
            setp(writeBuffer.data(), writeBuffer.data() + nBufferSize);
            pFarthestPptr = pptr();
        }

        if ((mode & std::ios::binary) == std::ios::binary)
            ss << "b";

        fp.reset(reinterpret_cast<VSIVirtualHandle*>(
            VSIFOpenL(filename.c_str(), ss.str().c_str())));
    }

    ~VSIStreamBuffer()
    {
        // if data to write then sync
        if (pptr())
        {
            sync();
        }
    }

    /// C.f. C++ standard section 27.5.2.4.3
    /** It is essential to override this virtual function for the stream
        member function readsome to work correctly (c.f. 27.6.1.3, alinea 30)
     */
    virtual std::streamsize showmanyc()
    {
        int_type const failure = traits_type::eof();
        int_type status = underflow();
        if (status == failure)
            return -1;
        return egptr() - gptr();
    }

    /// C.f. C++ standard section 27.5.2.4.3
    virtual int_type underflow()
    {
        int_type const failure = traits_type::eof();

        off_type nRead = (off_type)fp->Read(readBuffer.data(), 1, nBufferSize);

        nReadBufferEndInVSIFile += nRead;
        setg(readBuffer.data(), readBuffer.data(), readBuffer.data() + nRead);

        // ^^^27.5.2.3.1 (4)
        if (nRead == 0)
            return failure;

        return traits_type::to_int_type(readBuffer.data()[0]);
    }

    /// C.f. C++ standard section 27.5.2.4.5
    virtual int_type overflow(int_type c = traits_type::eof())
    {
        pFarthestPptr = (std::max)(pFarthestPptr, pptr());
        off_type nWritten = (off_type)(pFarthestPptr - pbase());

        // write current buffer
        if (nWritten)
            fp->Write(pbase(), 1, nWritten);

        if (!traits_type::eq_int_type(c, traits_type::eof()))
        {
            char cs = traits_type::to_char_type(c);
            fp->Write(&cs, 1, 1);
            nWritten++;
        }

        if (nWritten)
        {
            nWriteBufferEndInVSIFile += nWritten;
            setp(pbase(), epptr());
            // ^^^ 27.5.2.4.5 (5)
            pFarthestPptr = pptr();
        }
        return traits_type::eq_int_type(c, traits_type::eof())
                   ? traits_type::not_eof(c)
                   : c;
    }

    /// Update the VSI file to reflect the state of this stream buffer
    /** Empty the write buffer into the VSI file object and set the seek
        position of the latter accordingly (C++ standard section 27.5.2.4.2).
        If there is no write buffer or it is empty, but there is a non-empty
        read buffer, set the VSI file object seek position to the
        seek position in that read buffer.
    */
    virtual int sync()
    {
        int result = 0;
        pFarthestPptr = (std::max)(pFarthestPptr, pptr());
        if (pFarthestPptr && pFarthestPptr > pbase())
        {
            off_type delta = pptr() - pFarthestPptr;
            int_type status = overflow();

            if (traits_type::eq_int_type(status, traits_type::eof()))
                result = -1;

            if (delta > 0)
                fp->Seek(delta, SEEK_CUR);
        }
        else if (gptr() && gptr() < egptr())
        {
            fp->Seek(gptr() - egptr(), SEEK_CUR);
        }
        return result;
    }

    /// C.f. C++ standard section 27.5.2.4.2
    /** This implementation is optimised to look whether the position is within
        the buffers, so as to avoid calling VSI seek or tell.
    */
    virtual pos_type seekoff(off_type off, std::ios_base::seekdir way,
                             std::ios_base::openmode which = std::ios_base::in |
                                                             std::ios_base::out)
    {
        /* In practice, "which" is either std::ios_base::in or out
           since we end up here because either seekp or seekg was called
           on the stream using this buffer. That simplifies the code
           in a few places.
        */
        int const failure = off_type(-1);

        // we need the read buffer to contain something!
        if (which == std::ios_base::in && !gptr())
        {
            if (traits_type::eq_int_type(underflow(), traits_type::eof()))
            {
                return failure;
            }
        }

        // compute the whence parameter for VSI seek
        int whence;
        switch (way)
        {
        case std::ios_base::beg:
            whence = 0;
            break;
        case std::ios_base::cur:
            whence = 1;
            break;
        case std::ios_base::end:
            whence = 2;
            break;
        default:
            return failure;
        }

        // Let's have a go
        off_type result = static_cast<off_type>(fp->Tell());
        if (!seekWithinBuffer(off, way, which, result))
        {
            // we need to call VSI
            if (which == std::ios_base::out)
                overflow();

            if (way == std::ios_base::cur)
            {
                if (which == std::ios_base::in)
                    off -= egptr() - gptr();
                else if (which == std::ios_base::out)
                    off += pptr() - pbase();
            }

            fp->Seek(off, whence);
            result = static_cast<off_type>(fp->Tell());

            if (which == std::ios_base::in)
                underflow();
        }
        return result;
    }

    /// C.f. C++ standard section 27.5.2.4.2
    virtual pos_type seekpos(pos_type sp,
                             std::ios_base::openmode which = std::ios_base::in |
                                                             std::ios_base::out)
    {
        return VSIStreamBuffer::seekoff(sp, std::ios_base::beg, which);
    }

private:
    PDALVirtualHandleUniquePtr fp{};
    std::string fname;
    std::size_t nBufferSize;
    std::vector<char> readBuffer;
    std::vector<char> writeBuffer;
    off_type nReadBufferEndInVSIFile;
    off_type nWriteBufferEndInVSIFile;
    char* pFarthestPptr;

    bool seekWithinBuffer(off_type off, std::ios_base::seekdir way,
                          std::ios_base::openmode which, off_type& result)
    {
        // Buffer range and current position
        off_type bufBegin, bufEnd, bufCur, upperBound;
        off_type nBufferEndInVSIFile;

        if (which == std::ios_base::in)
        {
            nBufferEndInVSIFile = nReadBufferEndInVSIFile;
            bufBegin = reinterpret_cast<std::streamsize>(eback());
            bufCur = reinterpret_cast<std::streamsize>(gptr());
            bufEnd = reinterpret_cast<std::streamsize>(egptr());
            upperBound = bufEnd;
        }
        else
        {
            nBufferEndInVSIFile = nWriteBufferEndInVSIFile;
            bufBegin = reinterpret_cast<std::streamsize>(pbase());
            bufCur = reinterpret_cast<std::streamsize>(pptr());
            bufEnd = reinterpret_cast<std::streamsize>(epptr());
            pFarthestPptr = (std::max)(pFarthestPptr, pptr());
            upperBound = reinterpret_cast<std::streamsize>(pFarthestPptr) + 1;
        }

        // Sought position in "buffer coordinate"
        off_type bufSought;
        if (way == std::ios_base::cur)
        {
            bufSought = bufCur + off;
        }
        else if (way == std::ios_base::beg)
        {
            bufSought = bufEnd + (off - nBufferEndInVSIFile);
        }
        else
        {
            // end
            return false;
        }

        // if the sought position is not in the buffer, give up
        if (bufSought < bufBegin || bufSought >= upperBound)
            return false;

        if (which == std::ios_base::in)
            gbump(bufSought - bufCur);
        else
            pbump(bufSought - bufCur);

        result = nBufferEndInVSIFile + (bufSought - bufEnd);
        return true;
    }
};

class VSIStream : public std::iostream
{
private:
    ::pdal::VSI::VSIStreamBuffer vsiBuf;

public:
    VSIStream(const std::string& filename, std::ios::openmode mode)
        : vsiBuf(filename, mode)
    {
        rdbuf(&vsiBuf);
    }
};
} // namespace VSI
} // namespace pdal