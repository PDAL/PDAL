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

#include <pdal/GDALUtils.hpp>
#include <pdal/Utils.hpp>
#include <boost/bind/placeholders.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif


// This is cheap hackery.  GDALUtils is the first file compiled, and 
// we want to make sure we have the correct boost before we go and 
// compile a bunch of junk.  boost::phoenix has a sub-namespace 
// called boostphoenix that gets renamed pdalboostphoenix as part of the 
// bcp extraction process.  If the user is using embedded boost, we can
// know right away based on this sub-namespace.  All of the boost 
// headers do boost=pdalboostphoenix; for a top namespace, however, so 
// merely testing that is no good.  If someone has a better sub-namespace 
// than the one out of phoenix which probably includes half of boost in the 
// process, please switch this.
#include <boost/spirit/include/phoenix_object.hpp>

#ifdef PDAL_EMBED_BOOST
// If this fails, it is because we've included a boost that is not 
// the embedded one ahead of the one in our tree.  Maybe the user 
// forgot to clean up the CMakeCache.txt or something, or it is 
// legitimately the wrong one. 
namespace pho = boost::pdalboostphoenix;
#else
namespace pho = boost::phoenix;
#endif


namespace pdal
{
namespace gdal
{

Debug::Debug(bool isDebug, pdal::LogPtr log)
: m_isDebug(isDebug)
, m_log(log)
{
    if (m_isDebug)
    {
        const char* gdal_debug = ::pdal::Utils::getenv("CPL_DEBUG");
        if (gdal_debug == 0)
        {
            pdal::Utils::putenv("CPL_DEBUG=ON");
        }        
        m_gdal_callback = boost::bind(&Debug::log, this, _1, _2, _3);
    }
    else
    {
        m_gdal_callback = boost::bind(&Debug::error, this, _1, _2, _3);
    }


#if GDAL_VERSION_MAJOR == 1 && GDAL_VERSION_MINOR >= 9
    CPLPushErrorHandlerEx(&Debug::trampoline, this);
#else
    CPLPushErrorHandler(&Debug::trampoline);
#endif    
}

void Debug::log(::CPLErr code, int num, char const* msg)
{
    std::ostringstream oss;
    
    if (code == CE_Failure || code == CE_Fatal) {
        oss <<"GDAL Failure number=" << num << ": " << msg;
        throw pdal::gdal_error(oss.str());
    } else if (code == CE_Debug) {
        oss << "GDAL debug: " << msg;
        m_log->get(logDEBUG) << oss.str() << std::endl;
        return;
    } else {
        return;
    }
}

void Debug::error(::CPLErr code, int num, char const* msg)
{
    std::ostringstream oss;
    if (code == CE_Failure || code == CE_Fatal) {
        oss <<"GDAL Failure number=" << num << ": " << msg;
        throw pdal::gdal_error(oss.str());
    } else {
        return;
    }
}

Debug::~Debug()
{
    CPLPopErrorHandler();

}


//----------------------------------------------------------------------------

#ifdef PDAL_HAVE_GDAL
VSILFileBuffer::VSILFileBuffer(VSILFILE* fp)
    : m_fp(fp)
{
    return;
}


std::streamsize VSILFileBuffer::read(char* s, std::streamsize n)
{
    // Read up to n characters from the underlying data source
    // into the buffer s, returning the number of characters
    // read; return -1 to indicate EOF
    size_t result = VSIFReadL/*fread*/(s, 1, (size_t)n, m_fp);
    if (result == 0)
        return -1;
    return result;
}


std::streamsize VSILFileBuffer::write(const char* s, std::streamsize n)
{
    // Write up to n characters from the buffer
    // s to the output sequence, returning the 
    // number of characters written
    size_t result = VSIFWriteL/*fwrite*/(s, 1, (size_t)n, m_fp);
    if (result != n)
        return -1;
    return result;
}


std::streampos VSILFileBuffer::seek(boost::iostreams::stream_offset off, std::ios_base::seekdir way) 
{
    // Advances the read/write head by off characters, 
    // returning the new position, where the offset is 
    // calculated from:
    //  - the start of the sequence if way == ios_base::beg
    //  - the current position if way == ios_base::cur
    //  - the end of the sequence if way == ios_base::end

    int myway = 0;
    if (way == std::ios_base::beg) myway = SEEK_SET;
    else if (way == std::ios_base::cur) myway = SEEK_CUR;
    else if (way == std::ios_base::end) myway = SEEK_END;

    long myoff = (long)off;
    int result = VSIFSeekL/*fseek*/(m_fp, myoff, myway);
    if (result != 0)
    {
        return -1;
    }
    return VSIFTellL/*ftell*/(m_fp);
}
#endif

}} // namespace pdal::gdal
