/************************************************************************
 * Copyright (c) 2012, CARIS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of CARIS nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/

#include "Utils.hpp"

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include "boost_range_algorithm_count.hpp"
#include <boost/filesystem.hpp>
#include <boost/filesystem/detail/utf8_codecvt_facet.hpp>

namespace csar
{
namespace utils
{

//************************************************************************
//! convert a caris_type to a pdal::dimension::Interpretation
/*!
\param in_type
    \li caris_type to convert
\return
    \li pdal::dimension::Interpretation of \e in_type
*/
//************************************************************************
pdal::dimension::Interpretation carisTypeToInterpretation(caris_type in_type)
{
    using namespace pdal::dimension;

    switch (in_type)
    {
        case CARIS_TYPE_FLOAT32:
        case CARIS_TYPE_FLOAT64:
            return Float;
        case CARIS_TYPE_INT8:
            return SignedByte; // or SignedInteger
        case CARIS_TYPE_INT16:
        case CARIS_TYPE_INT32:
        case CARIS_TYPE_INT64:
            return SignedInteger;
        case CARIS_TYPE_UINT8:
            return UnsignedByte; // or UnsignedInteger
        case CARIS_TYPE_UINT16:
        case CARIS_TYPE_UINT32:
        case CARIS_TYPE_UINT64:
            return UnsignedInteger;
        default:
            assert(false && "Invalid caris_type");
            return Undefined;
    }
}

//************************************************************************
//! get the size of a caris_type
/*!
\param in_type
    \li caris_type to convert
\return
    \li number of bytes of an element of type \e in_type
*/
//************************************************************************
pdal::dimension::size_type carisTypeToSize(caris_type in_type)
{
    using namespace pdal::dimension;

    switch (in_type)
    {
        case CARIS_TYPE_INT8:
        case CARIS_TYPE_UINT8:
            return 1;
        case CARIS_TYPE_INT16:
        case CARIS_TYPE_UINT16:
            return 2;
        case CARIS_TYPE_FLOAT32:
        case CARIS_TYPE_INT32:
        case CARIS_TYPE_UINT32:
            return 4;
        case CARIS_TYPE_FLOAT64:
        case CARIS_TYPE_INT64:
        case CARIS_TYPE_UINT64:
            return 8;
        default:
            assert(false && "Invalid caris_type");
            return 0;
    }
}

//************************************************************************
//! convert a filesystem path to a URI
/*!
\param in_path
    \li filesystem path
\return
    \li Absolute URI to \e in_path in utf-8
*/
//************************************************************************
std::string systemPathToURI(std::string const& in_path)
{
    boost::filesystem::path fsPath = boost::filesystem::absolute(in_path);
    std::string path = fsPath.string(boost::filesystem::detail::utf8_codecvt_facet());

#ifdef WIN32
    // convert /'s and add inital /
    path = "/" + path;
    boost::replace_all(path, "\\", "/");
#endif

    const char validChars[] =
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "0123456789"
        "-._~"
        "!$&'()*+,;=:@/";

    std::string uri = "file://";

    BOOST_FOREACH(char c, path)
    {
        if (boost::range::count(validChars, c))
        {
            uri += c;
        }
        else
        {
            // percent encode
            struct
            {
                char operator()(uint8_t in_v)
                {
                    return (in_v < 10)
                           ? (in_v + '0')
                           : (in_v - 10 + 'A');
                }
            } toHexChar;

            char hexStr[4] =
            {
                '%',
                toHexChar((c & 0xf0) >> 4),
                toHexChar(c & 0x0f),
                '\0'
            };

            uri += hexStr;
        }
    }

    return uri;
}

}
}
