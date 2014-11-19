/******************************************************************************
* Copyright (c) 2014, Howard Butler (howard@hobu.co)
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


#include <boost/algorithm/string.hpp>

#ifdef PDAL_HAVE_LAZPERF
#include <tlaz/common/common.hpp>
#include <tlaz/compressor.hpp>
#include <tlaz/decompressor.hpp>

#include <tlaz/encoder.hpp>
#include <tlaz/decoder.hpp>
#include <tlaz/formats.hpp>
#include <tlaz/las.hpp>
#endif

#include <pdal/Dimension.hpp>
#include <pdal/PointContext.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Compression.hpp>
#include <pdal/Charbuf.hpp>



#include <map>
#include <vector>

namespace pdal
{


namespace compression
{


namespace CompressionType
{

enum Enum
{
    None = 0,
    Ght = 1,
    Dimensional = 2,
    Lazperf = 3,
    Unknown = 256
};

} // namespace CompressionType


template <typename CompressionStream> inline void Compress(PointContextRef ctx,
              const PointBuffer& buffer,
              CompressionStream& output,
              CompressionType::Enum ctype,
              PointId start,
              PointId end)
{

#ifdef PDAL_HAVE_LAZPERF
    using namespace laszip;
    using namespace laszip::formats;

    typedef encoders::arithmetic<CompressionStream> EncoderType;

    EncoderType encoder(output);
    auto compressor = make_dynamic_compressor(encoder);
    const Dimension::IdList& dims = ctx.dims();
//     std::cout << listDims(ctx) << std::endl;
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        Dimension::Type::Enum t = ctx.dimType(*di);
        size_t s = ctx.dimSize(*di);
        if (t == Dimension::Type::Signed32)
        {
            if (s == 4)
            {
                compressor->template add_field<int>();
            }
            else
                throw pdal_error("Dimension is type Signed32 but size != 4!");
        }
        else if (t == Dimension::Type::Signed16)
        {
            if (s == 2)
            {
                compressor->template add_field<short>();
            }
            else
                throw pdal_error("Dimension is type Signed16 but size != 2  !");
        }
        else if (t == Dimension::Type::Signed8)
        {
            if (s == 1)
            {
                compressor->template add_field<char>();
            }
            else
                throw pdal_error("Dimension is type Signed8 but size != 1!");
        }
        else if (t == Dimension::Type::Signed64)
        {
            if (s == 8)
            {
                compressor->template add_field<int>();
                compressor->template add_field<int>();
            }
            else
                throw pdal_error("Dimension is type Signed64 but size != 8!");
        }
        else if (t == Dimension::Type::Unsigned32)
        {
            if (s == 4)
            {
                compressor->template add_field<unsigned int>();
            }
            else
                throw pdal_error("Dimension is type Unsigned32 but size != 4!");
        }
        else if (t == Dimension::Type::Unsigned16)
        {
            if (s == 2)
            {
                compressor->template add_field<unsigned short>();
            }
            else
                throw pdal_error("Dimension is type Unsigned16 but size != 2!");
        }
        else if (t == Dimension::Type::Unsigned8)
        {
            if (s == 1)
            {
                compressor->template add_field<unsigned char>();
            }
            else
                throw pdal_error("Dimension is type Unsigned8 but size != 1!");
        }
        else if (t == Dimension::Type::Unsigned64)
        {
            if (s == 8)
            {
                compressor->template add_field<unsigned int>();
                compressor->template add_field<unsigned int>();
            }
            else
                throw pdal_error("Dimension is type Unsigned64 but size != 8!");
        }
        else if (t == Dimension::Type::Double)
        {
            if (s == 8)
            {
                compressor->template add_field<unsigned int>();
                compressor->template add_field<unsigned int>();
//                 compressor->template add_field<las::gpstime>();
            }
            else
                throw pdal_error("Dimension is type Double but size != 8!");
        }
        else if (t == Dimension::Type::Float)
        {
            if (s == 4)
            {
                compressor->template add_field<int>();
            }
            else
                throw pdal_error("Dimension is type Float but size != 4!");
        }
        else
        {

            std::ostringstream oss;
            oss << "Unhandled compression for dimension of type '" << Dimension::interpretationName(t) << "' for dimension '" << Dimension::name(*di) << "'";
            throw pdal_error(oss.str());
        }

    }

    size_t pointSize = ctx.pointSize();
    if (end == 0) // Set to max
        end = buffer.size();
    std::vector<char> bytes;
    bytes.resize(pointSize * (end - start));

    const char* pos = bytes.data() + (pointSize * start);

    Charbuf buf(bytes);
    std::ostream strm(&buf);
    buffer.getBytes(strm, start, end);

    char* end_pos = bytes.data() + (pointSize * end);
    while (pos != end_pos)
    {
        compressor->compress(pos);
        pos += pointSize;
    }

    encoder.done();
#endif
}



template <typename CompressionStream> inline PointBufferPtr Decompress(PointContextRef ctx,
                                                                       CompressionStream& strm,
                                                                       size_t howMany,
                                                                       CompressionType::Enum ctype)
{
    std::vector<char> output;
#ifdef PDAL_HAVE_LAZPERF
    using namespace laszip;
    using namespace laszip::formats;

    typedef decoders::arithmetic<CompressionStream> DecoderType;


    DecoderType decoder(strm);
    auto decompressor = make_dynamic_decompressor(decoder);
    const Dimension::IdList& dims = ctx.dims();

//     std::cout << listDims(ctx) << std::endl;
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        Dimension::Type::Enum t = ctx.dimType(*di);
        size_t s = ctx.dimSize(*di);
        if (t == Dimension::Type::Signed32)
        {
            if (s == 4)
            {
                decompressor->template add_field<int>();
            }
            else
                throw pdal_error("Dimension is type Signed32 but size != 4!");
        }
        else if (t == Dimension::Type::Signed16)
        {
            if (s == 2)
            {
                decompressor->template add_field<short>();
            }
            else
                throw pdal_error("Dimension is type Signed16 but size != 2  !");
        }
        else if (t == Dimension::Type::Signed8)
        {
            if (s == 1)
            {
                decompressor->template add_field<char>();
            }
            else
                throw pdal_error("Dimension is type Signed8 but size != 1!");
        }
        else if (t == Dimension::Type::Signed64)
        {
            if (s == 8)
            {
                decompressor->template add_field<int>();
                decompressor->template add_field<int>();
            }
            else
                throw pdal_error("Dimension is type Signed64 but size != 8!");
        }
        else if (t == Dimension::Type::Unsigned32)
        {
            if (s == 4)
            {
                decompressor->template add_field<unsigned int>();
            }
            else
                throw pdal_error("Dimension is type Unsigned32 but size != 4!");
        }
        else if (t == Dimension::Type::Unsigned16)
        {
            if (s == 2)
            {
                decompressor->template add_field<unsigned short>();
            }
            else
                throw pdal_error("Dimension is type Unsigned16 but size != 2!");
        }
        else if (t == Dimension::Type::Unsigned8)
        {
            if (s == 1)
            {
                decompressor->template add_field<unsigned char>();
            }
            else
                throw pdal_error("Dimension is type Unsigned8 but size != 1!");
        }
        else if (t == Dimension::Type::Unsigned64)
        {
            if (s == 8)
            {
                decompressor->template add_field<unsigned int>();
                decompressor->template add_field<unsigned int>();
            }

            else
                throw pdal_error("Dimension is type Unsigned64 but size != 8!");
        }
        else if (t == Dimension::Type::Double)
        {
            if (s == 8)
            {
                decompressor->template add_field<unsigned int>();
                decompressor->template add_field<unsigned int>();
//                 decompressor->template add_field<las::gpstime>();
            }
            else
                throw pdal_error("Dimension is type Double but size != 8!");
        }
        else if (t == Dimension::Type::Float)
        {
            if (s == 4)
            {
                decompressor->template add_field<int>();
            }
            else
                throw pdal_error("Dimension is type Float but size != 4!");
        }
        else
        {

            std::ostringstream oss;
            oss << "Unhandled compression for dimension of type '" << Dimension::interpretationName(t) << "' for dimension '" << Dimension::name(*di) << "'";
            throw pdal_error(oss.str());
        }

    }


    size_t point_size = ctx.pointSize();
    size_t byte_len = howMany * point_size;
    output.resize(byte_len);

    char* pos = output.data();

    char* end_pos = pos + (byte_len);
    while (pos != end_pos)
    {
        decompressor->decompress(pos);
        pos+=point_size;
    }

#endif

    Charbuf charstreambuf(output, output.size());
    std::istream istrm(&charstreambuf);

    PointBufferPtr b = PointBufferPtr(new PointBuffer(istrm, ctx, 0, howMany));
    return b;


}

} // compression
} // namespace pdal


