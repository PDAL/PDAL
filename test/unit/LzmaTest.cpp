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

#include <pdal/pdal_test_main.hpp>

#include <random>

#include <pdal/compression/LzmaCompression.hpp>

using namespace pdal;

TEST(Compression, lzma)
{
    std::default_random_engine generator;
    std::uniform_int_distribution<int> dist((std::numeric_limits<int>::min)());

    // Choosing a size that isn't a multiple of the internal buffer.
    // Trying to make something that compresses reasonably well.
    std::vector<int> orig(1000357);
    int val = dist(generator);
    for (size_t i = 0; i < orig.size(); ++i)
    {
        orig[i] = val++;
        if (i % 100 == 0)
            val = dist(generator);
    }

    std::vector<char> compressed;
    auto cb = [&compressed](char *buf, size_t bufsize)
    {
        static size_t total = 0;
        compressed.insert(compressed.end(), buf, buf + bufsize);
        total += bufsize;
    };

    size_t s = orig.size() * sizeof(int);
    char *sp = reinterpret_cast<char *>(orig.data());
    LzmaCompressor compressor(cb);
    compressor.compress(sp, s);
    compressor.done();

    auto verifier = [&sp](char *buf, size_t bufsize)
    {
        EXPECT_EQ(memcmp(buf, sp, bufsize), 0);
        sp += bufsize;
    };

    LzmaDecompressor decompressor(verifier);
    decompressor.decompress(compressed.data(), compressed.size());
    decompressor.done();
}

