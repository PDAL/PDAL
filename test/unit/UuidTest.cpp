/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <pdal/util/Uuid.hpp>

using namespace pdal;

TEST(UuidTest, test)
{
    std::string s("5CE0E9A5-6015-FEC5-AADF-A328AE398115");
    unsigned char id[16] = {0x5c, 0xe0, 0xe9, 0xa5, 0x60, 0x15, 0xfe, 0xc5,
                            0xaa, 0xdf, 0xa3, 0x28, 0xae, 0x39, 0x81, 0x15 };
    Uuid uuid((char *)id);
    EXPECT_EQ(uuid.toString(), s);

    Uuid uuid2(s);
    unsigned char buf[16];
    uuid2.pack((char *)buf);
    for (size_t i = 0; i < 16; ++i)
        EXPECT_EQ(id[i], buf[i]);
}

TEST(UuidTest, initialization)
{
    Uuid uuid;

    EXPECT_TRUE(uuid.isNull());

    Uuid uuid2(std::string("foo"));
    EXPECT_TRUE(uuid2.isNull());
}
