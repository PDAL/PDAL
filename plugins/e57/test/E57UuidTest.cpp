/******************************************************************************
 * Copyright (c) 2019, Helix Re Inc.
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
 *     * Neither the name of Helix Re Inc. nor the
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

#include <gtest/gtest.h>

#include "plugins/e57/io/Uuid.hpp"

using namespace pdal::uuidGenerator;

std::vector<std::string> split(const std::string& s, char c)
{
    std::string::size_type i = 0;
    std::string::size_type j = s.find(c);
    std::vector<std::string> v;
    while (j != std::string::npos)
    {
        v.push_back(s.substr(i, j - i));
        i = ++j;
        j = s.find(c, j);

        if (j == std::string::npos)
            v.push_back(s.substr(i, s.length()));
    }
    return v;
}

TEST(UUID, uuidGenerate)
{
    std::string id = generate_uuid();
    ASSERT_EQ(id.size(), (unsigned long)36);

    std::vector<std::string> components = split(id, '-');
    ASSERT_EQ(components.size(), (unsigned long)5);
    ASSERT_EQ(components[0].size(), (unsigned long)8);
    ASSERT_EQ(components[1].size(), (unsigned long)4);
    ASSERT_EQ(components[2].size(), (unsigned long)4);
    ASSERT_EQ(components[3].size(), (unsigned long)4);
    ASSERT_EQ(components[4].size(), (unsigned long)12);
}