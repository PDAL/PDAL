/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include "Random.hpp"

namespace pdal
{
namespace Utils
{

Random::Random()
{
    std::vector<int32_t> seed;
    std::random_device rd;
    for (size_t i = 0; i < std::mt19937::state_size; ++i)
        seed.push_back(rd());
    std::seed_seq seedSeq(seed.begin(), seed.end());
    m_generator.seed(seedSeq);
}

Random::Random(int32_t seed)
{
    std::seed_seq seedSeq {seed};
    m_generator.seed(seedSeq);
}

Random::Random(const std::vector<int32_t> seed)
{
    std::seed_seq seedSeq(seed.begin(), seed.end());
    m_generator.seed(seedSeq);
}

Random::Random(const std::string& seed)
{
    std::vector<int32_t> v;
    int32_t s = 0;
    int i = 0;
    for (char c : seed)
    {
        s |= c << (8 * i++);
        if (i == 4)
        {
            v.push_back(s);
            i = 0;
            s = 0;
        }
    }
    if (i)
        v.push_back(s);
    std::seed_seq seedSeq(v.begin(), v.end());
    m_generator.seed(seedSeq);
}

std::mt19937& Random::generator()
{
    return m_generator;
}

unsigned int Random::quick()
{
    std::random_device rd;

    return rd();
}

} // namespace Utils
} // namespace pdal
