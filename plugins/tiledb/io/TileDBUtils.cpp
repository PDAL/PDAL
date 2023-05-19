/******************************************************************************
 * Copyright (c) 2023 TileDB, Inc.
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

#include "TileDBUtils.hpp"

namespace pdal
{

void DomainBounds::parsePair(std::istringstream& ss, const std::string& dimName)
{
    Utils::eatwhitespace(ss);
    if (!Utils::eatcharacter(ss, '['))
        throw error("No opening '[' for '" + dimName + "' range.");

    double low{0.0};
    ss >> low;
    if (!ss.good())
        throw error("No valid minimum value for '" + dimName + "' range.");

    Utils::eatwhitespace(ss);
    if (!Utils::eatcharacter(ss, ','))
        throw error("No ',' separating minimum/maximum values on '" + dimName +
                    "'.");

    double high{0.0};
    ss >> high;
    if (!ss.good())
        throw error("No valid maximum value for '" + dimName + "' range.");

    Utils::eatwhitespace(ss);
    if (!Utils::eatcharacter(ss, ']'))
        throw error("No closing ']' for '" + dimName + "' range.");

    m_data.push_back({low, high});
}

void DomainBounds::parse(std::string input)
{
    m_data.clear();

    static thread_local Utils::IStringStreamClassicLocale ss;
    ss.str(input);

    Utils::eatwhitespace(ss);
    if (!Utils::eatcharacter(ss, '('))
        throw error("No opening '('.");
    Utils::eatwhitespace(ss);
    if (ss.peek() == ')')
        return;
    std::vector<std::string> names{"X", "Y", "Z", "GpsTime"};
    for (uint32_t index{0}; index < 4; ++index)
    {
        parsePair(ss, names[0]);
        Utils::eatwhitespace(ss);
        auto nextChar = ss.get();
        if (nextChar == ')')
            return;
        if (index == 3)
            throw error("Can only add 4 ranges to the bounding box.");
        if (nextChar != ',')
            throw error("No comma separating '" + names[index] + "' and '" +
                        names[index + 1] + "' dimensions.");
    }
    throw error("No closing ')' for the bounding box.");
}

std::istream& operator>>(std::istream& in, DomainBounds& bounds)
{
    std::string boundsLine;
    std::getline(in, boundsLine);
    bounds.parse(boundsLine);
    return in;
}

std::ostream& operator<<(std::ostream& out, const DomainBounds& bounds)
{
    if (bounds.empty())
    {
        out << "()";
        return out;
    }

    Utils::StringStreamClassicLocale ss;
    ss.precision(16);
    ss << "(";
    auto ndim = bounds.ndim();
    if (ndim >= 1)
        ss << "[" << bounds.minX() << ", " << bounds.maxX() << "]";
    if (ndim >= 2)
        ss << ", [" << bounds.minY() << ", " << bounds.maxY() << "]";
    if (ndim >= 3)
        ss << ", [" << bounds.minZ() << ", " << bounds.maxZ() << "]";
    if (ndim >= 4)
        ss << ", [" << bounds.minGpsTime() << ", " << bounds.maxGpsTime()
           << "]";
    ss << ")";
    out << ss.str();
    return out;
}

} // namespace pdal
