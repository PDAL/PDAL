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

#pragma once

#include <array>
#include <iostream>
#include <istream>
#include <stdexcept>
#include <vector>

#include <pdal/util/Utils.hpp>

namespace pdal
{

struct DomainBounds
{
public:
    /** Runtime error for DomainBounds. */
    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err) {}
    };

    /**
     Default constructor creates empty bounds.
     */
    DomainBounds() = default;

    /**
     Constructor for 3D bounds.

     \param minx Minimum X value.
     \param miny Minimum Y value.
     \param minz Minimum Z value.
     \param maxx Maximum X value.
     \param maxy Maximum Y value.
     \param maxz Maximum Z value.
     */
    DomainBounds(double minx, double miny, double minz, double maxx,
                 double maxy, double maxz)
        : m_data{{minx, maxx}, {miny, maxy}, {minz, maxz}}
    {
    }

    /**
     Constructor for 3D bounds.

     \param minx Minimum X value.
     \param miny Minimum Y value.
     \param minz Minimum Z value.
     \param mintm Minimum GpsTime value.
     \param maxx Maximum X value.
     \param maxy Maximum Y value.
     \param maxz Maximum Z value.
     \param maxtm Maximum GpsTime value.
     */
    DomainBounds(double minx, double miny, double minz, double mintm,
                 double maxx, double maxy, double maxz, double maxtm)
        : m_data{{minx, maxx}, {miny, maxy}, {minz, maxz}, {mintm, maxtm}}
    {
    }

    /**
      Returns if any values were set on the domain.
     */
    inline bool empty() const
    {
        return m_data.empty();
    }

    inline size_t ndim() const
    {
        return m_data.size();
    }

    /**
      Parse string and use input to update this object.

      \param s String representation of the domain bounds.
      \param pos The position the string parsing is at. On input it is the
                 location to begin parsing the domain at. On output it is the
                 position after the domain bounds.
     */
    void parse(std::string input);

    /**
      Parse a range from an input string and add it to the end of the bounds.

      \param ss Input stream to parse data from
      \param dimName Dimension name to use for error messages

     */
    void parsePair(std::istringstream& ss, const std::string& dimName);

    /** Minimum X value. */
    double minX() const
    {
        return m_data[0][0];
    }

    /** Maximum X value. */
    double maxX() const
    {
        return m_data[0][1];
    }

    /** Minimum Y value. */
    double minY() const
    {
        return m_data[1][0];
    }

    /** Maximum Y value. */
    double maxY() const
    {
        return m_data[1][1];
    }

    /** Minimum Z value. */
    double minZ() const
    {
        return m_data[2][0];
    }

    /** Maximum Z value. */
    double maxZ() const
    {
        return m_data[2][1];
    }

    /** Minimum GpsTime value. */
    double minGpsTime() const
    {
        return m_data[3][0];
    }

    /** Maximum GpsTime value. */
    double maxGpsTime() const
    {
        return m_data[3][1];
    }

    friend PDAL_DLL std::istream& operator>>(std::istream& in,
                                             DomainBounds& bounds);

    friend PDAL_DLL std::ostream& operator<<(std::ostream& out,
                                             const DomainBounds& bounds);

private:
    std::vector<std::array<double, 2>> m_data;
};

std::istream& operator>>(std::istream& in, DomainBounds& bounds);

std::ostream& operator<<(std::ostream& out, const DomainBounds& bounds);

} // namespace pdal
