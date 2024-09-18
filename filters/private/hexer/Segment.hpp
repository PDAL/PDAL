/******************************************************************************
 * Copyright (c) 2014, Hobu Inc. (howard@hobu.co)
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
 *     * Neither the name of the Howard Butler or Hobu, Inc.
 *       the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <iostream>

#include "HexId.hpp"

namespace hexer
{

struct Segment
{
    Segment(HexId hex, int edge) : hex(hex), edge(edge)
    {}

    bool horizontal() const
    { return edge == 0 || edge == 3; }

    /// Hexagon who's side is the segment.
    HexId hex;
    /// Which side of the hexagon.
    int edge;
};

inline bool operator == (const Segment& s1, const Segment& s2)
{
    return s1.hex == s2.hex && s1.edge == s2.edge;
}

inline bool operator != (const Segment& s1, const Segment& s2)
{
    return !(s1 == s2);
}

inline std::ostream& operator << (std::ostream& out, const Segment& s)
{
    out << "(" << s.hex.i << "," << s.hex.j << " - " << s.edge << ")";
    return out;
}

} // namespace hexer
