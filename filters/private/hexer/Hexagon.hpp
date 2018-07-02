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

#include <stdint.h>

#include "Mathpair.hpp"

namespace hexer
{

class Hexagon
{
public:
    Hexagon(int x, int y) : m_x(x), m_y(y), m_count(0), m_dense(false),
        m_dense_neighbors(0)
        {}

    uint64_t key()
    {
        return key(m_x, m_y);
    }

    void increment()
       { m_count++; }

    static uint64_t key(int32_t x, int32_t y)
    {
        uint32_t ux = (uint32_t)x;
        uint32_t uy = (uint32_t)y;
        return (ux | ((uint64_t)uy << 32));
    }

    int x() const
        { return m_x; }

    int y() const
        { return m_y; }

    bool xodd() const
        { return (x() % 2 != 0); }

    bool xeven() const
        { return !xodd(); }

    void setDense()
        { m_dense = true; }

    bool dense() const
        { return m_dense; }

    int count() const
        { return m_count; }

    void setCount(int count)
        { m_count = count; }

    void setDenseNeighbor(int dir)
        { m_dense_neighbors |= (1 << dir); }

    // We're surrounded by dense neighbors when the six low bits are set.
    bool surrounded() const
        { return (m_dense && (m_dense_neighbors == 0x3F)); }

    bool possibleRoot() const
    {
        const int TOP = (1 << 0);     // Top is side 0.
        return (m_dense && ((m_dense_neighbors & TOP) == 0) );
    }

    bool less(const Hexagon *h) const;
    bool yless(Hexagon *h) const;
    Coord neighborCoord(int dir) const;

private:
    int32_t m_x;
    int32_t m_y;
    int m_count;
    bool m_dense;
    int m_dense_neighbors;
};

class HexCompare
{
public:
    bool operator()(const Hexagon *h1, const Hexagon *h2) const
        { return h1->less(h2); }
};

} // namespace

