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

#include "HexGrid.hpp"
#include "HexInfo.hpp"

namespace hexer
{

class HexIter
{
public:
    HexIter(HexGrid::HexMap::iterator iter, HexGrid *grid) :
        m_iter(iter), m_grid(grid)
    {
        advance();
    }

    HexIter& operator++()
    {
        m_iter++;
        advance();
        return *this;
    }

    HexInfo operator*()
    {
        HexInfo info;
        Hexagon& hex = m_iter->second;
        info.m_pos.m_x = hex.x();
        info.m_pos.m_y = hex.y();
        info.m_center.m_x = m_grid->width() * hex.x();
        info.m_center.m_y = m_grid->height() * hex.y();
        if (hex.xodd())
            info.m_center.m_y += (m_grid->height() / 2);
        info.m_density = hex.count();
        return info;
    }

    bool operator == (const HexIter& iter)
        { return m_iter == iter.m_iter; }
    bool operator != (const HexIter& iter)
        { return m_iter != iter.m_iter; }

private:
    void advance()
    {
        while (m_iter != m_grid->m_hexes.end())
        {
            if (m_iter->second.count())
                break;
            m_iter++;
        }
    }

    HexGrid::HexMap::iterator m_iter;
    HexGrid *m_grid;
};

} // namespace hexer

