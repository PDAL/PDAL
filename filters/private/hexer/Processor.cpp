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

#include <assert.h>
#include <sstream>
#include <cmath>

#include "Processor.hpp"

#include "HexGrid.hpp"

#include "Mathpair.hpp"

namespace hexer
{
    double distance(const Point& p1, const Point& p2)
    {
        double xdist = p2.m_x - p1.m_x;
        double ydist = p2.m_y - p1.m_y;
        return std::sqrt(xdist * xdist + ydist * ydist);
    }

    // Compute hex size based on distance between consecutive points and
    // density.  The probably needs some work based on more data.
    double computeHexSize(const std::vector<Point>& samples, int density)
    {
        double dist = 0;
        for (std::vector<Point>::size_type i = 0; i < samples.size() - 1; ++i)
        {
           Point p1 = samples[i];
           Point p2 = samples[i + 1];
           dist += distance(p1, p2);
        }
        return ((density * dist) / samples.size());
    }


void process(HexGrid *grid, PointReader reader)
{
    double x, y;
    void* context;

    while (reader(x, y, context))
        grid->addPoint(x, y);
    grid->findShapes();
    grid->findParentPaths();
}

void processHexes(HexGrid *grid, HexReader reader)
{
    int x, y;
    void* ctx;

    assert(grid->width() > 0);
    assert(grid->denseLimit() < 0);

    while (reader(x, y, ctx))
        grid->addDenseHexagon(x, y);
    grid->findShapes();
    grid->findParentPaths();
}

} //namespace hexer
