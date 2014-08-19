/******************************************************************************
* Copyright (c) 2014, Connor Manning (connor@hobu.co)
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

#include <vector>

#include <pdal/pdal_export.hpp>

namespace pdal
{

class PointBuffer;

class PDAL_DLL QuadIndex
{
public:
    QuadIndex(const PointBuffer& pointBuffer);
    ~QuadIndex();

    // Build the quadtree index.  Could throw a runtime_error.
    void build();

    // Get bounds of the quad tree.  Return false if the tree has not been
    // built.
    bool getBounds(
            double& xMin,
            double& yMin,
            double& xMax,
            double& yMax) const;

    // All getPoints queries will return an empty vector if the tree has not
    // been successfully built prior to the getPoints call.

    // Return all points at depth levels strictly less than depthEnd.
    // A depthEnd value of zero returns all points in the tree.
    std::vector<std::size_t> getPoints(
            std::size_t depthEnd = 0) const;

    // Return all points at depth levels between [depthBegin, depthEnd).
    // A depthEnd value of zero will return all points at levels >= depthBegin.
    std::vector<std::size_t> getPoints(
            std::size_t depthBegin,
            std::size_t depthEnd) const;

    // Return all points within the query bounding box, searching only up to
    // depth levels strictly less than depthEnd.
    // A depthEnd value of zero will return all existing points that fall
    // within the query range regardless of depth.
    std::vector<std::size_t> getPoints(
            double xMin,
            double yMin,
            double xMax,
            double yMax,
            std::size_t depthEnd = 0) const;

    // Return all points within the bounding box, searching at tree depth
    // levels from [depthBegin, depthEnd).
    // A depthEnd value of zero will return all points within the query range
    // that have a tree level >= depthBegin.
    std::vector<std::size_t> getPoints(
            double xMin,
            double yMin,
            double xMax,
            double yMax,
            std::size_t depthBegin,
            std::size_t depthEnd) const;

private:
    struct QImpl;
    std::unique_ptr<QImpl> m_qImpl;

    // Disable copying and assignment.
    QuadIndex(const QuadIndex&);
    QuadIndex& operator=(QuadIndex&);
};

} // namespace pdal

