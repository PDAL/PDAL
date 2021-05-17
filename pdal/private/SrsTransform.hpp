/******************************************************************************
 * Copyright (c) 2019, Hobu Inc.
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

#include <pdal/pdal_internal.hpp>

class OGRCoordinateTransformation;
class OGRSpatialReference;

namespace pdal
{

class SpatialReference;

class PDAL_DLL SrsTransform
{
public:
    /// Object that performs transformation from a \src spatial reference
    /// to a \dest spatial reference.
    SrsTransform(OGRSpatialReference src, OGRSpatialReference dst);
    SrsTransform(const SpatialReference& src, const SpatialReference& dst);
    SrsTransform(const SpatialReference& src,
                 std::vector<int> srcOrder,
                 const SpatialReference& dst,
                 std::vector<int> dstOrder);
    ~SrsTransform();

    /// Get the underlying transformation.
    /// \return  Pointer to the underlying coordinate transform.
    OGRCoordinateTransformation *get() const;

    /// Transform the X, Y and Z of a point in place.
    /// \param x  X coordinate
    /// \param y  Y coordinate
    /// \param z  Z coordinate
    /// \return  True if the transformation was successful
    bool transform(double& x, double& y, double& z) const;

    /// Transform a set of points in place.
    /// \param x  X coordinates
    /// \param y  Y coordinates
    /// \param z  Z coordinates
    /// \return  True if the transformation was successful
    bool transform(std::vector<double>& x, std::vector<double>& y,
        std::vector<double>& z) const;

private:
    std::unique_ptr<OGRCoordinateTransformation> m_transform;
};

} // namespace pdal

