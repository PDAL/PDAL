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

#include "SrsTransform.hpp"
#include <pdal/SpatialReference.hpp>

#include <ogr_spatialref.h>

namespace pdal
{

SrsTransform::SrsTransform()
{}

SrsTransform::SrsTransform(const SrsTransform& src)
{
    if (src.valid())
        set(*(src.m_transform->GetSourceCS()), *(src.m_transform->GetTargetCS()));
}


SrsTransform::SrsTransform(SrsTransform&& src) : m_transform(std::move(src.m_transform))
{}


SrsTransform::~SrsTransform()
{}


void SrsTransform::setSrcEpoch(double epoch)
{
#if GDAL_VERSION_NUM >= GDAL_COMPUTE_VERSION(3,4,0)
    OGRSpatialReference target(*m_transform->GetTargetCS());
    OGRSpatialReference source(*m_transform->GetSourceCS());
    source.SetCoordinateEpoch(epoch);
    set(target, source);
#endif
}

void SrsTransform::setDstEpoch(double epoch)
{
#if GDAL_VERSION_NUM >= GDAL_COMPUTE_VERSION(3,4,0)
    OGRSpatialReference target(*m_transform->GetTargetCS());
    OGRSpatialReference source(*m_transform->GetSourceCS());
    target.SetCoordinateEpoch(epoch);
    set(target, source);
#endif
}

SrsTransform& SrsTransform::operator=(SrsTransform&& src)
{
    m_transform = std::move(src.m_transform);
    return *this;
}

SrsTransform::SrsTransform(const SpatialReference& src, const SpatialReference& dst)
{
    set(src, dst);
}


SrsTransform::SrsTransform(const OGRSpatialReference& srcRef, const OGRSpatialReference& dstRef)
{
    set(srcRef, dstRef);
}


SrsTransform::SrsTransform(const SpatialReference& src,
                           std::vector<int> srcOrder,
                           const SpatialReference& dst,
                           std::vector<int> dstOrder)
{
    OGRSpatialReference srcRef(src.getWKT2().data());
    srcRef.SetCoordinateEpoch(src.getEpoch());
    OGRSpatialReference dstRef(dst.getWKT2().data());
    dstRef.SetCoordinateEpoch(dst.getEpoch());

// Starting with version 3, the axes (X, Y, Z or lon, lat, h or whatever)
// are mapped according to the WKT definition.  In particular, this means
// that for EPSG:4326 the mapping is X -> lat, Y -> lon, rather than the
// more conventional X -> lon, Y -> lat.  Setting this flag reverses things
// such that the traditional ordering is maintained.  There are other
// SRSes where this comes up.  See "axis order issues" in the GDAL WKT2
// discussion for more info.
//
    if (srcOrder.size())
        srcRef.SetDataAxisToSRSAxisMapping(srcOrder);
    if (dstOrder.size())
        dstRef.SetDataAxisToSRSAxisMapping(dstOrder);
    m_transform.reset(OGRCreateCoordinateTransformation(&srcRef, &dstRef));
}

void SrsTransform::set(const SpatialReference& src, const SpatialReference& dst)
{
    OGRSpatialReference osrSrc(src.getWKT2().data());
    osrSrc.SetCoordinateEpoch(src.getEpoch());
    OGRSpatialReference osrDst(dst.getWKT2().data());
    osrDst.SetCoordinateEpoch(dst.getEpoch());
    set(osrSrc, osrDst);
}


void SrsTransform::set(OGRSpatialReference src, OGRSpatialReference dst)
{
// Starting with version 3 of GDAL, the axes (X, Y, Z or lon, lat, h or whatever)
// are mapped according to the WKT definition.  In particular, this means
// that for EPSG:4326 the mapping is X -> lat, Y -> lon, rather than the
// more conventional X -> lon, Y -> lat.  Setting this flag reverses things
// such that the traditional ordering is maintained.  There are other
// SRSes where this comes up.  See "axis order issues" in the GDAL WKT2
// discussion for more info.
//
    src.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    dst.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    m_transform.reset(OGRCreateCoordinateTransformation(&src, &dst));
}


OGRCoordinateTransformation *SrsTransform::get() const
{
    return m_transform.get();
}


bool SrsTransform::transform(double& x, double& y, double& z) const
{
    return m_transform && m_transform->Transform(1, &x, &y, &z);
}


bool SrsTransform::transform(std::vector<double>& x, std::vector<double>& y,
    std::vector<double>& z) const
{
    if (x.size() != y.size() && y.size() != z.size())
        throw pdal_error("SrsTransform::called with vectors of different "
            "sizes.");
    int err = m_transform->Transform(x.size(), x.data(), y.data(), z.data());
    return (err == OGRERR_NONE);
}

} // namespace pdal
