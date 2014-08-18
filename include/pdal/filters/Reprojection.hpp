/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/Filter.hpp>

#include <boost/shared_ptr.hpp>

namespace pdal
{
class PointBuffer;
namespace gdal
{
class Debug;
}

namespace filters
{

class PDAL_DLL Reprojection : public Filter
{
public:
    SET_STAGE_NAME("filters.reprojection", "Reprojection Filter")
    SET_STAGE_LINK("http://www.pdal.io/stages/filters.inplacereprojection.html")     
#ifdef PDAL_HAVE_GDAL
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif

    Reprojection(const Options&);
    Reprojection(const SpatialReference& outSRS);
    Reprojection(const SpatialReference& inSRS, const SpatialReference& outSRS);

private:
    virtual void processOptions(const Options& options);
    virtual void ready(PointContext ctx);
    virtual void filter(PointBuffer& buffer);

    void updateBounds();
    void transform(double& x, double& y, double& z);

    SpatialReference m_inSRS;
    SpatialReference m_outSRS;
    bool m_inferInputSRS;

    typedef boost::shared_ptr<void> ReferencePtr;
    typedef boost::shared_ptr<void> TransformPtr;
    ReferencePtr m_in_ref_ptr;
    ReferencePtr m_out_ref_ptr;
    TransformPtr m_transform_ptr;
    boost::shared_ptr<pdal::gdal::Debug> m_gdal_debug;

    Reprojection& operator=(const Reprojection&); // not implemented
    Reprojection(const Reprojection&); // not implemented
};

} // namespace filter
} // namespace pdal

