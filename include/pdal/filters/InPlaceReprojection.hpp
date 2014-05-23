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

#ifndef INCLUDED_FILTERS_INPLACEREPROJECTIONFILTER_HPP
#define INCLUDED_FILTERS_INPLACEREPROJECTIONFILTER_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>

#include <boost/shared_ptr.hpp>


namespace pdal
{
class PointBuffer;
namespace gdal
{
class GlobalDebug;
}
}

namespace pdal
{
namespace filters
{


class PDAL_DLL InPlaceReprojection : public Filter
{
public:
    SET_STAGE_NAME("filters.inplacereprojection",
        "In place Reprojection Filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.inplacereprojection.html")     
#ifdef PDAL_HAVE_GDAL
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif
    
    InPlaceReprojection(const Options& options) : Filter(options)
         {}
    static Options getDefaultOptions();

private:
    virtual void processOptions(const Options& options);
    virtual void buildSchema(Schema *schema);
    virtual void ready(PointContext ctx);
    virtual void filter(PointBuffer& buffer);

    Dimension *appendDimension(Schema *schema, Dimension *src);
    void reprojectOffsets(double& x, double& y, double& z);
    void updateBounds(PointBuffer& buffer);
    void transform(double& x, double& y, double& z) const;

    typedef boost::shared_ptr<void> ReferencePtr;
    typedef boost::shared_ptr<void> TransformPtr;

    SpatialReference m_inSRS;
    SpatialReference m_outSRS;
    ReferencePtr m_in_ref_ptr;
    ReferencePtr m_out_ref_ptr;
    TransformPtr m_transform_ptr;
    boost::optional<double> m_offset_x;
    boost::optional<double> m_offset_y;
    boost::optional<double> m_offset_z;
    boost::optional<double> m_scale_x;
    boost::optional<double> m_scale_y;
    boost::optional<double> m_scale_z;
    std::string m_x_name;
    std::string m_y_name;
    std::string m_z_name;
    bool m_markIgnored;
    bool m_doOffsetZ;
    Dimension *m_srcDimX;
    Dimension *m_srcDimY;
    Dimension *m_srcDimZ;
    Dimension *m_dimX;
    Dimension *m_dimY;
    Dimension *m_dimZ;

    InPlaceReprojection& operator=(
        const InPlaceReprojection&); // not implemented
    InPlaceReprojection(const InPlaceReprojection&); // not implemented
};


} // namespace filter
} // namespace pdal

#endif
