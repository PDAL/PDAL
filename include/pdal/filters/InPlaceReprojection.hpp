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
class Debug;
}
}

namespace pdal
{
namespace filters
{


class PDAL_DLL InPlaceReprojection : public Filter
{
public:
    SET_STAGE_NAME("filters.inplacereprojection", "In place Reprojection Filter")

    InPlaceReprojection(Stage& prevStage, const Options&);
    InPlaceReprojection(Stage& prevStage,
                        const SpatialReference& outSRS);
    InPlaceReprojection(Stage& prevStage,
                        const SpatialReference& inSRS,
                        const SpatialReference& outSRS);

    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    bool supportsIterator(StageIteratorType t) const
    {
        if (t == StageIterator_Sequential) return true;

        return false;
    }

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const
    {
        return NULL;
    }

    void processBuffer(PointBuffer& data) const;

    double getScaledValue(PointBuffer& data,
                          Dimension const& d,
                          std::size_t pointIndex) const;
    void setScaledValue(PointBuffer& data,
                        double value,
                        Dimension const& d,
                        std::size_t pointIndex) const;

    void transform(double& x, double& y, double& z) const;

private:


    SpatialReference m_inSRS;
    SpatialReference m_outSRS;
    bool m_inferInputSRS;

    typedef boost::shared_ptr<void> ReferencePtr;
    typedef boost::shared_ptr<void> TransformPtr;
    ReferencePtr m_in_ref_ptr;
    ReferencePtr m_out_ref_ptr;
    TransformPtr m_transform_ptr;

    boost::shared_ptr<pdal::gdal::Debug> m_gdal_debug;

    InPlaceReprojection& operator=(const InPlaceReprojection&); // not implemented
    InPlaceReprojection(const InPlaceReprojection&); // not implemented
};

namespace iterators
{
namespace sequential
{


class PDAL_DLL InPlaceReprojection : public pdal::FilterSequentialIterator
{
public:
    InPlaceReprojection(const pdal::filters::InPlaceReprojection& filter, PointBuffer& buffer);

protected:
    virtual void readBufferBeginImpl(PointBuffer&);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    void alterSchema(pdal::PointBuffer&);

    std::map<dimension::id, dimension::id> m_dimension_map;
    
    dimension::id m_new_x_id;
    dimension::id m_new_y_id;
    dimension::id m_new_z_id;

    dimension::id m_old_x_id;
    dimension::id m_old_y_id;
    dimension::id m_old_z_id;
    bool alteredSchema;
    void updateBounds(PointBuffer&);
    

    const pdal::filters::InPlaceReprojection& m_reprojectionFilter;
};


}
} // iterators::sequential


}
} // namespaces

#endif
