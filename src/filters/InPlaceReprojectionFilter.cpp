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

#include <pdal/filters/InPlaceReprojectionFilter.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <pdal/Dimension.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/filters/InPlaceReprojectionFilterIterator.hpp>

#ifdef PDAL_HAVE_GDAL
#include <gdal.h>
#include <ogr_spatialref.h>
#endif

namespace pdal { namespace filters {


#ifdef PDAL_HAVE_GDAL
    struct OGRSpatialReferenceDeleter
    {
       template <typename T>
       void operator()(T* ptr)
       {
           ::OSRDestroySpatialReference(ptr);
       }
    };

    struct OSRTransformDeleter
    {
       template <typename T>
       void operator()(T* ptr)
       {
           ::OCTDestroyCoordinateTransformation(ptr);
       }
    };


    struct GDALSourceDeleter
    {
       template <typename T>
       void operator()(T* ptr)
       {
           ::GDALClose(ptr);
       }
    };
#endif



InPlaceReprojectionFilter::InPlaceReprojectionFilter(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_outSRS(options.getValueOrThrow<pdal::SpatialReference>("out_srs"))
    , m_inferInputSRS(false)
    , m_x(DimensionId::X_i32)
    , m_y(DimensionId::Y_i32)
    , m_z(DimensionId::Z_i32)
    , m_x_scale(1.0)
    , m_y_scale(1.0)
    , m_z_scale(1.0)
    , m_x_offset(0.0)
    , m_y_offset(0.0)
    , m_z_offset(0.0)
{
    if (options.hasOption("in_srs"))
    {
        m_inSRS = options.getValueOrThrow<pdal::SpatialReference>("in_srs");
        m_inferInputSRS = false;
    }
    else
    {
        m_inferInputSRS = true;
    }
    
    return;
}


void InPlaceReprojectionFilter::initialize()
{
    Filter::initialize();

    checkImpedance();

    if (m_inferInputSRS)
    {
        m_inSRS = getPrevStage().getSpatialReference();
    }

#ifdef PDAL_HAVE_GDAL
    
    m_in_ref_ptr = ReferencePtr(OSRNewSpatialReference(0), OGRSpatialReferenceDeleter());
    m_out_ref_ptr = ReferencePtr(OSRNewSpatialReference(0), OGRSpatialReferenceDeleter());
    
    int result = OSRSetFromUserInput(m_in_ref_ptr.get(), m_inSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE) 
    {
        std::ostringstream msg; 
        msg << "Could not import input spatial reference for ReprojectionFilter:: " 
            << CPLGetLastErrorMsg() << " code: " << result 
            << " wkt: '" << m_inSRS.getWKT() << "'";
        throw std::runtime_error(msg.str());
    }
    
    result = OSRSetFromUserInput(m_out_ref_ptr.get(), m_outSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE) 
    {
        std::ostringstream msg; 
        msg << "Could not import output spatial reference for ReprojectionFilter:: " 
            << CPLGetLastErrorMsg() << " code: " << result 
            << " wkt: '" << m_outSRS.getWKT() << "'";
        std::string message(msg.str());
        throw std::runtime_error(message);
    }
    m_transform_ptr = TransformPtr(OCTNewCoordinateTransformation( m_in_ref_ptr.get(), m_out_ref_ptr.get()), OSRTransformDeleter());
    
    if (!m_transform_ptr.get())
    {
        std::ostringstream msg; 
        msg << "Could not construct CoordinateTransformation in ReprojectionFilter:: ";
        std::string message(msg.str());
        throw std::runtime_error(message);
    }    
    
#endif
    
    setSpatialReference(m_outSRS);

    updateBounds();

    return;
}


const Options InPlaceReprojectionFilter::getDefaultOptions() const
{
    Options options;
    Option in_srs("in_srs", std::string(""),"Input SRS to use to override -- fetched from previous stage if not present");
    Option out_srs("out_srs", std::string(""), "Output SRS to reproject to");
    Option x("x_dim", std::string("X"), "Dimension name to use for 'X' data");
    Option y("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");
    Option z("z_dim", std::string("Z"), "Dimension name to use for 'Z' data");
    Option x_scale("scale_x", 1.0, "Scale for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
    Option y_scale("scale_y", 1.0, "Scale for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
    Option z_scale("scale_z", 1.0, "Scale for output Z data in the case when 'Z' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
    Option x_offset("offset_x", 0.0, "Offset for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '0.0'.  If not set, the Dimensions's scale will be used");
    Option y_offset("offset_y", 0.0, "Offset for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '0.0'.  If not set, the Dimensions's scale will be used");
    Option z_offset("offset_z", 0.0, "Offset for output Z data in the case when 'Z' dimension data are to be scaled.  Defaults to '0.0'.  If not set, the Dimensions's scale will be used");
    options.add(in_srs);
    options.add(out_srs);
    options.add(x);
    options.add(y);
    options.add(z);
    options.add(x_scale);
    options.add(y_scale);
    options.add(z_scale);
    options.add(x_offset);
    options.add(y_offset);
    options.add(z_offset);
        
    return options;
}


void InPlaceReprojectionFilter::updateBounds()
{
    const Bounds<double>& oldBounds = getBounds();

    double minx = oldBounds.getMinimum(0);
    double miny = oldBounds.getMinimum(1);
    double minz = oldBounds.getMinimum(2);
    double maxx = oldBounds.getMaximum(0);
    double maxy = oldBounds.getMaximum(1);
    double maxz = oldBounds.getMaximum(2);
    
    try {

        transform(minx, miny, minz);
        transform(maxx, maxy, maxz);
        
    } catch (pdal::pdal_error&) 
    {
        return;
    }

    Bounds<double> newBounds(minx, miny, minz, maxx, maxy, maxz);

    setBounds(newBounds);

    return;
}


void InPlaceReprojectionFilter::checkImpedance()
{
    Schema& schema = this->getSchemaRef();

    std::vector<Dimension>& dims = schema.getDimensions();
    
    std::string x_name;
    std::string y_name;
    std::string z_name;
    
    try
    {
        x_name = getOptions().getValueOrThrow<std::string>("x_dim");
    } catch (pdal::option_not_found const&)
    {
        x_name = "X";
    }

    try
    {
        y_name = getOptions().getValueOrThrow<std::string>("y_dim");
    } catch (pdal::option_not_found const&)
    {
        y_name = "Y";
    }

    try
    {
        z_name = getOptions().getValueOrThrow<std::string>("z_dim");
    } catch (pdal::option_not_found const&)
    {
        z_name = "Z";
    }


    std::vector<Dimension>::iterator i;
    for (i = dims.begin(); i != dims.end(); ++i)
    {
        if (i->getName() == x_name)
        {
            m_x = *i;
            try
            {
                m_x_scale = getOptions().getValueOrThrow<double>("scale_x");
            } catch (pdal::option_not_found const&)
            {
                m_x_scale = i->getNumericScale();
            }
            i->setNumericScale(m_x_scale);
            try
            {
                m_x_offset = getOptions().getValueOrThrow<double>("offset_x");
            } catch (pdal::option_not_found const&)
            {
                m_x_offset = i->getNumericOffset();
            }
            i->setNumericOffset(m_x_offset);
        }
        if (i->getName() == y_name)
        {
            m_y = *i;
            try
            {
                m_y_scale = getOptions().getValueOrThrow<double>("scale_y");
            } catch (pdal::option_not_found const&)
            {
                m_y_scale = i->getNumericScale();
            }
            i->setNumericScale(m_y_scale);
            try
            {
                m_y_offset = getOptions().getValueOrThrow<double>("offset_y");
            } catch (pdal::option_not_found const&)
            {
                m_y_offset = i->getNumericOffset();
            }
            i->setNumericOffset(m_y_offset);
        }
        if (i->getName() == z_name)
        {
            m_z = *i;
            try
            {
                m_z_scale = getOptions().getValueOrThrow<double>("scale_z");
            } catch (pdal::option_not_found const&)
            {
                m_z_scale = i->getNumericScale();
            }
            i->setNumericScale(m_z_scale);
            try
            {
                m_z_offset = getOptions().getValueOrThrow<double>("offset_z");
            } catch (pdal::option_not_found const&)
            {
                m_z_offset = i->getNumericOffset();
            }
            i->setNumericOffset(m_z_offset);
        }        
    }

    return;
}


void InPlaceReprojectionFilter::transform(double& x, double& y, double& z) const
{

#ifdef PDAL_HAVE_GDAL
    int ret = 0;

    ret = OCTTransform(m_transform_ptr.get(), 1, &x, &y, &z);    
    if (!ret)
    {
        std::ostringstream msg; 
        msg << "Could not project point for ReprojectionTransform::" << CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }
#else
    boost::ignore_unused_variable_warning(x);
    boost::ignore_unused_variable_warning(y);
    boost::ignore_unused_variable_warning(z);
#endif
 
    return;
}

double InPlaceReprojectionFilter::getScaledValue(PointBuffer& data, Dimension const& d, std::size_t pointIndex, boost::int32_t fieldIndex) const
{
    double output(0.0);
        
    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);
    
    switch (d.getDataType())
    {
        case Dimension::Float:
            flt = data.getField<float>(pointIndex, fieldIndex);
            output = static_cast<double>(flt);
            break;
        case Dimension::Double:
            output = data.getField<double>(pointIndex, fieldIndex);
            break;
        
        case Dimension::Int8:
            i8 = data.getField<boost::int8_t>(pointIndex, fieldIndex);
            output = d.applyScaling<boost::int8_t>(i8);
            break;
        case Dimension::Uint8:
            u8 = data.getField<boost::uint8_t>(pointIndex, fieldIndex);
            output = d.applyScaling<boost::uint8_t>(u8);
            break;
        case Dimension::Int16:
            i16 = data.getField<boost::int16_t>(pointIndex, fieldIndex);
            output = d.applyScaling<boost::int16_t>(i16);
            break;
        case Dimension::Uint16:
            u16 = data.getField<boost::uint16_t>(pointIndex, fieldIndex);
            output = d.applyScaling<boost::uint16_t>(u16);
            break;
        case Dimension::Int32:
            i32 = data.getField<boost::int32_t>(pointIndex, fieldIndex);
            output = d.applyScaling<boost::int32_t>(i32);
            break;
        case Dimension::Uint32:
            u32 = data.getField<boost::uint32_t>(pointIndex, fieldIndex);
            output = d.applyScaling<boost::uint32_t>(u32);
            break;
        case Dimension::Int64:
            i64 = data.getField<boost::int64_t>(pointIndex, fieldIndex);
            output = d.applyScaling<boost::int64_t>(i64);
            break;
        case Dimension::Uint64:
            u64 = data.getField<boost::uint64_t>(pointIndex, fieldIndex);
            output = d.applyScaling<boost::uint64_t>(u64);
            break;
        case Dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case Dimension::Undefined:
            throw pdal_error("Dimension data type unable to be reprojected");
    }    
    
    return output;
}
void InPlaceReprojectionFilter::setScaledValue(PointBuffer& data, 
                                               double value, 
                                               Dimension const& d, 
                                               std::size_t pointIndex, 
                                               boost::int32_t fieldIndex) const
{

    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);
    
    switch (d.getDataType())
    {
        case Dimension::Float:
            flt = static_cast<float>(value);
            data.setField<float>(pointIndex, fieldIndex, flt);
            break;
        case Dimension::Double:
            data.setField<double>(pointIndex, fieldIndex, value);
            break;
        case Dimension::Int8:
            i8 = d.removeScaling<boost::int8_t>(value);
            data.setField<boost::int8_t>(pointIndex, fieldIndex, i8);
            break;
        case Dimension::Uint8:
            u8 = d.removeScaling<boost::uint8_t>(value);
            data.setField<boost::uint8_t>(pointIndex, fieldIndex, u8);
            break;
        case Dimension::Int16:
            i16 = d.removeScaling<boost::int16_t>(value);
            data.setField<boost::int16_t>(pointIndex, fieldIndex, i16);
            break;
        case Dimension::Uint16:
            u16 = d.removeScaling<boost::uint16_t>(value);
            data.setField<boost::uint16_t>(pointIndex, fieldIndex, u16);
            break;
        case Dimension::Int32:
            i32 = d.removeScaling<boost::int32_t>(value);
            data.setField<boost::int32_t>(pointIndex, fieldIndex, i32);
            break;
        case Dimension::Uint32:
            u32 = d.removeScaling<boost::uint32_t>(value);
            data.setField<boost::uint32_t>(pointIndex, fieldIndex, u32);
            break;
        case Dimension::Int64:
            i64 = d.removeScaling<boost::int64_t>(value);
            data.setField<boost::int64_t>(pointIndex, fieldIndex, i64);
            break;
        case Dimension::Uint64:
            u64 = d.removeScaling<boost::uint64_t>(value);
            data.setField<boost::uint64_t>(pointIndex, fieldIndex, u64);
            break;
        case Dimension::Pointer:
        case Dimension::Undefined:
            throw pdal_error("Dimension data type unable to be reprojected");
    }    
        
    
}

void InPlaceReprojectionFilter::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    const Schema& schema = this->getSchema();
    
    Dimension const& d_x = schema.getDimension(m_x.getId());
    Dimension const& d_y = schema.getDimension(m_y.getId());
    Dimension const& d_z = schema.getDimension(m_z.getId());
    
    const int indexX = schema.getDimensionIndex(d_x);
    const int indexY = schema.getDimensionIndex(d_y);
    const int indexZ = schema.getDimensionIndex(d_z);

std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
std::cout.precision(8);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        double x = getScaledValue(data, m_x, pointIndex, indexX);
        double y = getScaledValue(data, m_y, pointIndex, indexY);
        double z = getScaledValue(data, m_z, pointIndex, indexZ);
        
        // std::cout << "input: " << x << " y: " << y << " z: " << z << std::endl;
        this->transform(x,y,z);
        // std::cout << "output: " << x << " y: " << y << " z: " << z << std::endl;
        
        setScaledValue(data, x, d_x, pointIndex, indexX);
        setScaledValue(data, y, d_y, pointIndex, indexY);
        setScaledValue(data, z, d_z, pointIndex, indexZ);

        // std::cout << "set: " << getScaledValue(data, d_x, pointIndex, indexX) 
        //           << " y: " << getScaledValue(data, d_y, pointIndex, indexY) 
        //           << " z: " << getScaledValue(data, d_z, pointIndex, indexZ) << std::endl;
        
        data.setNumPoints(pointIndex+1);
    }

    return;
}


pdal::StageSequentialIterator* InPlaceReprojectionFilter::createSequentialIterator() const
{
    return new InPlaceReprojectionFilterSequentialIterator(*this);
}

} } // namespaces
