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

#include <pdal/filters/InPlaceReprojection.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <pdal/PointBuffer.hpp>

#ifdef PDAL_HAVE_GDAL
#include <gdal.h>
#include <ogr_spatialref.h>
#include <pdal/GDALUtils.hpp>
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



InPlaceReprojection::InPlaceReprojection(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_outSRS(options.getValueOrThrow<pdal::SpatialReference>("out_srs"))
    , m_inferInputSRS(false)
    , m_x("X", dimension::SignedInteger, 4)
    , m_y("Y", dimension::SignedInteger, 4)
    , m_z("Z", dimension::SignedInteger, 4)
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


void InPlaceReprojection::initialize()
{
    Filter::initialize();

    checkImpedance();

    if (m_inferInputSRS)
    {
        m_inSRS = getPrevStage().getSpatialReference();
    }

#ifdef PDAL_HAVE_GDAL

    m_gdal_debug = boost::shared_ptr<pdal::gdal::Debug>( new pdal::gdal::Debug(isDebug(), log()));
    
    m_in_ref_ptr = ReferencePtr(OSRNewSpatialReference(0), OGRSpatialReferenceDeleter());
    m_out_ref_ptr = ReferencePtr(OSRNewSpatialReference(0), OGRSpatialReferenceDeleter());
    
    int result = OSRSetFromUserInput(m_in_ref_ptr.get(), m_inSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE) 
    {
        std::ostringstream msg; 
        msg << "Could not import input spatial reference for InPlaceReprojection:: " 
            << CPLGetLastErrorMsg() << " code: " << result 
            << " wkt: '" << m_inSRS.getWKT() << "'";
        throw std::runtime_error(msg.str());
    }
    
    result = OSRSetFromUserInput(m_out_ref_ptr.get(), m_outSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE) 
    {
        std::ostringstream msg; 
        msg << "Could not import output spatial reference for InPlaceReprojection:: " 
            << CPLGetLastErrorMsg() << " code: " << result 
            << " wkt: '" << m_outSRS.getWKT() << "'";
        std::string message(msg.str());
        throw std::runtime_error(message);
    }
    m_transform_ptr = TransformPtr(OCTNewCoordinateTransformation( m_in_ref_ptr.get(), m_out_ref_ptr.get()), OSRTransformDeleter());
    
    if (!m_transform_ptr.get())
    {
        std::ostringstream msg; 
        msg << "Could not construct CoordinateTransformation in InPlaceReprojection:: ";
        std::string message(msg.str());
        throw std::runtime_error(message);
    }    
    
#endif
    
    setSpatialReference(m_outSRS);

    updateBounds();

    return;
}


const Options InPlaceReprojection::getDefaultOptions() const
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


void InPlaceReprojection::updateBounds()
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


void InPlaceReprojection::checkImpedance()
{
    Schema& schema = this->getSchemaRef();

    schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();
          
    const std::string x_name = getOptions().getValueOrDefault<std::string>("x_dim", "X");
    const std::string y_name = getOptions().getValueOrDefault<std::string>("y_dim", "Y");
    const std::string z_name = getOptions().getValueOrDefault<std::string>("z_dim", "Z");

    schema::index_by_index::const_iterator i;
    std::vector<Dimension> new_dimensions;
    for (i = dims.begin(); i != dims.end(); ++i)
    {
        Dimension d(*i);
        if (i->getName() == x_name)
        {
            m_x = *i;

            m_x_scale = getOptions().getValueOrDefault<double>("scale_x", i->getNumericScale());
            d.setNumericScale(m_x_scale);

            m_x_offset = getOptions().getValueOrDefault<double>("offset_x", i->getNumericOffset());
            d.setNumericOffset(m_x_offset);
        }
        if (i->getName() == y_name)
        {
            m_y = *i;

            m_y_scale = getOptions().getValueOrDefault<double>("scale_y", i->getNumericScale());
            d.setNumericScale(m_y_scale);

            m_y_offset = getOptions().getValueOrDefault<double>("offset_y", i->getNumericOffset());
            d.setNumericOffset(m_y_offset);
        }
        if (i->getName() == z_name)
        {
            m_z = *i;

            m_z_scale = getOptions().getValueOrDefault<double>("scale_z", i->getNumericScale());
            d.setNumericScale(m_z_scale);

            m_z_offset = getOptions().getValueOrDefault<double>("offset_z", i->getNumericOffset());
            d.setNumericOffset(m_z_offset);
        }
        new_dimensions.push_back(d);
    }
    
    schema = Schema(new_dimensions);
    return;
}


void InPlaceReprojection::transform(double& x, double& y, double& z) const
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

double InPlaceReprojection::getScaledValue( PointBuffer& data, 
                                            Dimension const& d, 
                                            std::size_t pointIndex) const
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
    
    boost::uint32_t size = d.getByteSize();
    switch (d.getInterpretation())
    {
        case dimension::Float:
            if (size == 4)
            {
                flt = data.getField<float>(d, pointIndex);
                output = static_cast<double>(flt);                
            }
            if (size == 8)
            {
                output = data.getField<double>(d, pointIndex);
            }
            break;

        case dimension::SignedInteger:
        case dimension::SignedByte:
            if (size == 1)
            {
                i8 = data.getField<boost::int8_t>(d, pointIndex);
                output = d.applyScaling<boost::int8_t>(i8);
            }
            if (size == 2)
            {
                i16 = data.getField<boost::int16_t>(d, pointIndex);
                output = d.applyScaling<boost::int16_t>(i16);
            }
            if (size == 4)
            {
                i32 = data.getField<boost::int32_t>(d, pointIndex);
                output = d.applyScaling<boost::int32_t>(i32);
            }
            if (size == 8)
            {
                i64 = data.getField<boost::int64_t>(d, pointIndex);
                output = d.applyScaling<boost::int64_t>(i64);
            }
            break;
            
        case dimension::UnsignedInteger:
        case dimension::UnsignedByte:
            if (size == 1)
            {
                u8 = data.getField<boost::uint8_t>(d, pointIndex);
                output = d.applyScaling<boost::uint8_t>(u8);
            }
            if (size == 2)
            {
                u16 = data.getField<boost::uint16_t>(d, pointIndex);
                output = d.applyScaling<boost::uint16_t>(u16);
            }
            if (size == 4)
            {
                u32 = data.getField<boost::uint32_t>(d, pointIndex);
                output = d.applyScaling<boost::uint32_t>(u32);
            }
            if (size == 8)
            {
                u64 = data.getField<boost::uint64_t>(d, pointIndex);
                output = d.applyScaling<boost::uint64_t>(u64);
            }
            break;

        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be reprojected");
    }    
    
    return output;
}
void InPlaceReprojection::setScaledValue(PointBuffer& data, 
                                               double value, 
                                               Dimension const& d, 
                                               std::size_t pointIndex) const
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
    

    boost::uint32_t size = d.getByteSize();
    switch (d.getInterpretation())
    {
        case dimension::Float:
            if (size == 4)
            {
                flt = static_cast<float>(value);
                data.setField<float>(d, pointIndex, flt);
            }
            if (size == 8)
            {
                data.setField<double>(d, pointIndex, value);
            }
            break;

        case dimension::SignedInteger:
        case dimension::SignedByte:
            if (size == 1)
            {
                i8 = d.removeScaling<boost::int8_t>(value);
                data.setField<boost::int8_t>(d, pointIndex, i8);
            }
            if (size == 2)
            {
                i16 = d.removeScaling<boost::int16_t>(value);
                data.setField<boost::int16_t>(d, pointIndex, i16);
            }
            if (size == 4)
            {
                i32 = d.removeScaling<boost::int32_t>(value);
                data.setField<boost::int32_t>(d, pointIndex, i32);
            }
            if (size == 8)
            {
                i64 = d.removeScaling<boost::int64_t>(value);
                data.setField<boost::int64_t>(d, pointIndex, i64);
            }
            break;
            
        case dimension::UnsignedInteger:
        case dimension::UnsignedByte:
            if (size == 1)
            {
                u8 = d.removeScaling<boost::uint8_t>(value);
                data.setField<boost::uint8_t>(d, pointIndex, u8);
            }
            if (size == 2)
            {
                u16 = d.removeScaling<boost::uint16_t>(value);
                data.setField<boost::uint16_t>(d, pointIndex, u16);
            }
            if (size == 4)
            {
                u32 = d.removeScaling<boost::uint32_t>(value);
                data.setField<boost::uint32_t>(d, pointIndex, u32);
            }
            if (size == 8)
            {
                u64 = d.removeScaling<boost::uint64_t>(value);
                data.setField<boost::uint64_t>(d, pointIndex, u64);
            }
            break;

        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be reprojected");

    }    
        
    
}

void InPlaceReprojection::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    const Schema& schema = this->getSchema();
    
    Dimension const& d_x = schema.getDimension(getOptions().getValueOrDefault<std::string>("x_dim", "X"));
    Dimension const& d_y = schema.getDimension(getOptions().getValueOrDefault<std::string>("y_dim", "Y"));
    Dimension const& d_z = schema.getDimension(getOptions().getValueOrDefault<std::string>("z_dim", "Z"));
    
    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        double x = getScaledValue(data, m_x, pointIndex);
        double y = getScaledValue(data, m_y, pointIndex);
        double z = getScaledValue(data, m_z, pointIndex);
        
        // std::cout << "input: " << x << " y: " << y << " z: " << z << std::endl;
        this->transform(x,y,z);
        // std::cout << "output: " << x << " y: " << y << " z: " << z << std::endl;
        
        setScaledValue(data, x, d_x, pointIndex);
        setScaledValue(data, y, d_y, pointIndex);
        setScaledValue(data, z, d_z, pointIndex);

        // std::cout << "set: " << getScaledValue(data, d_x, pointIndex, indexX) 
        //           << " y: " << getScaledValue(data, d_y, pointIndex, indexY) 
        //           << " z: " << getScaledValue(data, d_z, pointIndex, indexZ) << std::endl;
        
        data.setNumPoints(pointIndex+1);
    }

    return;
}


pdal::StageSequentialIterator* InPlaceReprojection::createSequentialIterator() const
{
    return new pdal::filters::iterators::sequential::InPlaceReprojection(*this);
}


namespace iterators { namespace sequential {


InPlaceReprojection::InPlaceReprojection(const pdal::filters::InPlaceReprojection& filter)
    : pdal::FilterSequentialIterator(filter)
    , m_reprojectionFilter(filter)
{
    return;
}


boost::uint32_t InPlaceReprojection::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);

    m_reprojectionFilter.processBuffer(data);

    return numRead;
}


boost::uint64_t InPlaceReprojection::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool InPlaceReprojection::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

} } // iterators::sequential


} } // namespaces
