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
#include <pdal/GlobalEnvironment.hpp>

#ifdef PDAL_HAVE_GDAL
#include <gdal.h>
#include <ogr_spatialref.h>
#include <pdal/GDALUtils.hpp>
#endif

namespace pdal
{
namespace filters
{


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

#endif



InPlaceReprojection::InPlaceReprojection(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_outSRS(options.getValueOrThrow<pdal::SpatialReference>("out_srs"))
    , m_inferInputSRS(false)
    , m_new_x_id(boost::uuids::nil_uuid())
    , m_new_y_id(boost::uuids::nil_uuid())
    , m_new_z_id(boost::uuids::nil_uuid())
    , m_old_x_id(boost::uuids::nil_uuid())
    , m_old_y_id(boost::uuids::nil_uuid())
    , m_old_z_id(boost::uuids::nil_uuid())
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

InPlaceReprojection::~InPlaceReprojection()
{

}
void InPlaceReprojection::initialize()
{
    Filter::initialize();

    if (m_inferInputSRS)
    {
        m_inSRS = getPrevStage().getSpatialReference();
    }

#ifdef PDAL_HAVE_GDAL

    pdal::GlobalEnvironment::get().getGDALDebug()->addLog(log());

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
    m_transform_ptr = TransformPtr(OCTNewCoordinateTransformation(m_in_ref_ptr.get(), m_out_ref_ptr.get()), OSRTransformDeleter());

    if (!m_transform_ptr.get())
    {
        std::ostringstream msg;
        msg << "Could not construct CoordinateTransformation in InPlaceReprojection:: ";
        std::string message(msg.str());
        throw std::runtime_error(message);
    }

#endif

    setSpatialReference(m_outSRS);

    Schema& s = getSchemaRef();
    s = alterSchema(s);


    return;
}


void InPlaceReprojection::setDimension(std::string const& name,
                                       dimension::id& old_id,
                                       dimension::id& new_id,
                                       Schema& schema,
                                       double scale,
                                       double offset)
{


    Dimension const& old_dim = schema.getDimension(name);

    log()->get(logDEBUG2) << "found '" << name <<"' dimension " << old_dim << std::endl;

    Dimension derived(old_dim.getName(), old_dim.getInterpretation(), old_dim.getByteSize(), old_dim.getDescription());
    derived.setNumericScale(scale);
    derived.setNumericOffset(offset);
    derived.createUUID();
    derived.setNamespace(getName());
    derived.setParent(old_dim.getUUID());
    schema.appendDimension(derived);

    old_id = old_dim.getUUID();
    new_id = derived.getUUID();

    log()->get(logDEBUG2) << "source    dimension: " << old_dim << std::endl;
    log()->get(logDEBUG2) << "derived dimension: " << derived << std::endl;

    log()->get(logDEBUG2) << "source  id: " << old_id << std::endl;
    log()->get(logDEBUG2) << "derived id: " << new_id << std::endl;

    bool markIgnored = getOptions().getValueOrDefault<bool>("ignore_old_dimensions", true);
    if (markIgnored)
    {
        Dimension const& dim = schema.getDimension(old_id);
        log()->get(logDEBUG2) << "marking " << name << " as ignored with uuid "  << old_id << std::endl;

        Dimension d(dim);
        boost::uint32_t flags = d.getFlags();
        d.setFlags(flags | dimension::IsIgnored);
        schema.setDimension(d);
    }

}

void InPlaceReprojection::reprojectOffsets(double& offset_x,
        double& offset_y,
        double& offset_z)
{

#ifdef PDAL_HAVE_GDAL
    int ret = 0;

    double dummy_z(0.0);
    bool doOffsetZ = getOptions().getValueOrDefault<bool>("do_offset_z", false);

    double* z = doOffsetZ ? &offset_z : &dummy_z;
    ret = OCTTransform(m_transform_ptr.get(), 1, &offset_x, &offset_y, z);
    if (!ret)
    {
        std::ostringstream msg;
        msg << "Could not project offset for InPlaceReprojection::" << CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }
#else

#endif

}

Schema InPlaceReprojection::alterSchema(Schema& schema)
{


    const std::string x_name = getOptions().getValueOrDefault<std::string>("x_dim", "X");
    const std::string y_name = getOptions().getValueOrDefault<std::string>("y_dim", "Y");
    const std::string z_name = getOptions().getValueOrDefault<std::string>("z_dim", "Z");

    log()->get(logDEBUG2) << "x_dim '" << x_name <<"' requested" << std::endl;
    log()->get(logDEBUG2) << "y_dim '" << y_name <<"' requested" << std::endl;
    log()->get(logDEBUG2) << "z_dim '" << z_name <<"' requested" << std::endl;

    Dimension const& dimX = schema.getDimension(x_name);
    Dimension const& dimY = schema.getDimension(y_name);
    Dimension const& dimZ = schema.getDimension(z_name);

    log()->get(logDEBUG3) << "Fetched x_name: " << dimX;
    log()->get(logDEBUG3) << "Fetched y_name: " << dimY;
    log()->get(logDEBUG3) << "Fetched z_name: " << dimZ;

    /* Start with the incoming dimension offsets */
    double offset_x = dimX.getNumericOffset();
    double offset_y = dimY.getNumericOffset();
    double offset_z = dimZ.getNumericOffset();

    /* Reproject incoming offsets to new coordinate system */
    log()->floatPrecision(8);
    log()->get(logDEBUG2) << "original offset x,y: " << offset_x <<"," << offset_y << std::endl;

    try
    {
        reprojectOffsets(offset_x, offset_y, offset_z);        
        log()->get(logDEBUG2) << "reprojected offset x,y: " 
                              << offset_x <<"," 
                              << offset_y << std::endl;
    } catch (pdal::pdal_error&)
    {
        // If we failed to reproject the offsets, we're going to just use
        // what we have
        /* If user-specified offsets exist, use those instead of the reprojected offsets */
        offset_x = getOptions().getValueOrDefault<double>("offset_x", offset_x);
        offset_y = getOptions().getValueOrDefault<double>("offset_y", offset_y);
        offset_z = getOptions().getValueOrDefault<double>("offset_z", offset_z);
        log()->get(logDEBUG2) << "Default offset used due to inability to reproject original offset x,y: " 
                              << offset_x <<"," 
                              << offset_y << std::endl;
    }




    /* Read any user-specified scales */
    double scale_x = getOptions().getValueOrDefault<double>("scale_x", dimX.getNumericScale());
    double scale_y = getOptions().getValueOrDefault<double>("scale_y", dimY.getNumericScale());
    double scale_z = getOptions().getValueOrDefault<double>("scale_z", dimZ.getNumericScale());

    /* Apply scaling/offset to output schema */
    setDimension(x_name, m_old_x_id, m_new_x_id, schema, scale_x, offset_x);
    setDimension(y_name, m_old_y_id, m_new_y_id, schema, scale_y, offset_y);
    setDimension(z_name, m_old_z_id, m_new_z_id, schema, scale_z, offset_z);

    return schema;

}

Options InPlaceReprojection::getDefaultOptions()
{
    Options options;
    Option in_srs("in_srs", std::string(""),"Input SRS to use to override -- fetched from previous stage if not present");
    Option out_srs("out_srs", std::string(""), "Output SRS to reproject to");
    Option x("x_dim", std::string("X"), "Dimension name to use for 'X' data");
    Option y("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");
    Option z("z_dim", std::string("Z"), "Dimension name to use for 'Z' data");
    Option x_scale("scale_x", 1.0f, "Scale for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
    Option y_scale("scale_y", 1.0f, "Scale for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
    Option z_scale("scale_z", 1.0f, "Scale for output Z data in the case when 'Z' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
    Option x_offset("offset_x", 0.0f, "Offset for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '0.0'.  If not set, the Dimensions's scale will be used");
    Option y_offset("offset_y", 0.0f, "Offset for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '0.0'.  If not set, the Dimensions's scale will be used");
    Option z_offset("offset_z", 0.0f, "Offset for output Z data in the case when 'Z' dimension data are to be scaled.  Defaults to '0.0'.  If not set, the Dimensions's scale will be used");
    Option ignore_old_dimensions("ignore_old_dimensions", true, "Mark old, unprojected dimensions as ignored");
    Option do_offset_z("do_offset_z", false, "Should we re-offset Z data");
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
    options.add(ignore_old_dimensions);
    options.add(do_offset_z);

    return options;
}



void InPlaceReprojection::transform(double& x, double& y, double& z) const
{

#ifdef PDAL_HAVE_GDAL
    int ret = 0;

    ret = OCTTransform(m_transform_ptr.get(), 1, &x, &y, &z);
    if (!ret)
    {
        std::ostringstream msg;
        msg << "Could not project point for InPlaceReprojection::" << CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }
#else
    boost::ignore_unused_variable_warning(x);
    boost::ignore_unused_variable_warning(y);
    boost::ignore_unused_variable_warning(z);
#endif

    return;
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

        case dimension::RawByte:
        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be set to scaled value");

    }


}

pdal::StageSequentialIterator* InPlaceReprojection::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::InPlaceReprojection(*this, buffer);
}

pdal::StageRandomIterator* InPlaceReprojection::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::random::InPlaceReprojection(*this, buffer);
}


namespace iterators
{

namespace inplacereprojection
{

IteratorBase::IteratorBase(pdal::filters::InPlaceReprojection const& filter,
                           PointBuffer&)
    : m_reprojectionFilter(filter)
{

}

void IteratorBase::updateBounds(PointBuffer& buffer)
{
    const Bounds<double>& oldBounds = buffer.getSpatialBounds();

    double minx = oldBounds.getMinimum(0);
    double miny = oldBounds.getMinimum(1);
    double minz = oldBounds.getMinimum(2);
    double maxx = oldBounds.getMaximum(0);
    double maxy = oldBounds.getMaximum(1);
    double maxz = oldBounds.getMaximum(2);

    try
    {

        m_reprojectionFilter.transform(minx, miny, minz);
        m_reprojectionFilter.transform(maxx, maxy, maxz);

    }

    catch (pdal::pdal_error&)
    {
        return;
    }

    try
    {
        Bounds<double> newBounds(minx, miny, minz, maxx, maxy, maxz);
        buffer.setSpatialBounds(newBounds);
    }
    catch (pdal::bounds_error&)
    {
        try
        {
            Bounds<double> newBounds(minx, miny, oldBounds.getMinimum(2), maxx, maxy, oldBounds.getMaximum(2));
            buffer.setSpatialBounds(newBounds);
        }
        catch (pdal::bounds_error&)
        {
            Bounds<double> newBounds = buffer.calculateBounds(true);
            buffer.setSpatialBounds(newBounds);
        }
    }

    return;
}

void IteratorBase::projectData(PointBuffer& buffer, boost::uint32_t numPoints)
{

    const Schema& schema = buffer.getSchema();

    Dimension const& old_x = schema.getDimension(m_reprojectionFilter.getOldXId());
    Dimension const& old_y = schema.getDimension(m_reprojectionFilter.getOldYId());
    Dimension const& old_z = schema.getDimension(m_reprojectionFilter.getOldZId());

    Dimension const& new_x = schema.getDimension(m_reprojectionFilter.getNewXId());
    Dimension const& new_y = schema.getDimension(m_reprojectionFilter.getNewYId());
    Dimension const& new_z = schema.getDimension(m_reprojectionFilter.getNewZId());

    bool logOutput = m_reprojectionFilter.log()->getLevel() > logDEBUG3;


    if (logOutput)
    {
        m_reprojectionFilter.log()->floatPrecision(8);
        m_reprojectionFilter.log()->get(logDEBUG3) << "old_x: " << old_x;
        m_reprojectionFilter.log()->get(logDEBUG3) << "old_y: " << old_y;
        m_reprojectionFilter.log()->get(logDEBUG3) << "old_z: " << old_z;

        m_reprojectionFilter.log()->get(logDEBUG3) << "new_x: " << new_x;
        m_reprojectionFilter.log()->get(logDEBUG3) << "new_y: " << new_y;
        m_reprojectionFilter.log()->get(logDEBUG3) << "new_z: " << new_z;

    }

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        double x = buffer.applyScaling( old_x, pointIndex);
        double y = buffer.applyScaling( old_y, pointIndex);
        double z = buffer.applyScaling( old_z, pointIndex);

        if (logOutput)
        {
            m_reprojectionFilter.log()->floatPrecision(8);
            m_reprojectionFilter.log()->get(logDEBUG5) << "input: " << x << " y: " << y << " z: " << z << std::endl;
        }

        m_reprojectionFilter.transform(x,y,z);

        if (logOutput)
        {
            m_reprojectionFilter.log()->get(logDEBUG5) << "output: " << x << " y: " << y << " z: " << z << std::endl;

        }

        m_reprojectionFilter.setScaledValue(buffer, x, new_x, pointIndex);
        m_reprojectionFilter.setScaledValue(buffer, y, new_y, pointIndex);
        m_reprojectionFilter.setScaledValue(buffer, z, new_z, pointIndex);

        if (logOutput)
        {
            m_reprojectionFilter.log()->get(logDEBUG5) << "scaled: " << buffer.applyScaling(new_x, pointIndex)
                    << " y: " << buffer.applyScaling( new_y, pointIndex)
                    << " z: " << buffer.applyScaling(new_z, pointIndex) << std::endl;
        }

        buffer.setNumPoints(pointIndex+1);
    }
    if (logOutput)
        m_reprojectionFilter.log()->clearFloat();

    updateBounds(buffer);
}

} // inplacereprojection

namespace sequential
{


InPlaceReprojection::InPlaceReprojection(const pdal::filters::InPlaceReprojection& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , inplacereprojection::IteratorBase(filter, buffer)
{
    return;
}

boost::uint32_t InPlaceReprojection::readBufferImpl(PointBuffer& buffer)
{

    const boost::uint32_t numPoints = getPrevIterator().read(buffer);

    InPlaceReprojection::projectData(buffer, numPoints);

    return numPoints;
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

} // sequential

namespace random
{


InPlaceReprojection::InPlaceReprojection(const pdal::filters::InPlaceReprojection& filter, PointBuffer& buffer)
    : pdal::FilterRandomIterator(filter, buffer)
    , inplacereprojection::IteratorBase(filter, buffer)
{
    return;
}

boost::uint32_t InPlaceReprojection::readBufferImpl(PointBuffer& buffer)
{

    pdal::StageRandomIterator& iterator = getPrevIterator();

    const boost::uint32_t numPoints = iterator.read(buffer);

    InPlaceReprojection::projectData(buffer, numPoints);

    return numPoints;
}


boost::uint64_t InPlaceReprojection::seekImpl(boost::uint64_t count)
{

    return getPrevIterator().seek(count);
}

} // random



} // filters
} // pdal
} // namespaces
