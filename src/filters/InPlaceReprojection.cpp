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


void InPlaceReprojection::processOptions(const Options& options)
{
    if (options.hasOption("in_srs"))
        m_inSRS = m_options.getValueOrThrow<pdal::SpatialReference>("in_srs");
    m_outSRS = m_options.getValueOrThrow<pdal::SpatialReference>("out_srs");

    m_x_name = options.getValueOrDefault<std::string>("x_dim", "X");
    m_y_name = options.getValueOrDefault<std::string>("y_dim", "Y");
    m_z_name = options.getValueOrDefault<std::string>("z_dim", "Z");

    log()->get(logDEBUG2) << "x_dim '" << m_x_name <<"' requested" << std::endl;
    log()->get(logDEBUG2) << "y_dim '" << m_y_name <<"' requested" << std::endl;
    log()->get(logDEBUG2) << "z_dim '" << m_z_name <<"' requested" << std::endl;

    if (options.hasOption("scale_x"))
        m_scale_x = boost::optional<double>(
            options.getValueOrThrow<double>("scale_x"));
    if (options.hasOption("scale_y"))
        m_scale_y = boost::optional<double>(
            options.getValueOrThrow<double>("scale_y"));
    if (options.hasOption("scale_z"))
        m_scale_z = boost::optional<double>(
            options.getValueOrThrow<double>("scale_z"));

    if (options.hasOption("offset_x"))
        m_offset_x = boost::optional<double>(
            options.getValueOrThrow<double>("offset_x"));
    if (options.hasOption("offset_y"))
        m_offset_y = boost::optional<double>(
            options.getValueOrThrow<double>("offset_y"));
    if (options.hasOption("offset_z"))
        m_offset_z = boost::optional<double>(
            options.getValueOrThrow<double>("offset_z"));

    m_markIgnored = options.getValueOrDefault<bool>("ignore_old_dimensions",
        true);
    m_doOffsetZ = options.getValueOrDefault<bool>("do_offset_z", false);
}


Options InPlaceReprojection::getDefaultOptions()
{
    Options options;
    Option in_srs("in_srs", std::string(""),"Input SRS to use to override -- "
        "fetched from previous stage if not present");
    Option out_srs("out_srs", std::string(""), "Output SRS to reproject to");
    Option x("x_dim", std::string("X"), "Dimension name to use for 'X' data");
    Option y("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");
    Option z("z_dim", std::string("Z"), "Dimension name to use for 'Z' data");
    Option x_scale("scale_x", 1.0f, "Scale for output X data in the case "
        "when 'X' dimension data are to be scaled.  Defaults to '1.0'.  "
        "If not set, the Dimensions's scale will be used");
    Option y_scale("scale_y", 1.0f, "Scale for output Y data in the case "
        "when 'Y' dimension data are to be scaled.  Defaults to '1.0'.  "
        "If not set, the Dimensions's scale will be used");
    Option z_scale("scale_z", 1.0f, "Scale for output Z data in the case "
        "when 'Z' dimension data are to be scaled.  Defaults to '1.0'.  "
        "If not set, the Dimensions's scale will be used");
    Option x_offset("offset_x", 0.0f, "Offset for output X data in the case "
        "when 'X' dimension data are to be scaled.  Defaults to '0.0'.  "
        "If not set, the Dimensions's scale will be used");
    Option y_offset("offset_y", 0.0f, "Offset for output Y data in the case "
        "when 'Y' dimension data are to be scaled.  Defaults to '0.0'.  "
        "If not set, the Dimensions's scale will be used");
    Option z_offset("offset_z", 0.0f, "Offset for output Z data in the case "
        "when 'Z' dimension data are to be scaled.  Defaults to '0.0'.  "
        "If not set, the Dimensions's scale will be used");
    Option ignore_old_dimensions("ignore_old_dimensions", true,
        "Mark old, unprojected dimensions as ignored");
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


void InPlaceReprojection::buildSchema(Schema *schema)
{
    m_srcDimX = schema->getDimensionPtr(m_x_name);
    m_srcDimY = schema->getDimensionPtr(m_y_name);
    m_srcDimZ = schema->getDimensionPtr(m_z_name);

    log()->get(logDEBUG3) << "Fetched x_name: " << *m_srcDimX;
    log()->get(logDEBUG3) << "Fetched y_name: " << *m_srcDimY;
    log()->get(logDEBUG3) << "Fetched z_name: " << *m_srcDimZ;

    m_dimX = appendDimension(schema, m_srcDimX);
    m_dimY = appendDimension(schema, m_srcDimY);
    m_dimZ = appendDimension(schema, m_srcDimZ);
}


Dimension *InPlaceReprojection::appendDimension(Schema *schema, Dimension *src)
{
    Dimension dim(*src);
    dim.setParent(src->getUUID());
    dim.setNamespace(getName());
    dim.createUUID();
    Dimension *d = schema->appendDimension(dim);

    log()->get(logDEBUG2) << "source  dimension: " << *src << std::endl;
    log()->get(logDEBUG2) << "derived dimension: " << dim << std::endl;
    log()->get(logDEBUG2) << "source  id: " << src->getUUID() << std::endl;
    log()->get(logDEBUG2) << "derived id: " << dim.getUUID() << std::endl;

    if (m_markIgnored)
    {
        log()->get(logDEBUG2) << "marking " << getName() <<
            " as ignored with uuid "  << src->getUUID() << std::endl;
        src->setFlags(src->getFlags() | dimension::IsIgnored);
    }
    return d;
}


void InPlaceReprojection::ready(PointContext ctx)
{
    if (m_inSRS.empty())
        m_inSRS = getPrevStage().getSpatialReference();
    setSpatialReference(m_outSRS);

#ifdef PDAL_HAVE_GDAL
    pdal::GlobalEnvironment::get().getGDALDebug()->addLog(log());
    m_in_ref_ptr = ReferencePtr(OSRNewSpatialReference(0),

        OGRSpatialReferenceDeleter());
    m_out_ref_ptr = ReferencePtr(OSRNewSpatialReference(0),
        OGRSpatialReferenceDeleter());

    int result = OSRSetFromUserInput(m_in_ref_ptr.get(),
        m_inSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE)
    {
        std::ostringstream msg;
        msg << "Could not import input spatial reference for "
            "InPlaceReprojection:: " << CPLGetLastErrorMsg() << " code: " <<
            result << " wkt: '" << m_inSRS.getWKT() << "'";
        throw std::runtime_error(msg.str());
    }

    result = OSRSetFromUserInput(m_out_ref_ptr.get(),
        m_outSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE)
    {
        std::ostringstream msg;
        msg << "Could not import output spatial reference for "
            "InPlaceReprojection:: " << CPLGetLastErrorMsg() << " code: " <<
            result << " wkt: '" << m_outSRS.getWKT() << "'";
        throw std::runtime_error(msg.str());
    }
    m_transform_ptr = TransformPtr(
        OCTNewCoordinateTransformation(m_in_ref_ptr.get(),
            m_out_ref_ptr.get()), OSRTransformDeleter());

    if (!m_transform_ptr.get())
    {
        std::string msg = "Could not construct CoordinateTransformation in "
            "InPlaceReprojection::";
        throw std::runtime_error(msg);
    }

#endif

    if (!m_offset_x)
        m_offset_x = boost::optional<double>(m_dimX->getNumericOffset());
    if (!m_offset_y)
        m_offset_y = boost::optional<double>(m_dimY->getNumericOffset());
    if (!m_offset_z)
        m_offset_z = boost::optional<double>(m_dimZ->getNumericOffset());

    if (!m_scale_x)
        m_scale_x = boost::optional<double>(m_dimX->getNumericScale());
    if (!m_scale_y)
        m_scale_y = boost::optional<double>(m_dimY->getNumericScale());
    if (!m_scale_z)
        m_scale_z = boost::optional<double>(m_dimZ->getNumericScale());

    /* Reproject incoming offsets to new coordinate system */
    log()->floatPrecision(8);
    log()->get(logDEBUG2) << "original offset x,y: " << *m_offset_x <<
        "," << *m_offset_y << std::endl;

    try
    {
        double x(*m_offset_x);
        double y(*m_offset_y);
        double z(*m_offset_z);
        reprojectOffsets(x, y, z);        
        log()->get(logDEBUG2) << "reprojected offset x,y: " 
            << x << "," << y << std::endl;
        *m_offset_x = x;
        *m_offset_y = y;
        *m_offset_z = z;
    } catch (pdal::pdal_error&)
    {
        // If we failed to reproject the offsets, we're going to just use
        // what we have
        log()->get(logDEBUG2) << "Default offset used due to inability "
            "to reproject original offset x,y: " << *m_offset_x <<
            "," << *m_offset_y << std::endl;
    }

    m_dimX->setNumericScale(*m_scale_x);
    m_dimY->setNumericScale(*m_scale_y);
    m_dimZ->setNumericScale(*m_scale_z);
    m_dimX->setNumericOffset(*m_offset_x);
    m_dimY->setNumericOffset(*m_offset_y);
    m_dimZ->setNumericOffset(*m_offset_z);
}


void InPlaceReprojection::reprojectOffsets(double& x, double& y, double& z)
{
    double temp = z;
#ifdef PDAL_HAVE_GDAL
    int ret = OCTTransform(m_transform_ptr.get(), 1, &x, &y, &z);
    if (!ret)
    {
        std::ostringstream msg;
        msg << "Could not project offset for InPlaceReprojection::" << CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }
#endif
    if (!m_doOffsetZ)
       z = temp;
}


//ABELL
void InPlaceReprojection::updateBounds(PointBuffer& buffer)
{
    Bounds<double> bounds = buffer.getSpatialBounds();
    Bounds<double> newbounds = buffer.getSpatialBounds();

    try
    {
        newbounds.transform([this](double& x, double& y, double& z)
            { this->transform(x, y, z); } );
    }
    catch (pdal::pdal_error&)
    {
        return;
    }

    try
    {
        buffer.setSpatialBounds(newbounds);
    }
    catch (pdal::bounds_error&)
    {
        try
        {
            newbounds.setMinimum(2, bounds.getMinimum(2));
            newbounds.setMaximum(2, bounds.getMaximum(2));
            buffer.setSpatialBounds(newbounds);
        }
        catch (pdal::bounds_error&)
        {
            buffer.setSpatialBounds(buffer.calculateBounds(true));
        }
    }
}

void InPlaceReprojection::filter(PointBuffer& buffer)
{
std::cerr << "Filter #1!\n";
    bool logOutput = log()->getLevel() > logDEBUG3;
    if (logOutput)
    {
        log()->floatPrecision(8);
        log()->get(logDEBUG3) << "old_x: " << *m_srcDimX;
        log()->get(logDEBUG3) << "old_y: " << *m_srcDimY;
        log()->get(logDEBUG3) << "old_z: " << *m_srcDimZ;

        log()->get(logDEBUG3) << "new_x: " << *m_dimX;
        log()->get(logDEBUG3) << "new_y: " << *m_dimY;
        log()->get(logDEBUG3) << "new_z: " << *m_dimZ;
    }
std::cerr << "Filter #2!\n";
        
    for (PointId idx = 0; idx < buffer.size(); ++idx)
    {
        double x = buffer.getFieldAs<double>(*m_srcDimX, idx);
        double y = buffer.getFieldAs<double>(*m_srcDimY, idx);
        double z = buffer.getFieldAs<double>(*m_srcDimZ, idx);

        if (logOutput)
        {
            log()->floatPrecision(8);
            log()->get(logDEBUG5) << "input: " <<
                x << " y: " << y << " z: " << z << std::endl;
        }
std::cerr << "Filter #3!\n";

        transform(x, y, z);
std::cerr << "Filter #4!\n";

        if (logOutput)
        {
            log()->get(logDEBUG5) << "output: " <<
                x << " y: " << y << " z: " << z << std::endl;
        }

        buffer.setFieldUnscaled(*m_dimX, idx, x);
        buffer.setFieldUnscaled(*m_dimY, idx, y);
        buffer.setFieldUnscaled(*m_dimZ, idx, z);

        if (logOutput)
        {
            log()->get(logDEBUG5) << "scaled:" <<
                " x: " << buffer.getFieldAs<double>(*m_dimX, idx) <<
                " y: " << buffer.getFieldAs<double>(*m_dimY, idx) <<
                " z: " << buffer.getFieldAs<double>(*m_dimZ, idx) << std::endl;
        }
    }
    if (logOutput)
        log()->clearFloat();
    updateBounds(buffer);
}


void InPlaceReprojection::transform(double& x, double& y, double& z) const
{
#ifdef PDAL_HAVE_GDAL
    int ret = OCTTransform(m_transform_ptr.get(), 1, &x, &y, &z);
    if (!ret)
    {
        std::ostringstream msg;
        msg << "Could not project point for InPlaceReprojection::" <<
            CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }
#else
    boost::ignore_unused_variable_warning(x);
    boost::ignore_unused_variable_warning(y);
    boost::ignore_unused_variable_warning(z);
#endif
}

} // namespace filters
} // namespace pdal
