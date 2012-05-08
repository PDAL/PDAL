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

#include <pdal/drivers/las/Writer.hpp>

#include "LasHeaderWriter.hpp"

// local
#include "ZipPoint.hpp"

// laszip
#ifdef PDAL_HAVE_LASZIP
#include <laszip/laszipper.hpp>
#endif

#include <pdal/Stage.hpp>
#include <pdal/PointBuffer.hpp>

#include <iostream>

namespace pdal
{
namespace drivers
{
namespace las
{


Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_streamManager(options.getOption("filename").getValue<std::string>())
    , m_headerInitialized(false)
    , m_streamOffset(0)
{

    return;
}


Writer::Writer(Stage& prevStage, std::ostream* ostream)
    : pdal::Writer(prevStage, Options::none())
    , m_streamManager(ostream)
    , m_numPointsWritten(0)
    , m_headerInitialized(false)
    , m_streamOffset(0)
{
    return;
}


Writer::~Writer()
{
#ifdef PDAL_HAVE_LASZIP
    m_zipper.reset();
    m_zipPoint.reset();
#endif
    m_streamManager.close();
    return;
}


void Writer::initialize()
{
    pdal::Writer::initialize();

    m_streamManager.open();
        
    setCompressed(getOptions().getValueOrDefault("compression", false));

    if (getOptions().hasOption("a_srs"))
    {
        setSpatialReference(getOptions().getValueOrThrow<std::string>("a_srs"));
    } 
    

    return;
}


const Options Writer::getDefaultOptions() const
{
    Options options;

    Option filename("filename", "", "file to read from");
    Option compression("compression", false, "Do we LASzip-compress the data?");
    Option format("format", PointFormat3, "Point format to write");
    Option major_version("major_version", 1, "LAS Major version");
    Option minor_version("minor_version", 2, "LAS Minor version");
    Option day_of_year("day_of_year", 0, "Day of Year for file");
    Option year("year", 2011, "4-digit year value for file");
    Option system_id("system_id", LasHeader::SystemIdentifier, "System ID for this file");
    Option software_id("software_id", LasHeader::SoftwareIdentifier, "Software ID for this file");
    Option header_padding("header_padding", 0, "Header padding (space between end of VLRs and beginning of point data)");
    Option set_metadata("forward_metadata", false, "forward metadata into the file as necessary");

    options.add(major_version);
    options.add(minor_version);
    options.add(day_of_year);
    options.add(year);
    options.add(system_id);
    options.add(software_id);
    options.add(header_padding);
    options.add(format);
    options.add(filename);
    options.add(compression);
    return options;
}


void Writer::setCompressed(bool v)
{
    m_lasHeader.SetCompressed(v);
}


void Writer::setFormatVersion(boost::uint8_t majorVersion, boost::uint8_t minorVersion)
{
    m_lasHeader.SetVersionMajor(majorVersion);
    m_lasHeader.SetVersionMinor(minorVersion);
}


void Writer::setPointFormat(PointFormat pointFormat)
{
    m_lasHeader.setPointFormat(pointFormat);
}


void Writer::setDate(boost::uint16_t dayOfYear, boost::uint16_t year)
{
    m_lasHeader.SetCreationDOY(dayOfYear);
    m_lasHeader.SetCreationYear(year);
}


void Writer::setProjectId(const boost::uuids::uuid& id)
{
    m_lasHeader.SetProjectId(id);
}


void Writer::setSystemIdentifier(const std::string& systemId)
{
    m_lasHeader.SetSystemId(systemId);
}


void Writer::setGeneratingSoftware(const std::string& softwareId)
{
    m_lasHeader.SetSoftwareId(softwareId);
}


void Writer::setHeaderPadding(boost::uint32_t const& v)
{
    m_lasHeader.SetHeaderPadding(v);
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    m_streamOffset = m_streamManager.ostream().tellp();
    return;
}


void Writer::writeBufferBegin(PointBuffer const& data)
{
    if (m_headerInitialized) return;

    m_lasHeader.setBounds(getPrevStage().getBounds());

    const Schema& schema = data.getSchema();

    const Dimension& dimX = schema.getDimension("X");
    const Dimension& dimY = schema.getDimension("Y");
    const Dimension& dimZ = schema.getDimension("Z");

    m_lasHeader.SetScale(dimX.getNumericScale(), dimY.getNumericScale(), dimZ.getNumericScale());
    m_lasHeader.SetOffset(dimX.getNumericOffset(), dimY.getNumericOffset(), dimZ.getNumericOffset());

    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(8);


    m_lasHeader.setSpatialReference(getSpatialReference());


    bool useMetadata = getOptions().getValueOrDefault<bool>("forward_metadata", false);

    pdal::Metadata metadata = getPrevStage().collectMetadata();
    
    boost::optional<pdal::metadata::Entry const&> format = metadata.getEntryOptional("dataformatid");
    if (format && useMetadata)
    {
        setPointFormat(static_cast<PointFormat>(format->cast<boost::uint32_t>()));
    } 
    else 
    {
        setPointFormat(static_cast<PointFormat>(getOptions().getValueOrDefault<boost::uint32_t>("format", 3)));        
    }

    boost::optional<pdal::metadata::Entry const&> major = metadata.getEntryOptional("version_major");
    boost::optional<pdal::metadata::Entry const&> minor = metadata.getEntryOptional("version_minor");
    if (minor && useMetadata)
    {
        setFormatVersion((boost::uint8_t)getOptions().getValueOrDefault<boost::uint32_t>("major_version", 1),
                         minor->cast<boost::uint32_t>());
    } 
    else 
    {
        setFormatVersion((boost::uint8_t)getOptions().getValueOrDefault<boost::uint32_t>("major_version", 1),
                         (boost::uint8_t)getOptions().getValueOrDefault<boost::uint32_t>("minor_version", 2));     
    }
    
     

    boost::optional<pdal::metadata::Entry const&> year = metadata.getEntryOptional("creation_year");
    boost::optional<pdal::metadata::Entry const&> day = metadata.getEntryOptional("creation_doy");
    if (year && day && useMetadata)
    {
        setFormatVersion((boost::uint8_t)getOptions().getValueOrDefault<boost::uint32_t>("major_version", 1),
                         minor->cast<boost::uint32_t>());
        log()->get(logDEBUG) << "Setting version to " << getOptions().getValueOrDefault<boost::uint32_t>("major_version", 1)
                             << ", " << minor->cast<boost::uint32_t>() << std::endl;

    } 
    else 
    {
        if (!getOptions().getValueOrDefault<bool>("todays_date", false))
        {
            setDate((boost::uint16_t)getOptions().getValueOrDefault<boost::uint32_t>("year", 0),
                    (boost::uint16_t)getOptions().getValueOrDefault<boost::uint32_t>("day_of_year", 0)); 
            
        }
    }

    boost::optional<pdal::metadata::Entry const&> software_id = metadata.getEntryOptional("software_id");
    if (software_id && useMetadata)
    {
        try 
        {
            setGeneratingSoftware(software_id->cast<std::string>());
        } catch (std::bad_cast&) {}
    } 
    else 
    {
        setGeneratingSoftware(getOptions().getValueOrDefault<std::string>("software_id", LasHeader::SoftwareIdentifier));
    }    

    boost::optional<pdal::metadata::Entry const&> system_id = metadata.getEntryOptional("system_id");
    if (system_id && useMetadata)
    {
        try 
        {
            setSystemIdentifier(system_id->cast<std::string>());
        } catch (std::bad_cast&) {}
    } 
    else 
    {
        setSystemIdentifier(getOptions().getValueOrDefault<std::string>("system_id", LasHeader::SystemIdentifier));
    }    
    


    setHeaderPadding(getOptions().getValueOrDefault<boost::uint32_t>("header_padding", 0));



    LasHeaderWriter lasHeaderWriter(m_lasHeader, m_streamManager.ostream(), m_streamOffset);
    lasHeaderWriter.write();

    m_summaryData.reset();

    if (m_lasHeader.Compressed())
    {
#ifdef PDAL_HAVE_LASZIP
        if (!m_zipPoint)
        {
            PointFormat format = m_lasHeader.getPointFormat();
            boost::scoped_ptr<ZipPoint> z(new ZipPoint(format, m_lasHeader.getVLRs().getAll()));
            m_zipPoint.swap(z);
        }

        if (!m_zipper)
        {
            boost::scoped_ptr<LASzipper> z(new LASzipper());
            m_zipper.swap(z);

            bool stat(false);
            stat = m_zipper->open(m_streamManager.ostream(), m_zipPoint->GetZipper());
            if (!stat)
            {
                std::ostringstream oss;
                const char* err = m_zipper->get_error();
                if (err==NULL) err="(unknown error)";
                oss << "Error opening LASzipper: " << std::string(err);
                throw pdal_error(oss.str());
            }
        }
#else
        throw pdal_error("LASzip compression is not enabled for this compressed file!");
#endif
    }


    
    m_headerInitialized = true;

    return;

}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    m_lasHeader.SetPointRecordsCount(m_numPointsWritten);

    log()->get(logDEBUG) << "Wrote " << m_numPointsWritten << " points to the LAS file" << std::endl;

    m_streamManager.ostream().seekp(m_streamOffset);
    Support::rewriteHeader(m_streamManager.ostream(), m_summaryData);

    return;
}


void Writer::writeBufferEnd(PointBuffer const& /*data*/)
{
    return;
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& pointBuffer)
{
    const Schema& schema = pointBuffer.getSchema();

    PointFormat pointFormat = m_lasHeader.getPointFormat();

    const PointDimensions dimensions(schema,"");

    boost::uint32_t numValidPoints = 0;

    boost::uint8_t buf[1024]; // BUG: fixed size

    for (boost::uint32_t pointIndex=0; pointIndex<pointBuffer.getNumPoints(); pointIndex++)
    {
        boost::uint8_t* p = buf;

        // we always write the base fields
        const boost::int32_t x = pointBuffer.getField<boost::int32_t>(*dimensions.X, pointIndex);
        const boost::int32_t y = pointBuffer.getField<boost::int32_t>(*dimensions.Y, pointIndex);
        const boost::int32_t z = pointBuffer.getField<boost::int32_t>(*dimensions.Z, pointIndex);

        // std::clog << "x: " << x << " y: " << y << " z: " << z << std::endl;
        // std::clog << "positions.X: " << positions.X << " positions.Y: " << positions.Y << " positions.Z: " << positions.Z << std::endl;

        boost::uint16_t intensity(0);
        if (dimensions.Intensity)
            intensity = pointBuffer.getField<boost::uint16_t>(*dimensions.Intensity, pointIndex);

        boost::uint8_t returnNumber(0);
        if (dimensions.ReturnNumber)
            returnNumber = pointBuffer.getField<boost::uint8_t>(*dimensions.ReturnNumber, pointIndex);

        boost::uint8_t numberOfReturns(0);
        if (dimensions.NumberOfReturns)
            numberOfReturns = pointBuffer.getField<boost::uint8_t>(*dimensions.NumberOfReturns, pointIndex);

        boost::uint8_t scanDirectionFlag(0);
        if (dimensions.ScanDirectionFlag)
            scanDirectionFlag = pointBuffer.getField<boost::uint8_t>(*dimensions.ScanDirectionFlag, pointIndex);

        boost::uint8_t edgeOfFlightLine(0);
        if (dimensions.EdgeOfFlightLine)
            edgeOfFlightLine = pointBuffer.getField<boost::uint8_t>(*dimensions.EdgeOfFlightLine, pointIndex);

        boost::uint8_t bits = returnNumber | (numberOfReturns<<3) | (scanDirectionFlag << 6) | (edgeOfFlightLine << 7);

        boost::uint8_t classification(0);
        if (dimensions.Classification)
            classification = pointBuffer.getField<boost::uint8_t>(*dimensions.Classification, pointIndex);

        boost::int8_t scanAngleRank(0);
        if (dimensions.ScanAngleRank)
            scanAngleRank = pointBuffer.getField<boost::int8_t>(*dimensions.ScanAngleRank, pointIndex);

        boost::uint8_t userData(0);
        if (dimensions.UserData)
            userData = pointBuffer.getField<boost::uint8_t>(*dimensions.UserData, pointIndex);

        boost::uint16_t pointSourceId(0);
        if (dimensions.PointSourceId)
            pointSourceId = pointBuffer.getField<boost::uint16_t>(*dimensions.PointSourceId, pointIndex);

        Utils::write_field<boost::uint32_t>(p, x);
        Utils::write_field<boost::uint32_t>(p, y);
        Utils::write_field<boost::uint32_t>(p, z);
        Utils::write_field<boost::uint16_t>(p, intensity);
        Utils::write_field<boost::uint8_t>(p, bits);
        Utils::write_field<boost::uint8_t>(p, classification);
        Utils::write_field<boost::int8_t>(p, scanAngleRank);
        Utils::write_field<boost::uint8_t>(p, userData);
        Utils::write_field<boost::uint16_t>(p, pointSourceId);

        if (Support::hasTime(pointFormat))
        {
            double time(0.0);

            if (dimensions.Time)
                time = pointBuffer.getField<double>(*dimensions.Time, pointIndex);

            Utils::write_field<double>(p, time);
        }

        if (Support::hasColor(pointFormat))
        {
            boost::uint16_t red(0);
            boost::uint16_t green(0);
            boost::uint16_t blue(0);

            if (dimensions.Red)
                red = pointBuffer.getField<boost::uint16_t>(*dimensions.Red, pointIndex);
            if (dimensions.Green)
                green = pointBuffer.getField<boost::uint16_t>(*dimensions.Green, pointIndex);
            if (dimensions.Blue)
                blue = pointBuffer.getField<boost::uint16_t>(*dimensions.Blue, pointIndex);

            Utils::write_field<boost::uint16_t>(p, red);
            Utils::write_field<boost::uint16_t>(p, green);
            Utils::write_field<boost::uint16_t>(p, blue);
        }

#ifdef PDAL_HAVE_LASZIP
        if (m_zipPoint)
        {
            for (unsigned int i=0; i<m_zipPoint->m_lz_point_size; i++)
            {
                m_zipPoint->m_lz_point_data[i] = buf[i];
                // printf("%d %d\n", buf[i], i);
            }
            bool ok = m_zipper->write(m_zipPoint->m_lz_point);
            if (!ok)
            {
                std::ostringstream oss;
                const char* err = m_zipper->get_error();
                if (err==NULL) err="(unknown error)";
                oss << "Error writing point: " << std::string(err);
                throw pdal_error(oss.str());
            }
        }
        else
        {
            Utils::write_n(m_streamManager.ostream(), buf, Support::getPointDataSize(pointFormat));
        }
#else
        Utils::write_n(m_streamManager.ostream(), buf, Support::getPointDataSize(pointFormat));

#endif
        ++numValidPoints;

        const double xValue = (*dimensions.X).applyScaling<boost::int32_t>(x);
        const double yValue = (*dimensions.Y).applyScaling<boost::int32_t>(y);
        const double zValue = (*dimensions.Z).applyScaling<boost::int32_t>(z);
        m_summaryData.addPoint(xValue, yValue, zValue, returnNumber);
    }

    m_numPointsWritten = m_numPointsWritten+numValidPoints;
    return numValidPoints;
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

}
}
} // namespaces
