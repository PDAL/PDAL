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

namespace pdal { namespace drivers { namespace las {


Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_streamManager(options.getOption("filename").getValue<std::string>())
{

    return;
}


Writer::Writer(Stage& prevStage, std::ostream* ostream)
    : pdal::Writer(prevStage, Options::none())
    , m_streamManager(ostream)
    , m_numPointsWritten(0)
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
    
    setPointFormat(static_cast<PointFormat>(getOptions().getValueOrDefault<boost::uint32_t>("format", 3)));
    setFormatVersion((boost::uint8_t)getOptions().getValueOrDefault<boost::uint32_t>("major_version", 1),
                     (boost::uint8_t)getOptions().getValueOrDefault<boost::uint32_t>("minor_version", 2));
    setDate((boost::uint16_t)getOptions().getValueOrDefault<boost::uint32_t>("year", 0),
            (boost::uint16_t)getOptions().getValueOrDefault<boost::uint32_t>("day_of_year", 0));
    
    setHeaderPadding(getOptions().getValueOrDefault<boost::uint32_t>("header_padding", 0));
    setSystemIdentifier(getOptions().getValueOrDefault<std::string>("system_id", LasHeader::SystemIdentifier));
    setGeneratingSoftware(getOptions().getValueOrDefault<std::string>("software_id", LasHeader::SoftwareIdentifier));
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


void Writer::setProjectId(const pdal::external::boost::uuids::uuid& id)
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

void Writer::writeBegin(boost::uint64_t targetNumPointsToWrite)
{
    // need to set properties of the header here, based on prev->getHeader() and on the user's preferences
    m_lasHeader.setBounds( getPrevStage().getBounds() );

    const Schema& schema = getPrevStage().getSchema();

    const Dimension& dimX = schema.getDimension(DimensionId::X_i32);
    const Dimension& dimY = schema.getDimension(DimensionId::Y_i32);
    const Dimension& dimZ = schema.getDimension(DimensionId::Z_i32);

    m_lasHeader.SetScale(dimX.getNumericScale(), dimY.getNumericScale(), dimZ.getNumericScale());
    m_lasHeader.SetOffset(dimX.getNumericOffset(), dimY.getNumericOffset(), dimZ.getNumericOffset());

    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(8);
    
    boost::uint32_t cnt = static_cast<boost::uint32_t>(targetNumPointsToWrite);
    
    std::ostringstream oss;
    oss << "Writing " << cnt << " points to the LAS file";
    log(oss);
    
    m_lasHeader.SetPointRecordsCount(cnt);

    m_lasHeader.setSpatialReference(getSpatialReference());

    LasHeaderWriter lasHeaderWriter(m_lasHeader, m_streamManager.ostream());
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

    return;
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    m_lasHeader.SetPointRecordsCount(m_numPointsWritten);

    //std::streamsize const dataPos = 107; 
    //m_ostream.seekp(dataPos, std::ios::beg);
    //LasHeaderWriter lasHeaderWriter(m_lasHeader, m_ostream);
    //Utils::write_n(m_ostream, m_numPointsWritten, sizeof(m_numPointsWritten));
        
    m_streamManager.ostream().seekp(0);
    Support::rewriteHeader(m_streamManager.ostream(), m_summaryData);

    return;
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& pointBuffer)
{
    const Schema& schema = getPrevStage().getSchema();
    const Dimension& xDim = schema.getDimension(DimensionId::X_i32);
    const Dimension& yDim = schema.getDimension(DimensionId::Y_i32);
    const Dimension& zDim = schema.getDimension(DimensionId::Z_i32);
    PointFormat pointFormat = m_lasHeader.getPointFormat();

    const PointIndexes indexes(schema, pointFormat);
    const PointPositions positions(schema, pointFormat);

    boost::uint32_t numValidPoints = 0;

    boost::uint8_t buf[1024]; // BUG: fixed size

    for (boost::uint32_t pointIndex=0; pointIndex<pointBuffer.getNumPoints(); pointIndex++)
    {
        boost::uint8_t* p = buf;

        // we always write the base fields
        const boost::int32_t x = pointBuffer.getRawField<boost::int32_t>(pointIndex, positions.X);
        const boost::int32_t y = pointBuffer.getRawField<boost::int32_t>(pointIndex, positions.Y);
        const boost::int32_t z = pointBuffer.getRawField<boost::int32_t>(pointIndex, positions.Z);
        
        // std::clog << "x: " << x << " y: " << y << " z: " << z << std::endl;
        // std::clog << "positions.X: " << positions.X << " positions.Y: " << positions.Y << " positions.Z: " << positions.Z << std::endl;
        
        boost::uint16_t intensity(0);
        if (indexes.Intensity != -1)
            intensity = pointBuffer.getRawField<boost::uint16_t>(pointIndex, positions.Intensity);
        
        boost::uint8_t returnNumber(0); 
        if (indexes.ReturnNumber != -1)
            returnNumber = pointBuffer.getRawField<boost::uint8_t>(pointIndex, positions.ReturnNumber);
        
        boost::uint8_t numberOfReturns(0);
        if (indexes.NumberOfReturns != -1)
            numberOfReturns = pointBuffer.getRawField<boost::uint8_t>(pointIndex, positions.NumberOfReturns);
        
        boost::uint8_t scanDirectionFlag(0);
        if (indexes.ScanDirectionFlag != -1)
            scanDirectionFlag = pointBuffer.getRawField<boost::uint8_t>(pointIndex, positions.ScanDirectionFlag);
        
        boost::uint8_t edgeOfFlightLine(0);
        if (indexes.EdgeOfFlightLine != -1)
            edgeOfFlightLine = pointBuffer.getRawField<boost::uint8_t>(pointIndex, positions.EdgeOfFlightLine);

        boost::uint8_t bits = returnNumber | (numberOfReturns<<3) | (scanDirectionFlag << 6) | (edgeOfFlightLine << 7);
        
        boost::uint8_t classification(0);
        if (indexes.Classification != -1)
            classification = pointBuffer.getRawField<boost::uint8_t>(pointIndex, positions.Classification);
        
        boost::int8_t scanAngleRank(0);
        if (indexes.ScanAngleRank != -1)
            scanAngleRank = pointBuffer.getRawField<boost::int8_t>(pointIndex, positions.ScanAngleRank);
        
        boost::uint8_t userData(0);
        if (indexes.UserData != -1)
            userData = pointBuffer.getRawField<boost::uint8_t>(pointIndex, positions.UserData);

        boost::uint16_t pointSourceId(0);
        if (indexes.PointSourceId != -1)
            pointSourceId = pointBuffer.getRawField<boost::uint16_t>(pointIndex, indexes.PointSourceId);

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
            
            if (indexes.Time != -1) 
                time = pointBuffer.getRawField<double>(pointIndex, positions.Time);

            Utils::write_field<double>(p, time);
        }

        if (Support::hasColor(pointFormat))
        {
            boost::uint16_t red(0);
            boost::uint16_t green(0);
            boost::uint16_t blue(0);
            
            if (indexes.Red != -1)
                red = pointBuffer.getRawField<boost::uint16_t>(pointIndex, positions.Red);
            if (indexes.Green != -1)
                green = pointBuffer.getRawField<boost::uint16_t>(pointIndex, positions.Green);
            if (indexes.Blue != -1)
                blue = pointBuffer.getRawField<boost::uint16_t>(pointIndex, positions.Blue);
            
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

        const double xValue = xDim.applyScaling<boost::int32_t>(x);
        const double yValue = yDim.applyScaling<boost::int32_t>(y);
        const double zValue = zDim.applyScaling<boost::int32_t>(z);
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

} } } // namespaces
