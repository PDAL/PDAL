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

#include <pdal/exceptions.hpp>
#include <pdal/Stage.hpp>
#include <pdal/SchemaLayout.hpp>
#include <pdal/PointBuffer.hpp>

#include <iostream>

namespace pdal { namespace drivers { namespace las {


IMPLEMENT_STATICS(LasWriter, "drivers.las.writer", "Las Writer")


LasWriter::LasWriter(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_streamManager(options.getOption<std::string>("filename").getValue())
{

    return;
}


LasWriter::LasWriter(Stage& prevStage, std::ostream* ostream)
    : Writer(prevStage, Options::none())
    , m_streamManager(ostream)
    , m_numPointsWritten(0)
{
    return;
}


LasWriter::~LasWriter()
{
    m_streamManager.close();
    return;
}


void LasWriter::initialize()
{
    Writer::initialize();

    m_spatialReference = getPrevStage().getSpatialReference();

    m_streamManager.open();

    try
    {
        Option<bool> compression = getOptions().getOption<bool>("compression");
        setCompressed(compression.getValue());
    }
    catch (pdal::option_not_found const&)
    {
        // if we didn't get a compression option, no compression for you!
    }
    

    return;
}


const Options& LasWriter::s_getDefaultOptions()
{
    static Options options;

    Option<std::string> filename("filename", "", "file to read from");
    Option<bool> compression("compression", false, "Do we LASzip-compress the data?");
    options.add(filename);
    options.add(compression);
    return options;
}


void LasWriter::setCompressed(bool v)
{
    m_lasHeader.SetCompressed(v);
}


void LasWriter::setFormatVersion(boost::uint8_t majorVersion, boost::uint8_t minorVersion)
{
    m_lasHeader.SetVersionMajor(majorVersion);
    m_lasHeader.SetVersionMinor(minorVersion);
}


void LasWriter::setPointFormat(PointFormat pointFormat)
{
    m_lasHeader.setPointFormat(pointFormat);
}


void LasWriter::setDate(boost::uint16_t dayOfYear, boost::uint16_t year)
{
    m_lasHeader.SetCreationDOY(dayOfYear);
    m_lasHeader.SetCreationYear(year);
}


void LasWriter::setProjectId(const boost::uuids::uuid& id)
{
    m_lasHeader.SetProjectId(id);
}


void LasWriter::setSystemIdentifier(const std::string& systemId) 
{
    m_lasHeader.SetSystemId(systemId);
}


void LasWriter::setGeneratingSoftware(const std::string& softwareId)
{
    m_lasHeader.SetSoftwareId(softwareId);
}


void LasWriter::setSpatialReference(const SpatialReference& srs)
{
    m_spatialReference = srs;
}


void LasWriter::writeBegin(boost::uint64_t targetNumPointsToWrite)
{
    // need to set properties of the header here, based on prev->getHeader() and on the user's preferences
    m_lasHeader.setBounds( getPrevStage().getBounds() );

    const Schema& schema = getPrevStage().getSchema();

    int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    const Dimension& dimX = schema.getDimension(indexX);
    const Dimension& dimY = schema.getDimension(indexY);
    const Dimension& dimZ = schema.getDimension(indexZ);

    m_lasHeader.SetScale(dimX.getNumericScale(), dimY.getNumericScale(), dimZ.getNumericScale());
    m_lasHeader.SetOffset(dimX.getNumericOffset(), dimY.getNumericOffset(), dimZ.getNumericOffset());

    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(8);

    boost::uint32_t cnt = static_cast<boost::uint32_t>(targetNumPointsToWrite);
    m_lasHeader.SetPointRecordsCount(cnt);

    m_lasHeader.setSpatialReference(m_spatialReference);

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


void LasWriter::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
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


boost::uint32_t LasWriter::writeBuffer(const PointBuffer& PointBuffer)
{
    const SchemaLayout& schemaLayout = PointBuffer.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();
    PointFormat pointFormat = m_lasHeader.getPointFormat();

    const PointIndexes indexes(schema, pointFormat);

    boost::uint32_t numValidPoints = 0;

    boost::uint8_t buf[1024]; // BUG: fixed size

    for (boost::uint32_t pointIndex=0; pointIndex<PointBuffer.getNumPoints(); pointIndex++)
    {
        boost::uint8_t* p = buf;

        // we always write the base fields
        const boost::uint32_t x = PointBuffer.getField<boost::uint32_t>(pointIndex, indexes.X);
        const boost::uint32_t y = PointBuffer.getField<boost::uint32_t>(pointIndex, indexes.Y);
        const boost::uint32_t z = PointBuffer.getField<boost::uint32_t>(pointIndex, indexes.Z);
        const boost::uint16_t intensity = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.Intensity);
            
        const boost::uint8_t returnNumber = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.ReturnNumber);
        const boost::uint8_t numberOfReturns = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.NumberOfReturns);
        const boost::uint8_t scanDirectionFlag = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.ScanDirectionFlag);
        const boost::uint8_t edgeOfFlightLinet = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.EdgeOfFlightLine);

        const boost::uint8_t bits = returnNumber | (numberOfReturns<<3) | (scanDirectionFlag << 6) | (edgeOfFlightLinet << 7);

        const boost::uint8_t classification = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.Classification);
        const boost::int8_t scanAngleRank = PointBuffer.getField<boost::int8_t>(pointIndex, indexes.ScanAngleRank);
        const boost::uint8_t userData = PointBuffer.getField<boost::uint8_t>(pointIndex, indexes.UserData);
        const boost::uint16_t pointSourceId = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.PointSourceId);

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
            const double time = PointBuffer.getField<double>(pointIndex, indexes.Time);

            Utils::write_field<double>(p, time);
        }

        if (Support::hasColor(pointFormat))
        {
            const boost::uint16_t red = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.Red);
            const boost::uint16_t green = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.Green);
            const boost::uint16_t blue = PointBuffer.getField<boost::uint16_t>(pointIndex, indexes.Blue);

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
            assert(ok);
        }
        else
        {
            Utils::write_n(m_streamManager.ostream(), buf, Support::getPointDataSize(pointFormat));
        }
#else
            Utils::write_n(m_streamManager.ostream(), buf, Support::getPointDataSize(pointFormat));

#endif
        ++numValidPoints;

        const double xValue = schema.getDimension(indexes.X).applyScaling<boost::int32_t>(x);
        const double yValue = schema.getDimension(indexes.Y).applyScaling<boost::int32_t>(y);
        const double zValue = schema.getDimension(indexes.Z).applyScaling<boost::int32_t>(z);
        m_summaryData.addPoint(xValue, yValue, zValue, returnNumber);
    }

    m_numPointsWritten = m_numPointsWritten+numValidPoints;
    return numValidPoints;
}

} } } // namespaces
