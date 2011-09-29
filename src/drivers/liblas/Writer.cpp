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

#include <pdal/drivers/liblas/Writer.hpp>

#include <pdal/external/boost/uuid/uuid_io.hpp>

#include <liblas/header.hpp>
#include <liblas/writer.hpp>

#include <pdal/pdal_config.hpp>
#include <pdal/Schema.hpp>
#include <pdal/Stage.hpp>
#include <pdal/PointBuffer.hpp>

#include <pdal/drivers/las/Support.hpp>


namespace pdal { namespace drivers { namespace liblas {


Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_ostreamManager(options.getValueOrThrow<std::string>("filename"))
{
    return;
}


Writer::Writer(Stage& prevStage, std::ostream* ostream)
    : pdal::Writer(prevStage, Options::none())
    , m_ostreamManager(ostream)
    , m_externalWriter(NULL)
{
    return;
}


Writer::~Writer()
{
    return;
}


void Writer::initialize()
{
    pdal::Writer::initialize();

    m_ostreamManager.open();

    m_externalHeader = ::liblas::HeaderPtr(new ::liblas::Header);
    m_externalHeader->SetCompressed(false);

    setupExternalHeader();

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
    return options;
}


void Writer::setupExternalHeader()
{
    setFormatVersion(1,2);
    setPointFormat(::pdal::drivers::las::PointFormat3);

    setCompressed(false);

    setSystemIdentifier("PDAL");
    setGeneratingSoftware(GetVersionString());

    const Schema& schema = getPrevStage().getSchema();

    const Dimension& dimX = schema.getDimension(DimensionId::X_i32);
    const Dimension& dimY = schema.getDimension(DimensionId::Y_i32);
    const Dimension& dimZ = schema.getDimension(DimensionId::Z_i32);
    m_externalHeader->SetScale(dimX.getNumericScale(), dimY.getNumericScale(), dimZ.getNumericScale());
    m_externalHeader->SetOffset(dimX.getNumericOffset(), dimY.getNumericOffset(), dimZ.getNumericOffset());

    return;
}


void Writer::setCompressed(bool v)
{
    m_externalHeader->SetCompressed(v);
}

void Writer::setHeaderPadding(boost::uint32_t const& v)
{
    m_externalHeader->SetHeaderPadding(v);
}

void Writer::setFormatVersion(boost::uint8_t majorVersion, boost::uint8_t minorVersion)
{
    m_externalHeader->SetVersionMajor(majorVersion);
    m_externalHeader->SetVersionMinor(minorVersion);
}


void Writer::setPointFormat(::pdal::drivers::las::PointFormat pointFormat)
{
    m_externalHeader->SetDataFormatId((::liblas::PointFormatName)pointFormat);
}


void Writer::setDate(boost::uint16_t dayOfYear, boost::uint16_t year)
{
    m_externalHeader->SetCreationDOY(dayOfYear);
    m_externalHeader->SetCreationYear(year);
}


void Writer::setProjectId(const pdal::external::boost::uuids::uuid& uuid)
{
    std::string s = to_string(uuid);
    ::liblas::guid g(s);
    m_externalHeader->SetProjectId(g);
}


void Writer::setSystemIdentifier(const std::string& systemId) 
{
    m_externalHeader->SetSystemId(systemId);
}


void Writer::setGeneratingSoftware(const std::string& softwareId)
{
    m_externalHeader->SetSoftwareId(softwareId);
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    m_externalWriter = new ::liblas::Writer(m_ostreamManager.ostream(), *m_externalHeader);

    m_summaryData.reset();

    return;
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    delete m_externalWriter;
    m_externalWriter = NULL;

    //std::cout << m_summaryData;

    m_ostreamManager.ostream().seekp(0);
    ::pdal::drivers::las::Support::rewriteHeader(m_ostreamManager.ostream(), m_summaryData);

    return;
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& PointBuffer)
{
    const ::pdal::drivers::las::PointFormat pointFormat = (::pdal::drivers::las::PointFormat)m_externalHeader->GetDataFormatId();

    const Schema& schema = PointBuffer.getSchema();

    const pdal::drivers::las::PointIndexes indexes(schema, pointFormat);

    ::liblas::Point pt(m_externalHeader);

    boost::uint32_t numPoints = PointBuffer.getNumPoints();
    for (boost::uint32_t i=0; i<numPoints; i++)
    {
        const boost::int32_t x = PointBuffer.getField<boost::int32_t>(i, indexes.X);
        const boost::int32_t y = PointBuffer.getField<boost::int32_t>(i, indexes.Y);
        const boost::int32_t z = PointBuffer.getField<boost::int32_t>(i, indexes.Z);
        pt.SetRawX(x);
        pt.SetRawY(y);
        pt.SetRawZ(z);

        const boost::uint16_t intensity = PointBuffer.getField<boost::uint16_t>(i, indexes.Intensity);
        const boost::int8_t returnNumber = PointBuffer.getField<boost::int8_t>(i, indexes.ReturnNumber);
        const boost::int8_t numberOfReturns = PointBuffer.getField<boost::int8_t>(i, indexes.NumberOfReturns);
        const boost::int8_t scanDirFlag = PointBuffer.getField<boost::int8_t>(i, indexes.ScanDirectionFlag);
        const boost::int8_t edgeOfFlightLine = PointBuffer.getField<boost::int8_t>(i, indexes.EdgeOfFlightLine);
        const boost::uint8_t classification = PointBuffer.getField<boost::uint8_t>(i, indexes.Classification);
        const boost::int8_t scanAngleRank = PointBuffer.getField<boost::int8_t>(i, indexes.ScanAngleRank);
        const boost::uint8_t userData = PointBuffer.getField<boost::uint8_t>(i, indexes.UserData);
        const boost::uint16_t pointSourceId = PointBuffer.getField<boost::uint16_t>(i, indexes.PointSourceId);
        pt.SetIntensity(intensity);
        pt.SetReturnNumber(returnNumber);
        pt.SetNumberOfReturns(numberOfReturns);
        pt.SetScanDirection(scanDirFlag);
        pt.SetFlightLineEdge(edgeOfFlightLine);
        pt.SetClassification(classification);
        pt.SetScanAngleRank(scanAngleRank);
        pt.SetUserData(userData);
        pt.SetPointSourceID(pointSourceId);

        if (::pdal::drivers::las::Support::hasTime(pointFormat))
        {
            const double time = PointBuffer.getField<double>(i, indexes.Time);
            pt.SetTime(time);
        }

        if (::pdal::drivers::las::Support::hasColor(pointFormat))
        {
            const boost::uint16_t red = PointBuffer.getField<boost::uint16_t>(i, indexes.Red);
            const boost::uint16_t green = PointBuffer.getField<boost::uint16_t>(i, indexes.Green);
            const boost::uint16_t blue = PointBuffer.getField<boost::uint16_t>(i, indexes.Blue);
            ::liblas::Color color(red, green, blue);
            pt.SetColor(color);
        }

        if (::pdal::drivers::las::Support::hasWave(pointFormat))
        {
            assert(false);
        }

        bool ok = m_externalWriter->WritePoint(pt);
        assert(ok); // BUG

        const double xValue = schema.getDimension(DimensionId::X_i32).applyScaling<boost::int32_t>(x);
        const double yValue = schema.getDimension(DimensionId::Y_i32).applyScaling<boost::int32_t>(y);
        const double zValue = schema.getDimension(DimensionId::Z_i32).applyScaling<boost::int32_t>(z);

        m_summaryData.addPoint(xValue, yValue, zValue, returnNumber);
    }

    return numPoints;
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

} } } // namespaces
