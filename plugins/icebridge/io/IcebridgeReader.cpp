/******************************************************************************
* Copyright (c) 2014, Connor Manning, connor@hobu.co
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

#include "IcebridgeReader.hpp"
#include <pdal/util/FileUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <map>

namespace
{
    const std::vector<pdal::hdf5::Hdf5ColumnData> hdf5Columns =
    {
        { "instrument_parameters/time_hhmmss",  H5::PredType::NATIVE_FLOAT },
        { "latitude",                           H5::PredType::NATIVE_FLOAT },
        { "longitude",                          H5::PredType::NATIVE_FLOAT },
        { "elevation",                          H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/xmt_sigstr",   H5::PredType::NATIVE_INT   },
        { "instrument_parameters/rcv_sigstr",   H5::PredType::NATIVE_INT   },
        { "instrument_parameters/azimuth",      H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/pitch",        H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/roll",         H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/gps_pdop",     H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/pulse_width",  H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/rel_time",     H5::PredType::NATIVE_FLOAT }
    };
}

namespace pdal
{

static PluginInfo const s_info
{
    "readers.icebridge",
    "NASA HDF5-based IceBridge ATM reader. \n" \
        "See http://nsidc.org/data/docs/daac/icebridge/ilatm1b/index.html \n" \
        "for more information.",
    "http://pdal.io/stages/readers.icebridge.html"
};

CREATE_SHARED_STAGE(IcebridgeReader, s_info)

std::string IcebridgeReader::getName() const { return s_info.name; }

namespace
{

Dimension::IdList dimensions()
{
    using namespace Dimension;

    return IdList { Id::OffsetTime, Id::Y, Id::X, Id::Z, Id::StartPulse,
        Id::ReflectedPulse, Id::Azimuth, Id::Pitch, Id::Roll, Id::Pdop,
        Id::PulseWidth, Id::GpsTime };
}

} // unnamed namespace

void IcebridgeReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(dimensions());
}


void IcebridgeReader::ready(PointTableRef table)
{
    try
    {
        m_hdf5Handler.initialize(m_filename, hdf5Columns);
    }
    catch (const Hdf5Handler::error& err)
    {
        throwError(err.what());
    }
    m_index = 0;
    if (!m_metadataFile.empty())
    {
        m_mdReader.readMetadataFile(m_metadataFile, &m_metadata);
    }
}


point_count_t IcebridgeReader::read(PointViewPtr view, point_count_t count)
{
    //All data we read for icebridge is currently 4 bytes wide, so
    //  just allocate once and forget it.
    //This could be a huge allocation.  Perhaps we should do something
    //  in the icebridge handler?

    PointId startId = view->size();
    point_count_t remaining = m_hdf5Handler.getNumPoints() - m_index;
    count = (std::min)(count, remaining);

    std::unique_ptr<unsigned char>
        rawData(new unsigned char[count * sizeof(float)]);

    //Not loving the position-linked data, but fine for now.
    Dimension::IdList dims = dimensions();
    auto di = dims.begin();
    for (auto ci = hdf5Columns.begin(); ci != hdf5Columns.end(); ++ci, ++di)
    {
        PointId nextId = startId;
        PointId idx = m_index;
        const hdf5::Hdf5ColumnData& column = *ci;

        try
        {
            m_hdf5Handler.getColumnEntries(rawData.get(), column.name, count,
                m_index);
            void *p = (void *)rawData.get();

            // This is ugly but avoids a test in a tight loop.
            if (column.predType == H5::PredType::NATIVE_FLOAT)
            {
                // Offset time is in ms but icebridge stores in seconds.
                if (*di == Dimension::Id::OffsetTime)
                {
                    float *fval = (float *)p;
                    for (PointId i = 0; i < count; ++i)
                    {
                        view->setField(*di, nextId++, *fval * 1000);
                        fval++;
                    }
                }
                else if (*di == Dimension::Id::X)
                {
                    float *fval = (float *)p;
                    for (PointId i = 0; i < count; ++i)
                    {
                        double dval = (double)(*fval);
                        // Longitude is 0-360. Convert
                        dval = Utils::normalizeLongitude(dval);
                        view->setField(*di, nextId++, dval);
                        fval++;
                    }

                }
                else
                {
                    float *fval = (float *)p;
                    for (PointId i = 0; i < count; ++i)
                        view->setField(*di, nextId++, *fval++);
                }
            }
            else if (column.predType == H5::PredType::NATIVE_INT)
            {
                int32_t *ival = (int32_t *)p;
                for (PointId i = 0; i < count; ++i)
                    view->setField(*di, nextId++, *ival++);
            }
        }
        catch(const Hdf5Handler::error& err)
        {
            throwError(err.what());
        }
    }
    return count;
}

void IcebridgeReader::addArgs(ProgramArgs& args)
{
    args.add("metadata", "Metadata file", m_metadataFile);
}

void IcebridgeReader::initialize()
{
    if (!m_metadataFile.empty() && !FileUtils::fileExists(m_metadataFile))
    {
        throwError("Invalid metadata file: '" + m_metadataFile + "'");
    }

    // Data are WGS84 (4326) with ITRF2000 datum (6656)
    // See http://nsidc.org/data/docs/daac/icebridge/ilvis2/index.html for
    // background
    setSpatialReference("EPSG:4326");
}

void IcebridgeReader::done(PointTableRef table)
{
    m_hdf5Handler.close();
}


bool IcebridgeReader::eof()
{
    return m_index >= m_hdf5Handler.getNumPoints();
}

} // namespace pdal
