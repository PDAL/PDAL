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

#include "HdfReader.hpp"
#include <pdal/util/FileUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <map>

namespace
{
    // const std::vector<pdal::hdf5::Hdf5ColumnData> hdf5Columns =
    // {
    //     { "instrument_parameters/time_hhmmss",  H5::PredType::NATIVE_FLOAT },
    //     { "latitude",                           H5::PredType::NATIVE_FLOAT },
    //     { "longitude",                          H5::PredType::NATIVE_FLOAT },
    //     { "elevation",                          H5::PredType::NATIVE_FLOAT },
    //     { "instrument_parameters/xmt_sigstr",   H5::PredType::NATIVE_INT   },
    //     { "instrument_parameters/rcv_sigstr",   H5::PredType::NATIVE_INT   },
    //     { "instrument_parameters/azimuth",      H5::PredType::NATIVE_FLOAT },
    //     { "instrument_parameters/pitch",        H5::PredType::NATIVE_FLOAT },
    //     { "instrument_parameters/roll",         H5::PredType::NATIVE_FLOAT },
    //     { "instrument_parameters/gps_pdop",     H5::PredType::NATIVE_FLOAT },
    //     { "instrument_parameters/pulse_width",  H5::PredType::NATIVE_FLOAT },
    //     { "instrument_parameters/rel_time",     H5::PredType::NATIVE_FLOAT }
    // };
}

namespace pdal
{

static PluginInfo const s_info
{
    "readers.hdf",
    "HDF Reader (WIP)",
    "http://pdal.io/"
};

CREATE_SHARED_STAGE(HdfReader, s_info)

std::string HdfReader::getName() const { return s_info.name; }


void HdfReader::addDimensions(PointLayoutPtr layout)
{
    // layout->registerDims(dimensions());
    std::cout << "HdfReader::addDimensions begin" << std::endl;
    m_infos = m_hdf5Handler.getDimensionInfos();
    for(int i = 0; i < m_infos.size(); i++) {
        // Dimension::BaseType b = Dimension::BaseType::None;
        // Dimension::Type t = Dimension::Type::None;
        // if(info.hdf_type == H5T_INTEGER) {
        //     if(info.sign == H5T_SGN_NONE) {
        //         b = Dimension::BaseType::Unsigned;
        //     }
        //     else if(info.sign == H5T_SGN_2) {
        //         b = Dimension::BaseType::Signed;
        //     }
        // } else if(info.hdf_type == H5T_FLOAT) {
        //     b = Dimension::BaseType::Floating;
        // } else {
        //     throwError("Invalid hdf type");
        // }

        // t = Dimension::Type(unsigned(b) | info.size);
        // m_idlist.push_back(
        std::cout << unsigned(m_infos[i].id) << " : ";
        m_infos[i].id = layout->registerOrAssignDim(m_infos[i].name, m_infos[i].pdal_type);
        std::cout << unsigned(m_infos[i].id) << std::endl;
        // ); // TODO: add correct type
    };
    for(auto info: m_infos) {
        std::cout << unsigned(info.id) << std::endl;
    };
    std::cout << "HdfReader::addDimensions end" << std::endl;
}


void HdfReader::ready(PointTableRef table)
{
    std::cout << "HdfReader::ready" << std::endl;
    try
    {
        // m_hdf5Handler.initialize(m_filename, hdf5Columns);
        // m_hdf5Handler.initialize(m_filename);
    }
    catch (const Hdf5Handler::error& err)
    {
        throwError(err.what());
    }
    m_index = 0;
}


point_count_t HdfReader::read(PointViewPtr view, point_count_t count)
{
    //All data we read for icebridge is currently 4 bytes wide, so
    //  just allocate once and forget it.
    //This could be a huge allocation.  Perhaps we should do something
    //  in the icebridge handler?
    std::cout << "HdfReader::read" << std::endl;
    size_t point_size = 52; //TODO Fix
    PointId startId = view->size();
    point_count_t remaining = m_hdf5Handler.getNumPoints() - m_index;
    count = (std::min)(count, remaining);

    // std::unique_ptr<unsigned char>
    //     rawData(new unsigned char[count * point_size]);

    // for(std::size_t di = 0; di < m_idlist.size(); di++) {
        // Dimension::Id dimId = m_idlist[di];
    // std::cout << m_idlist.size() << std::endl;
    // for(auto dimId : m_idlist) {
    for(auto info : m_infos) {
        PointId nextId = startId;
        std::cout << (unsigned)info.id << ": ";
        for(uint64_t pi = 0; pi < m_hdf5Handler.getNumPoints(); pi++) {
            // void *p = (void *)rawData.get() + 0*pi*point_size + info.offset;
            void *p = m_hdf5Handler.getBuffer() + pi*point_size + info.offset;
            switch(info.pdal_type) {
                case Dimension::Type::Double:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((double *) p));
                    break;
                case Dimension::Type::Float:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((float *) p));
                    break;
                case Dimension::Type::Signed8:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((int8_t *) p));
                    break;
                case Dimension::Type::Signed16:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((int16_t *) p));
                    break;
                case Dimension::Type::Signed32:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((int32_t *) p));
                    break;
                case Dimension::Type::Signed64:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((int64_t *) p));
                    break;
                 case Dimension::Type::Unsigned8:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((uint8_t *) p));
                    break;
                case Dimension::Type::Unsigned16:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((uint16_t *) p));
                    break;
                case Dimension::Type::Unsigned32:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((uint32_t *) p));
                    break;
                case Dimension::Type::Unsigned64:
                    if(pi == 0) std::cout<< Dimension::interpretationName(info.pdal_type) <<std::endl;
                    view->setField(info.id, nextId++, * ((uint64_t *) p));
                    break;
                default:
                    view->setField(info.id, nextId, 0);
                    break;
            }
        }
    }

    //Not loving the position-linked data, but fine for now.
    // Dimension::IdList dims = dimensions();
    // auto di = dims.begin();
    // for (auto ci = hdf5Columns.begin(); ci != hdf5Columns.end(); ++ci, ++di)
    // {
    //     PointId nextId = startId;
    //     PointId idx = m_index;
    //     const hdf5::Hdf5ColumnData& column = *ci;

    //     try
    //     {
    //         m_hdf5Handler.getColumnEntries(rawData.get(), column.name, count,
    //             m_index);
    //         void *p = (void *)rawData.get();

    //         // This is ugly but avoids a test in a tight loop.
    //         if (column.predType == H5::PredType::NATIVE_FLOAT)
    //         {
    //             // Offset time is in ms but icebridge stores in seconds.
    //             if (*di == Dimension::Id::OffsetTime)
    //             {
    //                 float *fval = (float *)p;
    //                 for (PointId i = 0; i < count; ++i)
    //                 {
    //                     view->setField(*di, nextId++, *fval * 1000);
    //                     fval++;
    //                 }
    //             }
    //             else if (*di == Dimension::Id::X)
    //             {
    //                 float *fval = (float *)p;
    //                 for (PointId i = 0; i < count; ++i)
    //                 {
    //                     double dval = (double)(*fval);
    //                     // Longitude is 0-360. Convert
    //                     dval = Utils::normalizeLongitude(dval);
    //                     view->setField(*di, nextId++, dval);
    //                     fval++;
    //                 }

    //             }
    //             else
    //             {
    //                 float *fval = (float *)p;
    //                 for (PointId i = 0; i < count; ++i)
    //                     view->setField(*di, nextId++, *fval++);
    //             }
    //         }
    //         else if (column.predType == H5::PredType::NATIVE_INT)
    //         {
    //             int32_t *ival = (int32_t *)p;
    //             for (PointId i = 0; i < count; ++i)
    //                 view->setField(*di, nextId++, *ival++);
    //         }
    //     }
        // catch(const Hdf5Handler::error& err)
        // {
            // throwError(err.what());
        // }
    // }
    return count;
}

void HdfReader::addArgs(ProgramArgs& args)
{
    // args.add("metadata", "Metadata file", m_metadataFile);
    args.add("dataset", "HDF dataset to open", m_datasetName);
}

void HdfReader::initialize()
{
    std::cout << "HdfReader::initialize()" << std::endl;
    if (!m_metadataFile.empty() && !FileUtils::fileExists(m_metadataFile))
    {
        throwError("Invalid metadata file: '" + m_metadataFile + "'");
    }
    m_hdf5Handler.initialize(m_filename);

    // Data are WGS84 (4326) with ITRF2000 datum (6656)
    // See http://nsidc.org/data/docs/daac/icebridge/ilvis2/index.html for
    // background
    setSpatialReference(SpatialReference("EPSG:4326"));
}

void HdfReader::done(PointTableRef table)
{
    m_hdf5Handler.close();
}


bool HdfReader::eof()
{
    return m_index >= m_hdf5Handler.getNumPoints();
}

} // namespace pdal
