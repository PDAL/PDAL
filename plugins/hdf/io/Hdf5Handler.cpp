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

#include "Hdf5Handler.hpp"
#include <pdal/util/FileUtils.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/Dimension.hpp>

namespace pdal
{
// Dimension::Type getPdalType(DimInfo info) {
//     Dimension::BaseType b = Dimension::BaseType::None;
//     Dimension::Type t = Dimension::Type::None;
//     if(info.hdf_type == H5T_INTEGER) {
//         if(info.sign == H5T_SGN_NONE) {
//             b = Dimension::BaseType::Unsigned;
//         }
//         else if(info.sign == H5T_SGN_2) {
//             b = Dimension::BaseType::Signed;
//         }
//     } else if(info.hdf_type == H5T_FLOAT) {
//         b = Dimension::BaseType::Floating;
//     } else {
//         throwError("Invalid hdf type");
//     }

//     t = Dimension::Type(unsigned(b) | info.size);
//     return t;
// }
using namespace hdf5;



Hdf5Handler::Hdf5Handler()
    : m_chunkOffset(0)
    , m_numPoints(0)
    , m_columnDataMap()
{ }

void Hdf5Handler::setLog(pdal::LogPtr log) {
    m_logger = log;
}

DimInfo::DimInfo(
    const std::string& dimName,
    H5::IntType int_type)
{
    name = dimName;
    hdf_type = H5T_INTEGER;
    endianness = int_type.getOrder();
    sign = int_type.getSign();
    compound_size = int_type.getSize();
    member_size = int_type.getSize();
    offset = 0;
    pdal_type = sign == H5T_SGN_2 ?
        Dimension::Type(unsigned(Dimension::BaseType::Signed) | int_type.getSize()) :
        Dimension::Type(unsigned(Dimension::BaseType::Unsigned) | int_type.getSize());
}

DimInfo::DimInfo(
    const std::string& dimName,
    H5::FloatType float_type)
{
    name = dimName;
    hdf_type = H5T_FLOAT;
    endianness = float_type.getOrder();
    sign = H5T_SGN_ERROR;
    compound_size = float_type.getSize();
    member_size = float_type.getSize();
    offset = 0;
    pdal_type = Dimension::Type(unsigned(Dimension::BaseType::Floating) | float_type.getSize());
}

void Hdf5Handler::initialize(
        const std::string& filename,
        const std::string& dimName,
        const std::string& datasetName)
{
    try
    {
        m_h5File.reset(new H5::H5File(filename, H5F_ACC_RDONLY));
    }
    catch (const H5::FileIException&)
    {
        throw error("Could not open HDF5 file '" + filename + "'.");
    }

    m_dset = m_h5File.get()->openDataSet(datasetName);
    m_dspace = m_dset.getSpace();
    m_numPoints = m_dspace.getSelectNpoints();
    H5::DSetCreatPropList plist = m_dset.getCreatePlist();
    if(plist.getLayout() == H5D_CHUNKED) {
        int dimensionality = plist.getChunk(1, &m_chunkSize);
        if(dimensionality != 1)
            throw error("Only 1-dimensional arrays are supported.");
    } else {
        m_logger->get(LogLevel::Warning) << "Dataset not chunked; proceeding to read 1024 elements at a time" << std::endl;
        m_chunkSize = 1024;
    }
    m_logger->get(LogLevel::Warning) << "Chunk size: " << m_chunkSize << std::endl;
    m_logger->get(LogLevel::Warning) << "Num points: " << m_numPoints << std::endl;
    m_logger->get(LogLevel::Warning) << "Number of dataspace dimensions: " << m_dspace.getSimpleExtentNdims() << std::endl;
    H5::DataType dtype = m_dset.getDataType();
    H5T_class_t vauge_type = dtype.getClass();

    if(vauge_type == H5T_COMPOUND) {
        throw error("Compound types not supported");
    }
    else if(vauge_type == H5T_INTEGER) {
        m_dimInfos.push_back(
            DimInfo(
                dimName.empty() ? datasetName : dimName,
                m_dset.getIntType()
            )
        );
    }
    else if(vauge_type == H5T_FLOAT) {
        m_dimInfos.push_back(
            DimInfo(
                dimName.empty() ? datasetName : dimName,
                m_dset.getFloatType()
            )
        );
    } else {
        throw error("Unkown type: " + vauge_type);
    }
    // m_buf = malloc(dtype.getSize() * m_chunkSize); //TODO free
    // m_buf = malloc(dtype.getSize() * m_chunkSize); //TODO free
    m_data.resize(m_chunkSize*dtype.getSize());
    m_logger->get(LogLevel::Warning) << "m_data.size: " << m_data.size() << std::endl;
    m_logger->get(LogLevel::Warning) << "Chunk offset: " << m_chunkOffset << std::endl;
    // dspace.selectElements(H5S_SELECT_SET, m_chunkSize, &m_chunkOffset);
    // dspace.selectHyperslab(H5S_SELECT_SET, &m_chunkSize, &m_chunkOffset);
    // H5::DataSpace mspace( 1, &m_chunkOffset);
    // dset.read(m_buf, dtype, H5::DataSpace::ALL, dspace);
}

void Hdf5Handler::close()
{
    m_h5File->close();
}

// void *Hdf5Handler::getNextChunk() {
//     void *buf = malloc(m_dset.getDataType().getSize() * m_chunkSize);
//     m_logger->get(LogLevel::Warning) << "chunk size: " << m_chunkSize << ", chunk offset: "
//         << m_chunkOffset << std::endl;
//     m_dspace.selectHyperslab(H5S_SELECT_SET, &m_chunkSize, &m_chunkOffset);
//     m_dset.read(buf, m_dset.getDataType(), H5::DataSpace::ALL, m_dspace);
//     // m_dset.read(m_buf, m_dset.getDataType());
//     m_chunkOffset += m_chunkSize;
//     return buf;
// }

void *Hdf5Handler::getNextChunk() {
    // m_logger->get(LogLevel::Warning) << "chunk size: " << m_chunkSize << ", chunk offset: "
    //     << m_chunkOffset << std::endl;
    hsize_t elementsRemaining = m_numPoints - m_chunkOffset;
    // if(elementsRemaining < m_chunkSize) {
    //     selectionSize = elementsRemaining;
    // }
    hsize_t selectionSize = std::min(elementsRemaining, m_chunkSize);

    // m_logger->get(LogLevel::Warning) << "Points remainging: " << elementsRemaining;
    H5::DataSpace memspace(1, &selectionSize);
    m_dspace.selectHyperslab(H5S_SELECT_SET, &selectionSize, &m_chunkOffset);
    // m_logger->get(LogLevel::Warning) << "m_data: " << (void *)m_data.data() << std::endl;
    // m_logger->get(LogLevel::Warning) << "chunkOffset: " << m_chunkOffset << std::endl;
    // m_logger->get(LogLevel::Warning) << "chunkSize: " << selectionSize << std::endl;
    m_dset.read(m_data.data(),
                m_dset.getDataType(),
                memspace,
                m_dspace );
    m_chunkOffset += m_chunkSize;
    // m_logger->get(LogLevel::Warning) << "m_data[0] = " << *((double *)m_data.data()) << std::endl;
    return m_data.data();
}


uint64_t Hdf5Handler::getNumPoints() const
{
    return m_numPoints;
}

void Hdf5Handler::getColumnEntries(
        void* data,
        const std::string& dataSetName,
        const hsize_t numEntries,
        const hsize_t offset) const
{
    try
    {
        const ColumnData& columnData(getColumnData(dataSetName));

        columnData.dataSpace.selectHyperslab(
                H5S_SELECT_SET,
                &numEntries,
                &offset);

        const hsize_t outOffset = 0;
        const H5::DataSpace outSpace(1, &numEntries);
        outSpace.selectHyperslab(H5S_SELECT_SET, &numEntries, &outOffset);

        columnData.dataSet.read(
                data,
                columnData.predType,
                outSpace,
                columnData.dataSpace);
    }
    catch (const H5::Exception&)
    {
        throw error("Could not read from dataset.");
    }
}

std::vector<pdal::hdf5::DimInfo> Hdf5Handler::getDimensionInfos() {
    return m_dimInfos;
}
void *
Hdf5Handler::getBuffer() {
    return m_buf;
}

hsize_t
Hdf5Handler::getColumnNumEntries(const std::string& dataSetName) const
{
    hsize_t entries = 0;

    getColumnData(dataSetName).dataSpace.getSimpleExtentDims(&entries);

    return entries;
}

const Hdf5Handler::ColumnData&
Hdf5Handler::getColumnData(const std::string& dataSetName) const
{
    const auto columnDataIt(m_columnDataMap.find(dataSetName));

    if (columnDataIt == m_columnDataMap.end())
    {
        throw error("Could not retrieve column data.");
    }

    return columnDataIt->second;
}

} // namespace pdal

