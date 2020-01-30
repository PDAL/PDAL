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
    : m_numPoints(0)
    , m_columnDataMap()
{ }

void Hdf5Handler::initialize(
        const std::string& filename)
{
    try
    {
        m_h5File.reset(new H5::H5File(filename, H5F_ACC_RDONLY));

        std::cout << "Number of HD5 Objects: " << m_h5File.get()->getObjCount() <<std::endl;
        H5::DataSet dset = m_h5File.get()->openDataSet("/autzen");
        H5::DataSpace dspace = dset.getSpace();
        m_numPoints = 1065; //TODO FIX
        std::cout << "Number of dataspace dimensions: " << dspace.getSimpleExtentNdims() << std::endl;
        H5::CompType ctype = dset.getCompType();//H5::CompType(dset);
        std::cout << "Number of points: " << m_numPoints << std::endl;
        std::cout << "Point length: " << ctype.getSize() << std::endl;
        std::cout << "Number of HDF compound type members (PDAL dimensions): " << ctype.getNmembers() << std::endl;
        m_buf = malloc(ctype.getSize() * m_numPoints); //TODO free
        dset.read(m_buf, ctype);
        // print names
        for(int j = 0; j < ctype.getNmembers(); ++j) {
            // m_dimNames.push_back(ctype.getMemberName(j));

            H5T_class_t vauge_type = ctype.getMemberDataType(j).getClass();
            H5::IntType int_type = ctype.getMemberIntType(j);
            H5::FloatType float_type = ctype.getMemberFloatType(j);
            switch(vauge_type) {
                case H5T_COMPOUND:
                    std::cout << "Compound type";
                    std::cout << "Nested compound types not supported" << std::endl;
                    break;
                case H5T_INTEGER:
                    if(int_type.getSign() == H5T_SGN_NONE) {
                        m_dimInfos.push_back(DimInfo(
                            ctype.getMemberName(j),
                            vauge_type,
                            int_type.getOrder(),
                            int_type.getSign(),
                            int_type.getSize(),
                            ctype.getSize(),
                            ctype.getMemberOffset(j),
                            Dimension::Type(unsigned(Dimension::BaseType::Unsigned) | int_type.getSize()))
                        );
                        std::cout << "uint,  s:" << int_type.getSize() << ", e:" << int_type.getOrder();
                    } else if(int_type.getSign() == H5T_SGN_2) {
                        m_dimInfos.push_back(DimInfo(
                            ctype.getMemberName(j),
                            vauge_type,
                            int_type.getOrder(),
                            int_type.getSign(),
                            int_type.getSize(),
                            ctype.getSize(),
                            ctype.getMemberOffset(j),
                            Dimension::Type(unsigned(Dimension::BaseType::Signed) | int_type.getSize()))
                        );
                        std::cout << "sint,  s:" << int_type.getSize() << ", e:" << int_type.getOrder();
                    } else {
                        std::cout << "sign error";
                    }
                    break;
                case H5T_FLOAT:
                    m_dimInfos.push_back(DimInfo(
                        ctype.getMemberName(j),
                        vauge_type,
                        float_type.getOrder(),
                        H5T_SGN_ERROR,
                        ctype.getSize(),
                        float_type.getSize(),
                        ctype.getMemberOffset(j),
                        Dimension::Type(unsigned(Dimension::BaseType::Floating) | float_type.getSize()))
                    );
                    std::cout << "float, s:" << float_type.getSize() << ", e:" << float_type.getOrder();
                    break;
                default:
                    std::cout << "Unkown type: " << vauge_type;
            }
            std::cout << ", o:" << ctype.getMemberOffset(j) << ", " << ctype.getMemberName(j);
            std::cout  << std::endl;


        }
    }
    catch (const H5::FileIException&)
    {
        throw error("Could not open HDF5 file '" + filename + "'.");
    }

    try
    {
        // Open each HDF5 DataSet and its corresponding DataSpace.
        // for (const auto& col : columns)
        // {
        //     const std::string dataSetName = col.name;
        //     const H5::PredType predType = col.predType;
        //     const H5::DataSet dataSet = m_h5File->openDataSet(dataSetName);
        //     const H5::DataSpace dataSpace = dataSet.getSpace();

        //     m_columnDataMap.insert(std::make_pair(
        //                 dataSetName,
        //                 ColumnData(predType, dataSet, dataSpace)));

        //     // Does not check whether all the columns are the same length.
        //     m_numPoints = (std::max)((uint64_t)getColumnNumEntries(dataSetName),
        //         m_numPoints);
        // }
    }
    catch (const H5::Exception&)
    {
        throw error("Could not initialize data set information.");
    }
}

void Hdf5Handler::close()
{
    m_h5File->close();
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

