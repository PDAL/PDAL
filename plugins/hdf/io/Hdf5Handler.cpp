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

namespace pdal
{

using namespace hdf5;

Hdf5Handler::Hdf5Handler()
    : m_numPoints(0)
    , m_columnDataMap()
{ }

void Hdf5Handler::initialize(
        const std::string& filename,
        const std::vector<Hdf5ColumnData>& columns)
{
    try
    {
        int numPoints = 1065; // TODO pull this out of the file
        m_h5File.reset(new H5::H5File(filename, H5F_ACC_RDONLY));

        std::cout << "Number of HD5 Objects: " << m_h5File.get()->getObjCount() <<std::endl;
        H5::DataSet dset = m_h5File.get()->openDataSet("/autzen");
        H5::DataSpace dspace = dset.getSpace();
        std::cout << "Number of dataspacedimensions: " << dspace.getSimpleExtentNdims() << std::endl;
        H5::CompType ctype = dset.getCompType();//H5::CompType(dset);
        std::cout << "Point length: " << ctype.getSize() << std::endl;
        std::cout << "Number of HDF compound type members (PDAL dimensions): " << ctype.getNmembers() << std::endl;

        // print names
        for(int j = 0; j < ctype.getNmembers(); ++j) {
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
                        std::cout << "uint,  s:" << int_type.getSize() << ", e:" << int_type.getOrder();
                    } else if(int_type.getSign() == H5T_SGN_2) {
                        std::cout << "sint,  s:" << int_type.getSize() << ", e:" << int_type.getOrder();
                    } else {
                        std::cout << "sign error";
                    }
                    break;
                case H5T_FLOAT:
                    std::cout << "float, s:" << float_type.getSize() << ", e:" << float_type.getOrder();
                    break;
                default:
                    std::cout << "Unkown type: " << vauge_type;
            }
            std::cout << ", o:" << ctype.getMemberOffset(j) << ", " << ctype.getMemberName(j);
            std::cout  << std::endl;
        }

        // std::vector<char> data;
        //char data[numPoints*s];
        // autzen_t* data = new autzen_t[numPoints];
        // dset.read(data.data(), ctype, dspace);

        // std::cout << data.data() << std::endl << std::endl;
/*
        // std::cout << "Size of compound type: " << ctype.getSize() << std::endl;
        if(type_class == H5T_COMPOUND) std::cout << "Compound type" <<std::endl;

        H5::DataType dtype = dset.getDataType();
        H5T_class_t clas = dtype.getClass();
        std::cout << "clas: " << clas << std::endl;
        std::cout << "THING: " << H5Tget_native_type(clas, H5T_DIR_DEFAULT) << std::endl;
        // std::cout << dtype << std::endl;
        //dset.read(data, dtype, dspace);
        ctype.insertMember("red"  , HOFFSET(autzen_t, red), H5::PredType::STD_I16LE);
        ctype.insertMember("green", HOFFSET(autzen_t, green), H5::PredType::STD_I16LE);
        ctype.insertMember("blue" , HOFFSET(autzen_t, blue), H5::PredType::STD_I16LE);
        dset.read(data, ctype, dspace);
        // std::cout << dspace.getNumMembers(h5type) << std::endl;
        // char hex[2];
        // for(char *p = data; p < data+s*numPoints; p++) {
        //     sprintf(hex, "%X", *p);
        //     // std::cout << hex;
        // }

        autzen_t *struct_data = (autzen_t *) data;
        for(int j = 0; j < numPoints; j++) {
            autzen_t point = struct_data[j];
            std::cout << "Point number: " << j << ", RGB: " << point.red << ", " << point.green << ", " << point.blue << std::endl;
        }

        std::cout << std::endl;
        std::cout << "Got here!" << std::endl;
        */

        // auto accessPlist = m_h5File.get()->getAccessPlist();
        // for(auto i = 0; i < accessPlist.get) {

        // }

        // hid_t *p = (hid_t *)malloc(sizeof(hid_t) * objCount);
        // for(auto i = 0; i < objCount; ++i) {
        //     std::cout << "p[" << i << "]: " << p[i] << std::endl;
        // }
        // m_h5File.get()->getObjIDs(H5F_OBJ_DATASET, INT32_MAX, p);
        // for(auto i = 0; i < objCount; ++i) {
        //     auto thing = p[i];
        //     std::cout << "p[" << i << "]: " << thing << std::endl;
        // }
    }
    catch (const H5::FileIException&)
    {
        throw error("Could not open HDF5 file '" + filename + "'.");
    }

    try
    {
        // Open each HDF5 DataSet and its corresponding DataSpace.
        for (const auto& col : columns)
        {
            const std::string dataSetName = col.name;
            const H5::PredType predType = col.predType;
            const H5::DataSet dataSet = m_h5File->openDataSet(dataSetName);
            const H5::DataSpace dataSpace = dataSet.getSpace();

            m_columnDataMap.insert(std::make_pair(
                        dataSetName,
                        ColumnData(predType, dataSet, dataSpace)));

            // Does not check whether all the columns are the same length.
            m_numPoints = (std::max)((uint64_t)getColumnNumEntries(dataSetName),
                m_numPoints);
        }
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

