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

#include <boost/test/unit_test.hpp>

#include <pdal/SpatialReference.hpp>
#include <pdal/drivers/pipeline/Reader.hpp>
#include <pdal/filters/Scaling.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Options.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(ScalingFilterTest)

// 
// 
// 
// static void getDoublePoint(const pdal::PointBuffer& data, double& x, double& y, double& z, 
//                             boost::uint16_t& intensity, boost::int8_t& scan_angle, boost::uint16_t& green)
// {
//     using namespace pdal;
// 
//     const ::pdal::Schema& schema = data.getSchema();
// 
//     const int indexX = schema.getDimensionIndex(DimensionId::X_f64);
//     const int indexY = schema.getDimensionIndex(DimensionId::Y_f64);
//     const int indexZ = schema.getDimensionIndex(DimensionId::Z_f64);
//     const int indexIntensity = schema.getDimensionIndex(DimensionId::Las_Intensity);
//     const int indexScanAngle = schema.getDimensionIndex(DimensionId::Las_ScanAngleRank);
//     const int indexGreen = schema.getDimensionIndex(DimensionId::Green_u16);
// 
//     x = data.getField<double>(0, indexX);
//     y = data.getField<double>(0, indexY);
//     z = data.getField<double>(0, indexZ);
//     scan_angle = data.getField<boost::int8_t>(0, indexScanAngle);
//     intensity = data.getField<boost::uint16_t>(0, indexIntensity);
//     green = data.getField<boost::uint16_t>(0, indexGreen);
// 
//     return;
// }
// 
BOOST_AUTO_TEST_CASE(ScalingFilterTest_test_1)
{
    pdal::Option option("filename", Support::datapath("pipeline/pipeline_scaling.xml"));
    pdal::Options options(option);

    pdal::drivers::pipeline::Reader reader(options);
    reader.initialize();
    
    pdal::filters::Scaling const* filter = static_cast<pdal::filters::Scaling const*>(reader.getManager().getStage());
    pdal::Options opt = filter->getCurrentOptions();
    // std::cout << "filter ops: " << opt << std::endl;

    const pdal::Schema& schema = filter->getSchema();
    pdal::PointBuffer data2(schema, 1);

    pdal::StageSequentialIterator* iter = filter->createSequentialIterator(data2);
    boost::uint32_t numRead = iter->read(data2);
    BOOST_CHECK(numRead == 1);
    delete iter;
        // 
        // double x=0, y=0, z=0;
        // boost::uint16_t intensity(0);
        // boost::int8_t scan_angle(06);
        // boost::uint16_t green(0);
        // getDoublePoint(data2, x, y, z, intensity, scan_angle, green);
        // 
        // BOOST_CHECK_CLOSE(x, 470692.44, 1);
        // BOOST_CHECK_CLOSE(y, 4602888.90, 1);
        // BOOST_CHECK_CLOSE(z, 16.00, 1);
        // BOOST_CHECK_EQUAL(intensity, 0);
        // BOOST_CHECK_EQUAL(scan_angle, -13);
        // BOOST_CHECK_EQUAL(green, 12);

    return;
}



BOOST_AUTO_TEST_SUITE_END()
