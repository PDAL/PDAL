/******************************************************************************
 * Copyright (c) 2021, Guilhem Villemin (guilhem.villemin@gmail.com)
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

#include <pdal/Filter.hpp>
#include <pdal/pdal_test_main.hpp>

#include <io/SbetSmrmsgReader.hpp>
#include <pdal/pdal_types.hpp>
#include "Support.hpp"

namespace pdal
{


void checkPoint(const PointView& data, PointId index,
        double gpsTime, double npRMS, double epRMS, double dpRMS,
        double nvRMS, double evRMS, double dvRMS, double rollRMS,
        double pitchRMS, double headingRMS)
{
    auto checkDimension = [&data,index](Dimension::Id dim,
        double expected)
    {
        double actual = data.getFieldAs<double>(dim, index);
        EXPECT_NEAR(expected, actual, .0001);
    };

    checkDimension(Dimension::Id::GpsTime, gpsTime);
    //Postion RMS
    checkDimension(Dimension::Id::NorthPositionRMS, npRMS);
    checkDimension(Dimension::Id::EastPositionRMS, epRMS);
    checkDimension(Dimension::Id::DownPositionRMS, dpRMS);
    //Velocity RMS
    checkDimension(Dimension::Id::NorthVelocityRMS, nvRMS);
    checkDimension(Dimension::Id::EastVelocityRMS, evRMS);
    checkDimension(Dimension::Id::DownVelocityRMS, dvRMS);
    //Angles RMS
    checkDimension(Dimension::Id::RollRMS, rollRMS);
    checkDimension(Dimension::Id::PitchRMS, pitchRMS);
    checkDimension(Dimension::Id::HeadingRMS, headingRMS);
}

TEST(SmrmsgReader, ReadSmrmsg)
{
    SmrmsgReader reader;
    Options options;
    options.add("filename", Support::datapath("smrmsg/smrmsg.smrmsg"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 21902u);

    checkPoint(*view.get(), 0, 536258, 0.056279, 0.057791, 0.070774, 0.006642, 0.008034, 0.008131, 0.236985, 0.239420,3.010802);
    checkPoint(*view.get(), 1, 536259, 0.054151, 0.055892, 0.068654, 0.006113, 0.007200, 0.007629, 0.235971, 0.238542,3.010596);
    checkPoint(*view.get(), 2, 536260, 0.052268, 0.054339, 0.066672, 0.006119, 0.006675, 0.007411, 0.235108, 0.237804,3.010406);
}

} // namespace pdal