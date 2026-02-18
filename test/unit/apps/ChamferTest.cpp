/******************************************************************************
 * Copyright (c) 2020, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <string>

#include <pdal/PDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(Chamfer, kernel)
{
    std::string A = Support::datapath("autzen/autzen-thin.las");
    std::string B = Support::datapath("las/autzen_trim.las");
    std::string output;

    const std::string cmd =
        Support::binpath(Support::exename("pdal")) + " chamfer " + A + " " + B;

    EXPECT_EQ(Utils::run_shell_command(cmd, output), 0);
    EXPECT_TRUE(output.find("\"chamfer\": 5.907628766e+10") !=
                std::string::npos);
}

TEST(Chamfer, distance)
{
    PointTable table;
    PointLayoutPtr layout(table.layout());

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    PointViewPtr src(new PointView(table));
    src->setField(Dimension::Id::X, 0, 0.0);
    src->setField(Dimension::Id::Y, 0, 0.0);
    src->setField(Dimension::Id::Z, 0, 0.0);

    PointViewPtr cand(new PointView(table));
    cand->setField(Dimension::Id::X, 0, 1.0);
    cand->setField(Dimension::Id::Y, 0, 0.0);
    cand->setField(Dimension::Id::Z, 0, 0.0);

    cand->setField(Dimension::Id::X, 1, 0.0);
    cand->setField(Dimension::Id::Y, 1, 2.0);
    cand->setField(Dimension::Id::Z, 1, 0.0);

    EXPECT_EQ(6.0, Utils::computeChamfer(src, cand));

    cand->setField(Dimension::Id::X, 1, 0.0);
    cand->setField(Dimension::Id::Y, 1, 0.0);
    cand->setField(Dimension::Id::Z, 1, 3.0);

    EXPECT_EQ(11.0, Utils::computeChamfer(src, cand));

    src->setField(Dimension::Id::X, 0, 1.0);
    src->setField(Dimension::Id::Y, 0, 1.0);
    src->setField(Dimension::Id::Z, 0, 1.0);

    EXPECT_EQ(10.0, Utils::computeChamfer(src, cand));
}
