/******************************************************************************
* Copyright (c) 2025, Kasparas Karlauskas (kasparas.karlauskas@gmail.com)
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

#include <vector>
#include <string>
#include <unordered_map>

#include <pdal/pdal_test_main.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

namespace pdal {

class CustomGroundClassTest : public ::testing::TestWithParam<std::string>
{
    protected:
        std::unordered_map<uint8_t, size_t> classCounts(
            const std::string filterTypeName,
            const uint8_t groundClass,
            const uint8_t otherClass,
            const uint8_t startingClass,
            const bool onlyGround
        )
        {
            StageFactory f;

            Stage* reader(f.createStage("readers.las"));
            Options rOpts;
            rOpts.add("filename", Support::datapath("las/warsaw_small.las"));
            reader->setOptions(rOpts);

            Stage* assign(f.createStage("filters.assign"));
            Options aOpts;
            aOpts.add("assignment", "Classification[:]=" + std::to_string(startingClass));
            assign->setInput(*reader);
            assign->setOptions(aOpts);

            const std::string filterType = "filters." + filterTypeName;
            Stage* filter(f.createStage(filterType));
            Options fOpts;
            fOpts.add("ground_class", groundClass);
            fOpts.add("other_class", otherClass);
            fOpts.add("only_ground", onlyGround);
            filter->setInput(*assign);
            filter->setOptions(fOpts);

            PointTable t;
            filter->prepare(t);
            PointViewSet s = filter->execute(t);
            
            PointViewPtr v = *s.begin();

            std::unordered_map<uint8_t, size_t> classCounts;
            for (PointId id = 0; id < v->size(); ++id)
            {
                uint8_t cl = v->getFieldAs<uint8_t>(Dimension::Id::Classification, id);
                classCounts[cl]++;
            }
            return classCounts;
        }
};

TEST_P(CustomGroundClassTest, CustomGroundClass)
{
    const uint8_t groundClass = 7;
    const uint8_t otherClass = 5;
    const uint8_t startingClass = 9;
    const std::string filterTypeName = GetParam();

    // Both classes custom
    std::unordered_map<uint8_t, size_t> classCounts = this->classCounts(
        filterTypeName, groundClass, otherClass, startingClass, /*onlyGround=*/false);

    // If the ground classification filter processes a certain range of returns by default,
    // some points will retain their original class.
    EXPECT_TRUE((2 <= classCounts.size()) && (classCounts.size() <= 3));
    // We must observe at least one groundClass and otherClass point regardless of returns
    // segmentation in the filter.
    EXPECT_GT(classCounts[groundClass], 0u);
    EXPECT_GT(classCounts[otherClass], 0u);

    // Same class for groundClass and otherClass, expect an error
    EXPECT_ANY_THROW(
        this->classCounts(
            filterTypeName, groundClass, groundClass, startingClass, /*onlyGround=*/false);
    );

    // Only ground class is custom, expect part of the original classification to remain
    classCounts = this->classCounts(
        filterTypeName, groundClass, otherClass, startingClass, /*onlyGround=*/true);

    EXPECT_EQ(classCounts.size(), 2);
    EXPECT_GT(classCounts[groundClass], 0u);
    EXPECT_GT(classCounts[startingClass], 0u);
}

std::vector<std::string> groundFilterTypes = {
    "csf",
    "pmf",
    "skewnessbalancing",
    "smrf"
};

INSTANTIATE_TEST_SUITE_P(
    GroundFilterTest,
    CustomGroundClassTest,
    ::testing::ValuesIn(groundFilterTypes)
);

} // namespace pdal
