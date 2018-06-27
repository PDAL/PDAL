/******************************************************************************
* Copyright (c) 2016, Howard Butler <howard@hobu.co>
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

#include <pdal/pdal_test_main.hpp>

#include <pdal/Artifact.hpp>
#include <pdal/ArtifactManager.hpp>
#include <pdal/PointTable.hpp>

#include <pdal/Options.hpp>

#include "Support.hpp"


namespace pdal
{

class TestArtifact : public Artifact
{
public:
    TestArtifact(const std::string& s) : m_val(s)
    {}

    std::string m_val;
};

class Foo
{
};

class TestArtifact2 : public Artifact
{
};

TEST(ArtifactTest, simple)
{
    using TAPtr = std::shared_ptr<TestArtifact>;
    TAPtr ta(new TestArtifact("MyTest"));

    PointTable t;
    t.artifactManager().put("MyTest", ta);
    EXPECT_EQ(t.artifactManager().get<TestArtifact>("foo"), nullptr);
    EXPECT_NE(t.artifactManager().get<TestArtifact>("MyTest"), nullptr);
    EXPECT_EQ(t.artifactManager().get<TestArtifact>("MyTest")->m_val, "MyTest");
    EXPECT_EQ(t.artifactManager().get<Foo>("MyTest"), nullptr);
    EXPECT_EQ(t.artifactManager().get<TestArtifact2>("MyTest"), nullptr);
}

TEST(ArtifactTest, replace)
{
    using TAPtr = std::shared_ptr<TestArtifact>;
    using TAPtr2 = std::shared_ptr<TestArtifact2>;
    TAPtr ta(new TestArtifact("MyTest"));
    TAPtr taa(new TestArtifact("MyTestA"));
    TAPtr2 ta2(new TestArtifact2);

    PointTable t;
    EXPECT_FALSE(t.artifactManager().exists("MyTest"));
    EXPECT_FALSE(t.artifactManager().replace("MyTest", ta));
    t.artifactManager().put("MyTest", ta);
    EXPECT_FALSE(t.artifactManager().replace("MyTest", ta2));
    EXPECT_TRUE(t.artifactManager().replace("MyTest", taa));
    EXPECT_TRUE(t.artifactManager().exists("MyTest"));
    EXPECT_EQ(t.artifactManager().get<TestArtifact>("MyTest")->m_val,
        "MyTestA");
    EXPECT_FALSE(t.artifactManager().erase("MyOtherTest"));
    EXPECT_TRUE(t.artifactManager().erase("MyTest"));
    EXPECT_FALSE(t.artifactManager().exists("MyTest"));
}

TEST(ArtifactTest, replaceOrPut)
{
    using TAPtr = std::shared_ptr<TestArtifact>;
    using TAPtr2 = std::shared_ptr<TestArtifact2>;

    TAPtr ta(new TestArtifact("MyTest"));
    TAPtr taa(new TestArtifact("MyTestA"));
    TAPtr2 ta2(new TestArtifact2);

    PointTable t;
    EXPECT_FALSE(t.artifactManager().exists("MyTest"));
    EXPECT_TRUE(t.artifactManager().replaceOrPut("MyTest", ta));
    EXPECT_EQ(t.artifactManager().get<TestArtifact>("MyTest")->m_val,
        "MyTest");
    EXPECT_TRUE(t.artifactManager().exists("MyTest"));
    EXPECT_TRUE(t.artifactManager().replaceOrPut("MyTest", taa));
    EXPECT_TRUE(t.artifactManager().exists("MyTest"));
    EXPECT_EQ(t.artifactManager().get<TestArtifact>("MyTest")->m_val,
        "MyTestA");
    EXPECT_FALSE(t.artifactManager().replaceOrPut("MyTest", ta2));
}

TEST(ArtifactTest, key_access)
{
    using TAPtr = std::shared_ptr<TestArtifact>;
    TAPtr ta(new TestArtifact("MyTest"));

    PointTable t;
    EXPECT_TRUE(t.artifactManager().keys().empty());

    t.artifactManager().put("MyTest", ta);
    EXPECT_EQ(t.artifactManager().keys().size(), 1U);
    EXPECT_EQ(t.artifactManager().keys().at(0), "MyTest");

    t.artifactManager().put("MyTest2", ta);
    auto keys = t.artifactManager().keys();
    EXPECT_EQ(keys.size(), 2U);
    EXPECT_EQ(std::find(keys.begin(), keys.end(), "Foo"), keys.end());
    EXPECT_NE(std::find(keys.begin(), keys.end(), "MyTest"), keys.end());
    EXPECT_NE(std::find(keys.begin(), keys.end(), "MyTest2"), keys.end());
}

} // namespace pdal
