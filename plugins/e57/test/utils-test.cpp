#include <gtest/gtest.h>

#include "../io/utils.hpp"

using namespace pdal::e57plugin;

TEST(E57Utils, e57ToPdalTranslation)
{
    ASSERT_EQ(e57ToPdal("cartesianX"),pdal::Dimension::Id::X);
    ASSERT_EQ(e57ToPdal("fake"),pdal::Dimension::Id::Unknown);
    ASSERT_EQ(e57ToPdal("cartesianY"),pdal::Dimension::Id::Y);
    ASSERT_EQ(e57ToPdal("cartesianZ"),pdal::Dimension::Id::Z);
    ASSERT_EQ(e57ToPdal("colorRed"),pdal::Dimension::Id::Red);
    ASSERT_EQ(e57ToPdal("colorGreen"),pdal::Dimension::Id::Green);
    ASSERT_EQ(e57ToPdal("colorBlue"),pdal::Dimension::Id::Blue);
    ASSERT_EQ(e57ToPdal("intensity"),pdal::Dimension::Id::Intensity);
}

TEST(E57Utils, pdalToE57Translation)
{
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::X),"cartesianX");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Y),"cartesianY");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Z),"cartesianZ");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Red),"colorRed");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Green),"colorGreen");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Blue),"colorBlue");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Intensity),"intensity");
}

TEST(E57Utils, rescaleE57ToPdal)
{
    ASSERT_FLOAT_EQ(rescaleE57ToPdalValue("fake",10.0,{0,100}),10.0) ;
    ASSERT_FLOAT_EQ(rescaleE57ToPdalValue("intensity",0.5,{0,1}),std::numeric_limits<uint16_t>::max()/2.0) ;
    ASSERT_FLOAT_EQ(rescaleE57ToPdalValue("intensity",5,{0,10}),std::numeric_limits<uint16_t>::max()/2.0) ;
}


TEST(E57Utils, getPdalBounds)
{
    using pdal::Dimension::Id;
    auto pdalTypes = {Id::Red,Id::Green,Id::Blue,Id::Intensity};
    for (auto type: pdalTypes)
    {
        ASSERT_NO_THROW(getPdalBounds(type));
    }
    ASSERT_ANY_THROW(getPdalBounds(pdal::Dimension::Id::Unknown));
}