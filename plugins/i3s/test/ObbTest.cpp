#include <pdal/pdal_test_main.hpp>

#include <nlohmann/json.hpp>

#include "Support.hpp"

#include "../io/Obb.hpp"

namespace pdal
{
namespace i3s
{

TEST(ObbTest, obb)
{
    auto doesIntersect = [](Obb c)
    {
        NL::json base = R"(
            {
                "center" : [ 0, 0, 0 ],
                "halfSize" : [ 2, 1, 1.5 ],
                "quaternion" : [ 0, 0, 0, 1 ]
            }
        )"_json;
        Obb b(base);
        EXPECT_EQ(b.intersect(c), c.intersect(b));
        return b.intersect(c);
    };

    NL::json clip = R"(
        {
            "center" : [ 2, 1, 0 ],
            "halfSize" : [
                2.12132034355,
                0.707106781186,
                1
            ],
            "quaternion" : [
                0,
                0,
                -0.3826834324,
                0.9238795325
            ]
        }
    )"_json;

    Obb c(clip);
    EXPECT_TRUE(doesIntersect(c));
    c.setCenter({2, 1, -1});
    EXPECT_TRUE(doesIntersect(c));
    c.setCenter({2, 1, -2.5});
    EXPECT_TRUE(doesIntersect(c));
    c.setCenter({2, 1, -2.51});
    EXPECT_FALSE(doesIntersect(c));
    c.setCenter({2, 3, 0});
    EXPECT_FALSE(doesIntersect(c));
    c.setCenter({2, 2, 0});
    EXPECT_TRUE(doesIntersect(c));
}

} // namespace i3s
} // namespace pdal

