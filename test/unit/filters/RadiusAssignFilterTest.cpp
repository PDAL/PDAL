#include <pdal/pdal_test_main.hpp>

#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include "Support.hpp"

#include <filters/StatsFilter.hpp>

namespace pdal
{

TEST(RadiusAssignFilterTest, basic_usage)
{
    Options ro;
    ro.add("filename", Support::datapath("filters/radiusassign/grid_20x10x10.txt"));
    StageFactory factory;
    Stage& r = *(factory.createStage("readers.text"));
    r.setOptions(ro);

    std::vector<std::string> is3ds = {"false", "false", "true", "true"};
    std::vector<double> radVals = {1, 3.5, 1, 3.5};
    std::vector<unsigned int> nbUpdatedPoints = {0, 0, 0, 0};
    std::vector<unsigned int> nbExpectedUpdatedPoints = {10, 370, 1, 179};

    for (size_t ii = 0; ii < radVals.size(); ii++)
    {
        Options ao;
        ao.add("value", "Classification=1");
        ao.add("where", "(X == 5) && (Y == 5) && (Z == 5)");

        Stage& a = *(factory.createStage("filters.assign"));
        a.setInput(r);
        a.setOptions(ao);

        Options fo;
        fo.add("is3d", is3ds[ii]);
        fo.add("reference_domain", "Classification[1:1]");
        fo.add("update_expression", "Classification = 2");
        fo.add("radius", radVals[ii]);

        Stage& f = *(factory.createStage("filters.radiusassign"));
        f.setInput(a);
        f.setOptions(fo);

        PointTable table;
        f.prepare(table);
        PointViewSet viewSet = f.execute(table);
        PointViewPtr view = *viewSet.begin();

        EXPECT_EQ(1u, viewSet.size());

        PointRef point(*view, 0);

        for (PointId id = 0; id < view->size(); ++id)
        {
            point.setPointId(id);
            if (view->getFieldAs<unsigned int>(Dimension::Id::Classification, id) == 2)
                nbUpdatedPoints[ii] += 1;
        }
        // Check that some points are updated
        EXPECT_TRUE(nbUpdatedPoints[ii] == nbExpectedUpdatedPoints[ii])
            << "Found: '" << nbUpdatedPoints[ii] << "'" << std::endl
            << "expected: '" << nbExpectedUpdatedPoints[ii] <<"'" << std::endl;
    }
}



TEST(RadiusAssignFilterTest, with_z_limit)
{
    Options ro;
    ro.add("filename", Support::datapath("filters/radiusassign/grid_20x10x10.txt"));
    StageFactory factory;
    Stage& r = *(factory.createStage("readers.text"));
    r.setOptions(ro);

    std::vector<double> maxs2dAbove = {-1, 0, 1, -1, -1};
    std::vector<double> maxs2dBelow = {-1, -1, -1, 0, 1};
    std::vector<unsigned int> nbUpdatedPoints = {0, 0, 0, 0, 0};
    std::vector<unsigned int> nbExpectedUpdatedPoints = {10, 3, 4, 8, 9};

    for (size_t ii = 0; ii < nbUpdatedPoints.size(); ii++)
    {
        Options ao;
        ao.add("value", "Classification=1");
        ao.add("where", "(X == 5) && (Y == 5) && (Z == 7)");

        Stage& a = *(factory.createStage("filters.assign"));
        a.setInput(r);
        a.setOptions(ao);

        Options fo;
        fo.add("is3d", "false");
        fo.add("reference_domain", "Classification[1:1]");
        fo.add("update_expression", "Classification = 2");
        fo.add("radius", 1);
        fo.add("max2d_above", maxs2dAbove[ii]);
        fo.add("max2d_below", maxs2dBelow[ii]);


        Stage& f = *(factory.createStage("filters.radiusassign"));
        f.setInput(a);
        f.setOptions(fo);

        PointTable table;
        f.prepare(table);
        PointViewSet viewSet = f.execute(table);
        PointViewPtr view = *viewSet.begin();

        EXPECT_EQ(1u, viewSet.size());

        PointRef point(*view, 0);

        for (PointId id = 0; id < view->size(); ++id)
        {
            point.setPointId(id);
            if (view->getFieldAs<unsigned int>(Dimension::Id::Classification, id) == 2)
                nbUpdatedPoints[ii] += 1;
        }
        // Check that some points are updated
        EXPECT_TRUE(nbUpdatedPoints[ii] == nbExpectedUpdatedPoints[ii])
            << "Found: '" << nbUpdatedPoints[ii] << "'" << std::endl
            << "expected: '" << nbExpectedUpdatedPoints[ii] <<"'" << std::endl;
    }
}

TEST(RadiusAssignFilterTest, with_src_domain)
{
    Options ro;
    ro.add("filename", Support::datapath("filters/radiusassign/grid_20x10x10.txt"));
    StageFactory factory;
    Stage& r = *(factory.createStage("readers.text"));
    r.setOptions(ro);

    Options ao;
    ao.add("value", "Classification=1");
    ao.add("where", "(X == 5) && (Y == 5) && (Z == 5)");

    Stage& a = *(factory.createStage("filters.assign"));
    a.setInput(r);
    a.setOptions(ao);

    Options fo;
    fo.add("is3d", "false");
    fo.add("reference_domain", "Classification[1:1]");
    fo.add("src_domain", "Z[0:5]");
    fo.add("update_expression", "Classification = 2");
    fo.add("radius", 1);


    Stage& f = *(factory.createStage("filters.radiusassign"));
    f.setInput(a);
    f.setOptions(fo);

    PointTable table;
    f.prepare(table);
    PointViewSet viewSet = f.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());

    PointRef point(*view, 0);

    size_t nbUpdatedPoints=0;
    for (PointId id = 0; id < view->size(); ++id)
    {
        point.setPointId(id);
        if (view->getFieldAs<unsigned int>(Dimension::Id::Classification, id) == 2)
            nbUpdatedPoints += 1;
    }
    // Check that some points are updated
    EXPECT_TRUE(nbUpdatedPoints == 6)
        << "Found: '" << nbUpdatedPoints << "'" << std::endl
        << "expected: '5'" << std::endl;

}


TEST(RadiusAssignFilterTest, missing_param)
{
    Options ro;
    ro.add("filename", Support::datapath("las/4_6.las"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    std::vector<unsigned int> nbUpdatedPoints = {0, 0};

    Options fo;
    fo.add("src_domain", "Classification[2:2]");
    fo.add("reference_domain", "Classification[1:1]");
    fo.add("radius", 1);

    Stage& f = *(factory.createStage("filters.radiusassign"));
    f.setInput(r);
    f.setOptions(fo);

    PointTable table;
    // Expect error due to missing update_expression argument
    EXPECT_ANY_THROW(f.prepare(table));
    PointViewSet viewSet = f.execute(table);
}


} // namespace pdal
