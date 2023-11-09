#include <pdal/pdal_test_main.hpp>

#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include "Support.hpp"

#include <filters/StatsFilter.hpp>

namespace pdal
{

TEST(RadiusSearchFilterTest, basic_usage)
{
    Options ro;
    ro.add("filename", Support::datapath("las/4_6.las"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    std::vector<double> radVals = {1, 3.5};
    std::vector<unsigned int> nbUpdatedPoints = {0, 0};

    for (size_t ii; ii < radVals.size(); ii++)
    {
        Options fo;
        fo.add("src_domain", "Classification[2:2]");
        fo.add("reference_domain", "Classification[1:1]");
        fo.add("update_expression", "UserData = 1");
        fo.add("radius", radVals[ii]);

        Stage& f = *(factory.createStage("filters.radiussearch"));
        f.setInput(r);
        f.setOptions(fo);

        PointTable table;
        f.prepare(table);
        PointViewSet viewSet = f.execute(table);
        PointViewPtr view = *viewSet.begin();

        EXPECT_EQ(1u, viewSet.size());

        // Get ids for points within src_domain and points with output_dim = output_value
        std::set<PointId> srcDomainIds;
        std::set<PointId> outputValIds;
        PointRef point(*view, 0);

        for (PointId id = 0; id < view->size(); ++id)
        {
            point.setPointId(id);
            if (view->getFieldAs<double>(Dimension::Id::Classification, id) == 2.)
                srcDomainIds.emplace(id);
            if (view->getFieldAs<unsigned int>(Dimension::Id::UserData, id) == 1)
                outputValIds.emplace(id);
        }
        // Check that some points are updated
        EXPECT_GT(outputValIds.size(), 0);
        // Check that only points with classification in src_domain have been flagged with UserData = 1
        EXPECT_TRUE(std::includes(srcDomainIds.begin(), srcDomainIds.end(),
            outputValIds.begin(), outputValIds.end()));

        // Check that not all points with classification in src_domain have been flagged with UderData = 1
        EXPECT_GT(srcDomainIds.size(), outputValIds.size());
        nbUpdatedPoints[ii] = outputValIds.size();
    }
    // Check that using a bigger radius updates more points
    EXPECT_GT(nbUpdatedPoints[1], nbUpdatedPoints[0]);
}


TEST(RadiusSearchFilterTest, missing_param)
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

    Stage& f = *(factory.createStage("filters.radiussearch"));
    f.setInput(r);
    f.setOptions(fo);

    PointTable table;
    // Expect error due to missing update_expression argument
    EXPECT_ANY_THROW(f.prepare(table));
    PointViewSet viewSet = f.execute(table);
}


} // namespace pdal
