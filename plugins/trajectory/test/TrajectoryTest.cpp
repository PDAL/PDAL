#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"
#include <pdal/StageFactory.hpp>

namespace pdal
{

TEST(TrajectoryTest, t1)
{
    StageFactory f;

    Stage& r = *(f.createStage("readers.las"));
    Options lasOpts;
    lasOpts.add("filename", Support::datapath("laz/C2_L2.laz"));
    r.setOptions(lasOpts);

    Stage& t = *(f.createStage("filters.trajectory"));
    t.setInput(r);

    Options txtOpts;
    txtOpts.add("order", "GpsTime, X, Y, Z");
    txtOpts.add("precision", 6);
    txtOpts.add("keep_unspecified", false);
    txtOpts.add("write_header", false);
    txtOpts.add("delimiter", " ");
    txtOpts.add("filename", Support::temppath("tran.txt"));
    Stage& w = *(f.createStage("writers.text"));
    w.setOptions(txtOpts);
    w.setInput(t);

    PointTable table;
    w.prepare(table);
    w.execute(table);

    EXPECT_EQ(Support::diff_files(Support::temppath("tran.txt"),
        Support::datapath("text/C2_L2-traj.txt")), 0);
}

} // namespace pdal
