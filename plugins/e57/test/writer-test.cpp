//
// Created by Nicolas Chaulet on 2018-12-14.
//
#include <stdio.h>
#include <pdal/pdal_test_main.hpp>
#include "Support.hpp"

#include "../io/e57writer.hpp"
#include "../io/e57reader.hpp"
#include "../io/utils.hpp"

using namespace pdal;

TEST(E57Writer, testCtr)
{
    Options ops;
    std::string outfile =  Support::datapath("e57/test.e57");
    ops.add("filename",outfile);
    E57Writer writer;
    writer.setOptions(ops);
    PointTable table;
    writer.prepare(table);
    remove(outfile.c_str());
}

PointViewSet writertest_readE57(std::string filename,PointTableRef table)
{
  Options ops;
  ops.add("filename",filename);
  E57Reader reader;
  reader.setOptions(ops);
  reader.prepare(table);
  return reader.execute(table);
}


TEST(E57WRiter,testWrite)
{
    std::string outfile(Support::datapath("e57/test.e57"));
    std::string infile(Support::datapath("e57/A4.e57"));

    remove(outfile.c_str());

    E57Reader r;
    Options ro;

    ro.add("filename", infile);
    r.setOptions(ro);

    {
        E57Writer w;
        Options wo;

        wo.add("filename", outfile);
        w.setOptions(wo);
        w.setInput(r);


        PointTable t;

        w.prepare(t);
        w.execute(t);
    }
    PointTable tablein;
    auto viewin = writertest_readE57(infile,tablein);
    auto cloudin = *viewin.begin();
    PointTable tableout;
    auto viewout = writertest_readE57(outfile,tableout);
    auto cloudout = *viewout.begin();

    auto expectedDimensions = {pdal::Dimension::Id::X,pdal::Dimension::Id::Y,pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Red,pdal::Dimension::Id::Green,pdal::Dimension::Id::Blue,pdal::Dimension::Id::Intensity};
    for (int i =0; i < cloudout->size();i++)
    {
        auto ptB = cloudin->point(i);
        auto pt = cloudout->point(i);
        for (auto& dim: expectedDimensions)
        {
            ASSERT_TRUE(pt.hasDim(dim));
            ASSERT_FLOAT_EQ(pt.getFieldAs<double>(dim),ptB.getFieldAs<double>(dim));
        }
    }

   remove(outfile.c_str());

}