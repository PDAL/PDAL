
#include <pdal/pdal_test_main.hpp>

#include <filters/M3C2Filter.hpp>

#include <io/LasReader.hpp>
#include <io/BufferReader.hpp>
#include <io/TextWriter.hpp>

#include "Support.hpp"

namespace pdal
{


TEST(M3C2FilterTest, test1)
{
    using namespace Dimension;
    PointTable table;

    Options ro1;
    Options ro2;
    LasReader r1;
    LasReader r2;
    BufferReader r3;
    ro1.add("filename", Support::datapath("autzen/autzen-bmx-2010.las"));
    ro2.add("filename", Support::datapath("autzen/autzen-bmx-2023.las"));
    r1.setOptions(ro1);
    r2.setOptions(ro2);

    M3C2Filter filter;
    Options fo;
    fo.add("normal_radius", 10);
    fo.add("cyl_radius", 20);
    fo.add("cyl_halflen", 10);
    fo.add("sample_pct", 100);

    filter.setOptions(fo);

    // 3 points picked from the first file to use as core points
    PointViewPtr inView(new PointView(table));
    inView->setField(Id::X, 0, 194496.64);
    inView->setField(Id::Y, 0, 259241.37);
    inView->setField(Id::Z, 0, 434.12);
    inView->setField(Id::X, 1, 194478.38);
    inView->setField(Id::Y, 1, 259249.33);
    inView->setField(Id::Z, 1, 423.75);
    inView->setField(Id::X, 2, 194486.95);
    inView->setField(Id::Y, 2, 259234.12);
    inView->setField(Id::Z, 2, 430.74);
    r3.addView(inView);

    filter.prepare(table);

    filter.setInput(r1);
    filter.setInput(r2);
    filter.setInput(r3);
    
    PointViewSet viewSet = filter.execute(table);
    /*
    PointViewPtr viewOut = *viewSet.begin();

    Dimension::Id distance = viewOut->layout()->findDim("m3c2_distance");
    for (int i = 0; i < viewOut->size(); ++i)
        std::cout << viewOut->getFieldAs<float>(distance, i) << std::endl;*/
}

} // namespace pdal