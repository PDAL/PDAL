#include <pdal/PointView.hpp>
#include <pdal/BufferReader.hpp>
#include <pdal/Pointtable->hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>

#include <vector>

struct Point
{
    double x;
    double y;
    double z;
};

std::vector<Point> getMyData()
{
    std::vector<Point> output;
    Point p;

    for (int i = 0; i < 1000; ++i)
    {
        p.x = -93.0 + i*0.001;
        p.y = 42.0 + i*0.001;
        p.z = 106.0 + i;
        output.push_back(p);
    }
    return output;
}


void fillView(pdal::PointViewPtr view, std::vector<Point> const& data)
{
    for (int i = 0; i < data.size(); ++i)
    {
        Point const& pt = data[i];
        view->setField<double>(pdal::Dimension::Id::X, i, pt.x);
        view->setField<double>(pdal::Dimension::Id::Y, i, pt.y);
        view->setField<double>(pdal::Dimension::Id::Z, i, pt.z);
    }
}


int main(int argc, char* argv[])
{
    using namespace pdal;

    Options options;
    options.add("filename", "myfile.las");

    PointTable table;
    table.registerDim(Dimension::Id::X);
    table.registerDim(Dimension::Id::Y);
    table.registerDim(Dimension::Id::Z);

    {
        PointViewPtr view(new PointView(table));

        std::vector<Point> data = getMyData();
        fillView(view, data);

        BufferReader reader;
        reader.addView(view);

        LasWriter writer;

        writer.setInput(&reader);
        writer.setOptions(options);
        writer.prepare(table);
        writer.execute(table);
    }
}
