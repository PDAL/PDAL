#include <pdal/PointBuffer.hpp>
#include <pdal/BufferReader.hpp>
#include <pdal/PointContext.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>

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

void fillBuffer(pdal::PointBufferPtr buffer, std::vector<Point> const& data)
{
  for (int i = 0; i < data.size(); ++i)
  {
    Point const& pt = data[i];
    buffer->setField<double>(pdal::Dimension::Id::X, i, pt.x);
    buffer->setField<double>(pdal::Dimension::Id::Y, i, pt.y);
    buffer->setField<double>(pdal::Dimension::Id::Z, i, pt.z);
  }
}

int main(int argc, char* argv[])
{
  pdal::Options options;
  options.add("filename", "myfile.las");
  
  pdal::PointContextRef ctx;
  ctx.registerDim(pdal::Dimension::Id::X);
  ctx.registerDim(pdal::Dimension::Id::Y);
  ctx.registerDim(pdal::Dimension::Id::Z);

  {
    pdal::PointBufferPtr buffer = pdal::PointBufferPtr(new pdal::PointBuffer(ctx));

    std::vector<Point> data = getMyData();

    fillBuffer(buffer, data);

    pdal::BufferReader reader;
    reader.addBuffer(buffer);

    pdal::StageFactory f;
    pdal::WriterPtr writer(f.createWriter("writers.las"));
    writer->setInput(&reader);
    writer->setOptions(options);
    writer->prepare(ctx);
    writer->execute(ctx);
  }
}
