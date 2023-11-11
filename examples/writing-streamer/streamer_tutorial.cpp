#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/Reader.hpp>

#include <vector>

namespace pdal {
    
// A class to produce a point cloud point-by-point, rather than
// having it all in memory at the same time. It will be streamed to
// disk. See the GDALReader class for how to add more fields
// and read from disk.
class PDAL_DLL StreamedPointCloud : public Reader, public Streamable
{
public:
    std::string getName() const;
    StreamedPointCloud();
    ~StreamedPointCloud();

private:
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t num);
    virtual void done(PointTableRef table);
    virtual bool processOne(PointRef& point);
    virtual void addArgs(ProgramArgs& args);

    point_count_t m_count, m_size;
};
    
std::string StreamedPointCloud::getName() const
{
    return "streamed_point_cloud";
}

StreamedPointCloud::StreamedPointCloud()
    : m_count(0), m_size(0)
{}

StreamedPointCloud::~StreamedPointCloud()
{
}

// Set the size of the cloud. Will ask for a point till the counter
// reaches this size.
void StreamedPointCloud::initialize()
{
    m_size = 500;
}

// Set the cloud dimensions.
void StreamedPointCloud::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(pdal::Dimension::Id::X);
    layout->registerDim(pdal::Dimension::Id::Y);
    layout->registerDim(pdal::Dimension::Id::Z);
}

void StreamedPointCloud::addArgs(ProgramArgs& args)
{
}

void StreamedPointCloud::ready(PointTableRef table)
{
    m_count = 0;
}

// This function is used when a point cloud is formed fully in memory.
// Not applicable here.
point_count_t StreamedPointCloud::read(PointViewPtr view, point_count_t numPts)
{
    throw pdal_error("The read() function must not be called in streaming mode.");
    return -1;
}

// Create one point at a time.
bool StreamedPointCloud::processOne(PointRef& point)
{
    if (m_count == m_size)
        return false; // done

    double x = m_count * 0.1;
    double y = m_count * 0.2;
    double z = m_count * 0.3;
    
    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);

    m_count++;
    return true;
}

void StreamedPointCloud::done(PointTableRef table)
{
}

} // end namespace pdal

int main(int argc, char* argv[])
{
    using namespace pdal;
    
    // A point cloud. Only one point at a time will be in memory
    StreamedPointCloud stream_cloud;

    // Will copy here each point and then stream it to disk.
    // buf_size is the number of points that will be
    // processed and kept in memory at the same time. 
    // A somewhat bigger value may result in some efficiencies.
    int buf_size = 5;
    FixedPointTable t(buf_size);
    stream_cloud.prepare(t);

    // Set the output filename
    Options write_options;
    write_options.add("filename", "output.las");
    
    // StageFactory always "owns" stages it creates. They'll be destroyed with
    // the factory.
    StageFactory factory;
    Stage *writer = factory.createStage("writers.las");

    // Stream the point cloud to disk
    writer->setInput(stream_cloud);
    writer->setOptions(write_options);
    writer->prepare(t);
    writer->execute(t);
 
    return 0;   
}
