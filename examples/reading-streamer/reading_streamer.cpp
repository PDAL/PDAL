#include <io/LasReader.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <iostream>

namespace pdal
{

// A class to read a point cloud from a file point by point, without
// loading it fully in memory. The points are printed to the screen,
// but this can be replaced with any other processing.
 
class PDAL_DLL StreamProcessor: public Writer, public Streamable
{

public:
    std::string getName() const;
    StreamProcessor();
    ~StreamProcessor();

private:

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void writeView(const PointViewPtr view);
    virtual bool processOne(PointRef& point);

    StreamProcessor& operator=(const StreamProcessor&) = delete;
    StreamProcessor(const StreamProcessor&) = delete;
    StreamProcessor(const StreamProcessor&&) = delete;
};

std::string StreamProcessor::getName() const { return "sample streamer"; }

StreamProcessor::StreamProcessor() {}

StreamProcessor::~StreamProcessor() {}

void StreamProcessor::addArgs(ProgramArgs& args)
{
}

void StreamProcessor::initialize()
{
}

// This will be called for each point in the cloud.
bool StreamProcessor::processOne(PointRef& point)
{
   // Print the point coordinates
   std::cout << "Process point: " 
             << point.getFieldAs<double>(Dimension::Id::X) <<  ", " 
             << point.getFieldAs<double>(Dimension::Id::Y) << ", " 
             << point.getFieldAs<double>(Dimension::Id::Z) << std::endl;
    return true;  
}

void StreamProcessor::writeView(const PointViewPtr view)
{
    throw pdal_error("The writeView() function must not be called in streaming mode.");
}

} // namespace pdal

int main(int argc, char* argv[])
{
 
    using namespace pdal;
    
    // Set the input point cloud    
    Options read_options;
    read_options.add("filename", "input.las");
    LasReader reader;
    reader.setOptions(read_options);

    // buf_size is the number of points that will be
    // processed and kept in this table at the same time. 
    // A somewhat bigger value may result in some efficiencies.
    int buf_size = 100;
    FixedPointTable t(buf_size);
    reader.prepare(t);

    // Read each point and print it to the screen
    StreamProcessor writer;
    Options write_options;
    writer.setOptions(write_options);
    writer.setInput(reader);
    writer.prepare(t);
    writer.execute(t);
    
    return 0;
}
