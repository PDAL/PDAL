#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>
#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include <io/LasHeader.hpp>
#include <pdal/Options.hpp>

#include <vector>

namespace pdal {

// A filter to multiply each point's coordinates by a factor. Each point
// is processed in streaming mode, without loading the entire point cloud into
// memory. Adjust appropriately the scale and offset in the header of the output
// file.
    
class PDAL_EXPORT FilterStreamer : public Filter, public Streamable
{
public:
    std::string getName() const;
    FilterStreamer(double factor);
    ~FilterStreamer();

private:
    virtual bool processOne(PointRef& point);
    double m_factor;
};
    
std::string FilterStreamer::getName() const
{
    return "filter_streamer";
}

FilterStreamer::FilterStreamer(double factor): m_factor(factor) 
{
}

FilterStreamer::~FilterStreamer() 
{
}

// Apply a transform to each point
bool FilterStreamer::processOne(PointRef& point)
{
    
    // Apply the scale factor
    double x = point.getFieldAs<double>(Dimension::Id::X) * m_factor;
    double y = point.getFieldAs<double>(Dimension::Id::Y) * m_factor;
    double z = point.getFieldAs<double>(Dimension::Id::Z) * m_factor;
    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);

    return true;
}

} // end namespace pdal

int main(int argc, char* argv[])
{
    using namespace pdal;
    
    // buf_size is the number of points that will be
    // processed and kept in this table at the same time. 
    // A somewhat bigger value may result in some efficiencies.
    int buf_size = 5;
    FixedPointTable t(buf_size);
    
    // Set the input point cloud    
    Options read_options;
    read_options.add("filename", "input.las");
    std::cout << "Reading: input.las\n";
    LasReader reader;
    reader.setOptions(read_options);
    reader.prepare(t); 
    
    // Get the scale and offset from the input cloud header
    // Must be run after the table is prepared
    const LasHeader & header = reader.header();
    double offset[3] = {header.offsetX(), header.offsetY(), header.offsetZ()};
    double scale[3] = {header.scaleX(), header.scaleY(), header.scaleZ()};

    // Will multiply each point by this factor
    double factor = 2.0;
    std::cout << "Applying factor: " << factor << std::endl;
    
    // Set up the filter
    FilterStreamer streamer(factor);
    streamer.setInput(reader);
    streamer.prepare(t);

    // Set up the output file
    Options write_options;
    write_options.add("filename", "output.las");
    std::cout << "Writing: output.las\n";
    
    // The scale and offset for the output file will be adjusted
    // given that we multiply each point by a factor
    write_options.add("offset_x", factor * offset[0]);
    write_options.add("offset_y", factor * offset[1]);
    write_options.add("offset_z", factor * offset[2]);
    write_options.add("scale_x",  factor * scale[0]);
    write_options.add("scale_y",  factor * scale[1]);
    write_options.add("scale_z",  factor * scale[2]);
    
    // Write the output file
    LasWriter writer;
    writer.setOptions(write_options);
    writer.setInput(streamer);
    writer.prepare(t);
    writer.execute(t);

    return 0;   
}
