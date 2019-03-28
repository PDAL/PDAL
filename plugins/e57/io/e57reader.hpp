#ifndef _E57READER_H_
#define _E57READER_H_

#include <string>

#include <pdal/pdal_types.hpp>
#include <E57Format.h>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "scan.hpp"

namespace pdal {
class E57Reader: public Reader, public Streamable
{

class ChunkReader
{
public:
    ChunkReader(pdal::point_count_t pointOffset,pdal::point_count_t maxPointRead,
        std::shared_ptr<e57::Scan> scan, 
        const std::set<std::string>& e57Dimensions); // TODO make this 1000 a static variable
    ~ChunkReader();

    // returns false if the index falls out of the [pointOffset,pointOffset + m_maxPointRead] interval
    bool isInScope(pdal::point_count_t index) const;

    bool isInChunk(pdal::point_count_t index) const;

    void setPoint(pdal::point_count_t pointIndex, pdal::PointRef point, 
        const std::set<std::string>& e57Dimensions) const;

    // Reads a new chunk of data
    pdal::point_count_t read(pdal::point_count_t index);

private:
    pdal::point_count_t m_startIndex;
    pdal::point_count_t m_pointOffset;
    pdal::point_count_t m_maxPointRead;
    const pdal::point_count_t m_defaultChunkSize;
    std::map<std::string,std::vector<double>> m_doubleBuffers;
    std::vector<e57::SourceDestBuffer> m_e57buffers;
    std::unique_ptr<e57::CompressedVectorReader> m_dataReader;
    std::shared_ptr<e57::Scan> m_scan;
};

public:
    E57Reader(): Reader(), Streamable() {};
    E57Reader(std::string filename);
    ~E57Reader();

    std::string getName() const;

    /// Extract the header and return in a human readable format
    std::string getHeader() const;

    /// Extract the file's structure and return in human readable format
    std::string getSummary() const;

    /// Get the total number of points in the scan
    point_count_t getNumberPoints() const;

    /// Gets the scan index of a given point index
    int getScanIndex(pdal::point_count_t) const;

    /// Gets the dimensions present within the set of clouds
    std::set<std::string> getDimensions();

    /// returns the scans
   std::vector<std::shared_ptr<e57::Scan>> getScans() const;

private:

    /* Pdal section */
    virtual void addDimensions(PointLayoutPtr layout);
    // // virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual bool processOne(PointRef& point);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    // virtual void done(PointTableRef table);

    void openFile_(std::string filename);
    void setupReader_(pdal::point_count_t pointNumber);
    e57::Node getNode_(const e57::Node &parent, std::string childName) const;
    point_count_t getNumberPoints_() const;
    void extractScans_();

    // members
    std::unique_ptr<e57::ImageFile> m_imf; 
    std::vector<std::shared_ptr<e57::Scan>> m_scans;
    std::shared_ptr<spdlog::logger> m_logger;

    // Allows construction by filename
    std::string m_filenameManual;

    // E57 dimensions that are common to all scans
    std::set<std::string> m_validDimensions;

    // Chunk data reader
    std::unique_ptr<ChunkReader> m_chunk;
    int m_currentScanIndex;
    point_count_t m_currentPoint; // Variable for streaming mode

    // Cache the total number of points
    point_count_t m_pointCount;
};
}

#endif // _E57READER_H_
