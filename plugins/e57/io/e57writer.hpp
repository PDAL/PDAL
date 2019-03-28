//
// Created by Nicolas Chaulet on 2018-12-14.
//

#ifndef E57IO_E57WRITER_HPP
#define E57IO_E57WRITER_HPP

#include <pdal/pdal_types.hpp>
#include <E57Format.h>
#include <pdal/Writer.hpp>
#include <pdal/Streamable.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace pdal
{
class E57Writer : public pdal::Writer, public pdal::Streamable 
{

class ChunkWriter
{
public:
    ChunkWriter(const std::vector<std::string>& dimensionsToWrite,e57::CompressedVectorNode& vectorNode);
    ~ChunkWriter();

    void write(pdal::PointRef& point);
    void finalise();

private:
    const pdal::point_count_t m_defaultChunkSize;
    pdal::point_count_t m_currentIndex;
    std::map<std::string,std::vector<double>> m_doubleBuffers;
    std::vector<e57::SourceDestBuffer> m_e57buffers;
    std::unique_ptr<e57::CompressedVectorWriter> m_dataWriter;
};

public:
    E57Writer();
    ~E57Writer();

    std::string getName() const;

private:
    // Implement pdal::Writer interface
    virtual void addArgs(ProgramArgs &args);
    virtual void initialize();
    virtual bool processOne(PointRef & point);
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);

    void setupFileHeader_();
    void setupWriter_();

    std::shared_ptr<spdlog::logger> m_logger;

    // Writer parameters
    std::string m_filename;
    bool m_doublePrecision;

    // e57 file objects
    std::unique_ptr<e57::ImageFile> m_imageFile;
    std::unique_ptr<e57::StructureNode> m_rootNode;
    std::unique_ptr<ChunkWriter> m_chunkWriter;
    std::unique_ptr<e57::StructureNode>  m_scanNode;

    // What do we write?
    std::vector<std::string> m_dimensionsToWrite;
};
}


#endif //E57IO_E57WRITER_HPP
