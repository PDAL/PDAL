#include "PipelineExecutor.hpp"
#include <pdal/PDALUtils.hpp>

namespace pdal
{

namespace executor {

PipelineExecutor::PipelineExecutor(std::string const& json)
    : m_json(json)
    , m_executed(false)
    , m_logLevel(pdal::LogLevel::Error)
{
}


std::string PipelineExecutor::getPipeline() const
{
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    std::stringstream strm;
    pdal::PipelineWriter::writePipeline(m_manager.getStage(), strm);
    return strm.str();
}


std::string PipelineExecutor::getMetadata() const
{
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    std::stringstream strm;
    MetadataNode root = m_manager.getMetadata().clone("metadata");
    pdal::Utils::toJSON(root, strm);
    return strm.str();
}


std::string PipelineExecutor::getSchema() const
{
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    std::stringstream strm;
    MetadataNode root = m_manager.pointTable().toMetadata().clone("schema");
    pdal::Utils::toJSON(root, strm);
    return strm.str();
}


int64_t PipelineExecutor::execute()
{
    std::stringstream strm;
    strm << m_json;
    m_manager.readPipeline(strm);
    point_count_t count = m_manager.execute();

    m_executed = true;

    return count;
}


void PipelineExecutor::setLogStream(std::ostream& strm)
{

    LogPtr log = pdal::LogPtr(new pdal::Log("pypipeline", &strm));
    log->setLevel(m_logLevel);
    m_manager.setLog(log);

}


void PipelineExecutor::setLogLevel(int level)
{
    m_logLevel = static_cast<pdal::LogLevel>(level);
    setLogStream(m_logStream);
}


int PipelineExecutor::getLogLevel() const
{
    return static_cast<int>(m_logLevel);
}


std::string PipelineExecutor::getLog() const
{
    return m_logStream.str();
}


}} //namespace pdal::executor

