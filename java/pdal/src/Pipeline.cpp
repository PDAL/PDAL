#include "Pipeline.hpp"
#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

namespace libpdaljava
{

Pipeline::Pipeline(std::string const& json)
    : m_json(json)
    , m_schema("")
    , m_manager(-1)
{ }

void Pipeline::execute()
{
    std::stringstream strm;
    strm << m_json;
    m_manager.readPipeline(strm);
    m_manager.execute();
#ifdef PDAL_HAVE_LIBXML2
    pdal::XMLSchema schema(m_manager.pointTable().layout());
    m_schema = schema.xml();
#endif
    strm.str("");
    pdal::PipelineWriter::writePipeline(m_manager.getStage(), strm);
    m_json = strm.str();

    std::cout << m_manager.views().size() << std::endl;
}

pdal::PointViewSet Pipeline::getArrays() const
{
    return m_manager.views();
}
}
//namespace libpdaljava

