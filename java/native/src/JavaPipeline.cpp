#include "JavaPipeline.hpp"
#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

namespace libpdaljava
{

Pipeline::Pipeline(std::string const& json)
    : m_executor(json)
{

}

Pipeline::~Pipeline()
{
}

void Pipeline::setLogLevel(int level)
{
    m_executor.setLogLevel(level);
}

int Pipeline::getLogLevel() const
{
    return static_cast<int>(m_executor.getLogLevel());
}

int64_t Pipeline::execute()
{

    int64_t count = m_executor.execute();
    return count;
}

bool Pipeline::validate()
{
    return m_executor.validate();
}

pdal::PointViewSet Pipeline::getPointViews() const
{
    if (!m_executor.executed())
        throw java_error("call execute() before fetching arrays");

    return m_executor.getManagerConst().views();
}
} //namespace libpdaljava
