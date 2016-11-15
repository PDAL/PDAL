#pragma once

#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/PipelineExecutor.hpp>

#include <string>
#include <sstream>
#undef toupper
#undef tolower
#undef isspace

namespace libpdaljava
{

class java_error : public std::runtime_error
{
public:
    inline java_error(std::string const& msg) : std::runtime_error(msg)
        {}
};

class Pipeline {
public:
    Pipeline(std::string const& xml);
    ~Pipeline();

    int64_t execute();
    bool validate();
    inline std::string getPipeline() const
    {
        return m_executor.getPipeline();
    }
    inline std::string getMetadata() const
    {
        return m_executor.getMetadata();
    }
    inline std::string getSchema() const
    {
        return m_executor.getSchema();
    }
    inline std::string getLog() const
    {
        return m_executor.getLog();
    }
    pdal::PointViewSet getPointViews() const;

    void setLogLevel(int level);
    int getLogLevel() const;

private:

    pdal::PipelineExecutor m_executor;

};

}
