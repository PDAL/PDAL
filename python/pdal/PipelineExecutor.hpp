#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/util/FileUtils.hpp>

#include <string>

namespace pdal
{
namespace executor
{

class PipelineExecutor {
public:
    PipelineExecutor(std::string const& json);
    ~PipelineExecutor(){};

    int64_t execute();
    bool executed() const { return m_executed; }

    std::string getPipeline() const;
    std::string getMetadata() const;
    std::string getSchema() const;
    std::string getLog() const;
    void setLogLevel(int level);
    int getLogLevel() const;

    PipelineManager const& getManagerConst() const { return m_manager; }
    PipelineManager & getManager() { return m_manager; }


//     std::vector<PArray> getArrays() const;

private:
    void setLogStream(std::ostream& strm);

    std::string m_json;
    pdal::PipelineManager m_manager; // no progress reporting
    bool m_executed;
    std::stringstream m_logStream;
    pdal::LogLevel m_logLevel;

};

}
}
