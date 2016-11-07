#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/util/FileUtils.hpp>

#include <string>

namespace pdal
{
namespace executor
{

/**
  An executor hides the management of constructing, executing, and
  fetching data from a PipelineManager.

  It is constructed with JSON defining a pipeline.
*/

class PipelineExecutor {
public:

    /**
      Construct a PipelineExecutor

      \param json Pipeline JSON defining the PDAL operations
    */
    PipelineExecutor(std::string const& json);

    /**
      dtor
    */
    ~PipelineExecutor(){};

    /**
      Execute the pipeline

      \return total number of points produced by the pipeline.
    */
    int64_t execute();

    /**
      \return the transliterated pipeline
    */
    std::string getPipeline() const;

    /**
      \return computed metadata for the pipeline and all stages
    */
    std::string getMetadata() const;

    /**
      \return computed schema for the pipeline
    */
    std::string getSchema() const;

    /**
      \return log output for the executed pipeline. use
      setLogLevel to adjust verbosity.
    */
    std::string getLog() const;

    /**
      set the log verbosity. Use values 0-8.
    */
    void setLogLevel(int level);

    /**
      \return log verbosity
    */
    int getLogLevel() const;

    /**
      \return has the pipeline been executed
    */
    inline bool executed() const
    {
        return m_executed;
    }


    /**
      \return a const reference to the pipeline manager
    */
    PipelineManager const& getManagerConst() const { return m_manager; }

    /**
      \return a reference to the pipeline manager
    */
    PipelineManager & getManager() { return m_manager; }

private:
    void setLogStream(std::ostream& strm);

    std::string m_json;
    pdal::PipelineManager m_manager;
    bool m_executed;
    std::stringstream m_logStream;
    pdal::LogLevel m_logLevel;

};

}
}
