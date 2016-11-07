#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/plang/Array.hpp>


#include "PipelineExecutor.hpp"

#include <string>
#include <sstream>
#undef toupper
#undef tolower
#undef isspace

#define PY_ARRAY_UNIQUE_SYMBOL LIBPDALPYTHON_ARRAY_API
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <numpy/arrayobject.h>


namespace libpdalpython
{

class python_error : public std::runtime_error
{
public:
    inline python_error(std::string const& msg) : std::runtime_error(msg)
        {}
};

    typedef pdal::plang::Array* PArray;

class Pipeline {
public:
    Pipeline(std::string const& xml);
    ~Pipeline();

    int64_t execute();
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
    std::vector<PArray> getArrays() const;


    void setLogLevel(int level);
    int getLogLevel() const;

private:

    pdal::executor::PipelineExecutor m_executor;

};

}
