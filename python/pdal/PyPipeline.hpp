#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/plang/Array.hpp>

#include "PipelineExecutor.hpp"

#include <string>
#undef toupper
#undef tolower
#undef isspace

#define PY_ARRAY_UNIQUE_SYMBOL LIBPDALPYTHON_ARRAY_API
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <numpy/arrayobject.h>


namespace libpdalpython
{
//     typedef std::shared_ptr<pdal::plang::Array> PArray;
    typedef pdal::plang::Array* PArray;

class Pipeline {
public:
    Pipeline(std::string const& xml);
    ~Pipeline(){};

    int64_t execute();
    inline const char* getJSON() const { return m_executor.getJSON(); }
    inline const char* getSchema() const { return m_executor.getSchema(); }
    std::vector<PArray> getArrays() const;

private:
    pdal::executor::Pipeline m_executor;

};

}
