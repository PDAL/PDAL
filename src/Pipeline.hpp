#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/util/FileUtils.hpp>

#include <string>
#undef toupper
#undef tolower
#undef isspace

//#define PY_ARRAY_UNIQUE_SYMBOL LIBPDALPYTHON_ARRAY_API
//#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

//#include <numpy/arrayobject.h>


namespace libpdaljava
{
//     typedef std::shared_ptr<pdal::plang::Array> PArray;
//     typedef pdal::plang::Array* PArray;
class Pipeline {
public:
    Pipeline(std::string const& xml);
    ~Pipeline(){};

    void execute();
    inline const char* getJSON() const { return m_json.c_str(); }
    inline const char* getSchema() const { return m_schema.c_str(); }
    pdal::PointViewSet getArrays() const;
    inline int test() const { return 22; }

private:
    std::string m_json;
    std::string m_schema;
    pdal::PipelineManager m_manager; // no progress reporting

};

}

