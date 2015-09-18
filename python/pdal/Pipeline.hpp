#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/plang/Array.hpp>

#include <string>
#include <Python.h>
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

    void execute();
    inline const char* getXML() const { return m_xml.c_str(); }
    inline const char* getSchema() const { return m_schema.c_str(); }
    std::vector<PArray> getArrays() const;

private:
    std::string m_xml;
    std::string m_schema;
    pdal::PipelineManager m_manager; // no progress reporting

};

}
