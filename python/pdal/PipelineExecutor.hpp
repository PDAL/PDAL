#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/util/FileUtils.hpp>

#include <string>

namespace pdal
{
namespace executor
{

class Pipeline {
public:
    Pipeline(std::string const& json);
    ~Pipeline(){};

    int64_t execute();
    inline const char* getJSON() const { return m_json.c_str(); }
    inline const char* getSchema() const { return m_schema.c_str(); }
    PipelineManager const& manager() const { return m_manager; }


//     std::vector<PArray> getArrays() const;

private:
    std::string m_json;
    std::string m_schema;
    pdal::PipelineManager m_manager; // no progress reporting

};

}
}
