#include "Pipeline.hpp"
#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

#include <pdal/plang/Array.hpp>


namespace libpdalpython
{

Pipeline::Pipeline(std::string const& xml)
    : m_xml(xml)
    , m_schema("")
    , m_manager(-1)
{
    auto initNumpy = []()
    {
#undef NUMPY_IMPORT_ARRAY_RETVAL
#define NUMPY_IMPORT_ARRAY_RETVAL
        import_array();
    };
    initNumpy();
}

void Pipeline::execute()
{
    std::stringstream strm;
    strm << m_xml;
    m_manager.readPipeline(strm);
    m_manager.execute();
#ifdef PDAL_HAVE_LIBXML2
    pdal::XMLSchema schema(m_manager.pointTable().layout());
    m_schema = schema.xml();
#endif

    strm.str("");
    pdal::PipelineWriter::writePipeline(m_manager.getStage(), strm);

    m_xml = strm.str();

}

std::vector<PArray> Pipeline::getArrays() const
{
    std::vector<PArray> output;
    const pdal::PointViewSet& pvset = m_manager.views();


    for (auto i: pvset)
    {
        PArray array = new pdal::plang::Array;
        array->update(i);
        output.push_back(array);
    }
    return output;
}
} //namespace libpdalpython

