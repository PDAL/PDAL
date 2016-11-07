#include "PyPipeline.hpp"
#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif


namespace libpdalpython
{

Pipeline::Pipeline(std::string const& json)
    : m_executor(json)
{
    auto initNumpy = []()
    {
#undef NUMPY_IMPORT_ARRAY_RETVAL
#define NUMPY_IMPORT_ARRAY_RETVAL
        import_array();
    };

    initNumpy();
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

std::vector<PArray> Pipeline::getArrays() const
{
    std::vector<PArray> output;

    if (!m_executor.executed())
        throw python_error("call execute() before fetching arrays");

    const pdal::PointViewSet& pvset = m_executor.getManagerConst().views();

    for (auto i: pvset)
    {
        PArray array = new pdal::plang::Array;
        array->update(i);
        output.push_back(array);
    }
    return output;
}
} //namespace libpdalpython

