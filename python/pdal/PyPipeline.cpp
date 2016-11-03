#include "PyPipeline.hpp"
#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

#include <pdal/plang/Array.hpp>


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

int64_t Pipeline::execute()
{
    return m_executor.execute();

}

std::vector<PArray> Pipeline::getArrays() const
{
    std::vector<PArray> output;
    const pdal::PointViewSet& pvset = m_executor.manager().views();


    for (auto i: pvset)
    {
        PArray array = new pdal::plang::Array;
        array->update(i);
        output.push_back(array);
    }
    return output;
}
} //namespace libpdalpython

