#include "JavaPipeline.hpp"
#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif


namespace libpdaljava
{

Pipeline::Pipeline(std::string const& json)
    : m_executor(json)
{

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

bool Pipeline::validate()
{
    return m_executor.validate();
}

pdal::PointViewSet Pipeline::getPointViews() const
{
    if (!m_executor.executed())
        throw java_error("call execute() before fetching arrays");

    return m_executor.getManagerConst().views();

    //const pdal::PointViewSet& pvset = m_executor.getManagerConst().views();

    /* [array([ (635619.85, 850064.04, 447.01, 1, 1, 2, 0, 0, 1, 15.0, 125, 7327, 246092.20788099366, 52, 65, 68),
           (635640.42, 849758.79, 422.74, 25, 1, 1, 1, 0, 1, 6.0, 124, 7327, 246092.68201508585, 39, 57, 56),
           (635650.9500000001, 850244.03, 424.93, 33, 1, 1, 1, 0, 2, -11.0, 126, 7329, 247175.60983893464, 109, 104, 122),
           ...,
           (638972.01, 852465.78, 442.32, 1, 2, 3, 0, 0, 1, 0.0, 126, 7332, 248668.47601131082, 116, 118, 138),
           (638972.93, 853326.9400000001, 439.44, 0, 2, 3, 0, 0, 1, 8.0, 124, 7334, 249764.54700494712, 91, 99, 115),
           (638982.55, 853131.86, 456.92, 56, 1, 1, 0, 0, 1, -13.0, 128, 7332, 248667.42579574048, 120, 111, 124)],
          dtype=[(u'X', '<f8'), (u'Y', '<f8'), (u'Z', '<f8'), (u'Intensity', '<u2'), (u'ReturnNumber', 'u1'), (u'NumberOfReturns', 'u1'), (u'ScanDirectionFlag', 'u1'), (u'EdgeOfFlightLine', 'u1'), (u'Classification', 'u1'), (u'ScanAngleRank', '<f4'), (u'UserData', 'u1'), (u'PointSourceId', '<u2'), (u'GpsTime', '<f8'), (u'Red', '<u2'), (u'Green', '<u2'), (u'Blue', '<u2')])]
     */

    /*for (auto i: pvset)
    {
        PArray array = new pdal::plang::Array;
        array->update(i);
        output.push_back(array);
    }
    return output;*/
}
} //namespace libpdaljava
