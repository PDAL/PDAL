#include <pdal/PointRef.hpp>
#include <pdal/PointView.hpp>

namespace pdal
{

PointRef::PointRef(PointView& v, PointId idx) : m_table(&v.table()),
    m_idx(v.tableId(idx)), m_view(&v), m_viewIdx(idx), m_orig(true)
{}

PointRef& PointRef::operator=(const PointRef& r)
{
    assert(m_table == r.m_table);

    if (m_orig)
        m_view->setTableId(m_viewIdx, r.m_idx);
    m_idx = r.m_idx;
    m_viewIdx = r.m_viewIdx;

    return *this;
}

void PointRef::setFieldInternal(Dimension::Id dim, void *val)
{
    if (m_view && m_viewIdx == m_view->size())
        m_idx = m_view->addPoint();
    m_table->setFieldInternal(dim, m_idx, val);
}

void PointRef::setPointId(PointId idx)
{
    if (m_view)
    {
        m_viewIdx = idx;
        m_idx = m_view->tableId(idx);
    }
    else
        m_idx = idx;
}

void PointRef::swap(const PointRef& r1, const PointRef& r2)
{
    assert(r1.m_view);
    assert(r2.m_view);

    r1.m_view->swapItems(r1.m_viewIdx, r2.m_viewIdx);
}

void PointRef::swap(PointRef& r1, PointRef& r2)
{
    assert(r1.m_view);
    assert(r2.m_view);

    PointRef tmp = r1;
    r1 = r2;
    r2 = tmp;
}

} // namespace pdal
