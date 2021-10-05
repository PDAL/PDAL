//
// Created by Chloe T on 10/5/21.
//

#include "XYZTmUtils.hpp"


namespace pdal
{

void BOX4D::clear()
{
    BOX3D::clear();
    mintm = HIGHEST;
    maxtm = LOWEST;
}

bool BOX4D::empty() const
{
    return  BOX3D::empty() && mintm == HIGHEST && maxtm == LOWEST;
}

bool BOX4D::valid() const
{
    return !empty();
}

BOX4D& BOX4D::grow(double x, double y, double z, double tm)
{
    BOX3D::grow(x, y, z);
    if (tm < mintm) mintm = tm;
    if (tm > maxtm) maxtm = tm;
    return *this;
}

Bounds::Bounds(const BOX4D& box) : m_box(box)
{}























static StaticPluginInfo const s_info
{
    "readers.XYZTimeFauxReader",
    "XYZ time Faux Reader",
    "none"
}

CREATE_STATIC_STAGE(XYZTimeFauxReader, s_info);

std::string XYZTimeFauxReader::getName() const
{
    return s_info.name;
}

void XYZTimeFauxReader::addArgs(ProgramArgs& args)
{
    args.add("bounds", "X/Y/Z/time limits",
                 m_bounds, BOX4D(0., 0., 0., 0.,
                                        1., 1., 1., 1));
}

void XYZTimeFauxReader::prepared(PointTableRef table)
{
    if (!m_countArg->set())
        throwError("Argument 'count' needs a value and none was provided.");

}

void XYZTimeFauxReader::initialize()
{
    m_generator.seed((uint32_t)std::time(NULL));
    m_uniformX.reset(new urd(m_bounds.minx, m_bounds.maxx));
    m_uniformY.reset(new urd(m_bounds.miny, m_bounds.maxy));
    m_uniformZ.reset(new urd(m_bounds.minz, m_bounds.maxz));
}

void XYZTimeFauxReader::addDimensions(PointLayoutPtr layout)
{
    Dimension::IdList ids = {
        Dimension::Id::X,
        Dimension::Id::Y,
        Dimension::Id::Z,
        Dimension::Id::GpsTime,
        Dimension::Id::Density
    };
    layout->registerDims(ids);
}

void XYZTimeFauxReader::ready(PointTableRef table)
    { m_index = 0; }

#pragma warning(push)
#pragma warning(disable : 4244)

bool XYZTimeFauxReader::processOne(PointRef& point)
{
    double x(0);
    double y(0);
    double z(0);
    double tm(0);
    double density = 1.0;

    if (m_index >= m_count)
        return false;

    x = (*m_uniformX)(m_generator);
    y = (*m_uniformY)(m_generator);
    z = (*m_uniformZ)(m_generator);
    if (m_count > 1)
        tm = m_bounds.mintm + ((m_bounds.maxtm - m_bounds.mintm) / (m_count - 1)) * m_index;
    else
        tm = m_bounds.mintm;

    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    point.setField(Dimension::Id::GpsTime, tm);
    point.setField(Dimension::Id::Density, density);
    m_index++;
    return true;
}

#pragma warning(pop)

point_count_t XYZTimeFauxReader::read(PointViewPtr view, point_count_t count)
{
    for (PointId idx = 0; idx < count; ++idx) {
        PointRef point = view->point(idx);
        if (!processOne(point))
            break;
        if (m_cb)
            m_cb(*view, idx);
    };
    return count;
}

} //namespace pdal;


