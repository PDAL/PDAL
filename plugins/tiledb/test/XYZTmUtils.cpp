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

void BOX4D::clear()
{
    BOX3D::clear();
    mintm = HIGHEST;
    maxtm = LOWEST;
}

void BOX4D::parse(const std::string& s, std::string::size_type& pos)
{
    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != '(')
        throw error("No opening '('.");
    parsePair<BOX4D>(s, pos, minx, maxx);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ',')
        throw error("No comma separating 'X' and 'Y' dimensions.");
    parsePair<BOX4D>(s, pos, miny, maxy);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ',')
        throw error("No comma separating 'Y' and 'Z' dimensions.");
    parsePair<BOX4D>(s, pos, minz, maxz);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos] != ',' && s[pos] != ')')
        throw error("No comma separating 'Z' and 'time' dimensions.");
    else if (s[pos++] != ')')
    {
        parsePair<BOX4D>(s, pos, mintm, maxtm);
        pos++;
    }

    pos += Utils::extractSpaces(s, pos);
}

std::istream& operator>>(std::istream& in, BOX4D& box)
{
    std::string s;

    std::getline(in, s);
    std::string::size_type pos(0);

    box.parse(s, pos);
    if (pos != s.size())
        throw BOX4D::error("Invalid characters following valid 4d-bounds.");
    return in;
}

Bounds4D::Bounds4D(const BOX4D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = box.minz;
    m_box.maxz = box.maxz;
    m_box.mintm = box.mintm;
    m_box.maxtm = box.maxtm;
}

Bounds4D::Bounds4D(const BOX3D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = box.minz;
    m_box.maxz = box.maxz;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

Bounds4D::Bounds4D(const BOX2D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

BOX4D Bounds4D::to4d() const
{
    if (!is4d())
        return BOX4D();
    return m_box;
}

BOX3D Bounds4D::to3d() const
{
    if (is2d())
        return BOX3D();
    return m_box.to3d();
}

bool Bounds4D::is4d() const
{
    return (m_box.mintm != HIGHEST || m_box.maxtm != LOWEST);
}

void Bounds4D::reset(const BOX4D& box)
{
    m_box = box;
}

void Bounds4D::reset(const BOX3D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = box.minz;
    m_box.maxz = box.maxz;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

void Bounds4D::reset(const BOX2D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

void Bounds4D::grow(double x, double y)
{
    if (!is4d() && !is3d())
    {
        m_box.minx = (std::min)(x, m_box.minx);
        m_box.miny = (std::min)(y, m_box.miny);
        m_box.maxx = (std::max)(x, m_box.maxx);
        m_box.maxy = (std::max)(y, m_box.maxy);
    }
}

void Bounds4D::grow(double x, double y, double z)
{
    if (!is4d())
    {
        m_box.minx = (std::min)(x, m_box.minx);
        m_box.miny = (std::min)(y, m_box.miny);
        m_box.minz = (std::min)(z, m_box.minz);
        m_box.maxx = (std::max)(x, m_box.maxx);
        m_box.maxy = (std::max)(y, m_box.maxy);
        m_box.maxz = (std::max)(z, m_box.maxz);
    }
}

void Bounds4D::grow(double x, double y, double z, double tm)
{
    m_box.grow(x, y, z, tm);
}

void Bounds4D::parse(const std::string& s, std::string::size_type& pos)
{
    try
    {
        BOX4D box4d;
        box4d.parse(s, pos);
        set(box4d);
    }
    catch (const BOX4D::error&)
    {
        try
        {
            BOX3D box3d;
            box3d.parse(s, pos);
            set(box3d);
        }
        catch (const BOX3D::error&)
        {
            try
            {
                pos = 0;
                BOX2D box2d;
                box2d.parse(s, pos);
                set(box2d);
            }
            catch (const BOX2D::error& err)
            {
                throw Bounds4D::error(err.what());
            }
        }
    }
}

void Bounds4D::set(const BOX4D& box)
{
    m_box = box;
}

void Bounds4D::set(const BOX3D& box)
{
    m_box = BOX4D(box);
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

void Bounds4D::set(const BOX2D& box)
{
    m_box = BOX4D(box);
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

std::istream& operator>>(std::istream& in, Bounds4D& bounds)
{
    std::string s;

    std::getline(in, s);
    std::string::size_type pos(0);

    bounds.parse(s, pos);
    return in;
}

std::ostream& operator<<(std::ostream& out, const Bounds4D& bounds)
{
    if (bounds.is4d())
        out << bounds.to4d();
    else if (bounds.is3d())
        out << bounds.to3d();
    else
        out << bounds.to2d();
    return out;
}


















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


