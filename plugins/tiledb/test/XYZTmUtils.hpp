//
// Created by Chloe T on 10/3/21.
//
#include <ctime>

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <io/FauxReader.hpp>

#include <pdal/PluginHelper.hpp>

#include <pdal/util/Bounds.hpp>


namespace pdal
{

class PDAL_DLL BOX4D : public BOX3D
{
public:
    struct error : public std::runtime_error
        {
        error(const std::string& err) : std::runtime_error(err) {}
        };

    using BOX3D::maxx;
    using BOX3D::maxy;
    using BOX3D::maxz;
    using BOX3D::minx;
    using BOX3D::miny;
    using BOX3D::minz;
    double mintm; ///< Minimum time value.
    double maxtm; ///< Maximum time value.

    BOX4D()
    {
        clear();
    }

    BOX4D(const BOX4D& box) : BOX3D(box), mintm(box.mintm), maxtm(box.maxtm) {}

    BOX4D& operator=(const BOX4D& box) = default;

    explicit BOX4D(const BOX3D& box) : BOX3D(box), mintm(0), maxtm(0) {}

    explicit BOX4D(const BOX2D& box) : BOX3D(box.minx, box.miny, 0, box.maxx, box.maxy, 0), mintm(0), maxtm(0) {}

    BOX4D(double minx, double miny, double minz, double mintm, double maxx,
          double maxy, double maxz, double maxtm)
          : BOX3D(minx, miny, minz, maxx, maxy, maxz), mintm(mintm), maxtm(maxtm)
          {}

    bool empty() const;

    bool valid() const;

    BOX4D& grow(double x, double y, double z, double tm);

    void clear();

    bool contains(double x, double y, double z, double tm) const
    {
        return BOX3D::contains(x, y, z) && mintm <= tm && tm <= maxtm;
    }

    bool contains(const BOX4D& other) const
    {
        return BOX3D::contains(other) &&
        mintm <= other.mintm && other.maxtm <= maxtm;
    }

    bool equal(const BOX4D& other) const
    {
        return  BOX3D::contains(other) &&
        mintm == other.mintm && maxtm == other.maxtm;
    }

    bool operator==(BOX4D const& rhs) const
    {
        return equal(rhs);
    }

    bool operator!=(BOX4D const& rhs) const
    {
        return (!equal(rhs));
    }

    BOX3D& grow(const BOX3D& other)
    {
        BOX2D::grow(other);
        if (other.minz < minz) minz = other.minz;
        if (other.maxz > maxz) maxz = other.maxz;
        return *this;
    }

    BOX3D& grow(double dist)
    {
        BOX2D::grow(dist);
        minz -= dist;
        maxz += dist;
        return *this;
    }

    void clip(const BOX3D& other)
    {
        BOX2D::clip(other);
        if (other.minz < minz) minz = other.minz;
        if (other.maxz > maxz) maxz = other.maxz;
    }

    bool overlaps(const BOX3D& other) const
    {
        return BOX2D::overlaps(other) &&
        minz <= other.maxz && maxz >= other.minz;
    }

    BOX3D to3d() const
    {
        return *this;
    }

    BOX2D to2d() const
    {
        return *this;
    }

    void parse(const std::string& s, std::string::size_type& pos);
};


class PDAL_DLL Bounds4D : public Bounds
{
public:

    explicit Bounds4D(const BOX4D& box);
    explicit Bounds4D(const BOX3D& box) : Bounds(box)
    {}
    explicit Bounds4D(const BOX2D& box) : Bounds(box)
    {}

    BOX4D to4d() const;
    bool is4d() const;
    void reset(const BOX4D& box);
    void grow(double x, double y, double z, double tm);

    friend PDAL_DLL std::istream& operator >> (std::istream& in,
        Bounds& bounds);
    friend PDAL_DLL std::ostream& operator << (std::ostream& out,
        const Bounds& bounds);

private:
    BOX4D m_box;

    void set(const BOX4D& box);
};

inline std::ostream& operator << (std::ostream& ostr, const BOX4D& bounds)
{
    if (bounds.empty())
    {
        ostr << "()";
        return ostr;
    }

    auto savedPrec = ostr.precision();
    ostr.precision(16); // or..?
    ostr << "(";
    ostr << "[" << bounds.minx << ", " << bounds.maxx << "], " <<
    "[" << bounds.miny << ", " << bounds.maxy << "], " <<
    "[" << bounds.minz << ", " << bounds.maxz << "], " <<
    "[" << bounds.mintm << ", " << bounds.maxtm << "]";
    ostr << ")";
    ostr.precision(savedPrec);
    return ostr;
}

extern PDAL_DLL std::istream& operator>>(std::istream& istr, BOX4D& bounds);

PDAL_DLL std::istream& operator >> (std::istream& in, Bounds4D& bounds);

PDAL_DLL std::ostream& operator << (std::ostream& out, const Bounds4D& bounds);

namespace Utils
{
    template<>
    inline StatusWithReason fromString(const std::string& s, BOX4D& bounds)
    {
        try
        {
            std::istringstream iss(s);
            iss >> bounds;
        }
        catch (BOX4D::error& error)
        {
            std::string msg = "Error parsing '" + s + "': " + error.what();
            return StatusWithReason(-1, msg);
        }
        return true;
    }

    template<>
    inline StatusWithReason fromString(const std::string& s, Bounds4D& bounds)
    {
        try
        {
            std::istringstream iss(s);
            iss >> bounds;
        }
        catch (Bounds4D::error& error)
        {
            std::string msg = "Error parsing '" + s + "': " + error.what();
            return StatusWithReason(-1, msg);
        }
        return true;
    }
}

class PDAL_DLL XYZTimeFauxReader : public Reader, public Streamable
{
public:
    XYZTimeFauxReader() {};

    std::string getName() const;

private:
    using urd = std::uniform_real_distribution<double>;
    std::mt19937 m_generator;
    int m_numReturns;
    point_count_t m_index;
    std::unique_ptr<urd> m_uniformX;
    std::unique_ptr<urd> m_uniformY;
    std::unique_ptr<urd> m_uniformZ;
    BOX4D m_bounds;

    virtual void addArgs(ProgramArgs& args);

    virtual void prepared(PointTableRef table);

    virtual void initialize();

    virtual void addDimensions(PointLayoutPtr layout);

    virtual void ready(PointTableRef table);

    virtual bool processOne(PointRef& point);

    virtual point_count_t read(PointViewPtr view, point_count_t count);

    virtual bool eof()
        { return false; }

    XYZTimeFauxReader& operator=(const XYZTimeFauxReader&);
    XYZTimeFauxReader(const XYZTimeFauxReader&);
};

}