//
// Created by Chloe T on 10/7/21.
//

#pragma once

#include <pdal/util/Bounds.hpp>

namespace pdal
{

class PDAL_DLL BOX2D_ : public BOX2D
{
public:
    BOX2D_() : BOX2D()
    {}
    BOX2D_(const BOX3D& box) : BOX2D(box.minx, box.miny, box.maxx, box.maxy)
    {}
    using BOX2D::maxx;
    using BOX2D::maxy;
    using BOX2D::minx;
    using BOX2D::miny;
};

class PDAL_DLL BOX4D : private BOX3D
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
        { clear(); }

    BOX4D(const BOX4D& box) :
    BOX3D(box), mintm(box.mintm), maxtm(box.maxtm)
    {}

    BOX4D& operator=(const BOX4D& box) = default;

    explicit BOX4D(const BOX3D& box) :
    BOX3D(box), mintm(0), maxtm(0) {}

    explicit BOX4D(const BOX2D_& box) :
    BOX3D(box.minx, box.miny, 0, box.maxx, box.maxy, 0), mintm(0), maxtm(0) {}

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
        return  BOX3D::equal(other) &&
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

    BOX4D& grow(const BOX4D& other)
    {
        BOX3D::grow(other);
        if (other.mintm < mintm) mintm = other.mintm;
        if (other.maxtm > maxtm) maxtm = other.maxtm;
        return *this;
    }

    BOX4D& grow(double dist)
    {
        BOX3D::grow(dist);
        mintm -= dist;
        maxtm += dist;
        return *this;
    }

    void clip(const BOX4D& other)
    {
        BOX3D::clip(other);
        if (other.mintm > mintm && other.mintm < maxtm) mintm = other.mintm;
        if (other.maxtm < maxtm && other.maxtm > mintm) maxtm = other.maxtm;
    }

    bool overlaps(const BOX4D& other) const
    {
        return BOX3D::overlaps(other) &&
        mintm <= other.maxtm && maxtm >= other.mintm;
    }

    BOX3D to3d() const
    {
        return *this;
    }

    BOX2D_ to2d() const
    {
        return this->to3d();
    }

    std::string toBox(uint32_t precision = 8) const
    {
        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "box4d(" << minx << " " << miny << " " << minz << " " << mintm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << maxtm << ")";
        return oss.str();
    }

    std::string toWKT(uint32_t precision = 8) const
    {
        if (empty())
            return std::string();

        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "POLYHEDRON ZM ( ";

        // Inner sufaces in Tesseract
        oss << "((" << minx << " " << miny << " " << minz << " " << mintm << ", " <<
        maxx << " " << miny << " " << minz << " " << mintm << ", " <<
        maxx << " " << maxy << " " << minz << " " << mintm << ", " <<
        minx << " " << maxy << " " << minz << " " << mintm << ", " <<
        minx << " " << miny << " " << minz << " " << mintm << ", " <<
        ")), ";
        oss << "((" << minx << " " << miny << " " << minz << " " << mintm << ", " <<
        maxx << " " << miny << " " << minz << " " << mintm << ", " <<
        maxx << " " << miny << " " << maxz << " " << mintm << ", " <<
        minx << " " << miny << " " << maxz << " " << mintm << ", " <<
        minx << " " << miny << " " << minz << " " << mintm << ", " <<
        ")), ";
        oss << "((" << maxx << " " << miny << " " << minz << " " << mintm << ", " <<
        maxx << " " << maxy << " " << minz << " " << mintm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        maxx << " " << miny << " " << maxz << " " << mintm << ", " <<
        maxx << " " << miny << " " << minz << " " << mintm << ", " <<
        ")), ";
        oss << "((" << maxx << " " << maxy << " " << minz << " " << mintm << ", " <<
        minx << " " << maxy << " " << minz << " " << mintm << ", " <<
        minx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        maxx << " " << maxy << " " << minz << " " << mintm << ", " <<
        ")), ";
        oss << "((" << minx << " " << maxy << " " << minz << " " << mintm << ", " <<
        minx << " " << miny << " " << minz << " " << mintm << ", " <<
        minx << " " << miny << " " << maxz << " " << mintm << ", " <<
        minx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        minx << " " << maxy << " " << minz << " " << mintm << ", " <<
        ")), ";
        oss << "((" << minx << " " << miny << " " << maxz << " " << mintm << ", " <<
        maxx << " " << miny << " " << maxz << " " << mintm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        minx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        minx << " " << miny << " " << maxz << " " << mintm << ", " <<
        ")), ";

        // Outer surfaces in Tesseract
        oss << "((" << minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << minz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << minz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        minx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << maxx << " " << miny << " " << minz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << maxx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << minx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        minx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << minx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        minx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        ")), ";

        // Top down-dented surfaces in Tesseract
        oss << "((" << minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << minz << " " << mintm << ", " <<
        minx << " " << miny << " " << minz << " " << mintm << ", " <<
        minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << minx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        minx << " " << miny << " " << maxz << " " << mintm << ", " <<
        minx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        minx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        minx << " " << miny << " " << maxz << " " << mintm << ", " <<
        minx << " " << miny << " " << minz << " " << mintm << ", " <<
        minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << minx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        minx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        minx << " " << maxy << " " << minz << " " << mintm << ", " <<
        minx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        ")), ";

        // Bottom up-dented surfaces in Tesseract
        oss << "((" << maxx << " " << miny << " " << minz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << minz << " " << mintm << ", " <<
        maxx << " " << miny << " " << minz << " " << mintm << ", " <<
        maxx << " " << miny << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << maxx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        maxx << " " << miny << " " << maxz << " " << mintm << ", " <<
        maxx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << maxx << " " << miny << " " << minz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << maxz << " " << mintm << ", " <<
        maxx << " " << miny << " " << minz << " " << mintm << ", " <<
        maxx << " " << miny << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << maxx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        maxx << " " << maxy << " " << minz << " " << mintm << ", " <<
        maxx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        ")), ";

        // Side inward-indented surfaces in Tesseract - left
        oss << "((" << minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << minz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << minz << " " << mintm << ", " <<
        minx << " " << miny << " " << minz << " " << mintm << ", " <<
        minx << " " << miny << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << minx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << miny << " " << maxz << " " << mintm << ", " <<
        minx << " " << miny << " " << maxz << " " << mintm << ", " <<
        minx << " " << miny << " " << maxz << " " << maxtm << ", " <<
        ")), ";

        // Side inward-indented surfaces in Tesseract - right
        oss << "((" << minx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << minz << " " << mintm << ", " <<
        minx << " " << maxy << " " << minz << " " << mintm << ", " <<
        minx << " " << maxy << " " << minz << " " << maxtm << ", " <<
        ")), ";
        oss << "((" << minx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        maxx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        minx << " " << maxy << " " << maxz << " " << mintm << ", " <<
        minx << " " << maxy << " " << maxz << " " << maxtm << ", " <<
        ")), ";

        oss << " )";

        return oss.str();
    }

    static const BOX4D& getDefaultSpatialExtent();

    void parse(const std::string& s, std::string::size_type& pos);
};


class PDAL_DLL Bounds4D : public Bounds
{
public:
    struct error : public std::runtime_error
        {
        error(const std::string& err) : std::runtime_error(err)
        {}
        };

    Bounds4D()
    {}

    explicit Bounds4D(const BOX4D& box);
    explicit Bounds4D(const BOX3D& box);
    explicit Bounds4D(const BOX2D& box);

    BOX4D to4d() const;
    BOX3D to3d() const;
    BOX2D to2d() const;
    bool is4d() const;
    bool is3d() const;
    bool is2d() const;
    bool empty() const;
    bool valid() const;
    void reset(const BOX4D& box);
    void reset(const BOX3D& box);
    void reset(const BOX2D& box);
    void grow(double x, double y);
    void grow(double x, double y, double z);
    void grow(double x, double y, double z, double tm);
    void parse(const std::string& s, std::string::size_type& pos);

    friend PDAL_DLL std::istream& operator >> (std::istream& in,
        Bounds4D& bounds);
    friend PDAL_DLL std::ostream& operator << (std::ostream& out,
        const Bounds4D& bounds);

private:
    BOX4D m_box;

    void set(const BOX4D& box);
    void set(const BOX3D& box);
    void set(const BOX2D_& box);
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

} //namespace Utils;

} //namespace pdal;
