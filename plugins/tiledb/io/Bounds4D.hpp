/******************************************************************************
 * Copyright (c) 2021 TileDB, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
 *       names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior
 *       written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 ****************************************************************************/

#pragma once

#include <pdal/util/Bounds.hpp>

namespace pdal
{
// dummy class to allow access to private BOX2D in constructor and conversion
class BOX2D_ : public BOX2D
{
public:
    BOX2D_() : BOX2D() {}
    BOX2D_(const BOX3D& box) : BOX2D(box.minx, box.miny, box.maxx, box.maxy) {}
    using BOX2D::maxx;
    using BOX2D::maxy;
    using BOX2D::minx;
    using BOX2D::miny;
};

class BOX4D : private BOX3D
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

    explicit BOX4D(const BOX2D_& box)
        : BOX3D(box.minx, box.miny, 0, box.maxx, box.maxy, 0), mintm(0),
          maxtm(0)
    {
    }

    BOX4D(double minx, double miny, double minz, double mintm, double maxx,
          double maxy, double maxz, double maxtm)
        : BOX3D(minx, miny, minz, maxx, maxy, maxz), mintm(mintm), maxtm(maxtm)
    {
    }

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
        return BOX3D::contains(other) && mintm <= other.mintm &&
               other.maxtm <= maxtm;
    }

    bool equal(const BOX4D& other) const
    {
        return BOX3D::equal(other) && mintm == other.mintm &&
               maxtm == other.maxtm;
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
        if (other.mintm < mintm)
            mintm = other.mintm;
        if (other.maxtm > maxtm)
            maxtm = other.maxtm;
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
        if (other.mintm > mintm && other.mintm < maxtm)
            mintm = other.mintm;
        if (other.maxtm < maxtm && other.maxtm > mintm)
            maxtm = other.maxtm;
    }

    bool overlaps(const BOX4D& other) const
    {
        return BOX3D::overlaps(other) && mintm <= other.maxtm &&
               maxtm >= other.mintm;
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
        Utils::StringStreamClassicLocale oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "box4d(" << minx << " " << miny << " " << minz << " " << mintm
            << ", " << maxx << " " << maxy << " " << maxz << " " << maxtm
            << ")";
        return oss.str();
    }

    static const BOX4D& getDefaultSpatialExtent();

    void parse(const std::string& s, std::string::size_type& pos);
};

class Bounds4D : public Bounds
{
public:
    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err) {}
    };

    Bounds4D() {}

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

    friend PDAL_DLL std::istream& operator>>(std::istream& in,
                                             Bounds4D& bounds);
    friend PDAL_DLL std::ostream& operator<<(std::ostream& out,
                                             const Bounds4D& bounds);

private:
    BOX4D m_box;

    void set(const BOX4D& box);
    void set(const BOX3D& box);
    void set(const BOX2D_& box);
};

inline std::ostream& operator<<(std::ostream& ostr, const BOX4D& bounds)
{
    if (bounds.empty())
    {
        ostr << "()";
        return ostr;
    }
    Utils::StringStreamClassicLocale ss;
    ss.precision(16); // or..?
    ss << "(";
    ss << "[" << bounds.minx << ", " << bounds.maxx << "], "
       << "[" << bounds.miny << ", " << bounds.maxy << "], "
       << "[" << bounds.minz << ", " << bounds.maxz << "], "
       << "[" << bounds.mintm << ", " << bounds.maxtm << "]";
    ss << ")";
    ostr << ss.str();
    return ostr;
}

extern std::istream& operator>>(std::istream& istr, BOX4D& bounds);

std::istream& operator>>(std::istream& in, Bounds4D& bounds);

std::ostream& operator<<(std::ostream& out, const Bounds4D& bounds);

namespace Utils
{
template <>
inline StatusWithReason fromString(const std::string& s, BOX4D& bounds)
{
    try
    {
        Utils::IStringStreamClassicLocale iss(s);
        iss >> bounds;
    }
    catch (BOX4D::error& error)
    {
        std::string msg = "Error parsing '" + s + "': " + error.what();
        return StatusWithReason(-1, msg);
    }
    return true;
}

template <>
inline StatusWithReason fromString(const std::string& s, Bounds4D& bounds)
{
    try
    {
        Utils::IStringStreamClassicLocale iss(s);
        iss >> bounds;
    }
    catch (Bounds4D::error& error)
    {
        std::string msg = "Error parsing '" + s + "': " + error.what();
        return StatusWithReason(-1, msg);
    }
    return true;
}

} // namespace Utils

} // namespace pdal
