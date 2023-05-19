#pragma once

#ifdef DELAUNATOR_HEADER_ONLY
#define INLINE inline
#else
#define INLINE
#endif

#if WINDOWS
#undef max
#endif // WINDOWS

#include <limits>
#include <vector>

#include <stdint.h>


namespace delaunator {

// using shorter value type for indices rather than size_t
// to consume less memory (it is unlikely that the triangulation
// would be done on more than 4 billion points)
typedef uint32_t index_t;

constexpr index_t INVALID_INDEX =
    (std::numeric_limits<index_t>::max)();

class Point
{
public:
    Point(double x, double y) : m_x(x), m_y(y)
    {}
    Point() : m_x(0), m_y(0)
    {}


    double x() const
    { return m_x; }

    double y() const
    { return m_y; }

private:
    double m_x;
    double m_y;
};

class Delaunator {

public:
    std::vector<double> const& coords;
    std::vector<index_t> triangles;
    std::vector<index_t> halfedges;
    std::vector<index_t> hull_prev;
    std::vector<index_t> hull_next;
    std::vector<index_t> hull_tri;
    index_t hull_start;

    INLINE Delaunator(std::vector<double> const& in_coords);
    INLINE double get_hull_area();

private:
    std::vector<index_t> m_hash;
    Point m_center;
    index_t m_hash_size;
    std::vector<index_t> m_edge_stack;

    INLINE index_t legalize(index_t a);
    INLINE index_t hash_key(double x, double y) const;
    INLINE index_t add_triangle(
        index_t i0,
        index_t i1,
        index_t i2,
        index_t a,
        index_t b,
        index_t c);
    INLINE void link(index_t a, index_t b);
};

} //namespace delaunator

#undef INLINE
