// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2010-2012 Barend Gehrels, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_GEOMETRIES_ADAPTED_BOOST_POLYGON_RING_HPP
#define BOOST_GEOMETRY_GEOMETRIES_ADAPTED_BOOST_POLYGON_RING_HPP

// Adapts Geometries from Boost.Polygon for usage in Boost.Geometry
// pdalboost::polygon::polygon_data -> pdalboost::geometry::ring

#include <cstddef>
#include <boost/polygon/polygon.hpp>

#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/coordinate_dimension.hpp>
#include <boost/geometry/core/coordinate_type.hpp>
#include <boost/geometry/core/mutable_range.hpp>
#include <boost/geometry/core/tags.hpp>


#ifndef DOXYGEN_NO_TRAITS_SPECIALIZATIONS

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace geometry
{

namespace traits
{

template <typename CoordinateType>
struct tag<pdalboost::polygon::polygon_data<CoordinateType> >
{
    typedef ring_tag type;
};

template <typename CoordinateType>
struct clear<pdalboost::polygon::polygon_data<CoordinateType> >
{
    static inline void apply(pdalboost::polygon::polygon_data<CoordinateType>& data)
    {
        // There is no "clear" function but we can assign
        // a newly (and therefore empty) constructed polygon
        pdalboost::polygon::assign(data, pdalboost::polygon::polygon_data<CoordinateType>());
    }
};

template <typename CoordinateType>
struct push_back<pdalboost::polygon::polygon_data<CoordinateType> >
{
    typedef pdalboost::polygon::point_data<CoordinateType> point_type;

    static inline void apply(pdalboost::polygon::polygon_data<CoordinateType>& data,
         point_type const& point)
    {
        // Boost.Polygon's polygons are not appendable. So create a temporary vector,
        // add a record and set it to the original. Of course: this is not efficient.
        // But there seems no other way (without using a wrapper)
        std::vector<point_type> temporary_vector
            (
                pdalboost::polygon::begin_points(data),
                pdalboost::polygon::end_points(data)
            );
        temporary_vector.push_back(point);
        data.set(temporary_vector.begin(), temporary_vector.end());
    }
};




} // namespace traits

}} // namespace pdalboost::geometry


// Adapt Boost.Polygon's polygon_data to Boost.Range
// This just translates to
// polygon_data.begin() and polygon_data.end()
namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost
{
    template<typename CoordinateType>
    struct range_mutable_iterator<pdalboost::polygon::polygon_data<CoordinateType> >
    {
        typedef typename pdalboost::polygon::polygon_traits
            <
                pdalboost::polygon::polygon_data<CoordinateType>
            >::iterator_type type;
    };

    template<typename CoordinateType>
    struct range_const_iterator<pdalboost::polygon::polygon_data<CoordinateType> >
    {
        typedef typename pdalboost::polygon::polygon_traits
            <
                pdalboost::polygon::polygon_data<CoordinateType>
            >::iterator_type type;
    };

    template<typename CoordinateType>
    struct range_size<pdalboost::polygon::polygon_data<CoordinateType> >
    {
        typedef std::size_t type;
    };

} // namespace pdalboost


// Support Boost.Polygon's polygon_data for Boost.Range ADP
namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace polygon
{

template<typename CoordinateType>
inline typename polygon_traits
        <
            polygon_data<CoordinateType>
        >::iterator_type range_begin(polygon_data<CoordinateType>& polygon)
{
    return polygon.begin();
}

template<typename CoordinateType>
inline typename polygon_traits
        <
            polygon_data<CoordinateType>
        >::iterator_type range_begin(polygon_data<CoordinateType> const& polygon)
{
    return polygon.begin();
}

template<typename CoordinateType>
inline typename polygon_traits
        <
            polygon_data<CoordinateType>
        >::iterator_type range_end(polygon_data<CoordinateType>& polygon)
{
    return polygon.end();
}

template<typename CoordinateType>
inline typename polygon_traits
        <
            polygon_data<CoordinateType>
        >::iterator_type range_end(polygon_data<CoordinateType> const& polygon)
{
    return polygon.end();
}

template<typename CoordinateType>
inline std::size_t range_calculate_size(polygon_data<CoordinateType> const& polygon)
{
    return polygon.size();
}

}}

#endif // DOXYGEN_NO_TRAITS_SPECIALIZATIONS


#endif // BOOST_GEOMETRY_GEOMETRIES_ADAPTED_BOOST_POLYGON_RING_HPP
