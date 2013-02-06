// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2008-2012 Bruno Lalande, Paris, France.
// Copyright (c) 2008-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2009-2012 Mateusz Loskot, London, UK.

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_GEOMETRIES_ADAPTED_BOOST_TUPLE_HPP
#define BOOST_GEOMETRY_GEOMETRIES_ADAPTED_BOOST_TUPLE_HPP


#include <cstddef>

#include <boost/tuple/tuple.hpp>

#include <boost/geometry/core/coordinate_dimension.hpp>
#include <boost/geometry/core/coordinate_type.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/core/tags.hpp>


namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace geometry
{


#ifndef DOXYGEN_NO_TRAITS_SPECIALIZATIONS
namespace traits
{


template <typename T1, typename T2, typename T3, typename T4, typename T5,
          typename T6, typename T7, typename T8, typename T9, typename T10>
struct tag<pdalboost::tuple<T1, T2, T3, T4, T5, T6, T7, T8, T9, T10> >
{
    typedef point_tag type;
};


template <typename T1, typename T2, typename T3, typename T4, typename T5,
          typename T6, typename T7, typename T8, typename T9, typename T10>
struct coordinate_type<pdalboost::tuple<T1, T2, T3, T4, T5, T6, T7, T8, T9, T10> >
{
    typedef T1 type;
};


template <typename T1, typename T2, typename T3, typename T4, typename T5,
          typename T6, typename T7, typename T8, typename T9, typename T10>
struct dimension<pdalboost::tuple<T1, T2, T3, T4, T5, T6, T7, T8, T9, T10> >
    : pdalboost::mpl::int_
          <
              pdalboost::tuples::length
                  <
                      pdalboost::tuple<T1, T2, T3, T4, T5, T6, T7, T8, T9, T10>
                  >::value
          >
{};


template <typename T1, typename T2, typename T3, typename T4, typename T5,
          typename T6, typename T7, typename T8, typename T9, typename T10,
          std::size_t Dimension>
struct access
    <
        pdalboost::tuple<T1, T2, T3, T4, T5, T6, T7, T8, T9, T10>,
        Dimension
    >
{
    static inline T1 get(
        pdalboost::tuple<T1, T2, T3, T4, T5, T6, T7, T8, T9, T10> const& point)
    {
        return point.template get<Dimension>();
    }

    static inline void set(
        pdalboost::tuple<T1, T2, T3, T4, T5, T6, T7, T8, T9, T10>& point,
        T1 const& value)
    {
        point.template get<Dimension>() = value;
    }
};


} // namespace traits
#endif // DOXYGEN_NO_TRAITS_SPECIALIZATIONS


}} // namespace pdalboost::geometry


// Convenience registration macro to bind pdalboost::tuple to a CS
#define BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(CoordinateSystem) \
    namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace geometry { namespace traits { \
    template <typename T1, typename T2, typename T3, typename T4, typename T5, \
              typename T6, typename T7, typename T8, typename T9, typename T10> \
    struct coordinate_system<pdalboost::tuple<T1, T2, T3, T4, T5, T6, T7, T8, T9, T10> > \
    { \
        typedef CoordinateSystem type; \
    }; \
    }}}


#endif // BOOST_GEOMETRY_GEOMETRIES_ADAPTED_TUPLE_HPP
