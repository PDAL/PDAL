// Boost.Geometry Index
//
// Type-erased iterators
//
// Copyright (c) 2011-2013 Adam Wulkiewicz, Lodz, Poland.
//
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_TYPE_ERASED_ITERATORS_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_TYPE_ERASED_ITERATORS_HPP

#include <boost/type_erasure/any.hpp>
#include <boost/type_erasure/operators.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace geometry { namespace index { namespace detail {

template<typename T, typename ValueType, typename Reference, typename Pointer, typename DifferenceType>
struct single_pass_iterator_concept :
    ::pdalboost::mpl::vector<
        ::pdalboost::type_erasure::copy_constructible<T>,
        ::pdalboost::type_erasure::equality_comparable<T>,
        ::pdalboost::type_erasure::dereferenceable<Reference, T>,
        ::pdalboost::type_erasure::assignable<T>,
        ::pdalboost::type_erasure::incrementable<T>
    >
{};

template <typename ValueType, typename Reference, typename Pointer, typename DifferenceType>
struct single_pass_iterator_type
{
    typedef ::pdalboost::type_erasure::any<
        single_pass_iterator_concept<
            ::pdalboost::type_erasure::_self, ValueType, Reference, Pointer, DifferenceType
        >
    > type;
};

}}}} // namespace pdalboost::geometry::index::detail

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace type_erasure {

template<typename T, typename ValueType, typename Reference, typename Pointer, typename DifferenceType, typename Base>
struct concept_interface<
    ::pdalboost::geometry::index::detail::single_pass_iterator_concept<
        T, ValueType, Reference, Pointer, DifferenceType
    >, Base, T>
    : Base
{
    typedef ValueType value_type;
    typedef Reference reference;
    typedef Pointer pointer;
    typedef DifferenceType difference_type;
    typedef ::std::input_iterator_tag iterator_category;
};

}} // namespace pdalboost::type_erasure

#endif // BOOST_GEOMETRY_INDEX_DETAIL_TYPE_ERASED_ITERATORS_HPP
