// Boost.Bimap
//
// Copyright (c) 2006-2007 Matias Capeletto
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

/// \file relation/detail/to_mutable_relation_functor.hpp
/// \brief functors to convert types to mutable relations

#ifndef BOOST_BIMAP_RELATION_DETAIL_TO_MUTABLE_RELATION_FUNCTOR_HPP
#define BOOST_BIMAP_RELATION_DETAIL_TO_MUTABLE_RELATION_FUNCTOR_HPP

#if defined(_MSC_VER) && (_MSC_VER>=1200)
#pragma once
#endif

#include <boost/config.hpp>

#include <boost/bimap/relation/support/pair_type_by.hpp>
#include <boost/bimap/relation/detail/mutant.hpp>
#include <boost/bimap/relation/mutant_relation.hpp>

namespace pdalboost{} namespace boost = pdalboost; namespace pdalboost{
namespace bimaps {
namespace relation {
namespace detail {

/// \brief Functor used in map views

template< class Tag, class Relation >
struct pair_to_relation_functor
{
    const Relation
        operator()(const BOOST_DEDUCED_TYPENAME ::pdalboost::bimaps::relation::support::
                            pair_type_by<Tag,Relation>::type & p) const
    {
        return Relation(p);
    }
};

template< class Tag, class TA, class TB, class Info >
struct pair_to_relation_functor<
    Tag,::pdalboost::bimaps::relation::mutant_relation<TA,TB,Info,true> >
{
    typedef ::pdalboost::bimaps::relation::mutant_relation<TA,TB,Info,true> Relation;

    Relation &
        operator()( BOOST_DEDUCED_TYPENAME ::pdalboost::bimaps::relation::support::
            pair_type_by<Tag,Relation>::type & p ) const
    {
        return ::pdalboost::bimaps::relation::detail::mutate<Relation>(p);
    }

    const Relation &
        operator()( const BOOST_DEDUCED_TYPENAME ::pdalboost::bimaps::relation::support::
            pair_type_by<Tag,Relation>::type & p) const
    {
        return ::pdalboost::bimaps::relation::detail::mutate<Relation>(p);
    }
};


/// \brief Used in set views

template< class Relation >
struct get_mutable_relation_functor
{
    const Relation
    operator()( const BOOST_DEDUCED_TYPENAME Relation::above_view & r ) const
    {
        return Relation(r);
    }
};

template< class TA, class TB, class Info >
struct get_mutable_relation_functor< ::pdalboost::bimaps::relation::mutant_relation<TA,TB,Info,true> >
{
    typedef ::pdalboost::bimaps::relation::mutant_relation<TA,TB,Info,true> Relation;

    Relation &
    operator()( BOOST_DEDUCED_TYPENAME Relation::above_view & r ) const
    {
        return ::pdalboost::bimaps::relation::detail::mutate<Relation>(r);
    }

    const Relation &
    operator()( const BOOST_DEDUCED_TYPENAME Relation::above_view & r ) const
    {
        return ::pdalboost::bimaps::relation::detail::mutate<Relation>(r);
    }
};

} // namespace detail
} // namespace relation
} // namespace bimaps
} // namespace pdalboost


#endif // BOOST_BIMAP_RELATION_DETAIL_TO_MUTABLE_RELATION_FUNCTOR_HPP

