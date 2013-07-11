// Boost.TypeErasure library
//
// Copyright 2011 Steven Watanabe
//
// Distributed under the Boost Software License Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// $Id: normalize.hpp 83251 2013-03-02 19:23:44Z steven_watanabe $

#ifndef BOOST_TYPE_ERASURE_DETAIL_NORMALIZE_HPP_INCLUDED
#define BOOST_TYPE_ERASURE_DETAIL_NORMALIZE_HPP_INCLUDED

#include <boost/mpl/assert.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/identity.hpp>
#include <boost/mpl/is_sequence.hpp>
#include <boost/mpl/set.hpp>
#include <boost/mpl/map.hpp>
#include <boost/mpl/has_key.hpp>
#include <boost/mpl/insert.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/back_inserter.hpp>
#include <boost/mpl/inserter.hpp>
#include <boost/mpl/fold.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/copy.hpp>
#include <boost/mpl/at.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/type_erasure/detail/get_placeholders.hpp>
#include <boost/type_erasure/detail/rebind_placeholders.hpp>
#include <boost/type_erasure/detail/normalize_deduced.hpp>
#include <boost/type_erasure/relaxed.hpp>
#include <boost/type_erasure/builtin.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace type_erasure {

template<class F>
struct deduced;

template<class T, class U>
struct same_type;

namespace detail {

struct substitution_map_tag {};

// a wrapper around an mpl::map that
// defaults to the identity map.
template<class M>
struct substitution_map
{
    typedef substitution_map_tag tag;
    typedef M map_type;
};

}
}

namespace mpl {

template<>
struct at_impl< ::pdalboost::type_erasure::detail::substitution_map_tag>
{
    template<class Seq, class Key>
    struct apply
    {
        typedef typename ::pdalboost::mpl::eval_if<
            ::pdalboost::mpl::has_key<typename Seq::map_type, Key>,
            ::pdalboost::mpl::at<typename Seq::map_type, Key>,
            ::pdalboost::mpl::identity<Key>
        >::type type;
    };
};

template<>
struct has_key_impl< ::pdalboost::type_erasure::detail::substitution_map_tag>
{
    template<class Seq, class Key>
    struct apply : pdalboost::mpl::true_
    {};
};

}

namespace type_erasure {
namespace detail {

// given a partial substitution map from same_type,
// resolves a placeholder as far as possible.
template<class M, class T>
struct resolve_same_type
{
    typedef typename ::pdalboost::mpl::eval_if< ::pdalboost::mpl::has_key<M, T>,
        ::pdalboost::type_erasure::detail::resolve_same_type<
            M,
            typename ::pdalboost::mpl::at<M, T>::type
        >,
        ::pdalboost::mpl::identity<T>
    >::type type;
};

// Given the arguments to same_type, determines
// which should be the key and which should be
// the value in the substitution map.
template<class T, class U>
struct select_pair
{
    BOOST_MPL_ASSERT((::pdalboost::is_same<T, U>));
    typedef void type;
};

template<class T, class U>
struct select_pair<T, ::pdalboost::type_erasure::deduced<U> >
{
    typedef ::pdalboost::mpl::pair< ::pdalboost::type_erasure::deduced<U>, T> type;
};

template<class T, class U>
struct select_pair< ::pdalboost::type_erasure::deduced<T>, U>
{
    typedef ::pdalboost::mpl::pair< ::pdalboost::type_erasure::deduced<T>, U> type;
};

template<class T, class U>
struct select_pair<
    ::pdalboost::type_erasure::deduced<T>,
    ::pdalboost::type_erasure::deduced<U>
>
{
    typedef ::pdalboost::mpl::pair<
        ::pdalboost::type_erasure::deduced<T>,
        ::pdalboost::type_erasure::deduced<U>
    > type;
};

// M is a map of placeholder substitutions
template<class M, class T>
struct normalize_placeholder
{
    typedef typename ::pdalboost::mpl::eval_if< ::pdalboost::mpl::has_key<M, T>,
        ::pdalboost::type_erasure::detail::normalize_placeholder<
            M,
            typename ::pdalboost::mpl::at<M, T>::type
        >,
        ::pdalboost::mpl::identity<T>
    >::type type;
};

template<class M, class T>
struct normalize_placeholder<M, ::pdalboost::type_erasure::deduced<T> >
{
    typedef typename ::pdalboost::mpl::eval_if< ::pdalboost::mpl::has_key<M, T>,
        ::pdalboost::type_erasure::detail::normalize_placeholder<
            M,
            typename ::pdalboost::mpl::at<M, T>::type
        >,
        ::pdalboost::type_erasure::detail::normalize_deduced<
            M,
            T
        >
    >::type type;
};

// Takes a mpl::map of placeholder substitutions and
// fully resolves it.  i.e.  a -> b, b -> c, becomes
// a -> c, b -> c.  Also resolves deduced placeholders
// whose arguments are all resolved.
template<class M>
struct create_placeholder_map
{
    typedef typename ::pdalboost::mpl::fold<
        M,
        ::pdalboost::mpl::map0<>,
        ::pdalboost::mpl::insert<
            ::pdalboost::mpl::_1,
            ::pdalboost::mpl::pair<
                ::pdalboost::mpl::first< ::pdalboost::mpl::_2>,
                ::pdalboost::type_erasure::detail::normalize_placeholder<M, ::pdalboost::mpl::second< ::pdalboost::mpl::_2> >
            >
        >
    >::type type;
};

template<class Bindings, class P, class Out, class Sub>
struct convert_deduced
{
    typedef typename ::pdalboost::type_erasure::detail::rebind_placeholders_in_argument<
        typename P::first,
        Bindings
    >::type result1;
    typedef typename ::pdalboost::mpl::at<Sub, result1>::type result;
    typedef typename ::pdalboost::mpl::eval_if<
        ::pdalboost::mpl::has_key<Out, typename P::second>,
        ::pdalboost::mpl::identity<Out>,
        ::pdalboost::mpl::insert<Out, ::pdalboost::mpl::pair<typename P::second, result> >
    >::type type;
    BOOST_MPL_ASSERT((pdalboost::is_same<typename ::pdalboost::mpl::at<type, typename P::second>::type, result>));
};

template<class Bindings, class M, class Sub>
struct convert_deductions
{
    typedef typename ::pdalboost::mpl::fold<
        M,
        Bindings,
        ::pdalboost::type_erasure::detail::convert_deduced<
            Bindings, ::pdalboost::mpl::_2, ::pdalboost::mpl::_1, Sub
        >
    >::type type;
};

template<class Bindings, class P, class Out>
struct add_deduced
{
    typedef typename ::pdalboost::type_erasure::detail::rebind_placeholders_in_argument<
        typename P::first,
        Bindings
    >::type result;
    typedef typename ::pdalboost::mpl::eval_if<
        ::pdalboost::mpl::has_key<Out, typename P::second>,
        ::pdalboost::mpl::identity<Out>,
        ::pdalboost::mpl::insert<Out, ::pdalboost::mpl::pair<typename P::second, result> >
    >::type type;
    BOOST_MPL_ASSERT((pdalboost::is_same<typename ::pdalboost::mpl::at<type, typename P::second>::type, result>));
};

template<class Bindings, class M>
struct add_deductions
{
    typedef typename ::pdalboost::mpl::fold<
        M,
        Bindings,
        ::pdalboost::type_erasure::detail::add_deduced<
            Bindings, ::pdalboost::mpl::_2, ::pdalboost::mpl::_1
        >
    >::type type;
};

// Fold Op for normalize_concept_impl
template<class Out, class T>
struct insert_concept
{
    typedef ::pdalboost::mpl::pair<
        typename ::pdalboost::mpl::insert<typename Out::first, T>::type,
        typename Out::second
    > type;
};

template<class Out, class T, class U>
struct insert_concept<Out, ::pdalboost::type_erasure::same_type<T, U> >
{
    typedef typename ::pdalboost::type_erasure::detail::resolve_same_type<
        typename Out::second,
        T
    >::type t1;
    typedef typename ::pdalboost::type_erasure::detail::resolve_same_type<
        typename Out::second,
        U
    >::type t2;
    typedef ::pdalboost::mpl::pair<
        typename Out::first,
        typename ::pdalboost::mpl::eval_if<
            ::pdalboost::is_same<t1, t2>,
            ::pdalboost::mpl::identity<typename Out::second>,
            ::pdalboost::mpl::insert<
                typename Out::second,
                typename ::pdalboost::type_erasure::detail::select_pair<
                    t1,
                    t2
                >::type
            >
        >::type
    > type;
};

// flattens a concept returning an mpl::pair
// - first is an MPL sequence containing the leaf concepts
// - second is an MPL map of the placeholder substitutions
//   used to resolve same_type.
template<class Concept, class Out = ::pdalboost::mpl::pair< ::pdalboost::mpl::set0<>, ::pdalboost::mpl::map0<> > >
struct normalize_concept_impl
{
    typedef typename ::pdalboost::mpl::eval_if< ::pdalboost::mpl::is_sequence<Concept>,
        ::pdalboost::mpl::fold<Concept, Out, normalize_concept_impl< ::pdalboost::mpl::_2, ::pdalboost::mpl::_1> >,
        ::pdalboost::type_erasure::detail::insert_concept<Out, Concept>
    >::type type;
};

struct append_typeinfo
{
    template<class Set, class T>
    struct apply
    {
        typedef typename ::pdalboost::mpl::insert<
            Set,
            ::pdalboost::type_erasure::typeid_<T>
        >::type type;
    };
};

// Seq should be a flattened MPL sequence of leaf concepts.
// adds typeid_<P> for every placeholder used.
template<class Seq>
struct add_typeinfo
{
    typedef typename ::pdalboost::mpl::fold<
        Seq,
        ::pdalboost::mpl::set0<>,
        ::pdalboost::type_erasure::detail::get_placeholders<
            ::pdalboost::mpl::_2,
            ::pdalboost::mpl::_1
        >
    >::type placeholders;
    typedef typename ::pdalboost::mpl::fold<
        placeholders,
        Seq,
        ::pdalboost::type_erasure::detail::append_typeinfo
    >::type type;
};

template<class Concept>
struct get_placeholder_normalization_map
{
    typedef typename ::pdalboost::type_erasure::detail::create_placeholder_map<
        typename normalize_concept_impl<Concept>::type::second
    >::type type;
};

// Flattens a Concept to an mpl::vector of primitive
// concepts.  Resolves same_type and deduced placeholders.
template<class Concept>
struct normalize_concept
{
    typedef typename normalize_concept_impl<Concept>::type impl;
    typedef typename ::pdalboost::type_erasure::detail::create_placeholder_map<
        typename impl::second
    >::type substitutions;
    typedef typename ::pdalboost::mpl::fold<
        typename impl::first,
        ::pdalboost::mpl::set0<>,
        ::pdalboost::mpl::insert<
            ::pdalboost::mpl::_1,
            ::pdalboost::type_erasure::detail::rebind_placeholders<
                ::pdalboost::mpl::_2,
                ::pdalboost::type_erasure::detail::substitution_map<substitutions>
            >
        >
    >::type basic;
    typedef typename ::pdalboost::mpl::eval_if<
        ::pdalboost::type_erasure::is_relaxed<Concept>,
        ::pdalboost::type_erasure::detail::add_typeinfo<basic>,
        ::pdalboost::mpl::identity<basic>
    >::type concept_set;
    typedef typename ::pdalboost::mpl::copy<
        concept_set,
        ::pdalboost::mpl::back_inserter< ::pdalboost::mpl::vector0<> >
    >::type type;
};

// Returns an MPL sequence containing all the concepts
// in Concept.  If Concept is considered as a DAG,
// the result will be sorted topologically.
template<
    class Concept,
    class Map = typename ::pdalboost::type_erasure::detail::create_placeholder_map<
            typename ::pdalboost::type_erasure::detail::normalize_concept_impl<
                Concept
            >::type::second
        >::type,
    class Out = ::pdalboost::mpl::set0<>
>
struct collect_concepts
{
    typedef typename ::pdalboost::type_erasure::detail::rebind_placeholders<
        Concept,
        ::pdalboost::type_erasure::detail::substitution_map<Map>
    >::type transformed;
    typedef typename ::pdalboost::mpl::eval_if<
        ::pdalboost::is_same<transformed, void>,
        ::pdalboost::mpl::identity<Out>,
        ::pdalboost::mpl::insert<
            Out,
            transformed
        >
    >::type type1;
    typedef typename ::pdalboost::mpl::eval_if< ::pdalboost::mpl::is_sequence<Concept>,
        ::pdalboost::mpl::fold<Concept, type1, collect_concepts< ::pdalboost::mpl::_2, Map, ::pdalboost::mpl::_1> >,
        ::pdalboost::mpl::identity<type1>
    >::type type;
};

}
}
}

#endif
