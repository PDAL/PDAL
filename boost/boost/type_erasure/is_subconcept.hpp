// Boost.TypeErasure library
//
// Copyright 2012 Steven Watanabe
//
// Distributed under the Boost Software License Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// $Id: is_subconcept.hpp 80883 2012-10-06 01:28:13Z steven_watanabe $

#ifndef BOOST_TYPE_ERASURE_IS_SUBCONCEPT_HPP_INCLUDED
#define BOOST_TYPE_ERASURE_IS_SUBCONCEPT_HPP_INCLUDED

#include <boost/mpl/bool.hpp>
#include <boost/mpl/not.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/end.hpp>
#include <boost/mpl/find_if.hpp>
#include <boost/mpl/has_key.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/type_erasure/detail/normalize.hpp>
#include <boost/type_erasure/detail/rebind_placeholders.hpp>
#include <boost/type_erasure/static_binding.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace type_erasure {
namespace detail {

template<class Sub, class Super, class PlaceholderMap>
struct is_subconcept_impl {
    typedef typename ::pdalboost::type_erasure::detail::normalize_concept<
        Super>::concept_set super_set;
    typedef typename ::pdalboost::type_erasure::detail::get_placeholder_normalization_map<
        Super
    >::type placeholder_subs_super;
    
    typedef typename ::pdalboost::type_erasure::detail::normalize_concept<
        Sub>::type normalized_sub;
    typedef typename ::pdalboost::type_erasure::detail::get_placeholder_normalization_map<
        Sub
    >::type placeholder_subs_sub;

    typedef typename ::pdalboost::mpl::eval_if< ::pdalboost::is_same<PlaceholderMap, void>,
        pdalboost::mpl::identity<void>,
        ::pdalboost::type_erasure::detail::convert_deductions<
            PlaceholderMap,
            placeholder_subs_sub,
            placeholder_subs_super
        >
    >::type bindings;

    typedef typename ::pdalboost::mpl::if_< ::pdalboost::is_same<PlaceholderMap, void>,
        ::pdalboost::mpl::_1,
        ::pdalboost::type_erasure::detail::rebind_placeholders<
            ::pdalboost::mpl::_1,
            bindings
        >
    >::type transform;

    typedef typename ::pdalboost::is_same<
        typename ::pdalboost::mpl::find_if<normalized_sub,
            ::pdalboost::mpl::not_<
                ::pdalboost::mpl::has_key<
                    super_set,
                    transform
                >
            >
        >::type,
        typename ::pdalboost::mpl::end<normalized_sub>::type
    >::type type;
};

}

/**
 * @ref is_subconcept is a boolean metafunction that determines whether
 * one concept is a sub-concept of another.
 *
 * \code
 * is_subconcept<incrementable<>, incrementable<> >             -> true
 * is_subconcept<incrementable<>, addable<> >                   -> false
 * is_subconcept<incrementable<_a>, forward_iterator<_iter>,
 *   mpl::map<mpl::pair<_a, _iter> > >                          -> true
 * \endcode
 *
 * \tparam Sub The sub concept
 * \tparam Super The super concept
 * \tparam PlaceholderMap (optional) An MPL map with keys for
 *   every non-deduced placeholder in Sub.  The
 *   associated value of each key is the corresponding placeholder
 *   in Super.  If @c PlaceholderMap is omitted, @c Super and @c Sub
 *   are presumed to use the same set of placeholders.
 */
template<class Sub, class Super, class PlaceholderMap = void>
struct is_subconcept : ::pdalboost::type_erasure::detail::is_subconcept_impl<Sub, Super, PlaceholderMap>::type {
};

#ifndef BOOST_TYPE_ERASURE_DOXYGEN
template<class Sub, class Super, class PlaceholderMap>
struct is_subconcept<Sub, Super, static_binding<PlaceholderMap> > :
    ::pdalboost::type_erasure::detail::is_subconcept_impl<Sub, Super, PlaceholderMap>::type
{};
#endif

}
}

#endif
