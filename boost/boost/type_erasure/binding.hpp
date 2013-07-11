// Boost.TypeErasure library
//
// Copyright 2011 Steven Watanabe
//
// Distributed under the Boost Software License Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// $Id: binding.hpp 83251 2013-03-02 19:23:44Z steven_watanabe $

#ifndef BOOST_TYPE_ERASURE_BINDING_HPP_INCLUDED
#define BOOST_TYPE_ERASURE_BINDING_HPP_INCLUDED

#include <boost/config.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/find_if.hpp>
#include <boost/mpl/and.hpp>
#include <boost/mpl/not.hpp>
#include <boost/mpl/end.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/mpl/pair.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/type_erasure/static_binding.hpp>
#include <boost/type_erasure/is_subconcept.hpp>
#include <boost/type_erasure/detail/adapt_to_vtable.hpp>
#include <boost/type_erasure/detail/null.hpp>
#include <boost/type_erasure/detail/rebind_placeholders.hpp>
#include <boost/type_erasure/detail/vtable.hpp>
#include <boost/type_erasure/detail/normalize.hpp>
#include <boost/type_erasure/detail/instantiate.hpp>
#include <boost/type_erasure/detail/check_map.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace type_erasure {

namespace detail {

template<class Source, class Dest, class Map>
struct can_optimize_conversion : ::pdalboost::mpl::and_<
    ::pdalboost::is_same<Source, Dest>,
        ::pdalboost::is_same<
            typename ::pdalboost::mpl::find_if<
                Map,
                ::pdalboost::mpl::not_<
                    ::pdalboost::is_same<
                        ::pdalboost::mpl::first< ::pdalboost::mpl::_1>,
                        ::pdalboost::mpl::second< ::pdalboost::mpl::_1>
                    >
                >
            >::type,
            typename ::pdalboost::mpl::end<Map>::type
        >
    >::type
{};

}

/**
 * Stores the binding of a @c Concept to a set of actual types.
 * @c Concept is interpreted in the same way as with @ref any.
 */
template<class Concept>
class binding
{
    typedef typename ::pdalboost::type_erasure::detail::normalize_concept<
        Concept>::type normalized;
    typedef typename ::pdalboost::mpl::transform<normalized,
        ::pdalboost::type_erasure::detail::maybe_adapt_to_vtable< ::pdalboost::mpl::_1>
    >::type actual_concept;
    typedef typename ::pdalboost::type_erasure::detail::make_vtable<
        actual_concept>::type table_type;
    typedef typename ::pdalboost::type_erasure::detail::get_placeholder_normalization_map<
        Concept
    >::type placeholder_subs;
public:

    /**
     * \pre @ref relaxed must be in @c Concept.
     *
     * \throws Nothing.
     */
    binding() { BOOST_MPL_ASSERT((::pdalboost::type_erasure::is_relaxed<Concept>)); }
    
    /**
     * \pre @c Map must be an MPL map with an entry for each placeholder
     *      referred to by @c Concept.
     *
     * \throws Nothing.
     */
    template<class Map>
    explicit binding(const Map&)
      : impl((
            BOOST_TYPE_ERASURE_INSTANTIATE(Concept, Map),
            static_binding<Map>()
        ))
    {}
    
    /**
     * \pre @c Map must be an MPL map with an entry for each placeholder
     *      referred to by @c Concept.
     *
     * \throws Nothing.
     */
    template<class Map>
    binding(const static_binding<Map>&)
      : impl((
            BOOST_TYPE_ERASURE_INSTANTIATE(Concept, Map),
            static_binding<Map>()
        ))
    {}

    /**
     * Converts from another set of bindings.
     *
     * \pre Map must be an MPL map with an entry for each placeholder
     *      referred to by @c Concept.  The mapped type should be the
     *      corresponding placeholder in Concept2.
     *
     * \throws std::bad_alloc
     */
    template<class Concept2, class Map>
    binding(const binding<Concept2>& other, const Map&
#ifndef BOOST_TYPE_ERASURE_DOXYGEN
        , typename ::pdalboost::enable_if<
            ::pdalboost::mpl::and_<
                ::pdalboost::type_erasure::detail::check_map<Concept, Map>,
                ::pdalboost::type_erasure::is_subconcept<Concept, Concept2, Map>
            >
        >::type* = 0
#endif
        )
      : impl(
            other,
            static_binding<Map>(),
            ::pdalboost::type_erasure::detail::can_optimize_conversion<Concept2, Concept, Map>()
        )
    {}

    /**
     * Converts from another set of bindings.
     *
     * \pre Map must be an MPL map with an entry for each placeholder
     *      referred to by @c Concept.  The mapped type should be the
     *      corresponding placeholder in Concept2.
     *
     * \throws std::bad_alloc
     */
    template<class Concept2, class Map>
    binding(const binding<Concept2>& other, const static_binding<Map>&
#ifndef BOOST_TYPE_ERASURE_DOXYGEN
        , typename ::pdalboost::enable_if<
            ::pdalboost::mpl::and_<
                ::pdalboost::type_erasure::detail::check_map<Concept, Map>,
                ::pdalboost::type_erasure::is_subconcept<Concept, Concept2, Map>
            >
        >::type* = 0
#endif
        )
      : impl(
            other,
            static_binding<Map>(),
            ::pdalboost::type_erasure::detail::can_optimize_conversion<Concept2, Concept, Map>()
        )
    {}

    /**
     * \return true iff the sets of types that the placeholders
     *         bind to are the same for both arguments.
     *
     * \throws Nothing.
     */
    friend bool operator==(const binding& lhs, const binding& rhs)
    { return *lhs.impl.table == *rhs.impl.table; }
    
    /**
     * \return true iff the arguments do not map to identical
     *         sets of types.
     *
     * \throws Nothing.
     */
    friend bool operator!=(const binding& lhs, const binding& rhs)
    { return !(lhs == rhs); }

    /** INTERNAL ONLY */
    template<class T>
    typename T::type find() const { return impl.table->lookup((T*)0); }
private:
    template<class C2>
    friend class binding;
    /** INTERNAL ONLY */
    struct impl_type
    {
        impl_type() {
            table = &::pdalboost::type_erasure::detail::make_vtable_init<
                typename ::pdalboost::mpl::transform<
                    actual_concept,
                    ::pdalboost::type_erasure::detail::get_null_vtable_entry<
                        ::pdalboost::mpl::_1
                    >
                >::type,
                table_type
            >::type::value;
        }
        template<class Map>
        impl_type(const static_binding<Map>&)
        {
            table = &::pdalboost::type_erasure::detail::make_vtable_init<
                typename ::pdalboost::mpl::transform<
                    actual_concept,
                    ::pdalboost::type_erasure::detail::rebind_placeholders<
                        ::pdalboost::mpl::_1,
                        typename ::pdalboost::type_erasure::detail::add_deductions<
                            Map,
                            placeholder_subs
                        >::type
                    >
                >::type,
                table_type
            >::type::value;
        }
        template<class Concept2, class Map>
        impl_type(const binding<Concept2>& other, const static_binding<Map>&, pdalboost::mpl::false_)
          : manager(new table_type)
        {
            manager->template convert_from<
                typename ::pdalboost::type_erasure::detail::convert_deductions<
                    Map,
                    placeholder_subs,
                    typename binding<Concept2>::placeholder_subs
                >::type
            >(*other.impl.table);
            table = manager.get();
        }
        template<class Concept2, class Map>
        impl_type(const binding<Concept2>& other, const static_binding<Map>&, pdalboost::mpl::true_)
          : table(other.impl.table),
            manager(other.impl.manager)
        {}
        const table_type* table;
        ::pdalboost::shared_ptr<table_type> manager;
    } impl;
};

}
}

#endif
