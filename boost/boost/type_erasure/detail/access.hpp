// Boost.TypeErasure library
//
// Copyright 2011 Steven Watanabe
//
// Distributed under the Boost Software License Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// $Id: access.hpp 83321 2013-03-05 21:27:18Z steven_watanabe $

#ifndef BOOST_TYPE_ERASURE_DETAIL_ACCESS_HPP_INCLUDED
#define BOOST_TYPE_ERASURE_DETAIL_ACCESS_HPP_INCLUDED

#include <boost/type_erasure/detail/storage.hpp>
#include <boost/type_erasure/detail/any_base.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace type_erasure {

template<class Concept, class T>
class any;

template<class Concept, class T>
class param;

namespace detail {

struct access
{
    template<class Derived>
    static const typename Derived::table_type&
    table(const ::pdalboost::type_erasure::any_base<Derived>& arg)
    {
        return static_cast<const Derived&>(arg).table;
    }
    template<class Concept, class T>
    static const typename any<Concept, T>::table_type&
    table(const ::pdalboost::type_erasure::param<Concept, T>& arg)
    {
        return arg._impl.table;
    }
    template<class Derived>
    static ::pdalboost::type_erasure::detail::storage&
    data(::pdalboost::type_erasure::any_base<Derived>& arg)
    {
        return static_cast<Derived&>(arg).data;
    }
    template<class Concept, class T>
    static const ::pdalboost::type_erasure::detail::storage&
    data(::pdalboost::type_erasure::any_base< ::pdalboost::type_erasure::any<Concept, const T&> >& arg)
    {
        return static_cast< ::pdalboost::type_erasure::any<Concept, const T&>&>(arg).data;
    }
    template<class Derived>
    static const ::pdalboost::type_erasure::detail::storage&
    data(const ::pdalboost::type_erasure::any_base<Derived>& arg)
    {
        return static_cast<const Derived&>(arg).data;
    }
    template<class Concept, class T>
    static ::pdalboost::type_erasure::detail::storage&
    data(const ::pdalboost::type_erasure::any_base< ::pdalboost::type_erasure::any<Concept, T&> >& arg)
    {
        return const_cast< ::pdalboost::type_erasure::detail::storage&>(static_cast< const ::pdalboost::type_erasure::any<Concept, T&>&>(arg).data);
    }
    template<class Concept, class T>
    static const ::pdalboost::type_erasure::detail::storage&
    data(const ::pdalboost::type_erasure::any_base< ::pdalboost::type_erasure::any<Concept, const T&> >& arg)
    {
        return static_cast<const ::pdalboost::type_erasure::any<Concept, const T&>&>(arg).data;
    }
    template<class Concept, class T>
    static ::pdalboost::type_erasure::detail::storage&
    data(::pdalboost::type_erasure::param<Concept, T>& arg)
    {
        return arg._impl.data;
    }
    template<class Concept, class T>
    static const ::pdalboost::type_erasure::detail::storage&
    data(::pdalboost::type_erasure::param<Concept, const T&>& arg)
    {
        return arg._impl.data;
    }
    template<class Concept, class T>
    static const ::pdalboost::type_erasure::detail::storage&
    data(const ::pdalboost::type_erasure::param<Concept, T>& arg)
    {
        return arg._impl.data;
    }

#ifndef BOOST_NO_CXX11_RVALUE_REFERENCES

    template<class Derived>
    static ::pdalboost::type_erasure::detail::storage&&
    data(::pdalboost::type_erasure::any_base<Derived>&& arg)
    {
        return std::move(static_cast<Derived&>(arg).data);
    }
    template<class Concept, class T>
    static ::pdalboost::type_erasure::detail::storage&&
    data(::pdalboost::type_erasure::any_base< ::pdalboost::type_erasure::any<Concept, T&&> >& arg)
    {
        return std::move(static_cast< ::pdalboost::type_erasure::any<Concept, T&&>&>(arg).data);
    }
    template<class Concept, class T>
    static ::pdalboost::type_erasure::detail::storage&&
    data(const ::pdalboost::type_erasure::any_base< ::pdalboost::type_erasure::any<Concept, T&&> >& arg)
    {
        return std::move(const_cast< ::pdalboost::type_erasure::detail::storage&>(static_cast< const ::pdalboost::type_erasure::any<Concept, T&&>&>(arg).data));
    }

    template<class Concept, class T>
    static ::pdalboost::type_erasure::detail::storage&&
    data(::pdalboost::type_erasure::param<Concept, T>&& arg)
    {
        return std::move(arg._impl.data);
    }
    template<class Concept, class T>
    static ::pdalboost::type_erasure::detail::storage&&
    data(::pdalboost::type_erasure::param<Concept, T&&>& arg)
    {
        return std::move(arg._impl.data);
    }
    template<class Concept, class T>
    static ::pdalboost::type_erasure::detail::storage&&
    data(const ::pdalboost::type_erasure::param<Concept, T&&>& arg)
    {
        return std::move(const_cast< ::pdalboost::type_erasure::detail::storage&>(arg._impl.data));
    }

#endif

};

}
}
}

#endif
