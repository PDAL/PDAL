/*
 * Copyright (c) 2012 Glen Joseph Fernandes
 * glenfe at live dot com
 *
 * Distributed under the Boost Software License,
 * Version 1.0. (See accompanying file LICENSE_1_0.txt
 * or copy at http://boost.org/LICENSE_1_0.txt)
 */
#ifndef BOOST_SMART_PTR_MAKE_SHARED_ARRAY_HPP
#define BOOST_SMART_PTR_MAKE_SHARED_ARRAY_HPP

#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/detail/array_deleter.hpp>
#include <boost/smart_ptr/detail/array_traits.hpp>
#include <boost/smart_ptr/detail/make_array_helper.hpp>
#include <boost/smart_ptr/detail/sp_if_array.hpp>
#if !defined(BOOST_NO_CXX11_HDR_INITIALIZER_LIST)
#include <initializer_list>
#endif

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
    template<typename T>
    inline typename pdalboost::detail::sp_if_array<T>::type
    make_shared(std::size_t size) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        T1* p1 = 0;
        T2* p2 = 0;
        std::size_t n1 = size * pdalboost::detail::array_total<T1>::size;
        pdalboost::detail::make_array_helper<T2[]> a1(n1, &p2);
        pdalboost::detail::array_deleter<T2[]> d1(n1);
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[]>* D2;
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->init(p2);
        return pdalboost::shared_ptr<T>(s1, p1);
    }
#if !defined(BOOST_NO_CXX11_VARIADIC_TEMPLATES) && !defined(BOOST_NO_CXX11_RVALUE_REFERENCES)
    template<typename T, typename... Args>
    inline typename pdalboost::detail::sp_if_array<T>::type
    make_shared(std::size_t size, Args&&... args) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        T1* p1 = 0;
        T2* p2 = 0;
        std::size_t n1 = size * pdalboost::detail::array_total<T1>::size;
        pdalboost::detail::make_array_helper<T2[]> a1(n1, &p2);
        pdalboost::detail::array_deleter<T2[]> d1(n1);
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[]>* D2;
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->init(p2, pdalboost::detail::sp_forward<Args>(args)...);
        return pdalboost::shared_ptr<T>(s1, p1);
    }
    template<typename T, typename... Args>
    inline typename pdalboost::detail::sp_if_size_array<T>::type
    make_shared(Args&&... args) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        enum {
            N = pdalboost::detail::array_total<T>::size
        };
        T1* p1 = 0;
        T2* p2 = 0;
        pdalboost::detail::make_array_helper<T2[N]> a1(&p2);
        pdalboost::detail::array_deleter<T2[N]> d1;
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[N]>* D2;
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->init(p2, pdalboost::detail::sp_forward<Args>(args)...);
        return pdalboost::shared_ptr<T>(s1, p1);
    }
#endif
#if !defined(BOOST_NO_CXX11_UNIFIED_INITIALIZATION_SYNTAX)
    template<typename T>
    inline typename pdalboost::detail::sp_if_size_array<T>::type
    make_shared(const T& list) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        typedef const T2 T3;
        enum {
            N = pdalboost::detail::array_total<T>::size
        };
        T1* p1 = 0;
        T2* p2 = 0;
        T3* p3 = 0;
        pdalboost::detail::make_array_helper<T2[N]> a1(&p2);
        pdalboost::detail::array_deleter<T2[N]> d1;
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[N]>* D2;
        p3 = reinterpret_cast<T3*>(list);
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->init_list(p2, p3);
        return pdalboost::shared_ptr<T>(s1, p1);
    }
    template<typename T>
    inline typename pdalboost::detail::sp_if_array<T>::type
    make_shared(std::size_t size,
        const typename pdalboost::detail::array_inner<T>::type& list) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        typedef const T2 T3;
        enum {
            M = pdalboost::detail::array_total<T1>::size
        };
        T1* p1 = 0;
        T2* p2 = 0;
        T3* p3 = 0;
        std::size_t n1 = M * size;
        pdalboost::detail::make_array_helper<T2[]> a1(n1, &p2);
        pdalboost::detail::array_deleter<T2[]> d1(n1);
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[]>* D2;
        p3 = reinterpret_cast<T3*>(list);
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->template init_list<M>(p2, p3);
        return pdalboost::shared_ptr<T>(s1, p1);
    }
    template<typename T>
    inline typename pdalboost::detail::sp_if_size_array<T>::type
    make_shared(const typename pdalboost::detail::array_inner<T>::type& list) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        typedef const T2 T3;
        enum {
            M = pdalboost::detail::array_total<T1>::size,
            N = pdalboost::detail::array_total<T>::size
        };
        T1* p1 = 0;
        T2* p2 = 0;
        T3* p3 = 0;
        pdalboost::detail::make_array_helper<T2[N]> a1(&p2);
        pdalboost::detail::array_deleter<T2[N]> d1;
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[N]>* D2;
        p3 = reinterpret_cast<T3*>(list);
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->template init_list<M>(p2, p3);
        return pdalboost::shared_ptr<T>(s1, p1);
    }
#if !defined(BOOST_NO_CXX11_HDR_INITIALIZER_LIST)
    template<typename T>
    inline typename pdalboost::detail::sp_if_array<T>::type
    make_shared(std::initializer_list<typename pdalboost::detail::array_inner<T>::type> list) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        typedef const T2 T3;
        T1* p1 = 0;
        T2* p2 = 0;
        T3* p3 = 0;
        std::size_t n1 = list.size() * pdalboost::detail::array_total<T1>::size;
        pdalboost::detail::make_array_helper<T2[]> a1(n1, &p2);
        pdalboost::detail::array_deleter<T2[]> d1(n1);
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[]>* D2;
        p3 = reinterpret_cast<T3*>(list.begin());
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->init_list(p2, p3);
        return pdalboost::shared_ptr<T>(s1, p1);
    }
#endif
#if !defined(BOOST_NO_CXX11_RVALUE_REFERENCES)
    template<typename T>
    inline typename pdalboost::detail::sp_if_array<T>::type
    make_shared(std::size_t size,
        typename pdalboost::detail::array_base<T>::type&& value) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        T1* p1 = 0;
        T2* p2 = 0;
        std::size_t n1 = size * pdalboost::detail::array_total<T1>::size;
        pdalboost::detail::make_array_helper<T2[]> a1(n1, &p2);
        pdalboost::detail::array_deleter<T2[]> d1(n1);
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[]>* D2;
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->init(p2, pdalboost::detail::sp_forward<T2>(value));
        return pdalboost::shared_ptr<T>(s1, p1);
    }
    template<typename T>
    inline typename pdalboost::detail::sp_if_size_array<T>::type
    make_shared(typename pdalboost::detail::array_base<T>::type&& value) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        enum {
            N = pdalboost::detail::array_total<T>::size
        };
        T1* p1 = 0;
        T2* p2 = 0;
        pdalboost::detail::make_array_helper<T2[N]> a1(&p2);
        pdalboost::detail::array_deleter<T2[N]> d1;
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[N]>* D2;
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->init(p2, pdalboost::detail::sp_forward<T2>(value));
        return pdalboost::shared_ptr<T>(s1, p1);
    }
#endif
#endif
    template<typename T>
    inline typename pdalboost::detail::sp_if_array<T>::type
    make_shared_noinit(std::size_t size) {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        T1* p1 = 0;
        T2* p2 = 0;
        std::size_t n1 = size * pdalboost::detail::array_total<T1>::size;
        pdalboost::detail::make_array_helper<T2[]> a1(n1, &p2);
        pdalboost::detail::array_deleter<T2[]> d1(n1);
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[]>* D2;
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->noinit(p2);
        return pdalboost::shared_ptr<T>(s1, p1);
    }
    template<typename T>
    inline typename pdalboost::detail::sp_if_size_array<T>::type
    make_shared_noinit() {
        typedef typename pdalboost::detail::array_inner<T>::type T1;
        typedef typename pdalboost::detail::array_base<T1>::type T2;
        enum {
            N = pdalboost::detail::array_total<T>::size
        };
        T1* p1 = 0;
        T2* p2 = 0;
        pdalboost::detail::make_array_helper<T2[N]> a1(&p2);
        pdalboost::detail::array_deleter<T2[N]> d1;
        pdalboost::shared_ptr<T> s1(p1, d1, a1);
        typedef pdalboost::detail::array_deleter<T2[N]>* D2;
        p1 = reinterpret_cast<T1*>(p2);
        D2 d2 = static_cast<D2>(s1._internal_get_untyped_deleter());
        d2->noinit(p2);
        return pdalboost::shared_ptr<T>(s1, p1);
    }
}

#endif
