#ifndef BOOST_SERIALIZATION_SHARED_PTR_132_HPP
#define BOOST_SERIALIZATION_SHARED_PTR_132_HPP

// MS compatible compilers support #pragma once
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// shared_ptr.hpp: serialization for boost shared pointer

// (C) Copyright 2002 Robert Ramey - http://www.rrsd.com . 
// Use, modification and distribution is subject to the Boost Software
// License, Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

//  See http://www.boost.org for updates, documentation, and revision history.

// note: totally unadvised hack to gain access to private variables
// in shared_ptr and shared_count. Unfortunately its the only way to
// do this without changing shared_ptr and shared_count
// the best we can do is to detect a conflict here
#include <boost/config.hpp>

#include <list>
#include <cstddef> // NULL

#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/void_cast.hpp>

// mark base class as an (uncreatable) base class
#include <boost/serialization/detail/shared_ptr_132.hpp>

/////////////////////////////////////////////////////////////
// Maintain a couple of lists of loaded shared pointers of the old previous
// version (1.32)

namespace pdalboost_132 { 
namespace serialization {
namespace detail {

struct null_deleter {
    void operator()(void const *) const {}
};

} // namespace detail
} // namespace serialization
} // namespace pdalboost_132

/////////////////////////////////////////////////////////////
// sp_counted_base_impl serialization

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { 
namespace serialization {

template<class Archive, class P, class D>
inline void serialize(
    Archive & /* ar */,
    pdalboost_132::detail::sp_counted_base_impl<P, D> & /* t */,
    const unsigned int /*file_version*/
){
    // register the relationship between each derived class
    // its polymorphic base
    pdalboost::serialization::void_cast_register<
        pdalboost_132::detail::sp_counted_base_impl<P, D>,
        pdalboost_132::detail::sp_counted_base 
    >(
        static_cast<pdalboost_132::detail::sp_counted_base_impl<P, D> *>(NULL),
        static_cast<pdalboost_132::detail::sp_counted_base *>(NULL)
    );
}

template<class Archive, class P, class D>
inline void save_construct_data(
    Archive & ar,
    const 
    pdalboost_132::detail::sp_counted_base_impl<P, D> *t, 
    const BOOST_PFTO unsigned int /* file_version */
){
    // variables used for construction
    ar << pdalboost::serialization::make_nvp("ptr", t->ptr);
}

template<class Archive, class P, class D>
inline void load_construct_data(
    Archive & ar,
    pdalboost_132::detail::sp_counted_base_impl<P, D> * t, 
    const unsigned int /* file_version */
){
    P ptr_;
    ar >> pdalboost::serialization::make_nvp("ptr", ptr_);
    // ::new(t)pdalboost_132::detail::sp_counted_base_impl<P, D>(ptr_,  D()); 
    // placement
    // note: the original ::new... above is replaced by the one here.  This one
    // creates all new objects with a null_deleter so that after the archive
    // is finished loading and the shared_ptrs are destroyed - the underlying
    // raw pointers are NOT deleted.  This is necessary as they are used by the 
    // new system as well.
    ::new(t)pdalboost_132::detail::sp_counted_base_impl<
        P, 
        pdalboost_132::serialization::detail::null_deleter
    >(
        ptr_,  pdalboost_132::serialization::detail::null_deleter()
    ); // placement new
    // compensate for that fact that a new shared count always is 
    // initialized with one. the add_ref_copy below will increment it
    // every time its serialized so without this adjustment
    // the use and weak counts will be off by one.
    t->use_count_ = 0;
}

} // serialization
} // namespace pdalboost

/////////////////////////////////////////////////////////////
// shared_count serialization

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { 
namespace serialization {

template<class Archive>
inline void save(
    Archive & ar,
    const pdalboost_132::detail::shared_count &t,
    const unsigned int /* file_version */
){
    ar << pdalboost::serialization::make_nvp("pi", t.pi_);
}

template<class Archive>
inline void load(
    Archive & ar,
    pdalboost_132::detail::shared_count &t,
    const unsigned int /* file_version */
){
    ar >> pdalboost::serialization::make_nvp("pi", t.pi_);
    if(NULL != t.pi_)
        t.pi_->add_ref_copy();
}

} // serialization
} // namespace pdalboost

BOOST_SERIALIZATION_SPLIT_FREE(pdalboost_132::detail::shared_count)

/////////////////////////////////////////////////////////////
// implement serialization for shared_ptr< T >

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { 
namespace serialization {

template<class Archive, class T>
inline void save(
    Archive & ar,
    const pdalboost_132::shared_ptr< T > &t,
    const unsigned int /* file_version */
){
    // only the raw pointer has to be saved
    // the ref count is maintained automatically as shared pointers are loaded
    ar.register_type(static_cast<
        pdalboost_132::detail::sp_counted_base_impl<T *, pdalboost::checked_deleter< T > > *
    >(NULL));
    ar << pdalboost::serialization::make_nvp("px", t.px);
    ar << pdalboost::serialization::make_nvp("pn", t.pn);
}

template<class Archive, class T>
inline void load(
    Archive & ar,
    pdalboost_132::shared_ptr< T > &t,
    const unsigned int /* file_version */
){
    // only the raw pointer has to be saved
    // the ref count is maintained automatically as shared pointers are loaded
    ar.register_type(static_cast<
        pdalboost_132::detail::sp_counted_base_impl<T *, pdalboost::checked_deleter< T > > *
    >(NULL));
    ar >> pdalboost::serialization::make_nvp("px", t.px);
    ar >> pdalboost::serialization::make_nvp("pn", t.pn);
}

template<class Archive, class T>
inline void serialize(
    Archive & ar,
    pdalboost_132::shared_ptr< T > &t,
    const unsigned int file_version
){
    // correct shared_ptr serialization depends upon object tracking
    // being used.
    BOOST_STATIC_ASSERT(
        pdalboost::serialization::tracking_level< T >::value
        != pdalboost::serialization::track_never
    );
    pdalboost::serialization::split_free(ar, t, file_version);
}

} // serialization
} // namespace pdalboost

// note: change below uses null_deleter 
// This macro is used to export GUIDS for shared pointers to allow
// the serialization system to export them properly. David Tonge
#define BOOST_SHARED_POINTER_EXPORT_GUID(T, K)                     \
    typedef pdalboost_132::detail::sp_counted_base_impl<               \
        T *,                                                       \
        pdalboost::checked_deleter< T >                                \
    > __shared_ptr_ ## T;                                          \
    BOOST_CLASS_EXPORT_GUID(__shared_ptr_ ## T, "__shared_ptr_" K) \
    BOOST_CLASS_EXPORT_GUID(T, K)                                  \
    /**/

#define BOOST_SHARED_POINTER_EXPORT(T)                             \
    BOOST_SHARED_POINTER_EXPORT_GUID(                              \
        T,                                                         \
        BOOST_PP_STRINGIZE(T)                                      \
    )                                                              \
    /**/

#endif // BOOST_SERIALIZATION_SHARED_PTR_132_HPP
