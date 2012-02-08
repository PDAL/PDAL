#ifndef BOOST_SMART_PTR_MAKE_SHARED_HPP_INCLUDED
#define BOOST_SMART_PTR_MAKE_SHARED_HPP_INCLUDED

//  make_shared.hpp
//
//  Copyright (c) 2007, 2008 Peter Dimov
//
//  Distributed under the Boost Software License, Version 1.0.
//  See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
//
//  See http://www.boost.org/libs/smart_ptr/make_shared.html
//  for documentation.

#include <boost/config.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/type_traits/type_with_alignment.hpp>
#include <boost/type_traits/alignment_of.hpp>
#include <cstddef>
#include <new>

namespace pdalboost{} namespace boost = pdalboost; namespace pdalboost{

namespace detail
{

template< std::size_t N, std::size_t A > struct sp_aligned_storage
{
    union type
    {
        char data_[ N ];
        typename pdalboost::type_with_alignment< A >::type align_;
    };
};

template< class T > class sp_ms_deleter
{
private:

    typedef typename sp_aligned_storage< sizeof( T ), ::pdalboost::alignment_of< T >::value >::type storage_type;

    bool initialized_;
    storage_type storage_;

private:

    void destroy()
    {
        if( initialized_ )
        {
#if defined( __GNUC__ )

            // fixes incorrect aliasing warning
            T * p = reinterpret_cast< T* >( storage_.data_ );
            p->~T();

#else

            reinterpret_cast< T* >( storage_.data_ )->~T();

#endif

            initialized_ = false;
        }
    }

public:

    sp_ms_deleter(): initialized_( false )
    {
    }

    // optimization: do not copy storage_
    sp_ms_deleter( sp_ms_deleter const & ): initialized_( false )
    {
    }

    ~sp_ms_deleter()
    {
        destroy();
    }

    void operator()( T * )
    {
        destroy();
    }

    void * address()
    {
        return storage_.data_;
    }

    void set_initialized()
    {
        initialized_ = true;
    }
};

#if defined( BOOST_HAS_RVALUE_REFS )

template< class T > T&& sp_forward( T & t )
{
    return static_cast< T&& >( t );
}

#endif

} // namespace detail

#if !defined( BOOST_NO_FUNCTION_TEMPLATE_ORDERING )
# define BOOST_SP_MSD( T ) pdalboost::detail::sp_inplace_tag< pdalboost::detail::sp_ms_deleter< T > >()
#else
# define BOOST_SP_MSD( T ) pdalboost::detail::sp_ms_deleter< T >()
#endif

// Zero-argument versions
//
// Used even when variadic templates are available because of the new T() vs new T issue

template< class T > pdalboost::shared_ptr< T > make_shared()
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T();
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A > pdalboost::shared_ptr< T > allocate_shared( A const & a )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T();
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

#if defined( BOOST_HAS_VARIADIC_TMPL ) && defined( BOOST_HAS_RVALUE_REFS )

// Variadic templates, rvalue reference

template< class T, class Arg1, class... Args > pdalboost::shared_ptr< T > make_shared( Arg1 && arg1, Args && ... args )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( pdalboost::detail::sp_forward<Arg1>( arg1 ), pdalboost::detail::sp_forward<Args>( args )... );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class Arg1, class... Args > pdalboost::shared_ptr< T > allocate_shared( A const & a, Arg1 && arg1, Args && ... args )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( pdalboost::detail::sp_forward<Arg1>( arg1 ), pdalboost::detail::sp_forward<Args>( args )... );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

#elif defined( BOOST_HAS_RVALUE_REFS )

// For example MSVC 10.0

template< class T, class A1 >
pdalboost::shared_ptr< T > make_shared( A1 && a1 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T(
        pdalboost::detail::sp_forward<A1>( a1 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 && a1 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( 
        pdalboost::detail::sp_forward<A1>( a1 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2 >
pdalboost::shared_ptr< T > make_shared( A1 && a1, A2 && a2 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T(
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 && a1, A2 && a2 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( 
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3 >
pdalboost::shared_ptr< T > make_shared( A1 && a1, A2 && a2, A3 && a3 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T(
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 && a1, A2 && a2, A3 && a3 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( 
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4 >
pdalboost::shared_ptr< T > make_shared( A1 && a1, A2 && a2, A3 && a3, A4 && a4 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T(
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 && a1, A2 && a2, A3 && a3, A4 && a4 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( 
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5 >
pdalboost::shared_ptr< T > make_shared( A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T(
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( 
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5, class A6 >
pdalboost::shared_ptr< T > make_shared( A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5, A6 && a6 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T(
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 ), 
        pdalboost::detail::sp_forward<A6>( a6 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5, class A6 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5, A6 && a6 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( 
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 ), 
        pdalboost::detail::sp_forward<A6>( a6 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5, class A6, class A7 >
pdalboost::shared_ptr< T > make_shared( A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5, A6 && a6, A7 && a7 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T(
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 ), 
        pdalboost::detail::sp_forward<A6>( a6 ), 
        pdalboost::detail::sp_forward<A7>( a7 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5, class A6, class A7 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5, A6 && a6, A7 && a7 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( 
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 ), 
        pdalboost::detail::sp_forward<A6>( a6 ), 
        pdalboost::detail::sp_forward<A7>( a7 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8 >
pdalboost::shared_ptr< T > make_shared( A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5, A6 && a6, A7 && a7, A8 && a8 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T(
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 ), 
        pdalboost::detail::sp_forward<A6>( a6 ), 
        pdalboost::detail::sp_forward<A7>( a7 ), 
        pdalboost::detail::sp_forward<A8>( a8 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5, A6 && a6, A7 && a7, A8 && a8 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( 
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 ), 
        pdalboost::detail::sp_forward<A6>( a6 ), 
        pdalboost::detail::sp_forward<A7>( a7 ), 
        pdalboost::detail::sp_forward<A8>( a8 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8, class A9 >
pdalboost::shared_ptr< T > make_shared( A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5, A6 && a6, A7 && a7, A8 && a8, A9 && a9 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T(
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 ), 
        pdalboost::detail::sp_forward<A6>( a6 ), 
        pdalboost::detail::sp_forward<A7>( a7 ), 
        pdalboost::detail::sp_forward<A8>( a8 ), 
        pdalboost::detail::sp_forward<A9>( a9 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8, class A9 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 && a1, A2 && a2, A3 && a3, A4 && a4, A5 && a5, A6 && a6, A7 && a7, A8 && a8, A9 && a9 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( 
        pdalboost::detail::sp_forward<A1>( a1 ), 
        pdalboost::detail::sp_forward<A2>( a2 ), 
        pdalboost::detail::sp_forward<A3>( a3 ), 
        pdalboost::detail::sp_forward<A4>( a4 ), 
        pdalboost::detail::sp_forward<A5>( a5 ), 
        pdalboost::detail::sp_forward<A6>( a6 ), 
        pdalboost::detail::sp_forward<A7>( a7 ), 
        pdalboost::detail::sp_forward<A8>( a8 ), 
        pdalboost::detail::sp_forward<A9>( a9 )
        );

    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

#else

// C++03 version

template< class T, class A1 >
pdalboost::shared_ptr< T > make_shared( A1 const & a1 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 const & a1 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2 >
pdalboost::shared_ptr< T > make_shared( A1 const & a1, A2 const & a2 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 const & a1, A2 const & a2 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3 >
pdalboost::shared_ptr< T > make_shared( A1 const & a1, A2 const & a2, A3 const & a3 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 const & a1, A2 const & a2, A3 const & a3 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4 >
pdalboost::shared_ptr< T > make_shared( A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5 >
pdalboost::shared_ptr< T > make_shared( A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5, class A6 >
pdalboost::shared_ptr< T > make_shared( A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5, A6 const & a6 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5, a6 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5, class A6 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5, A6 const & a6 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5, a6 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5, class A6, class A7 >
pdalboost::shared_ptr< T > make_shared( A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5, A6 const & a6, A7 const & a7 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5, a6, a7 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5, class A6, class A7 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5, A6 const & a6, A7 const & a7 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5, a6, a7 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8 >
pdalboost::shared_ptr< T > make_shared( A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5, A6 const & a6, A7 const & a7, A8 const & a8 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5, a6, a7, a8 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5, A6 const & a6, A7 const & a7, A8 const & a8 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5, a6, a7, a8 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8, class A9 >
pdalboost::shared_ptr< T > make_shared( A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5, A6 const & a6, A7 const & a7, A8 const & a8, A9 const & a9 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ) );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5, a6, a7, a8, a9 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

template< class T, class A, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8, class A9 >
pdalboost::shared_ptr< T > allocate_shared( A const & a, A1 const & a1, A2 const & a2, A3 const & a3, A4 const & a4, A5 const & a5, A6 const & a6, A7 const & a7, A8 const & a8, A9 const & a9 )
{
    pdalboost::shared_ptr< T > pt( static_cast< T* >( 0 ), BOOST_SP_MSD( T ), a );

    pdalboost::detail::sp_ms_deleter< T > * pd = pdalboost::get_deleter< pdalboost::detail::sp_ms_deleter< T > >( pt );

    void * pv = pd->address();

    ::new( pv ) T( a1, a2, a3, a4, a5, a6, a7, a8, a9 );
    pd->set_initialized();

    T * pt2 = static_cast< T* >( pv );

    pdalboost::detail::sp_enable_shared_from_this( &pt, pt2, pt2 );
    return pdalboost::shared_ptr< T >( pt, pt2 );
}

#endif

#undef BOOST_SP_MSD

} // namespace pdalboost

#endif // #ifndef BOOST_SMART_PTR_MAKE_SHARED_HPP_INCLUDED
