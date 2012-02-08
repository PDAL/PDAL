/*==============================================================================
    Copyright (c) 2001-2010 Joel de Guzman
    Copyright (c) 2010 Thomas Heller

    Distributed under the Boost Software License, Version 1.0. (See accompanying
    file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
==============================================================================*/
namespace pdalboost{} namespace boost = pdalboost; namespace pdalboost{ namespace pdalboostphoenix { namespace tag { struct mem_fun_ptr {}; template <typename Ostream> inline Ostream &operator<<( Ostream & os , mem_fun_ptr) { os << "mem_fun_ptr"; return os; } } namespace expression { template < typename A0 = void , typename A1 = void , typename A2 = void , typename A3 = void , typename A4 = void , typename A5 = void , typename A6 = void , typename A7 = void , typename A8 = void , typename A9 = void , typename A10 = void , typename Dummy = void > struct mem_fun_ptr; template < typename A0 , typename A1 > struct mem_fun_ptr< A0 , A1 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 > {}; template < typename A0 , typename A1 , typename A2 > struct mem_fun_ptr< A0 , A1 , A2 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 , A2 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 > struct mem_fun_ptr< A0 , A1 , A2 , A3 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 , A2 , A3 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 > struct mem_fun_ptr< A0 , A1 , A2 , A3 , A4 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 , A2 , A3 , A4 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 > struct mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 , A2 , A3 , A4 , A5 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 > struct mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 , A6 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 , A2 , A3 , A4 , A5 , A6 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 > struct mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8 > struct mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8 , typename A9 > struct mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 , A9 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 , A9 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8 , typename A9 , typename A10 > struct mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 , A9 , A10 > : pdalboost::pdalboostphoenix::expr< tag:: mem_fun_ptr , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 , A9 , A10 > {}; } namespace rule { struct mem_fun_ptr : expression:: mem_fun_ptr < meta_grammar , pdalboost::proto::vararg< meta_grammar > > {}; } namespace functional { typedef pdalboost::proto::functional::make_expr< tag:: mem_fun_ptr > make_mem_fun_ptr; } namespace result_of { template <typename A0 = void , typename A1 = void , typename A2 = void , typename A3 = void , typename A4 = void , typename A5 = void , typename A6 = void , typename A7 = void , typename A8 = void , typename A9 = void, typename Dummy = void> struct make_mem_fun_ptr; template <typename A0> struct make_mem_fun_ptr <A0> : pdalboost::result_of< functional:: make_mem_fun_ptr( A0 ) > {}; template <typename A0 , typename A1> struct make_mem_fun_ptr <A0 , A1> : pdalboost::result_of< functional:: make_mem_fun_ptr( A0 , A1 ) > {}; template <typename A0 , typename A1 , typename A2> struct make_mem_fun_ptr <A0 , A1 , A2> : pdalboost::result_of< functional:: make_mem_fun_ptr( A0 , A1 , A2 ) > {}; template <typename A0 , typename A1 , typename A2 , typename A3> struct make_mem_fun_ptr <A0 , A1 , A2 , A3> : pdalboost::result_of< functional:: make_mem_fun_ptr( A0 , A1 , A2 , A3 ) > {}; template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4> struct make_mem_fun_ptr <A0 , A1 , A2 , A3 , A4> : pdalboost::result_of< functional:: make_mem_fun_ptr( A0 , A1 , A2 , A3 , A4 ) > {}; template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5> struct make_mem_fun_ptr <A0 , A1 , A2 , A3 , A4 , A5> : pdalboost::result_of< functional:: make_mem_fun_ptr( A0 , A1 , A2 , A3 , A4 , A5 ) > {}; template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6> struct make_mem_fun_ptr <A0 , A1 , A2 , A3 , A4 , A5 , A6> : pdalboost::result_of< functional:: make_mem_fun_ptr( A0 , A1 , A2 , A3 , A4 , A5 , A6 ) > {}; template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7> struct make_mem_fun_ptr <A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7> : pdalboost::result_of< functional:: make_mem_fun_ptr( A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 ) > {}; template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8> struct make_mem_fun_ptr <A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8> : pdalboost::result_of< functional:: make_mem_fun_ptr( A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 ) > {}; } template <typename A0> inline typename result_of:: make_mem_fun_ptr< A0 >::type make_mem_fun_ptr(A0 const& a0) { return functional::make_mem_fun_ptr()(a0); } template <typename A0 , typename A1> inline typename result_of:: make_mem_fun_ptr< A0 , A1 >::type make_mem_fun_ptr(A0 const& a0 , A1 const& a1) { return functional::make_mem_fun_ptr()(a0 , a1); } template <typename A0 , typename A1 , typename A2> inline typename result_of:: make_mem_fun_ptr< A0 , A1 , A2 >::type make_mem_fun_ptr(A0 const& a0 , A1 const& a1 , A2 const& a2) { return functional::make_mem_fun_ptr()(a0 , a1 , a2); } template <typename A0 , typename A1 , typename A2 , typename A3> inline typename result_of:: make_mem_fun_ptr< A0 , A1 , A2 , A3 >::type make_mem_fun_ptr(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3) { return functional::make_mem_fun_ptr()(a0 , a1 , a2 , a3); } template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4> inline typename result_of:: make_mem_fun_ptr< A0 , A1 , A2 , A3 , A4 >::type make_mem_fun_ptr(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4) { return functional::make_mem_fun_ptr()(a0 , a1 , a2 , a3 , a4); } template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5> inline typename result_of:: make_mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 >::type make_mem_fun_ptr(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4 , A5 const& a5) { return functional::make_mem_fun_ptr()(a0 , a1 , a2 , a3 , a4 , a5); } template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6> inline typename result_of:: make_mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 , A6 >::type make_mem_fun_ptr(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4 , A5 const& a5 , A6 const& a6) { return functional::make_mem_fun_ptr()(a0 , a1 , a2 , a3 , a4 , a5 , a6); } template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7> inline typename result_of:: make_mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 >::type make_mem_fun_ptr(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4 , A5 const& a5 , A6 const& a6 , A7 const& a7) { return functional::make_mem_fun_ptr()(a0 , a1 , a2 , a3 , a4 , a5 , a6 , a7); } template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8> inline typename result_of:: make_mem_fun_ptr< A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 >::type make_mem_fun_ptr(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4 , A5 const& a5 , A6 const& a6 , A7 const& a7 , A8 const& a8) { return functional::make_mem_fun_ptr()(a0 , a1 , a2 , a3 , a4 , a5 , a6 , a7 , a8); } } } namespace pdalboost{} namespace boost = pdalboost; namespace pdalboost{ namespace pdalboostphoenix { template <typename Dummy> struct meta_grammar::case_< :: pdalboost :: pdalboostphoenix :: tag:: mem_fun_ptr , Dummy > : enable_rule< :: pdalboost :: pdalboostphoenix :: rule:: mem_fun_ptr , Dummy > {}; } }
namespace pdalboost{} namespace boost = pdalboost; namespace pdalboost{ namespace pdalboostphoenix
{
    namespace detail {
        template <typename Object, typename MemPtr>
        struct mem_fun_ptr_gen
        {
            mem_fun_ptr_gen(Object const& obj, MemPtr ptr)
              : obj(obj)
              , ptr(ptr)
            {}
            typename pdalboostphoenix::expression::mem_fun_ptr<Object, MemPtr>::type const
            operator()() const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<Object, MemPtr>::make(obj, ptr);
            }
    
    
    
    
    
    
    
            template <typename A0>
            typename pdalboostphoenix::expression::mem_fun_ptr<
                Object
              , MemPtr
              , A0
            >::type const
            operator()(A0 const& a0) const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<
                    Object
                  , MemPtr
                  , A0
                >::make(obj, ptr, a0);
            }
    
    
    
    
    
    
    
            template <typename A0 , typename A1>
            typename pdalboostphoenix::expression::mem_fun_ptr<
                Object
              , MemPtr
              , A0 , A1
            >::type const
            operator()(A0 const& a0 , A1 const& a1) const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<
                    Object
                  , MemPtr
                  , A0 , A1
                >::make(obj, ptr, a0 , a1);
            }
    
    
    
    
    
    
    
            template <typename A0 , typename A1 , typename A2>
            typename pdalboostphoenix::expression::mem_fun_ptr<
                Object
              , MemPtr
              , A0 , A1 , A2
            >::type const
            operator()(A0 const& a0 , A1 const& a1 , A2 const& a2) const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<
                    Object
                  , MemPtr
                  , A0 , A1 , A2
                >::make(obj, ptr, a0 , a1 , a2);
            }
    
    
    
    
    
    
    
            template <typename A0 , typename A1 , typename A2 , typename A3>
            typename pdalboostphoenix::expression::mem_fun_ptr<
                Object
              , MemPtr
              , A0 , A1 , A2 , A3
            >::type const
            operator()(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3) const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<
                    Object
                  , MemPtr
                  , A0 , A1 , A2 , A3
                >::make(obj, ptr, a0 , a1 , a2 , a3);
            }
    
    
    
    
    
    
    
            template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4>
            typename pdalboostphoenix::expression::mem_fun_ptr<
                Object
              , MemPtr
              , A0 , A1 , A2 , A3 , A4
            >::type const
            operator()(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4) const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<
                    Object
                  , MemPtr
                  , A0 , A1 , A2 , A3 , A4
                >::make(obj, ptr, a0 , a1 , a2 , a3 , a4);
            }
    
    
    
    
    
    
    
            template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5>
            typename pdalboostphoenix::expression::mem_fun_ptr<
                Object
              , MemPtr
              , A0 , A1 , A2 , A3 , A4 , A5
            >::type const
            operator()(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4 , A5 const& a5) const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<
                    Object
                  , MemPtr
                  , A0 , A1 , A2 , A3 , A4 , A5
                >::make(obj, ptr, a0 , a1 , a2 , a3 , a4 , a5);
            }
    
    
    
    
    
    
    
            template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6>
            typename pdalboostphoenix::expression::mem_fun_ptr<
                Object
              , MemPtr
              , A0 , A1 , A2 , A3 , A4 , A5 , A6
            >::type const
            operator()(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4 , A5 const& a5 , A6 const& a6) const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<
                    Object
                  , MemPtr
                  , A0 , A1 , A2 , A3 , A4 , A5 , A6
                >::make(obj, ptr, a0 , a1 , a2 , a3 , a4 , a5 , a6);
            }
    
    
    
    
    
    
    
            template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7>
            typename pdalboostphoenix::expression::mem_fun_ptr<
                Object
              , MemPtr
              , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7
            >::type const
            operator()(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4 , A5 const& a5 , A6 const& a6 , A7 const& a7) const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<
                    Object
                  , MemPtr
                  , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7
                >::make(obj, ptr, a0 , a1 , a2 , a3 , a4 , a5 , a6 , a7);
            }
    
    
    
    
    
    
    
            template <typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8>
            typename pdalboostphoenix::expression::mem_fun_ptr<
                Object
              , MemPtr
              , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8
            >::type const
            operator()(A0 const& a0 , A1 const& a1 , A2 const& a2 , A3 const& a3 , A4 const& a4 , A5 const& a5 , A6 const& a6 , A7 const& a7 , A8 const& a8) const
            {
                return pdalboostphoenix::expression::mem_fun_ptr<
                    Object
                  , MemPtr
                  , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8
                >::make(obj, ptr, a0 , a1 , a2 , a3 , a4 , a5 , a6 , a7 , a8);
            }
            Object const& obj;
            MemPtr ptr;
        };
    }
}}
