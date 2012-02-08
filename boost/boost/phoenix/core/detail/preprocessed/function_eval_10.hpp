/*=============================================================================
    Copyright (c) 2001-2007 Joel de Guzman

    Distributed under the Boost Software License, Version 1.0. (See accompanying
    file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
==============================================================================*/
namespace pdalboost{} namespace boost = pdalboost; namespace pdalboost{ namespace pdalboostphoenix { namespace detail { namespace tag { struct function_eval {}; template <typename Ostream> inline Ostream &operator<<( Ostream & os , function_eval) { os << "function_eval"; return os; } } namespace expression { template < typename A0 = void , typename A1 = void , typename A2 = void , typename A3 = void , typename A4 = void , typename A5 = void , typename A6 = void , typename A7 = void , typename A8 = void , typename A9 = void , typename Dummy = void > struct function_eval; template < typename A0 , typename A1 > struct function_eval< A0 , A1 > : pdalboost::pdalboostphoenix::expr< tag:: function_eval , A0 , A1 > {}; template < typename A0 , typename A1 , typename A2 > struct function_eval< A0 , A1 , A2 > : pdalboost::pdalboostphoenix::expr< tag:: function_eval , A0 , A1 , A2 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 > struct function_eval< A0 , A1 , A2 , A3 > : pdalboost::pdalboostphoenix::expr< tag:: function_eval , A0 , A1 , A2 , A3 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 > struct function_eval< A0 , A1 , A2 , A3 , A4 > : pdalboost::pdalboostphoenix::expr< tag:: function_eval , A0 , A1 , A2 , A3 , A4 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 > struct function_eval< A0 , A1 , A2 , A3 , A4 , A5 > : pdalboost::pdalboostphoenix::expr< tag:: function_eval , A0 , A1 , A2 , A3 , A4 , A5 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 > struct function_eval< A0 , A1 , A2 , A3 , A4 , A5 , A6 > : pdalboost::pdalboostphoenix::expr< tag:: function_eval , A0 , A1 , A2 , A3 , A4 , A5 , A6 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 > struct function_eval< A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 > : pdalboost::pdalboostphoenix::expr< tag:: function_eval , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8 > struct function_eval< A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 > : pdalboost::pdalboostphoenix::expr< tag:: function_eval , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 > {}; template < typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8 , typename A9 > struct function_eval< A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 , A9 > : pdalboost::pdalboostphoenix::expr< tag:: function_eval , A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8 , A9 > {}; typedef proto::functional::make_expr< tag:: function_eval > make_function_eval; } namespace rule { struct function_eval : expression:: function_eval < meta_grammar , proto::vararg< meta_grammar > > {}; } } } } namespace pdalboost{} namespace boost = pdalboost; namespace pdalboost{ namespace pdalboostphoenix { template <typename Dummy> struct meta_grammar::case_< :: pdalboost :: pdalboostphoenix :: detail :: tag:: function_eval , Dummy > : enable_rule< :: pdalboost :: pdalboostphoenix :: detail :: rule:: function_eval , Dummy > {}; } }
namespace pdalboost{} namespace boost = pdalboost; namespace pdalboost{ namespace pdalboostphoenix {
    namespace detail
    {
        template <typename T>
        T& help_rvalue_deduction(T& x)
        {
            return x;
        }
        
        template <typename T>
        T const& help_rvalue_deduction(T const& x)
        {
            return x;
        }
        struct function_eval
        {
            template <typename Sig>
            struct result;
            template <typename This, typename F, typename Context>
            struct result<This(F, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of<fn()>::type type;
            };
            template <typename F, typename Context>
            typename result<function_eval(F const&, Context const&)>::type
            operator()(F const & f, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)();
            }
            template <typename F, typename Context>
            typename result<function_eval(F &, Context const&)>::type
            operator()(F & f, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)();
            }
        
    
    
    
    
    
    
    
            template <
                typename This
              , typename F
              , typename A0
              , typename Context
            >
            struct result<This(F, A0, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A0 , Context ) >::type a0;
                typedef typename
                    pdalboost::result_of<fn(a0)>::type
                    type;
                
            };
            template <typename F, typename A0, typename Context>
            typename result<
                function_eval(
                    F const &
                  , A0 &
                  , Context const &
                )
            >::type
            operator()(F const & f, A0 & a0, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)));
            }
            template <typename F, typename A0, typename Context>
            typename result<
                function_eval(
                    F &
                  , A0 &
                  , Context const &
                )
            >::type
            operator()(F & f, A0 & a0, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)));
            }
    
    
    
    
    
    
    
            template <
                typename This
              , typename F
              , typename A0 , typename A1
              , typename Context
            >
            struct result<This(F, A0 , A1, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A0 , Context ) >::type a0; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A1 , Context ) >::type a1;
                typedef typename
                    pdalboost::result_of<fn(a0 , a1)>::type
                    type;
                
            };
            template <typename F, typename A0 , typename A1, typename Context>
            typename result<
                function_eval(
                    F const &
                  , A0 & , A1 &
                  , Context const &
                )
            >::type
            operator()(F const & f, A0 & a0 , A1 & a1, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)));
            }
            template <typename F, typename A0 , typename A1, typename Context>
            typename result<
                function_eval(
                    F &
                  , A0 & , A1 &
                  , Context const &
                )
            >::type
            operator()(F & f, A0 & a0 , A1 & a1, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)));
            }
    
    
    
    
    
    
    
            template <
                typename This
              , typename F
              , typename A0 , typename A1 , typename A2
              , typename Context
            >
            struct result<This(F, A0 , A1 , A2, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A0 , Context ) >::type a0; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A1 , Context ) >::type a1; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A2 , Context ) >::type a2;
                typedef typename
                    pdalboost::result_of<fn(a0 , a1 , a2)>::type
                    type;
                
            };
            template <typename F, typename A0 , typename A1 , typename A2, typename Context>
            typename result<
                function_eval(
                    F const &
                  , A0 & , A1 & , A2 &
                  , Context const &
                )
            >::type
            operator()(F const & f, A0 & a0 , A1 & a1 , A2 & a2, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)));
            }
            template <typename F, typename A0 , typename A1 , typename A2, typename Context>
            typename result<
                function_eval(
                    F &
                  , A0 & , A1 & , A2 &
                  , Context const &
                )
            >::type
            operator()(F & f, A0 & a0 , A1 & a1 , A2 & a2, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)));
            }
    
    
    
    
    
    
    
            template <
                typename This
              , typename F
              , typename A0 , typename A1 , typename A2 , typename A3
              , typename Context
            >
            struct result<This(F, A0 , A1 , A2 , A3, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A0 , Context ) >::type a0; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A1 , Context ) >::type a1; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A2 , Context ) >::type a2; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A3 , Context ) >::type a3;
                typedef typename
                    pdalboost::result_of<fn(a0 , a1 , a2 , a3)>::type
                    type;
                
            };
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3, typename Context>
            typename result<
                function_eval(
                    F const &
                  , A0 & , A1 & , A2 & , A3 &
                  , Context const &
                )
            >::type
            operator()(F const & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)));
            }
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3, typename Context>
            typename result<
                function_eval(
                    F &
                  , A0 & , A1 & , A2 & , A3 &
                  , Context const &
                )
            >::type
            operator()(F & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)));
            }
    
    
    
    
    
    
    
            template <
                typename This
              , typename F
              , typename A0 , typename A1 , typename A2 , typename A3 , typename A4
              , typename Context
            >
            struct result<This(F, A0 , A1 , A2 , A3 , A4, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A0 , Context ) >::type a0; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A1 , Context ) >::type a1; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A2 , Context ) >::type a2; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A3 , Context ) >::type a3; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A4 , Context ) >::type a4;
                typedef typename
                    pdalboost::result_of<fn(a0 , a1 , a2 , a3 , a4)>::type
                    type;
                
            };
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4, typename Context>
            typename result<
                function_eval(
                    F const &
                  , A0 & , A1 & , A2 & , A3 & , A4 &
                  , Context const &
                )
            >::type
            operator()(F const & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)));
            }
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4, typename Context>
            typename result<
                function_eval(
                    F &
                  , A0 & , A1 & , A2 & , A3 & , A4 &
                  , Context const &
                )
            >::type
            operator()(F & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)));
            }
    
    
    
    
    
    
    
            template <
                typename This
              , typename F
              , typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5
              , typename Context
            >
            struct result<This(F, A0 , A1 , A2 , A3 , A4 , A5, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A0 , Context ) >::type a0; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A1 , Context ) >::type a1; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A2 , Context ) >::type a2; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A3 , Context ) >::type a3; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A4 , Context ) >::type a4; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A5 , Context ) >::type a5;
                typedef typename
                    pdalboost::result_of<fn(a0 , a1 , a2 , a3 , a4 , a5)>::type
                    type;
                
            };
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5, typename Context>
            typename result<
                function_eval(
                    F const &
                  , A0 & , A1 & , A2 & , A3 & , A4 & , A5 &
                  , Context const &
                )
            >::type
            operator()(F const & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4 , A5 & a5, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a5, ctx)));
            }
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5, typename Context>
            typename result<
                function_eval(
                    F &
                  , A0 & , A1 & , A2 & , A3 & , A4 & , A5 &
                  , Context const &
                )
            >::type
            operator()(F & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4 , A5 & a5, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a5, ctx)));
            }
    
    
    
    
    
    
    
            template <
                typename This
              , typename F
              , typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6
              , typename Context
            >
            struct result<This(F, A0 , A1 , A2 , A3 , A4 , A5 , A6, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A0 , Context ) >::type a0; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A1 , Context ) >::type a1; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A2 , Context ) >::type a2; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A3 , Context ) >::type a3; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A4 , Context ) >::type a4; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A5 , Context ) >::type a5; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A6 , Context ) >::type a6;
                typedef typename
                    pdalboost::result_of<fn(a0 , a1 , a2 , a3 , a4 , a5 , a6)>::type
                    type;
                
            };
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6, typename Context>
            typename result<
                function_eval(
                    F const &
                  , A0 & , A1 & , A2 & , A3 & , A4 & , A5 & , A6 &
                  , Context const &
                )
            >::type
            operator()(F const & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4 , A5 & a5 , A6 & a6, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a5, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a6, ctx)));
            }
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6, typename Context>
            typename result<
                function_eval(
                    F &
                  , A0 & , A1 & , A2 & , A3 & , A4 & , A5 & , A6 &
                  , Context const &
                )
            >::type
            operator()(F & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4 , A5 & a5 , A6 & a6, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a5, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a6, ctx)));
            }
    
    
    
    
    
    
    
            template <
                typename This
              , typename F
              , typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7
              , typename Context
            >
            struct result<This(F, A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A0 , Context ) >::type a0; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A1 , Context ) >::type a1; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A2 , Context ) >::type a2; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A3 , Context ) >::type a3; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A4 , Context ) >::type a4; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A5 , Context ) >::type a5; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A6 , Context ) >::type a6; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A7 , Context ) >::type a7;
                typedef typename
                    pdalboost::result_of<fn(a0 , a1 , a2 , a3 , a4 , a5 , a6 , a7)>::type
                    type;
                
            };
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7, typename Context>
            typename result<
                function_eval(
                    F const &
                  , A0 & , A1 & , A2 & , A3 & , A4 & , A5 & , A6 & , A7 &
                  , Context const &
                )
            >::type
            operator()(F const & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4 , A5 & a5 , A6 & a6 , A7 & a7, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a5, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a6, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a7, ctx)));
            }
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7, typename Context>
            typename result<
                function_eval(
                    F &
                  , A0 & , A1 & , A2 & , A3 & , A4 & , A5 & , A6 & , A7 &
                  , Context const &
                )
            >::type
            operator()(F & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4 , A5 & a5 , A6 & a6 , A7 & a7, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a5, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a6, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a7, ctx)));
            }
    
    
    
    
    
    
    
            template <
                typename This
              , typename F
              , typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8
              , typename Context
            >
            struct result<This(F, A0 , A1 , A2 , A3 , A4 , A5 , A6 , A7 , A8, Context)>
            {
                typedef typename
                    remove_reference<
                        typename pdalboost::result_of<evaluator(F, Context)>::type
                    >::type
                    fn;
                typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A0 , Context ) >::type a0; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A1 , Context ) >::type a1; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A2 , Context ) >::type a2; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A3 , Context ) >::type a3; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A4 , Context ) >::type a4; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A5 , Context ) >::type a5; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A6 , Context ) >::type a6; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A7 , Context ) >::type a7; typedef typename pdalboost::result_of< pdalboost::pdalboostphoenix::evaluator( A8 , Context ) >::type a8;
                typedef typename
                    pdalboost::result_of<fn(a0 , a1 , a2 , a3 , a4 , a5 , a6 , a7 , a8)>::type
                    type;
                
            };
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8, typename Context>
            typename result<
                function_eval(
                    F const &
                  , A0 & , A1 & , A2 & , A3 & , A4 & , A5 & , A6 & , A7 & , A8 &
                  , Context const &
                )
            >::type
            operator()(F const & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4 , A5 & a5 , A6 & a6 , A7 & a7 , A8 & a8, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a5, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a6, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a7, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a8, ctx)));
            }
            template <typename F, typename A0 , typename A1 , typename A2 , typename A3 , typename A4 , typename A5 , typename A6 , typename A7 , typename A8, typename Context>
            typename result<
                function_eval(
                    F &
                  , A0 & , A1 & , A2 & , A3 & , A4 & , A5 & , A6 & , A7 & , A8 &
                  , Context const &
                )
            >::type
            operator()(F & f, A0 & a0 , A1 & a1 , A2 & a2 , A3 & a3 , A4 & a4 , A5 & a5 , A6 & a6 , A7 & a7 , A8 & a8, Context const & ctx) const
            {
                return pdalboost::pdalboostphoenix::eval(f, ctx)(help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a0, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a1, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a2, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a3, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a4, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a5, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a6, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a7, ctx)) , help_rvalue_deduction(pdalboost::pdalboostphoenix::eval(a8, ctx)));
            }
        
        };
        
    }
    template <typename Dummy>
    struct default_actions::when<detail::rule::function_eval, Dummy>
        : pdalboostphoenix::call<detail::function_eval>
    {};
}}
