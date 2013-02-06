// Copyright David Abrahams 2006. Distributed under the Boost
// Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef BOOST_CONCEPT_REQUIRES_DWA2006430_HPP
# define BOOST_CONCEPT_REQUIRES_DWA2006430_HPP

# include <boost/config.hpp>
# include <boost/parameter/aux_/parenthesized_type.hpp>
# include <boost/concept/assert.hpp>
# include <boost/preprocessor/seq/for_each.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { 

// Template for use in handwritten assertions
template <class Model, class More>
struct requires_ : More
{
# if BOOST_WORKAROUND(BOOST_MSVC, <= 1300)
    typedef typename More::type type;
# endif 
    BOOST_CONCEPT_ASSERT((Model));
};

// Template for use by macros, where models must be wrapped in parens.
// This isn't in namespace detail to keep extra cruft out of resulting
// error messages.
template <class ModelFn>
struct _requires_
{
    enum { value = 0 };
    BOOST_CONCEPT_ASSERT_FN(ModelFn);
};

template <int check, class Result>
struct Requires_ : ::pdalboost::parameter::aux::unaryfunptr_arg_type<Result>
{
# if BOOST_WORKAROUND(BOOST_MSVC, <= 1300)
    typedef typename ::pdalboost::parameter::aux::unaryfunptr_arg_type<Result>::type type;
# endif 
};

# if BOOST_WORKAROUND(BOOST_INTEL_WIN, BOOST_TESTED_AT(1010))
#  define BOOST_CONCEPT_REQUIRES_(r,data,t) | (::pdalboost::_requires_<void(*)t>::value)
# else 
#  define BOOST_CONCEPT_REQUIRES_(r,data,t) + (::pdalboost::_requires_<void(*)t>::value)
# endif

#if defined(NDEBUG) || BOOST_WORKAROUND(BOOST_MSVC, < 1300)

# define BOOST_CONCEPT_REQUIRES(models, result)                                    \
    typename ::pdalboost::parameter::aux::unaryfunptr_arg_type<void(*)result>::type

#elif BOOST_WORKAROUND(__BORLANDC__, BOOST_TESTED_AT(0x564))

// Same thing as below without the initial typename
# define BOOST_CONCEPT_REQUIRES(models, result)                                \
    ::pdalboost::Requires_<                                                        \
      (0 BOOST_PP_SEQ_FOR_EACH(BOOST_CONCEPT_REQUIRES_, ~, models)),           \
      ::pdalboost::parameter::aux::unaryfunptr_arg_type<void(*)result>          \
                     >::type

#else

// This just ICEs on MSVC6 :(
# define BOOST_CONCEPT_REQUIRES(models, result)                                        \
    typename ::pdalboost::Requires_<                                                       \
      (0 BOOST_PP_SEQ_FOR_EACH(BOOST_CONCEPT_REQUIRES_, ~, models)),                   \
      void(*)result                                                                 \
    >::type

#endif 

// C++0x proposed syntax changed.  This supports an older usage
#define BOOST_CONCEPT_WHERE(models,result) BOOST_CONCEPT_REQUIRES(models,result)

} // namespace pdalboost::concept_check

#endif // BOOST_CONCEPT_REQUIRES_DWA2006430_HPP
