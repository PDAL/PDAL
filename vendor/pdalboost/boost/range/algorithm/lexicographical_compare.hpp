//  Copyright Neil Groves 2009. Use, modification and
//  distribution is subject to the Boost Software License, Version
//  1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
//
// For more information, see http://www.boost.org/libs/range/
//
#ifndef BOOST_RANGE_ALGORITHM_LEXICOGRAPHICAL_COMPARE_HPP_INCLUDED
#define BOOST_RANGE_ALGORITHM_LEXICOGRAPHICAL_COMPARE_HPP_INCLUDED

#include <boost/concept_check.hpp>
#include <boost/range/begin.hpp>
#include <boost/range/end.hpp>
#include <boost/range/concepts.hpp>
#include <algorithm>

namespace pdalboost
{
    namespace range
    {

/// \brief template function lexicographic_compare
///
/// range-based version of the lexicographic_compare std algorithm
///
/// \pre SinglePassRange1 is a model of the SinglePassRangeConcept
/// \pre SinglePassRange2 is a model of the SinglePassRangeConcept
template<class SinglePassRange1, class SinglePassRange2>
inline bool lexicographical_compare(const SinglePassRange1& rng1,
                                    const SinglePassRange2& rng2)
{
    BOOST_RANGE_CONCEPT_ASSERT(( SinglePassRangeConcept<const SinglePassRange1> ));
    BOOST_RANGE_CONCEPT_ASSERT(( SinglePassRangeConcept<const SinglePassRange2> ));
    return std::lexicographical_compare(
        pdalboost::begin(rng1), pdalboost::end(rng1),
        pdalboost::begin(rng2), pdalboost::end(rng2));
}

/// \overload
template<class SinglePassRange1, class SinglePassRange2,
         class BinaryPredicate>
inline bool lexicographical_compare(const SinglePassRange1& rng1,
                                    const SinglePassRange2& rng2,
                                    BinaryPredicate pred)
{
    BOOST_RANGE_CONCEPT_ASSERT(( SinglePassRangeConcept<const SinglePassRange1> ));
    BOOST_RANGE_CONCEPT_ASSERT(( SinglePassRangeConcept<const SinglePassRange2> ));
    return std::lexicographical_compare(
        pdalboost::begin(rng1), pdalboost::end(rng1),
        pdalboost::begin(rng2), pdalboost::end(rng2), pred);
}

    } // namespace range
    using range::lexicographical_compare;
} // namespace pdalboost

#endif // include guard
