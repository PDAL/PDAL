// Copyright (C) 2007 Matthias Troyer
//
// Use, modification and distribution is subject to the Boost Software
// License, Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//
// This file contains helper data structures for use in transmitting
// properties. The basic idea is to optimize away any storage for the
// properties when no properties are specified.
#ifndef BOOST_PARALLEL_DETAIL_UNTRACKED_PAIR_HPP
#define BOOST_PARALLEL_DETAIL_UNTRACKED_PAIR_HPP

#ifndef BOOST_GRAPH_USE_MPI
#error "Parallel BGL files should not be included unless <boost/graph/use_mpi.hpp> has been included"
#endif

#include <boost/mpi/datatype.hpp>
#include <utility> // for std::pair
#include <boost/serialization/utility.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace parallel { namespace detail {

/**
 * This structure is like std::pair, with the only difference
 * that tracking in the serialization library is turned off.
 */
 
template<typename T, typename U>
struct untracked_pair : public std::pair<T,U>
{
  untracked_pair() {}

  untracked_pair(const T& t, const U& u)
  : std::pair<T,U>(t,u) {}

  template<class T1, class U1>
  untracked_pair(std::pair<T1,U1> const& p)
  : std::pair<T,U>(p) {}  
};

template<typename T, typename U>
inline untracked_pair<T, U>
make_untracked_pair(const T& t, const U& u)
{
  return untracked_pair<T,U>(t,u);
}

} } } // end namespace pdalboost::parallel::detail

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace mpi {

template<typename T, typename U>
struct is_mpi_datatype<pdalboost::parallel::detail::untracked_pair<T, U> >
  : is_mpi_datatype<std::pair<T,U> > {};

} } // end namespace pdalboost::mpi

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace serialization {

// pair
template<class Archive, class F, class S>
inline void serialize(
    Archive & ar,
    pdalboost::parallel::detail::untracked_pair<F, S> & p,
    const unsigned int /* file_version */
){
    ar & pdalboost::serialization::make_nvp("first", p.first);
    ar & pdalboost::serialization::make_nvp("second", p.second);
}

template<typename T, typename U>
struct is_bitwise_serializable<
        pdalboost::parallel::detail::untracked_pair<T, U> >
  : is_bitwise_serializable<std::pair<T, U> > {};

template<typename T, typename U>
struct implementation_level<pdalboost::parallel::detail::untracked_pair<T, U> >
 : mpl::int_<object_serializable> {} ;

template<typename T, typename U>
struct tracking_level<pdalboost::parallel::detail::untracked_pair<T, U> >
 : mpl::int_<track_never> {} ;

} } // end namespace pdalboost::serialization

#endif // BOOST_PARALLEL_DETAIL_UNTRACKED_PAIR_HPP
