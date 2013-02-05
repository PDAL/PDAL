// (C) Copyright 2006 Douglas Gregor <doug.gregor -at- gmail.com>

// Use, modification and distribution is subject to the Boost Software
// License, Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

//  Authors: Douglas Gregor
#ifndef BOOST_MPI_PYTHON_UTILITY_HPP
#define BOOST_MPI_PYTHON_UTILITY_HPP

/** @file utility.hpp
 *
 *  This file is a utility header for the Boost.MPI Python bindings.
 */
#include <boost/python.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace mpi { namespace python {

template<typename E>
class translate_exception
{
  explicit translate_exception(pdalboost::python::object type) : type(type) { }

public:
  static void declare(pdalboost::python::object type)
  {
    using pdalboost::python::register_exception_translator;
    register_exception_translator<E>(translate_exception(type));
  }

  void operator()(const E& e) const
  {
    using pdalboost::python::object;
    PyErr_SetObject(type.ptr(), object(e).ptr());
  }

private:
  pdalboost::python::object type;
};

} } } // end namespace pdalboost::mpi::python

#endif // BOOST_MPI_PYTHON_UTILITY_HPP
