// (C) Copyright 2006 Douglas Gregor <doug.gregor -at- gmail.com>

// Use, modification and distribution is subject to the Boost Software
// License, Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

//  Authors: Douglas Gregor

/** @file status.cpp
 *
 *  This file reflects the Boost.MPI @c status class into
 *  Python.
 */
#include <boost/python.hpp>
#include <boost/mpi.hpp>

using namespace pdalboost::python;
using namespace pdalboost::mpi;

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace mpi { namespace python {

extern const char* status_docstring;
extern const char* status_source_docstring;
extern const char* status_tag_docstring;
extern const char* status_error_docstring;
extern const char* status_cancelled_docstring;

void export_status()
{
  using pdalboost::python::arg;
  using pdalboost::python::object;
  
  class_<status>("Status", status_docstring, no_init)
    .add_property("source", &status::source, status_source_docstring)
    .add_property("tag", &status::tag, status_tag_docstring)
    .add_property("error", &status::error, status_error_docstring)
    .add_property("cancelled", &status::cancelled, status_cancelled_docstring)
    ;
}

} } } // end namespace pdalboost::mpi::python
