// Copyright David Abrahams 2002.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
#ifndef LIFE_SUPPORT_DWA200222_HPP
# define LIFE_SUPPORT_DWA200222_HPP
# include <boost/python/detail/prefix.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace python { namespace objects { 

BOOST_PYTHON_DECL PyObject* make_nurse_and_patient(PyObject* nurse, PyObject* patient);

}}} // namespace pdalboost::python::object

#endif // LIFE_SUPPORT_DWA200222_HPP
