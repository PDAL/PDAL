//
// Copyright (C) 2011 Mateusz Loskot <mateusz@loskot.net>
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// Blog article: http://mateusz.loskot.net/?p=2819


// http://python3porting.com/cextensions.html


#ifndef PDAL_PLANG_REDIRECTOR_H
#define PDAL_PLANG_REDIRECTOR_H

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include <pdal/pdal_internal.hpp>
#include <pdal/PointBuffer.hpp>

#include <boost/cstdint.hpp>
#include <boost/variant.hpp>
#include <functional>
#include <vector>
#include <iostream>

#include <Python.h>


namespace pdal
{
namespace plang
{

    
PyMODINIT_FUNC redirector_init(void);

class Redirector
{
public:
    typedef std::function<void(std::string)> stdout_write_type;

    static void init();
    static void set_stdout(stdout_write_type write);
    static void reset_stdout();

private:
    // Internal state
    static PyObject* g_stdout;
    static PyObject* g_stdout_saved;
};


} // namespaces
}

#endif

#endif
