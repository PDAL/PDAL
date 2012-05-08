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

#include <vector>
#include <iostream>

#include <Python.h>


namespace pdal
{
namespace plang
{
namespace emb
{

typedef std::function<void(std::string)> stdout_write_type;
void set_stdout(stdout_write_type write);
void reset_stdout();

PyMODINIT_FUNC /***PyInit_emb***/init(void);


} // emb namespace

}
} // namespaces

#endif

#endif
