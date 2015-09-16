//
// Copyright (C) 2011 Mateusz Loskot <mateusz@loskot.net>
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// Blog article: http://mateusz.loskot.net/?p=2819

// http://python3porting.com/cextensions.html

#pragma once

#include <Python.h>
#undef toupper
#undef tolower
#undef isspace

#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace pdal
{
namespace plang
{

PyMODINIT_FUNC redirector_init(void);

class Redirector
{
public:
    Redirector();
    ~Redirector();

    static PyObject* init();
    void set_stdout(std::ostream* ostr);
    void reset_stdout();

    typedef boost::function<void(std::string)> stdout_write_type;

private:
    void set_stdout(stdout_write_type write);

    // Internal state
    PyObject* m_stdout;
    PyObject* m_stdout_saved;
};

} // namespace plang
} // namespace pdal

