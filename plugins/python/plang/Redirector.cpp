//
// Copyright (C) 2011 Mateusz Loskot <mateusz@loskot.net>
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// Blog article: http://mateusz.loskot.net/?p=2819

#include "Redirector.hpp"

#pragma warning(disable: 4127)  // conditional expression is constant
#pragma warning(disable: 4068)  // gcc pragma warnings

#include <ostream>
#include <string>
#include <iostream>

namespace pdal
{
namespace plang
{

struct Stdout
{
    PyObject_HEAD
    Redirector::stdout_write_type write;
    Redirector::stdout_flush_type flush;
};


static PyObject* Stdout_write(PyObject* self, PyObject* args)
{
    std::size_t written(0);
    Stdout* selfimpl = reinterpret_cast<Stdout*>(self);
    if (selfimpl->write)
    {
        char* data;
        if (!PyArg_ParseTuple(args, "s", &data))
            return 0;

        std::string str(data);
        selfimpl->write(str);
        written = str.size();
    }
    return PyLong_FromSize_t(written);
}


static PyObject* Stdout_flush(PyObject* self, PyObject* /*args*/)
{
    Stdout *selfimpl = reinterpret_cast<Stdout *>(self);
    if (selfimpl->flush)
    {
        selfimpl->flush();
    }
    return Py_BuildValue("");
}


static PyMethodDef Stdout_methods[] =
{
    {"write", Stdout_write, METH_VARARGS, "sys.stdout.write"},
    {"flush", Stdout_flush, METH_VARARGS, "sys.stdout.flush"},
    {0, 0, 0, 0} // sentinel
};


static PyTypeObject StdoutType =
{
    PyVarObject_HEAD_INIT(0, 0)
    "redirector.StdoutType", /* tp_name */
    sizeof(Stdout), /* tp_basicsize */
    0, /* tp_itemsize */
    0, /* tp_dealloc */
    0, /* tp_print */
    0, /* tp_getattr */
    0, /* tp_setattr */
    0, /* tp_reserved */
    0, /* tp_repr */
    0, /* tp_as_number */
    0, /* tp_as_sequence */
    0, /* tp_as_mapping */
    0, /* tp_hash */
    0, /* tp_call */
    0, /* tp_str */
    0, /* tp_getattro */
    0, /* tp_setattro */
    0, /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT, /* tp_flags */
    "redirector.Stdout objects", /* tp_doc */
    0, /* tp_traverse */
    0, /* tp_clear */
    0, /* tp_richcompare */
    0, /* tp_weaklistoffset */
    0, /* tp_iter */
    0, /* tp_iternext */
    Stdout_methods, /* tp_methods */
    0, /* tp_members */
    0, /* tp_getset */
    0, /* tp_base */
    0, /* tp_dict */
    0, /* tp_descr_get */
    0, /* tp_descr_set */
    0, /* tp_dictoffset */
    0, /* tp_init */
    0, /* tp_alloc */
    0, /* tp_new */
    0, /* tp_free */
    0, /* tp_is_gc */
    0, /* tp_bases */
    0, /* tp_mro */
    0, /* tp_cache */
    0, /* tp_subclasses */
    0, /* tp_weaklist */
    0, /* tp_del */
    0, /* tp_version_tag */
    0, /* tp_finalilzer */
};


Redirector::Redirector()
    : m_stdout(NULL)
    , m_stdout_saved(NULL)
{
    return;
}


Redirector::~Redirector()
{
    return;
}


PyMODINIT_FUNC redirector_init(void)
{
    return Redirector::init();
}

static struct PyModuleDef redirectordef = {
    PyModuleDef_HEAD_INIT,
    "redirector",     /* m_name */
    "redirector.Stdout objects",  /* m_doc */
    -1,                  /* m_size */
    Stdout_methods,    /* m_methods */
    NULL,                /* m_reload */
    NULL,                /* m_traverse */
    NULL,                /* m_clear */
    NULL,                /* m_free */
};

PyObject* Redirector::init()
{
    StdoutType.tp_new = PyType_GenericNew;
    if (PyType_Ready(&StdoutType) < 0)
        return NULL;
    PyObject* m = PyModule_Create(&redirectordef);
    if (m)
    {
        //ABELL - This is bad code as the type cast is invalid. (type pun
        //  warning.)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
        Py_INCREF(reinterpret_cast<PyObject*>(&StdoutType));
        PyModule_AddObject(m, "Stdout", reinterpret_cast<PyObject*>(&StdoutType));
#pragma GCC diagnostic pop
    }
    return m;
}


void Redirector::set_stdout(std::ostream* ostr)
{
    stdout_write_type writeFunc = [ostr](std::string msg) { *ostr << msg; };
    stdout_flush_type flushFunc = [ostr]{ ostr->flush(); };

    this->set_stdout(writeFunc, flushFunc);
}


void Redirector::set_stdout(stdout_write_type write, stdout_flush_type flush)
{
    if (!m_stdout)
    {
        m_stdout_saved =
            PySys_GetObject(const_cast<char*>("stdout")); // borrowed
        m_stdout = StdoutType.tp_new(&StdoutType, 0, 0);
    }

    Stdout* impl = reinterpret_cast<Stdout*>(m_stdout);
    impl->write = write;
    impl->flush = flush;
    PySys_SetObject(const_cast<char*>("stdout"), m_stdout);
}


void Redirector::reset_stdout()
{
    if (m_stdout_saved)
        PySys_SetObject(const_cast<char*>("stdout"), m_stdout_saved);

    Py_XDECREF(m_stdout);
    m_stdout = 0;
}

} //namespace plang
} //namespace pdal

