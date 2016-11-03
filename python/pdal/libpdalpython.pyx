# distutils: language = c++

from libcpp.vector cimport vector
from libcpp.string cimport string
from cpython.version cimport PY_MAJOR_VERSION
cimport numpy as np
np.import_array()

from cpython cimport PyObject, Py_INCREF
from cython.operator cimport dereference as deref, preincrement as inc

cdef extern from "pdal/plang/Array.hpp" namespace "pdal::plang":
    cdef cppclass Array:
        void* getPythonArray() except+

cdef extern from "PyPipeline.hpp" namespace "libpdalpython":
    cdef cppclass Pipeline:
        Pipeline(const char* ) except +
        void execute() except +
        const char* getJSON()
        vector[Array*] getArrays() except +

cdef class PyPipeline:
    cdef Pipeline *thisptr      # hold a c++ instance which we're wrapping
    def __cinit__(self, unicode json):
        cdef char* x
        if PY_MAJOR_VERSION >= 3:
            py_byte_string = json.encode('UTF-8')
            x= py_byte_string
            self.thisptr = new Pipeline(x)
        else:
            self.thisptr = new Pipeline(json)
    def __dealloc__(self):
        del self.thisptr

    property json:
        def __get__(self):
            return self.thisptr.getJSON().decode('UTF-8')

    def arrays(self):
        v = self.thisptr.getArrays()
        output = []
        cdef vector[Array*].iterator it = v.begin()
        cdef Array* a
        while it != v.end():
            ptr = deref(it)
            a = ptr#.get()
            o = a.getPythonArray()
            output.append(<object>o)
            inc(it)
        return output

    def execute(self):
        if not self.thisptr:
            raise Exception("C++ Pipeline object not constructed!")
        self.thisptr.execute()

