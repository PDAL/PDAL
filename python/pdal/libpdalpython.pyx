# distutils: language = c++

from libcpp.vector cimport vector
from libcpp.string cimport string
from libc.stdint cimport uint32_t, int64_t
from libcpp cimport bool
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
        int64_t execute() except +
        bool validate() except +
        string getPipeline() except +
        string getMetadata() except +
        string getSchema() except +
        string getLog() except +
        vector[Array*] getArrays() except +
        int getLogLevel()
        void setLogLevel(int)

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

    property pipeline:
        def __get__(self):
            return self.thisptr.getPipeline().decode('UTF-8')

    property metadata:
        def __get__(self):
            return self.thisptr.getMetadata().decode('UTF-8')

    property loglevel:
        def __get__(self):
            return self.thisptr.getLogLevel()
        def __set__(self, v):
            self.thisptr.setLogLevel(v)

    property log:
        def __get__(self):

            return self.thisptr.getLog().decode('UTF-8')

    property schema:
        def __get__(self):
            import json

            j = self.thisptr.getSchema().decode('UTF-8')
            return json.loads(j)

    property arrays:
        def __get__(self):
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
        return self.thisptr.execute()

    def validate(self):
        if not self.thisptr:
            raise Exception("C++ Pipeline object not constructed!")
        return self.thisptr.validate()
