.. _readers.memoryview:

readers.memoryview
==================

The memoryview reader is a special stage that allows
the reading of point data arranged in rows directly from memory --
each point needs to have dimension data arranged at a fixed offset
from a base address of the point.
Before each point is read, the memoryview reader calls a function that
should return the point's base address, or a null pointer if there are no
points to be read.

Note that the memoryview reader does not currently work with columnar
data (data where individual dimensions are packed into arrays).

Usage
=====

The memoryview reader cannot be used from the command-line.  It is for use
by software using the PDAL API.

After creating an instance of the memoryview reader, the user should
call pushField() for every dimension that should be read from memory.
pushField() takes a single argument, a MemoryViewReader::Field, that consists
of a dimension name, a type and an offset from the point base address:

.. code-block:: c++

    struct Field
    {
        std::string m_name;
        Dimension::Type m_type;
        size_t m_offset;
    };

    void pushField(const Field&);

The user should also call setIncrementer(), a function that takes a
single argument, a std::function that receives the ID of the point to
be added and should return the base address of the point data, or a
null pointer if there are no more points to be read.

.. code-block:: c++

    using PointIncrementer = std::function<char *(PointId)>;

    void setIncrementer(PointIncrementer inc);


Options
-------

None.
