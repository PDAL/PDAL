.. _writing-filter:

===============================================================================
Writing a filter
===============================================================================

PDAL can be extended through the development of filter functions.

.. seealso::

    For more on filters and their role in PDAL, please refer to
    :ref:`overview`.

Every filter stage in PDAL is implemented as a plugin (sometimes referred to as
a "driver"). Filters native to PDAL, such as :ref:`filters.ferry`, are
implemented as *static* filters and are statically linked into the PDAL
library. Filters that require extra/optional dependencies, or are external to
the core PDAL codebase altogether, such as :ref:`filters.python`, are
implemented as *shared* filters, and are built as individual shared libraries,
discoverable by PDAL at runtime.

In this tutorial, we will give a brief example of a filter, with notes on how
to make it static or shared.


The header
...............................................................................

First, we provide a full listing of the filter header.

.. literalinclude:: ../../examples/writing-filter/MyFilter.hpp
   :language: cpp
   :linenos:

This header should be relatively straightforward, but we will point out one
method that must be declared for the plugin interface to be satisfied.

.. literalinclude:: ../../examples/writing-filter/MyFilter.hpp
   :language: cpp
   :lines: 16

In many instances, you should be able to copy this header template verbatim,
changing only the filter class name, includes, and member functions/variables
as required by your implementation.

The source
...............................................................................

Again, we start with a full listing of the filter source.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :linenos:

For your filter to be available to PDAL at runtime, it must adhere to the PDAL
plugin interface. As a convenience, we provide macros to do just this.

We begin by creating a ``PluginInfo`` struct containing three identifying
elements - the filter name, description, and a link to documentation.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 10-15
   :linenos:

PDAL requires that filter names always begin with ``filters.``, and end with a
string that uniquely identifies the filter. The description will be displayed
to users of the PDAL CLI (``pdal --drivers``).  When making a shared plugin,
the name of the shared library must correspond with the name of the filter
provided here.  The name of the generated shared object must be

::

    libpdal_plugin_filters_<filter name>.<shared library extension>

Next, we pass the following to the ``CREATE_SHARED_STAGE`` macro, passing in
the name of the stage and the ``PluginInfo`` struct.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 17

To create a static stage, we simply change ``CREATE_SHARED_STAGE`` to
``CREATE_STATC_STAGE``.

Finally, we implement a method to get the plugin name, which is primarily used
by the PDAL CLI when using the ``--drivers`` or ``--options`` arguments.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 19
   :linenos:

Now that the filter has implemented the proper plugin interface, we will begin
to implement some methods that actually implement the filter. The ``addArgs()``
method is used to register and bind any provided options to the stage. Here, we
get the value of ``param``, if provided, else we populate ``m_value`` with the
default value of ``1.0``. Option names, descriptions, and default values
specified in ``addArgs()`` will be displayed via the PDAL CLI with the
``--options`` argument.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 21-24
   :linenos:

In ``addDimensions()`` we make sure that the known ``Intensity`` dimension is
registered. We can also add a custom dimension, ``MyDimension``, which will be
populated within ``run()``.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 26-31
   :linenos:

Finally, we define ``run()``, which takes as input a ``PointViewPtr`` and
returns a ``PointViewSet``. It is here that we can transform existing
dimensions, add data to new dimensions, or selectively add/remove individual
points.

We suggest you take a closer look at our existing filters to get an idea of the
power of the ``Filter`` stage and inspiration for your own filters!


Compilation
...............................................................................

Set up a ``CMakeLists.txt`` file to compile your filter against PDAL:

.. literalinclude:: ../../examples/writing-filter/CMakeLists.txt
   :language: cmake
   :linenos:

.. note::

    CMakeLists.txt contents may vary slightly depending on your project
    requirements, operating system, and compilter.

