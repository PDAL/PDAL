.. _writing-filter:

===============================================================================
Writing a filter
===============================================================================

PDAL's command-line application can be extended through the development of
kernel functions. In this tutorial, we will give a brief example.

The header
-------------------------------------------------------------------------------

First, we provide a full listing of the filter header.

.. literalinclude:: ../../examples/writing-filter/MyFilter.hpp
   :language: cpp

In your MyFilter class, you will use two macros defined in Filter.hpp to
register some useful information about your filter. SET_STAGE_NAME sets the
filter name and description. These are extracted by the PDAL application to
inform the user of the filter's name and purpose. The name will be displayed
when the user types `pdal --help`.

.. literalinclude:: ../../examples/writing-filter/MyFilter.hpp
   :language: cpp
   :lines: 25

SET_STAGE_LINK will provide a link to a webpage that documents the filter.

.. literalinclude:: ../../examples/writing-filter/MyFilter.hpp
   :language: cpp
   :lines: 26

Plugin filters should also SET_PLUGIN_VERSION to register the version number of
the plugin.

.. literalinclude:: ../../examples/writing-filter/MyFilter.hpp
   :language: cpp
   :lines: 27

The source
-------------------------------------------------------------------------------

Again, we start with a full listing of the filter source.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp

In your filter implementation, you will use a third macro defined in
pdal_macros. This macro registers the plugin with the Filter factory. It is
only required by plugins.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 11

Native filters will use a different set of macros added directly to the
StageFactory.cpp file to register themselves.

.. code-block:: cpp

  MAKE_FILTER_CREATOR(myfilter, pdal::MyFilter)
  REGISTER_FILTER(myfilter, pdal::MyFilter);

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 13-16

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 18-21

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 23-28

