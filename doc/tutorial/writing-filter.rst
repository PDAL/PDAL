.. _writing-filter:

===============================================================================
Writing a filter
===============================================================================

:Author: Bradley Chambers
:Contact: brad.chambers@gmail.com
:Date: 11/11/2015


PDAL can be extended through the development of filter functions.

.. seealso::

    For more on filters and their role in PDAL, please refer to :ref:`the-pdal-model` and :ref:`implementing-a-filter`.

Every filter stage in PDAL is implemented as a plugin (sometimes referred to as a "driver"). Filters native to PDAL, such as :ref:`filters.ferry`, are implemented as _static_ filters and are statically linked into the PDAL library. Filters that require extra/optional dependencies, or are external to the core PDAL codebase altogether, such as :ref:`filters.ground`, are implemented as _shared_ filters, and are built as individual shared libraries, discoverable by PDAL at runtime.

In this tutorial, we will give a brief example of a filter, with notes on how to make it static or shared.


The header
...............................................................................

First, we provide a full listing of the filter header.

.. literalinclude:: ../../examples/writing-filter/MyFilter.hpp
   :language: cpp

This header should be relatively straightforward, but we will point out three methods that must be declared for the plugin interface to be satisfied.

.. literalinclude:: ../../examples/writing-filter/MyFilter.hpp
   :language: cpp
   :lines: 23-25

In many instances, you should be able to copy this header template verbatim, changing only the filter class name, includes, and member functions/variables as required by your implementation.

The source
...............................................................................

Again, we start with a full listing of the filter source.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp

For your filter to be available to PDAL at runtime, it must adhere to the PDAL plugin interface. As a convenience, we provide the macros in ``pdal_macros.hpp`` to do just this.

We begin by creating a ``PluginInfo`` struct containing three identifying elements - the filter name, description, and a link to documentation.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 14-16

PDAL requires that filter names always begin with ``filters.``, and end with a string that uniquely identifies the filter. The description will be displayed to users of the PDAL CLI (``pdal --drivers``).

Next, we pass the following to the ``CREATE_STATIC_PLUGIN`` macro, in order: PDAL plugin ABI major version, PDAL plugin ABI minor version, filter class name, stage type (``Filter``), and our ``PluginInfo`` struct.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 18

To create a shared plugin, we simply change ``CREATE_STATIC_PLUGIN`` to ``CREATE_SHARED_PLUGIN``.

Finally, we implement a method to get the plugin name, which is primarily used by the PDAL CLI when using the ``--drivers`` or ``--options`` arguments.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 20-23

Now that the filter has implemented the proper plugin interface, we will begin to implement some methods that actually implement the filter. First, ``getDefaultOptions()`` is used to advertise those options that the filter provides. Within PDAL, this is primarily used as a means of displaying options via the PDAL CLI with the ``--options`` argument. It provides the user with the option names, descriptions, and default values.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 25-30

The ``processOptions()`` method is used to parse any provided options. Here, we get the value of ``param``, if provided, else we populate ``m_value`` with the default value of ``1.0``.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 32-35

In ``addDimensions()`` we make sure that the known ``Intensity`` dimension is registered. We can also add a custom dimension, ``MyDimension``, which will be populated within ``run()``.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 37-41

Finally, we define ``run()``, which takes as input a ``PointViewPtr`` and returns a ``PointViewSet``. It is here that we can transform existing dimensions, add data to new dimensions, or selectively add/remove individual points.

.. literalinclude:: ../../examples/writing-filter/MyFilter.cpp
   :language: cpp
   :lines: 43-48

We suggest you take a closer look at our existing filters to get an idea of the power of the ``Filter`` stage and inspiration for your own filters!

StageFactory
...............................................................................

As of this writing, users must also make a couple of changes to ``StageFactory.cpp`` to properly register static plugins only (this is not required for shared plugins). It is our goal to eventually remove this requirement to further streamline development of add-on plugins.

First, add the following line to the beginning of ``StageFactory.cpp`` (adjusting the path and filename as necessary).

.. code-block:: cpp

    #include <MyFilter.hpp>

Next, add the following line of code to the ``StageFactory`` constructor.

.. code-block:: cpp

    PluginManager::initializePlugin(MyFilter_InitPlugin);
