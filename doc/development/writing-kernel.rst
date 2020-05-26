.. _writing-kernel:

===============================================================================
Writing a kernel
===============================================================================

:Author: Bradley Chambers
:Contact: brad.chambers@gmail.com
:Date: 11/02/2017


PDAL's command-line application can be extended through the development of
kernel functions. In this tutorial, we will give a brief example.

The header
-------------------------------------------------------------------------------

First, we provide a full listing of the kernel header.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.hpp
   :language: cpp
   :linenos:

As with other plugins, the MyKernel class needs to return a name.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.hpp
   :language: cpp
   :lines: 17


The source
-------------------------------------------------------------------------------

Again, we start with a full listing of the kernel source.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :linenos:

In your kernel implementation, you will use a macro defined in pdal_macros.
This macro registers the plugin with the PluginManager.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :lines: 23

To build up a processing pipeline in this example, we need to create two
objects: the :cpp:class:`pdal::PointTable`.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :lines: 35-53

To implement the actual kernel logic we implement execute().  In this case,
the kernel reads a las file, decimates the data (eliminates some points) and
writes the result to a text file.  The base kernel class provides functions
(makeReader, makeFilter, makeWriter) to create stages with options as desired.
The pipeline that has been created can be run by preparing and executing the
last stage in the pipeline.


When compiled, a dynamic library file will be created; in this case,
``libpdal_plugin_kernel_mykernel.dylib``

Put this file in whatever directory ``PDAL_DRIVER_PATH`` is pointing to.  Then,
if you run ``pdal --drivers``, you should see ``mykernel`` listed in the
possible commands.

To run this kernel, you would use ``pdal mykernel -i <input las file> -o
<output text file>``.

Compilation
...............................................................................

Set up a ``CMakeLists.txt`` file to compile your kernel against PDAL:

.. literalinclude:: ../../examples/writing-kernel/CMakeLists.txt
   :language: cmake
   :linenos:




