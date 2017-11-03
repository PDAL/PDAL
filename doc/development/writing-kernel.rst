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

As with other plugins, the MyKernel class needs to have the following three
methods declared for the plugin interface to be satisfied:

.. literalinclude:: ../../examples/writing-kernel/MyKernel.hpp
   :language: cpp
   :lines: 16-18


The source
-------------------------------------------------------------------------------

Again, we start with a full listing of the kernel source.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :linenos:

In your kernel implementation, you will use a macro defined in pdal_macros.
This macro registers the plugin with the Kernel factory. It is
only required by plugins.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :lines: 25-26

.. note::

    A static plugin macro can also be used to integrate the kernel with the
    main code.  This will not be described here.  Using this as a shared plugin
    will be described later.

To build up a processing pipeline in this example, we need to create two
objects: the :cpp:class:`pdal::PointTable` and the
:cpp:class:`pdal::StageFactory`. The latter is used to create the various
stages that will be used within the kernel.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :lines: 39-40

The :cpp:class:`pdal::Reader` is created from the
:cpp:class:`pdal::StageFactory`, and is specified by the stage name, in this
case an LAS reader. For brevity, we provide the reader a single option, the
filename of the file to be read.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :lines: 42-45

The :cpp:class:`pdal::Filter` is also created from the
:cpp:class:`pdal::StageFactory`.  Here, we create a decimation filter that will
pass every tenth point to subsequent stages. We also specify the input to this
stage, which is the reader.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :lines: 47-51

Finally, the :cpp:class:`pdal::Writer` is created from the
:cpp:class:`pdal::StageFactory`. This :ref:`writers.text`, takes as input the previous
stage (the :ref:`filters.decimation`) and the output filename as its sole option.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :lines: 53-57

The final two steps are to prepare and execute the pipeline. This is achieved
by calling prepare and execute on the final stage.

.. literalinclude:: ../../examples/writing-kernel/MyKernel.cpp
   :language: cpp
   :lines: 58-59

When compiled, a dynamic library file will be created; in this case,
``libpdal_plugin_kernel_mykernel.dylib``

Put this file in whatever directory ``PDAL_DRIVER_PATH`` is pointing to.  Then,
if you run ``pdal --help``, you should see ``mykernel`` listed in the possible
commands.

To run this kernel, you would use ``pdal mykernel -i <input las file> -o
<output text file>``.

Compilation
...............................................................................

Set up a ``CMakeLists.txt`` file to compile your kernel against PDAL:

.. literalinclude:: ../../examples/writing-kernel/CMakeLists.txt
   :language: cmake
   :linenos:




