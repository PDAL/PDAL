.. _writing:

=====================
Writing with PDAL
=====================

This tutorial will describe a complete example of using PDAL C++ objects to write
a LAS file. The example will show fetching data from your own data source rather than
interacting with a :ref:`reader stage <stage_index>`.

.. note::

     If you implement your own :ref:`reader <stage_index>` that conforms to
     PDAL's :cpp:class:`pdal::Stage`, you can implement a simple read-filter-write
     pipeline using :ref:`pipeline` and not have to code anything explicit
     yourself.

Includes
-------------------------------------------------------------------------------

First, our code.

.. literalinclude:: ../../examples/writing/tutorial.cpp
   :language: cpp

Take a closer look. We will need to include several PDAL headers.

.. literalinclude:: ../../examples/writing/tutorial.cpp
   :language: cpp
   :lines: 1-8

`BufferReader` will not be required by all users. Here is it used to populate a
bare `PointBuffer`. This will often be accomplished by a `Reader` stage.

Instead of directly including headers for individual stages, e.g., `LasWriter`,
we rely on the `StageFactory` which has the ability to query available stages
at runtime and return pointers to the created stages.

We proceed by providing a mechanism for generating dummy data for the x, y, and
z dimensions.

.. literalinclude:: ../../examples/writing/tutorial.cpp
   :language: cpp
   :lines: 10-30

.. literalinclude:: ../../examples/writing/tutorial.cpp
   :language: cpp
   :lines: 32-41

.. literalinclude:: ../../examples/writing/tutorial.cpp
   :language: cpp
   :lines: 43-70

Compiling and running the program
-------------------------------------------------------------------------------

.. note::

  Refer to :ref:`building` for information on how to build PDAL.

To build this example, simply copy the files tutorial.cpp and CMakeLists.txt
from the examples/writing directory of the PDAL source tree.

.. literalinclude:: ../../examples/writing/CMakeLists.txt
   :language: cmake

.. note::

  Refer to :ref:`using` for an explanation of the basic CMakeLists.

Begin by configuring your project using CMake (shown here on Unix) and building
using make.

.. code-block:: bash

  $ cd /PATH/TO/WRITING/TUTORIAL
  $ mkdir build
  $ cd build
  $ cmake ..
  $ make

After the project is built, you can run it by typing:

.. code-block:: bash

  $ ./tutorial


