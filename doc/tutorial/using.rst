.. _using:

===============================================================================
Using PDAL with CMake
===============================================================================

This tutorial will explain how to use PDAL in your own projects using CMake. A
more complete, working example can be found :ref:`here <writing>`.

.. note::

   We assume you have either :ref:`built or installed<building>` PDAL.

Basic CMake configuration
-------------------------------------------------------------------------------

Begin by creating a file named CMakeLists.txt that contains:

.. code-block:: cmake

  cmake_minimum_required(VERSION 2.8)
  project(MY_PDAL_PROJECT)
  find_package(PDAL 1.0.0 REQUIRED CONFIG)
  include_directories(${PDAL_INCLUDE_DIRS})
  link_directories(${PDAL_LIBRARY_DIRS})
  add_definitions(${PDAL_DEFINITIONS})
  set(CMAKE_CXX_FLAGS "-std=c++11")
  add_executable(tutorial tutorial.cpp)
  target_link_libraries(tutorial ${PDAL_LIBRARIES})

CMakeLists explained
-------------------------------------------------------------------------------

.. code-block:: cmake

  cmake_minimum_required(VERSION 2.8.12)

The `cmake_minimum_required` command specifies the minimum required version of
CMake. We use some recent additions to CMake in PDAL that require version
2.8.12.

.. code-block:: cmake

  project(MY_PDAL_PROJECT)

The CMake `project` command names your project and sets a number of useful
CMake variables.

.. code-block:: cmake

  find_package(PDAL 1.0.0 REQUIRED CONFIG)

We next ask CMake to locate the PDAL package, requiring version 1.0.0 or higher.

.. code-block:: cmake

  include_directories(${PDAL_INCLUDE_DIRS})
  link_directories(${PDAL_LIBRARY_DIRS})
  add_definitions(${PDAL_DEFINITIONS})

If PDAL is found, the following variables will be set:

* *PDAL_FOUND*: set to 1 if PDAL is found, otherwise unset
* *PDAL_INCLUDE_DIRS*: set to the paths to PDAL installed headers and the dependency headers
* *PDAL_LIBRARIES*: set to the file names of the built and installed PDAL libraries
* *PDAL_LIBRARY_DIRS*: set to the paths where PDAL libraries and 3rd party dependencies reside
* *PDAL_VERSION*: the detected version of PDAL
* *PDAL_DEFINITIONS*: list the needed preprocessor definitions and compiler flags

.. code-block:: cmake

  set(CMAKE_CXX_FLAGS "-std=c++11")

We haven't quite implemented the setting of *PDAL_DEFINITIONS* within the
`PDALConfig.cmake` file, so for now you should specify the c++11 compiler flag,
as we use it extensively throughout PDAL.

.. code-block:: cmake

  add_executable(tutorial tutorial.cpp)

We use the `add_executable` command to tell CMake to create an executable named
`tutorial` from the source file `tutorial.cpp`.

.. code-block:: cmake

  target_link_libraries(tutorial ${PDAL_LIBRARIES})

We assume that the tutorial executable makes calls to PDAL functions. To make
the linker aware of the PDAL libraries, we use `target_link_libraries` to link
`tutorial` against the *PDAL_LIBRARIES*.

Compiling the project
-------------------------------------------------------------------------------

Make a `build` directory, where compilation will occur:

.. code-block:: bash

  $ cd /PATH/TO/MY/PDAL/PROJECT
  $ mkdir build

Run cmake from within the build directory:

.. code-block:: bash

  $ cd build
  $ cmake ..

Now, build the project:

.. code-block:: bash

  $ make

The project is now built and ready to run:

.. code-block:: bash

  $ ./tutorial
