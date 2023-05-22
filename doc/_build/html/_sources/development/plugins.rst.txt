.. _writing-plugins:

==================================
Writing and building a PDAL Plugin
==================================

:Author: Andrew Bell
:Contact: andrew.bell.ia@gmail.com
:Date: 11/09/2021

A PDAL plugin is a specially-named dynamically linked library that serves as a stage
or a kernel (PDAL command). The PDAL program will
be able to use a properly-made plugin when it is placed in an appropriate location.
PDAL will search the following paths (relative to the current working directory) for
plugins: ``.``, ``./lib``, ``../lib``, ``./bin``, ``../bin``. You can also override the
default search path by setting the environment variable ``PDAL_DRIVER_PATH`` to a list
of directories that pdal should search for plugins.

PDAL stage plugins must be named:

::

    libpdal_plugin_<plugin type>_<plugin name>.<shared library extension>

where ``plugin name`` is one of ``reader``, ``writer`` or ``filter``.

PDAL kernel plugins must be named:

::

    libpdal_plugin_kernel_<plugin name>.<shared library extension>

See the tutorials :ref:`writing-reader`, :ref:`writing-filter` or :ref:`writing-writer` for
step-by-step instructions on creating a PDAL stage plugin. See :ref:`writing-kernel` for similar
information on creating a PDAL kernel plugin.  The tutorials provide a sample
CMakeLists.txt that can serve as a basis for building your plugin with a PDAL installation.
A simple macro, ``PDAL_CREATE_PLUGIN``,  is now provided with PDAL that makes it even easier
to build a plugin. You can use the macro by creating a file called CMakeLists.txt like this:

::

    cmake_minimum_required(VERSION 3.5)
    project(MY_READER LANGUAGES CXX)
    find_package(PDAL REQUIRED)

    set(SRCS MyGoodReader.cpp)

    PDAL_CREATE_PLUGIN(
        TYPE reader
        NAME mygood
        VERSION 1.0
        SOURCES ${SRCS}
    )

Once your plugin is built, copy it to an appropriate location so that it can be found by
PDAL and it should load and run.  If your plugin doesn't load, Use the PDAL `--debug` option
to get information about the plugin loading process.

