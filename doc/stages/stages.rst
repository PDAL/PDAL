.. _stages:

Stages
=======

The stages of a PDAL :ref:`pipeline` are divided into :ref:`readers`, :ref:`filters`
and :ref:`writers`. Stages may support :ref:`streaming mode <processing_modes>` or
not, depending on
their functionality or particular implementation.  Many stages are built into the
base PDAL library (the file pdalcpp.so on Unix, pdalcpp.dylib on OSX and pdalcpp.dll
on Windows).  PDAL can also load stages that have been built separately. These stages
are called plugins.

Stages are usually created as plugins for one of several reasons. First, a user may wish
to create a stage for their own purposes. In this case a user has no need to build
their stage into the PDAL library itself. Second, a stage may depend on some third-party
library that cannot be distributed with PDAL.  Providing the stage as a plugin eliminates
the direct dependency on a library and can simplify licensing issues.  Third, a stage may
be little used and its addition would unnecessarily increase the size of the PDAL library.

PDAL will automatically load plugins when necessary. PDAL plugins have a specific naming
pattern:

::

  libpdal_plugin_<plugin type>_<plugin name>.<shared library extension>

Where <plugin type> is "reader", "writer" or "filter" and <shared library extension> is
".dll" on Windows, ".dylib" on OSX and ".so" on UNIX systems.

The <plugin name> must start with a letter or number, which can be followed by letters,
numbers, or an underscore ('_').

PDAL looks for plugins in the directory that contains the PDAL library itself, as well
as the directories ``.``, ``./lib``, ``../lib``, ``./bin``, ``../bin``. Those paths
are relative to the current working directory.  These locations can be overridden by
setting the environment variable ``PDAL_DRIVER_PATH`` to a list of directories delimited
by ``;`` on Windows and ``:`` on other platforms.

You can use ``pdal --drivers`` to show stages that PDAL is able to load.  Verify the above
if you are having trouble loading specific plugins.
