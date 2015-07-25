.. _writing-reader:

===============================================================================
Writing a reader
===============================================================================

PDAL's command-line application can be extended through the development of
reader functions. In this tutorial, we will give a brief example.

The header
-------------------------------------------------------------------------------

First, we provide a full listing of the reader header.

.. literalinclude:: ../../examples/writing-reader/MyReader.hpp
   :language: cpp

In your MyReader class, you will use two macros defined in Reader.hpp to
register some useful information about your reader. SET_STAGE_NAME sets the
reader name and description. These are extracted by the PDAL application to
inform the user of the reader's name and purpose. The name will be displayed
when the user types `pdal --help`.

.. literalinclude:: ../../examples/writing-reader/MyReader.hpp
   :language: cpp
   :lines: 12

SET_STAGE_LINK will provide a link to a webpage that documents the reader.

.. literalinclude:: ../../examples/writing-reader/MyReader.hpp
   :language: cpp
   :lines: 13

The source
-------------------------------------------------------------------------------

Again, we start with a full listing of the reader source.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp

In your reader implementation, you will use a third macro defined in
pdal_macros. This macro registers the plugin with the Reader factory. It is
only required by plugins.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 24

Native readers will use a different set of macros added directly to the
ReaderFactory.cpp file to register themselves.

.. code-block:: cpp

  MAKE_READER_CREATOR(myreader, pdal::MyReader)
  REGISTER_READER(myreader, pdal::MyReader);

To build up a processing pipeline in this example, we need to create two
objects: the PointContext and the StageFactory. The latter is used to create
the various stages that will be used within the reader.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 50-51

The Reader is created from the StageFactory, and is specified by the stage
name, in this case an LAS reader. For brevity, we provide the reader a single
option, the filename of the file to be read.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 53-56

The Filter is also created from the StageFactory.  Here, we create a decimation
filter that will pass every tenth point to subsequent stages. We also specify
the input to this stage, which is the reader.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 58-62

Finally, the Writer is created from the StageFactory. This text writer, takes
as input the previous stage (the decimation filter) and the output filename as
its sole option.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 64-68

The final two steps are to prepare and execute the pipeline. This is achieved
by calling prepare and execute on the final stage.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 69-70
