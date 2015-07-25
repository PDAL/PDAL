.. _writing-writer:

===============================================================================
Writing a writer
===============================================================================

PDAL's command-line application can be extended through the development of
writer functions. In this tutorial, we will give a brief example.

The header
-------------------------------------------------------------------------------

First, we provide a full listing of the writer header.

.. literalinclude:: ../../examples/writing-writer/MyWriter.hpp
   :language: cpp

In your MyWriter class, you will use two macros defined in Writer.hpp to
register some useful information about your writer. SET_STAGE_NAME sets the
writer name and description. These are extracted by the PDAL application to
inform the user of the writer's name and purpose. The name will be displayed
when the user types `pdal --help`.

.. literalinclude:: ../../examples/writing-writer/MyWriter.hpp
   :language: cpp
   :lines: 12

SET_STAGE_LINK will provide a link to a webpage that documents the writer.

.. literalinclude:: ../../examples/writing-writer/MyWriter.hpp
   :language: cpp
   :lines: 13

The source
-------------------------------------------------------------------------------

Again, we start with a full listing of the writer source.

.. literalinclude:: ../../examples/writing-writer/MyWriter.cpp
   :language: cpp

In your writer implementation, you will use a third macro defined in
pdal_macros. This macro registers the plugin with the Writer factory. It is
only required by plugins.

.. literalinclude:: ../../examples/writing-writer/MyWriter.cpp
   :language: cpp
   :lines: 25

Native writers will use a different set of macros added directly to the
WriterFactory.cpp file to register themselves.

.. code-block:: cpp

  MAKE_WRITER_CREATOR(mywriter, pdal::MyWriter)
  REGISTER_WRITER(mywriter, pdal::MyWriter);

To build up a processing pipeline in this
example, we need to create two objects: the
PointContext and the StageFactory. The latter is
used to create the various stages that will be
used within the writer.

.. literalinclude:: ../../examples/writing-writer/MyWriter.cpp
   :language: cpp
   :lines: 50-51

The Reader is created from the StageFactory, and
is specified by the stage name, in this case an
LAS reader. For brevity, we provide the reader a
single option, the filename of the file to be
read.

.. literalinclude:: ../../examples/writing-writer/MyWriter.cpp
   :language: cpp
   :lines: 53-56

The Filter is also created from the StageFactory.
Here, we create a decimation filter that will
pass every tenth point to subsequent stages. We
also specify the input to this stage, which is
the reader.

.. literalinclude:: ../../examples/writing-writer/MyWriter.cpp
   :language: cpp
   :lines: 58-62

Finally, the Writer is created from the
StageFactory. This text writer, takes as input
the previous stage (the decimation filter) and
the output filename as its sole option.

.. literalinclude:: ../../examples/writing-writer/MyWriter.cpp
   :language: cpp
   :lines: 64-68

The final two steps are to prepare and execute
the pipeline. This is achieved by calling prepare
and execute on the final stage.

.. literalinclude:: ../../examples/writing-writer/MyWriter.cpp
   :language: cpp
   :lines: 69-70
