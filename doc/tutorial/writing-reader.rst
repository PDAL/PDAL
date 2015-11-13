.. _writing-reader:

===============================================================================
Writing a reader
===============================================================================

:Authors: Bradley Chambers, Scott Lewis
:Contact: brad.chambers@gmail.com
:Date: 11/13/2015


PDAL's command-line application can be extended through the development of
reader functions. In this tutorial, we will give a brief example.

The header
-------------------------------------------------------------------------------

First, we provide a full listing of the reader header.

.. literalinclude:: ../../examples/writing-reader/MyReader.hpp
   :language: cpp

In your MyReader class, you will declare the necessary methods and variables
needed to make the reader work and meet the plugin specifications.

.. literalinclude:: ../../examples/writing-reader/MyReader.hpp
   :language: cpp
   :lines: 16-18

These methods are required to fulfill the specs for defining a new plugin.

.. literalinclude:: ../../examples/writing-reader/MyReader.hpp
   :language: cpp
   :lines: 20-21

These methods are used for setting various defaults for the Reader.

.. literalinclude:: ../../examples/writing-reader/MyReader.hpp
   :language: cpp
   :lines: 24-26

``m_stream`` is used to process the input, while ``m_index`` is used to track
the index of the records.  ``m_scale_z`` is specific to MyReader, and will
be described later.

.. literalinclude:: ../../examples/writing-reader/MyReader.hpp
   :language: cpp
   :lines: 28-32

Various other override methods for the stage.  There are a few others that
could be overridden, which will not be discussed in this tutorial.

The source
-------------------------------------------------------------------------------

Again, we start with a full listing of the reader source.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp

In your reader implementation, you will use a macro defined in pdal_macros.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 7-12

This macro registers the plugin with the PDAL code.  In this case, we are
declaring this as a SHARED plugin, meaning that it will be located external
to the main PDAL instalation.  The macro is supplied with a version number
(major and minor), the class of the plugin, the parent class (in this case,
to identify it as a reader), and an object with information.  This information
includes the name of the plugin, a description, and a link to documentation.

Creating STATIC plugins requires a few more steps which will not be covered
in this tutorial.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 16-22

This method will process a set of default options for the reader.  In this
example, we are setting the z_scale value to a default of 1.0, indicating
that the Z values we read should remain as-is.  (In our reader, this could
be changed if, for example, the Z values in the file represented mm values,
and we want to represent them as m in the storage model).

The options will then be processed elsewhere and will be supplied values from
the pipeline configuration, etc.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 24-27

This method takes an Options object and populates the reader's private
variables with values found in the options object.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 29-35

This method registers the various dimensions the reader will use.  In our case,
we are using the X, Y, and Z built-in dimensions, as well as a custom
dimension MyData.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 48-52

This method is called when the Reader is ready for use.  It will only be
called once, regardless of the number of PointViews that are to be
processed.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 55-68

This is a helper function, which will convert a string value into the type
specified when it's called.  In our example, it will be used to convert
strings to doubles when reading from the input stream.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 71-78

This method is the main processing method for the reader.  It takes a
pointer to a Point View which we will build as we read from the file.  We
initialize some variables as well, and then reset the input stream with
the filename used for the reader.  Note that in other readers, the contents
of this method could be very different depending on the format of the file
being read, but this should serve as a good start for how to build the
PointView ojbect.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 80-82

In prepration for reading the file, we prepare to skip some header lines.  In
our case, the header is only a single line.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 83-87

Here we begin our main loop.  In our example file, the first line is a header,
and each line thereafter is a single point.  If the file had a different format
the method of looping and reading would have to change as appropriate.  We make
sure we are skipping the header lines here before moving on.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 91-99

Here we take the line we read in the for block header, split it, and make sure
that we have the proper number of fields.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 101-112

Here we take the values we read and put them into the PointView object.  The
X and Y fields are simply converted from the file and put into the respective
fields.  MyData is done likewise with the custom dimension we defined.  The Z
value is read, and multiplied by the scale_z option (defaulted to 1.0), before
the converted value is put into the field.

When putting the value into the PointView object, we pass in the Dimension
that we are assigning it to, the ID of the point (which is incremented in
each iteration of the loop), and the dimension value.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 111-122

Finally, we increment the nextId.  After the loop is done, we set the index
and number read, and return that value as the number of points read.
This could differ in cases where we read multiple streams, but that won't
be covered here.

.. literalinclude:: ../../examples/writing-reader/MyReader.cpp
   :language: cpp
   :lines: 124-127

When the read method is finished, the done method is called for any cleanup.
In this case, we simply make sure the stream is reset.


Compiling and Usage
-------------------------------------------------------------------------------
The MyReader.cpp code can be compiled.  For this example, we'll use cmake.
Here is the CMakeLists.txt file we will use:

.. literalinclude:: ../../examples/writing-reader/CMakeLists.txt

If this file is in the directory containing MyReader.hpp and MyReader.cpp,
simply run ``cmake .``, followed by ``make``.  This will generate a file called
``libpdal_plugin_reader_myreader.dylib``.

Put this dylib file into the directory pointed to by ``PDAL_DRIVER_PATH``, and
then when you run ``pdal --drivers``, you should see an entry for
readers.myreader.

To test the reader, we will put it into a pipeline and output a text file.

Please download the `pipeline-myreader.xml`_ and `test-reader-input.txt`_ files.

In the directory with those two files, run
``pdal pipeline pipeline-myreader.xml``.  You should have an output file
called ``output.txt``, which will have the same data as in the input file,
except in a CSV style format, and with the Z values scaled by .001.

.. _`pipeline-myreader.xml`: https://github.com/PDAL/PDAL/examples/writing-reader/pipeline-myreader.xml?raw=true
.. _`test-reader-input.txt`:https://github.com/PDAL/PDAL/examples/writing-reader/test-reader-input.txt?raw=true
