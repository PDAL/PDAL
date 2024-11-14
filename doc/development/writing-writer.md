---
Authors: Bradley Chambers, Scott Lewis
Contact: <mailto:brad.chambers@gmail.com>
Date: 10/26/2016
---

(writing-writer)=

# Writing a writer

PDAL's command-line application can be extended through the development of
writer functions. In this tutorial, we will give a brief example.

## The header

First, we provide a full listing of the writer header.

```{literalinclude} ../../examples/writing-writer/MyWriter.hpp
:language: cpp
:linenos: true
```

In your MyWriter class, you will declare the necessary methods and variables
needed to make the writer work and meet the plugin specifications.

```{literalinclude} ../../examples/writing-writer/MyWriter.hpp
:language: cpp
:linenos: true
:lines: 11
```

FileStreamPtr is defined to make the declaration of the stream easier to manage
later on.

```{literalinclude} ../../examples/writing-writer/MyWriter.hpp
:language: cpp
:lines: 19
```

Every stage must return a unique name.

```{literalinclude} ../../examples/writing-writer/MyWriter.hpp
:language: cpp
:lines: 22-26
```

These methods are used during various phases of the pipeline.  There are also
more methods, which will not be covered in this tutorial.

```{literalinclude} ../../examples/writing-writer/MyWriter.hpp
:language: cpp
:lines: 28-34
```

These are variables our Writer will use, such as the file to write to, the
newline character to use, the name of the data field to use to write the MyData
field, precision of the double outputs, the output stream, and the dimension
that corresponds to the data field for easier lookup.

As mentioned, there cen be additional configurations done as needed.

## The source

We will start with a full listing of the writer source.

```{literalinclude} ../../examples/writing-writer/MyWriter.cpp
:language: cpp
:linenos: true
```

In the writer implementation, we will use a macro defined in pdal_macros,
which is included in the include chain we are using.

```{literalinclude} ../../examples/writing-writer/MyWriter.cpp
:language: cpp
:lines: 9-16
```

Here we define a struct with information regarding the writer, such as the
name, a description, and a path to documentation.  We then use the macro
to create a SHARED stage, which means it will be external to the main PDAL
installation.  When using the macro, we specify the name of the Stage and
the PluginInfo struct we defined earlier.

When making a shared plugin, the name of the shared library must
correspond with the name of the writer
provided here.  The name of the generated shared object must be

```
libpdal_plugin_writer_<writer name>.<shared library extension>
```

```{literalinclude} ../../examples/writing-writer/MyWriter.cpp
:language: cpp
:linenos: true
:lines: 20-31
```

This struct is used for helping with the FileStreamPtr for cleanup.

```{literalinclude} ../../examples/writing-writer/MyWriter.cpp
:language: cpp
:linenos: true
:lines: 33-40
```

This method defines the arguments the writer provides and binds them to
private variables.

```{literalinclude} ../../examples/writing-writer/MyWriter.cpp
:language: cpp
:lines: 42-53
```

This method initializes our file stream in preparation for writing.

```{literalinclude} ../../examples/writing-writer/MyWriter.cpp
:language: cpp
:linenos: true
:lines: 56-72
```

The ready method is used to prepare the writer for any number of PointViews that
may be passed in.  In this case, we are setting the precision for our double
writes, looking up the dimension specified as the one to write into MyData,
and writing the header of the output file.

```{literalinclude} ../../examples/writing-writer/MyWriter.cpp
:language: cpp
:linenos: true
:lines: 75-91
```

This method is the main method for writing.  In our case, we are writing a very
simple file, with data in the format of X:Y:Z:MyData.  We loop through each
index in the PointView, and for each one we take the X, Y, and Z values, as well
as the value for the specified MyData dimension, and write this to the output
file.   In particular, note the reading of MyData; in our case, MyData is an
integer, but the field we are reading might be a double.  Converting from double
to integer is done via truncation, not rounding, so by adding .5 before making
the conversion will ensure rounding is done properly.

Note that in this case, the output format is pretty simple.  For more complex
outputs, you may need to generate helper methods (and possibly helper classes)
to help generate the proper output.  The key is reading in the appropriate
values from the PointView, and then writing those in whatever necessary format
to the output stream.

```{literalinclude} ../../examples/writing-writer/MyWriter.cpp
:language: cpp
:linenos: true
:lines: 94-97
```

This method is called when the writing is done.  In this case, it simply cleans
up the output stream by resetting it.

## Other fields

In addition to the X, Y, Z coordinates of a point, PDAL can write many other attributes. Their full
list and types are in [Dimension.json]. Most of these are not enabled by default. To enable them, the
LAS file format minor version should be set to 4, the value of the `extra_dims` writer option should be `all`,
and the attributes should be registered with the function ``registerDim()``.

## Compiling and usage

To compile this reader, we will use cmake.  Here is the CMakeLists.txt file we
will use for this process:

```{literalinclude} ../../examples/writing-writer/CMakeLists.txt
:linenos: true
```

If this file is in the directory with the MyWriter.hpp and MyWriter.cpp files,
simply run `cmake .` followed by `make`.  This will generate a file called
`libpdal_plugin_writer_mywriter.dylib`.

Put this dylib file into the directory pointed to by `PDAL_DRIVER_PATH`, and
then when you run `pdal --drivers`, you will see an entry for
writers.mywriter.

To test the writer, we will put it into a pipeline and read in a LAS file and
covert it to our output format.  For this example, use [interesting.las], and
run it through [pipeline-mywriter.json].

If those files are in the same directory, you would just run the command
`pdal pipeline pipeline-mywriter.json`, and it will generate an output file
called output.txt, which will be in the proper format.  From there, if you
wanted, you could run that output file through the MyReader that was created
in the previous tutorial, as well.

=======
## See also

The [LAS tutorial] has a high-level overview of how reading and writing a LAS file works.

[interesting.las]: https://github.com/PDAL/PDAL/raw/refs/heads/master/test/data/las/interesting.las
[pipeline-mywriter.json]: https://github.com/PDAL/PDAL/blob/master/examples/writing-writer/pipeline-mywriter.json?raw=true
[Dimension.json]: https://github.com/PDAL/PDAL/blob/master/pdal/Dimension.json
[LAS tutorial]: https://pdal.io/en/latest/tutorial/las.html
