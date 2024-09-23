.. _writers.matlab:

writers.matlab
==============

The **Matlab Writer** supports writing Matlab `.mat` files.

The produced files has a single variable, `PDAL`, an array struct.

.. image:: ./writers.matlab.png

.. note::

    The Matlab writer requires the Mat-File API from MathWorks, and
    it must be explicitly enabled at compile time with the
    ``BUILD_PLUGIN_MATLAB=ON`` variable

.. plugin::

Example
-------

.. code-block:: json

  [
      {
          "type":"readers.las",
          "filename":"inputfile.las"
      },
      {
          "type":"writers.matlab",
          "output_dims":"X,Y,Z,Intensity",
          "filename":"outputfile.mat"
      }
  ]

Options
-------

filename
  Output file name [Required]

output_dims
  A comma-separated list of dimensions to include in the output file.
  May also be specified as an array of strings. [Default: all available
  dimensions]

struct
  Array structure name to read [Default: "PDAL"]

.. include:: writer_opts.rst

