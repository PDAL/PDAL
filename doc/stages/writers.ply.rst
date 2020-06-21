.. _writers.ply:

writers.ply
===========

The **ply writer** writes the `polygon file format`_, a common file format
for storing three dimensional models.  The writer emits points as PLY vertices.
The writer can also emit a mesh as a set of faces.
:ref:`filters.greedyprojection` and :ref:`filters.poisson` create a
mesh suitable for output as faces.

.. embed::

Example
-------


.. code-block:: json

  [
      {
          "type":"readers.pcd",
          "filename":"inputfile.pcd"
      },
      {
          "type":"writers.ply",
          "storage_mode":"little endian",
          "filename":"outputfile.ply"
      }
  ]


Options
-------

filename
  ply file to write [Required]

storage_mode
  Type of ply file to write. Valid values are 'ascii', 'little endian',
  'big endian'.  [Default: "ascii"]

dims
  List of dimensions (and :ref:`types`) in the format
  ``<dimension_name>[=<type>] [,...]`` to write as output.
  (e.g., "Y=int32_t, X,Red=char")
  [Default: All dimensions with stored types]

faces
  Write a mesh as faces in addition to writing points as vertices.
  [Default: false]

sized_types
  PLY has variously been written with explicitly sized type strings
  ('int8', 'float32", 'uint32', etc.) and implied sized type strings
  ('char', 'float', 'int', etc.).  If true, explicitly sized type strings
  are used.  If false, implicitly sized type strings are used.
  [Default: true]

precision
  If specified, the number of digits to the right of the decimal place
  using f-style formatting.  Only permitted when 'storage_mode' is 'ascii'.
  See the `printf`_ reference for more information.
  [Default: g-style formatting (variable precision)]

.. _polygon file format: http://paulbourke.net/dataformats/ply/
.. _printf: https://en.cppreference.com/w/cpp/io/c/fprintf
