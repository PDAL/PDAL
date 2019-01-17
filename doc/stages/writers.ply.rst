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

    {
      "pipeline":[
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
    }


Options
-------

filename
  ply file to write [Required]

storage_mode
  Type of ply file to write. Valid values are 'ascii', 'little endian',
  'big endian', and 'default'. 'default' is binary output in the endianness
  of the machine. [Default: 'ascii']

dims
  List of dimensions to write as elements. [Default: all dimensions]

faces
  Write a mesh as faces in addition to writing points as vertices.
  [Default: false]

precision
  If specified, the number of digits to the right of the decimal place
  using f-style formatting.  Only permitted when 'storage_mode' is 'ascii'.
  See the `printf`_ reference for more information.
  [Default: g-style formatting (variable precision)]

.. _polygon file format: http://paulbourke.net/dataformats/ply/
.. _printf: https://en.cppreference.com/w/cpp/io/c/fprintf
