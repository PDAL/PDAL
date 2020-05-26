.. _readers.e57:

readers.e57
===========

The **E57 Reader** supports reading from E57 files.

The reader supports E57 files with Cartesian point clouds.

.. note::

   E57 files can contain multiple point clouds stored in a single
   file.  If that is the case, the reader will read all the points
   from all of the internal point clouds as one.

   Only dimensions present in all of the point clouds will be read.

.. note::

   Point clouds stored in spherical format are not supported.

.. note::

   The E57 `cartesianInvalidState` dimension is mapped to the Omit
   PDAL dimension.  A range filter can be used to filter out the
   invalid points.
   
.. plugin::

.. streamable::


Example 1
---------

.. code-block:: json

  [
      {
          "type":"readers.e57",
          "filename":"inputfile.e57"
      },
      {
          "type":"writers.text",
          "filename":"outputfile.txt"
      }
  ]


Example 2
---------
  
.. code-block:: json

  [
      {
          "type":"readers.e57",
          "filename":"inputfile.e57"
      },
      {
          "type":"filters.range",
          "limits":"Omit[0:0]"
      },
      {
          "type":"writers.text",
          "filename":"outputfile.txt"
      }
  ]

  
Options
-------

_`filename`
  E57 file to read [Required]

.. include:: reader_opts.rst
