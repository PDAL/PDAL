.. _readers.tiledb:

readers.tiledb
==============

Implements `TileDB`_ 1.4.1+ storage.

.. plugin::

.. streamable::

Example
-------

.. code-block:: json

  [
      {
        "type":"readers.tiledb",
        "array_name":"my_array"
      },
      {
        "type":"writers.las",
        "filename":"outputfile.las"
      }
  ]


Options
-------

array_name
  `TileDB`_ array to read from. [Required]

config_file
  `TileDB`_ configuration file [Optional]

chunk_size
  Size of chunks to read from TileDB array [Optional]

stats
  Dump query stats to stdout [Optional]

bbox3d
  TileDB subarray to read in format ([minx, maxx], [miny, maxy], [minz, maxz]) [Optional]

.. include:: reader_opts.rst

.. _TileDB: https://tiledb.io
