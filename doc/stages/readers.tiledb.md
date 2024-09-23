.. _readers.tiledb:

readers.tiledb
==============

Implements `TileDB`_ 2.3.0+ storage.

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
  `TileDB`_ array to read from. Synonymous with `filename`. [Required]

config_file
  `TileDB`_ configuration file [Optional]

chunk_size
  Size of chunks to read from TileDB array [Optional]

stats
  Dump query stats to stdout [Optional]

bbox3d
  TileDB subarray to read in format ([minx, maxx], [miny, maxy], [minz, maxz]) [Optional]

start_timestamp
  Opens the array between a timestamp range of start_timestamp and end_timestamp. Default is 0. [Optional]

end_timestamp
  Opens the array between a timestamp range of start_timestamp and end_timestamp. Default is UINT64_MAX. [Optional]

timestamp
  Synonymous with start_timestamp. [Optional]

strict
  Raise an error if the array contains a TileDB attribute not supported by PDAL, the default is set to true to raise an error for unsupported attribute types [Optional]

.. include:: reader_opts.rst

.. _TileDB: https://tiledb.io
