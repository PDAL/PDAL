.. _writers.tiledb:

writers.tiledb
==============

Implements `TileDB`_ 1.4.1+ reads from an array.

.. plugin::

.. streamable::

Example
-------

.. code-block:: json

  [
      {
          "type":"readers.las",
          "array_name":"input.las"
      },
      {
          "type":"writers.tiledb",
          "array_name":"output_array"
      }
  ]


Options
-------

array_name
  `TileDB`_ array to write to. [Required]

config_file
  `TileDB`_ configuration file [Optional]

tile_data_capacity
  Number of points per tile [Optional]

x_tile_size
  Tile size (x) in a Cartesian projection [Optional]

y_tile_size
  Tile size (y) in a Cartesian projection [Optional]

z_tile_size
  Tile size (z) in a Cartesian projection [Optional]

chunk_size
  Point cache size for chunked writes [Optional]

compression
  TileDB compression type for attributes, default is None [Optional]

compression_level
  TileDB compression level for chosen compression [Optional]

append
  Append to an existing TileDB array with the same schema [Optional]

stats
  Dump query stats to stdout [Optional]

filters
  JSON array or object of compression filters for either `coords` or `attributes` of the form {coords/attributename : {"compression": name, compression_options: value, ...}} [Optional]

.. include:: writer_opts.rst

By default TileDB will use the following set of compression filters for coordinates and attributes;

.. code-block:: json

  {
      "coords":[
          {"compression": "bit-shuffle"},
          {"compression": "gzip", "compression_level": 9}
      ],
      "Intensity":{"compression": "bzip2", "compression_level": 5},
      "ReturnNumber": {"compression": "zstd", "compression_level": 75},
      "NumberOfReturns": {"compression": "zstd", "compression_level": 75},
      "ScanDirectionFlag": {"compression": "bzip2", "compression_level": 5},
      "EdgeOfFlightLine": {"compression": "bzip2", "compression_level": 5},
      "Classification": {"compression": "gzip", "compression_level": 9},
      "ScanAngleRank": {"compression": "bzip2", "compression_level": 5},
      "UserData": {"compression": "gzip", "compression_level": 9},
      "PointSourceId": {"compression": "bzip2"},
      "Red": {"compression": "rle"},
      "Green": {"compression": "rle"},
      "Blue": {"compression": "rle"},
      "GpsTime": [
          {"compression": "bit-shuffle"},
          {"compression": "zstd", "compression_level": 75}
      ]
  }

.. _TileDB: https://tiledb.io
