.. _writers.tiledb:

writers.tiledb
==============

Implements `TileDB`_ 2.3.0+ reads from an array.

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
          "type":"filters.stats"
      },
      {
          "type":"writers.tiledb",
          "array_name":"output_array"
      }
  ]


Options
-------

array_name
  `TileDB`_ array to write to. Synonymous with `filename`. [Required]

config_file
  `TileDB`_ configuration file. [Optional]

data_tile_capacity
  Number of points per tile. Not used when `append=true`. [Optional]

x_tile_size
  Tile size (x). Floating point value used for determining on-disk data order. Not used when `append=true`. [Optional]

y_tile_size
  Tile size (y). Floating point value used for determining on-disk data order. Not used when `append=true`. [Optional]

z_tile_size
  Tile size (z). Floating point value used for determining on-disk data order. Not used when `append=true`. [Optional]

time_tile_size
  Tile size (time). Not used when `append=true`. [Optional]

x_domain_st
  Domain minimum for x. Not used when `append=true`. [Optional]

x_domain_end
  Domain maximum for x. Not used when `append=true`. [Optional]

y_domain_st
  Domain minimum for y. Not used when `append=true`. [Optional]

y_domain_end
  Domain maximum for y. Not used when `append=true`. [Optional]

z_domain_st
  Domain minimum for z. Not used when `append=true`. [Optional]

z_domain_end
  Domain maximum for z. Not used when `append=true`. [Optional]

time_domain_st
  Domain minimum for GpsTime. Not used when `append=true`. [Optional]

time_domain_end
  Domain maximum for GpsTime. Not used when `append=true`. [Optional]

use_time_dim
  Use GpsTime coordinate data as an array dimension instead of an array attribute. Not used when `append=true`. [Optional]

time_first
  Put the GpsTime dimension first instead of last. Only used when `use_time_dim=true`. Not used when `append=true`. Default is `false`. [Optional]

chunk_size
  Point cache size for chunked writes. [Optional]

compression
  The default TileDB compression filter to use. Only used if the dimension or attribute name is not included in `filters`. Not used when `append=true`. Default is None. [Optional]

compression_level
  The TileDB compression level to use for the default compression. Option is ignored if set to `-1`. Not used when `append=true`. Default is `-1`. [Optional]

append
  Instead of creating a new array, append to an existing array that has the dimensions stored as a TileDB dimension or TileDB attribute. Default is `false`.  [Optional]

stats
  Dump query stats to stdout. [Optional]

filters
  JSON array or object of compression filters for either `dims` or `attributes` of the form {dim/attributename : {"compression": name, compression_options: value, ...}}.  Not used when `append=true`. [Optional]

timestamp
  Sets the TileDB timestamp for this write. [Optional]

.. include:: writer_opts.rst

By default TileDB will use the following set of compression filters for coordinates and attributes;

.. code-block:: json

  {
      "X":{"compression": "zstd", "compression_level": 7},
      "Y":{"compression": "zstd", "compression_level": 7},
      "Z":{"compression": "zstd", "compression_level": 7},
      "Intensity":{"compression": "bzip2", "compression_level": 5},
      "ReturnNumber": {"compression": "zstd", "compression_level": 7},
      "NumberOfReturns": {"compression": "zstd", "compression_level": 7},
      "ScanDirectionFlag": {"compression": "bzip2", "compression_level": 5},
      "EdgeOfFlightLine": {"compression": "bzip2", "compression_level": 5},
      "Classification": {"compression": "gzip", "compression_level": 9},
      "ScanAngleRank": {"compression": "bzip2", "compression_level": 5},
      "UserData": {"compression": "gzip", "compression_level": 9},
      "PointSourceId": {"compression": "bzip2"},
      "Red": {"compression": "zstd", "compression_level": 7},
      "Green": {{"compression": "zstd", "compression_level": 7},
      "Blue": {{"compression": "zstd", "compression_level": 7},
      "GpsTime": {"compression": "zstd", "compression_level": 7}
  }

.. _TileDB: https://tiledb.io
