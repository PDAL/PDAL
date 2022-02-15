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
  `TileDB`_ array to write to. [Required]

config_file
  `TileDB`_ configuration file [Optional]

data_tile_capacity
  Number of points per tile [Optional]

x_tile_size
  Tile size (x) [Optional]

y_tile_size
  Tile size (y) [Optional]

z_tile_size
  Tile size (z) [Optional]
  
time_tile_size  
  Tile size (time) [Optional]

x_domain_st
  Domain minimum in x [Optional]

x_domain_end
  Domain maximum in x [Optional]

y_domain_st
  Domain minimum in y [Optional]

y_domain_end
  Domain maximum in y [Optional]

z_domain_st
  Domain minimum in z [Optional]

z_domain_end
  Domain maximum in z [Optional]

time_domain_st
  Domain minimum in GpsTime [Optional]

time_domain_end
  Domain maximum in GpsTime [Optional]
  
use_time_dim
  Use GpsTime coordinate data as array dimension [Optional]

time_first
  If writing 4D array with XYZ and Time, choose to put time dim first or last (default) [Optional]
  
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

timestamp
  Sets the TileDB timestamp for this write

.. include:: writer_opts.rst

By default TileDB will use the following set of compression filters for coordinates and attributes;

.. code-block:: json

  {
      "coords":[
        {"compression": "zstd", "compression_level": 7}
      ],
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
      "GpsTime": [  
        {"compression": "zstd", "compression_level": 7}
      ]
  }

.. _TileDB: https://tiledb.io
