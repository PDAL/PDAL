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
  Use GpsTime coordinate data as an array dimension instead of an array attribute. Not used when `append=true`. [Default: false]

time_first
  Put the GpsTime dimension first instead of last. Only used when `use_time_dim=true`. Not used when `append=true`. [Default: false]

chunk_size
  Point cache size for chunked writes. [Optional]

append
  Instead of creating a new array, append to an existing array that has the dimensions stored as a TileDB dimension or TileDB attribute. [Default: false]

stats
  Dump query stats to stdout. [Optional]

filters
  JSON array or object of compression filters for either dimenions or attributes of the form {dimension/attribute name : {"compression": name, compression_options: value, ...}}.  Not used when `append=true`. [Optional]

filter_profile
  Profile of compression filters to use for dimensions and attributes not provided in `filters`. Options include `balanced`, `aggressive`, and `none`. Not used when `append=true`. [Default: balanced]

scale_x, scale_y, scale_z
  Scale factor used for the float-scale filter for the X, Y, and Z dimensions, respectively, when using the `balanced` or `aggressive` filter profile. Not used when `append=true`. [Default: 0.01]

  Note: written value = (nominal value - offset) / scale.

offset_x, offset_y, offset_z
  Offset used for the float-scale filter for the  X, Y and Z dimenisons, respectively, when using the `balanced` or `aggressive` filter profile. Not used when `append=true`. [Default: 0.0]

  Note: written value = (nominal value - offset) / scale.

compression
  The default TileDB compression filter to use. Only used if the dimension or attribute name is not included in `filters`. Not used when `append=true`. [Default: none]

compression_level
  The TileDB compression level to use for the default compression. Option is ignored if set to `-1`. Not used when `append=true`. [Default: -1]

timestamp
  Sets the TileDB timestamp for this write. [Optional]

.. include:: writer_opts.rst


TileDB provides default filter profiles. The filters can be over-written by the `filters` option. If a TileDB attribute is not set by the filter profile or the `filter` option, the compression filter set by the compression option is used.

Filters set by the `balanced` (default) filter profile (the delta filter is skipped if using TileDB version less than 2.16.0):
.. code-block:: json

  {
      "X":{
        {"compression": "float-scale", "scale_float_factor": scale_x, "scale_float_offset": offset_x, "scale_float_bytewidth": 4},
        {"compression": "delta", "reinterpret_data": "INT32"},
        {"compression": "bit-shuffle"},
        {"compression": "zstd", "compression_level": 7}
      },
      "Y":{
        {"compression": "float-scale", "scale_float_factor": scale_y, "scale_float_offset": offset_y, "scale_float_bytewidth": 4},
        {"compression": "delta", "reinterpret_data": "INT32"},
        {"compression": "bit-shuffle"},
        {"compression": "zstd", "compression_level": 7}
      },
      "Z":{
        {"compression": "float-scale", "scale_float_factor": scale_z, "scale_float_offset": offset_z, "scale_float_bytewidth": 4},
        {"compression": "delta", "reinterpret_data": "INT32"},
        {"compression": "bit-shuffle"},
        {"compression": "zstd", "compression_level": 7}
      },
      "Intensity":{
        {"compression": "delta"},
        {"compression": "zstd", "compression_level": 5},
      },
      "BitFields":{"compression": "zstd", "compression_level": 5},
      "ReturnNumber": {"compression": "zstd", "compression_level": 5},
      "NumberOfReturns": {"compression": "zstd", "compression_level": 5},
      "ScanDirectionFlag": {"compression": "zstd", "compression_level": 5},
      "EdgeOfFlightLine": {"compression": "zstd", "compression_level": 5},
      "Classification": {"compression": "zstd", "compression_level": 5},
      "ScanAngleRank": {"compression": "zstd", "compression_level": 5},
      "UserData": {"compression": "zstd", "compression_level": 5},
      "PointSourceId": {"compression": "zstd", "compression_level": 5},
      "Red": {
        {"compression": "delta"},
        {"compression": "bit-width-reduction-filter"}
        {"compression": "zstd", "compression_level": 7},
      },
      "Green": {
        {"compression": "delta"},
        {"compression": "bit-width-reduction-filter"}
        {"compression": "zstd", "compression_level": 7},
      },
      "Blue": {
        {"compression": "delta"},
        {"compression": "bit-width-reduction-filter"}
        {"compression": "zstd", "compression_level": 7},
      },
      "GpsTime": {
        {"compression": "typed-view", "typed_view_output_datatype": int64},
        {"compression": "delta", "reinterpret_data": "INT64"},
        {"compression": "bit-width-reduction-filter"},
        {"compression": "zstd", "compression_level": 7},
      }
  }

Filters set by the `aggressive` filter profile (the delta filter is skipped if using TileDB version less than 2.16.0):
.. code-block:: json

  {
      "X":{
        {"compression": "float-scale", "scale_float_factor": scale_x, "scale_float_offset": offset_x, "scale_float_bytewidth": 4},
        {"compression": "delta", "reinterpret_data": "INT32"},
        {"compression": "bit-width-reduction"},
        {"compression": "bzip2", "compression_level": 9}
      },
      "Y":{
        {"compression": "float-scale", "scale_float_factor": scale_y, "scale_float_offset": offset_y, "scale_float_bytewidth": 4},
        {"compression": "delta", "reinterpret_data": "INT32"},
        {"compression": "bit-shuffle"},
        {"compression": "bzip2", "compression_level": 9}
      },
      "Z":{
        {"compression": "float-scale", "scale_float_factor": scale_z, "scale_float_offset": offset_z, "scale_float_bytewidth": 4},
        {"compression": "delta", "reinterpret_data": "INT32"},
        {"compression": "bit-width-reduction"},
        {"compression": "bzip2", "compression_level": 9}
      },
      "Intensity":{
        {"compression": "delta"},
        {"compression": "bit-width-reduction"},
        {"compression": "bzip2", "compression_level": 5},
      },
      "BitFields":{"compression": "bzip2", "compression_level": 9},
      "ReturnNumber": {"compression": "bzip2", "compression_level": 9},
      "NumberOfReturns": {"compression": "bzip2", "compression_level": 9},
      "ScanDirectionFlag": {"compression": "bzip2", "compression_level": 9},
      "EdgeOfFlightLine": {"compression": "bzip2", "compression_level": 9},
      "Classification": {"compression": "bzip2", "compression_level": 9},
      "ScanAngleRank": {"compression": "bzip2", "compression_level": 9},
      "UserData": {"compression": "gzip", "compression_level": 9},
      "PointSourceId": {"compression": "zstd", "compression_level": 9},
      "Red": {
        {"compression": "delta"},
        {"compression": "bit-width-reduction"}
        {"compression": "bzip2", "compression_level": 9},
      },
      "Green": {
        {"compression": "delta"},
        {"compression": "bit-width-reduction"}
        {"compression": "bzip2", "compression_level": 9},
      },
      "Blue": {
        {"compression": "delta"},
        {"compression": "bit-width-reduction"}
        {"compression": "bzip2", "compression_level": 9},
      },
      "GpsTime": {
        {"compression": "typed-view", "typed_view_output_datatype": int64},
        {"compression": "delta", "reinterpret_data": "INT64"},
        {"compression": "bit-width-reduction"},
        {"compression": "bzip2", "compression_level": 9},
      }
  }



The filter profile `none` does not set any default filters.

.. _TileDB: https://tiledb.io
