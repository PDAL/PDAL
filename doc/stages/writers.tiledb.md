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
          "filename":"input.las"
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
  Number of points per tile. Not used when `append=true`. [Default: 100,000]

cell_order
  The layout to use for TileDB cells. May be `auto`, `row-major`, `col-major`, or `hilbert`. Not used when `append=true`. [Default: auto]

tile_order
  The layout to use for TileDB tiles. May be `row-major` or `col-major`. Not used when `append=true`. [Default: row-major]

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

combine_bit_fields
  Store all sub-byte fields together in an attribute named `BitFields`. Not used when `append=true`. [Default: true]

chunk_size
  Point cache size for chunked writes. [Default: 1,000,000]

append
  Instead of creating a new array, append to an existing array that has the dimensions stored as a TileDB dimension or TileDB attribute. [Default: false]

stats
  Dump query stats to stdout. [Default: false]

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

allow_dups
  Allow duplicate points. [Default: true]

.. include:: writer_opts.rst


TileDB provides default filter profiles. The filters can be over-written by the `filters` option. If a TileDB attribute is not set by the filter profile or the `filter` option, the compression filter set by the compression option is used.

Filters set by the `balanced` (default) filter profile (the delta filter is skipped if using TileDB version less than 2.16.0):

* X

  1. Float-scale filter (factor=`scale_x`, offset=`offset_x`, scale_float_bytewidth=4)
  2. Delta filter (reinterpret_datatype=`INT32`)
  3. Bit shuffle filter
  4. Zstd filter (level=7)

* Y

  1. Float-scale filter (factor=`scale_y`, offset=`offset_y`, scale_float_bytewidth=4)
  2. Delta filter (reinterpret_datatype=`INT32`)
  3. Bit shuffle filter
  4. Zstd filter (level=7)

* Z

  1. Float-scale filter (factor=`scale_z`, offset=`offset_z`, scale_float_bytewidth=4)
  2. Delta filter (reinterpret_datatype=`INT32`)
  3. Bit shuffle filter
  4. Zstd filter (level=7)

* GPSTime

  1. Delta filter (reinterpret_datatype="INT64")
  2. Bit width reduction filter
  3. Zstd filter (level=7)

* Intensity

  1. Delta filter
  2. Zstd filter (level=5)

* BitFields

  1. Zstd filter (level=5)

* ReturnNumber

  1. Zstd filter (level=5)

* NumberOfReturns

  1. Zstd filter (level=5)

* ScanDirectionFlag

  1. Zstd filter (level=5)

* EdgeOfFlightLine

  1. Zstd filter (level=5)

* Classification

  1. Zstd filter (level=5)

* UserData

  1. Zstd filter (level=5)

* PointSourceId

  1. Zstd filter (level=5)

* Red

  1. Delta filter
  2. Bit width reduction filter
  3. Zstd filter (level=7)

* Green

  1. Delta filter
  2. Bit width reduction filter
  3. Zstd filter (level=7)

* Blue

  1. Delta filter
  2. Bit width reduction filter
  3. Zstd filter (level=7)

Filters set by the `aggressive` filter profile (the delta filter is skipped if using TileDB version less than 2.16.0):

* X

  1. Float-scale filter (factor=`scale_x`, offset=`offset_x`, scale_float_bytewidth=4)
  2. Delta filter (reinterpret_datatype=`INT32`)
  3. Bit width reduction filter
  4. BZIP2 filter (level=9)

* Y

  1. Float-scale filter (factor=`scale_y`, offset=`offset_y`, scale_float_bytewidth=4)
  2. Delta filter (reinterpret_datatype=`INT32`)
  3. Bit width reduction filter
  4. BZIP2 filter (level=9)

* Z

  1. Float-scale filter (factor=`scale_z`, offset=`offset_z`, scale_float_bytewidth=4)
  2. Delta filter (reinterpret_datatype=`INT32`)
  3. Bit width reduction filter
  4. BZIP2 filter (level=9)

* GPSTime

  1. Delta filter (reinterpret_datatype="INT64")
  2. Bit width reduction filter
  3. BZIP2 filter (level=9)

* Intensity

  1. Delta filter
  2. Bit width reduction
  3. BZIP2 filter (level=5)

* BitFields

  1. BZIP2 filter (level=9)

* ReturnNumber

  1. BZIP2 filter (level=9)

* NumberOfReturns

  1. BZIP2 filter (level=9)

* ScanDirectionFlag

  1. BZIP2 filter (level=9)

* EdgeOfFlightLine

  1. BZIP2 filter (level=9)

* Classification

  1. BZIP2 filter (level=9)

* UserData

  1. BZIP2 filter (level=9)

* PointSourceId

  1. BZIP2 filter (level=9)

* Red

  1. Delta filter
  2. Bit width reduction filter
  3. BZIP2 filter (level=9)

* Green

  1. Delta filter
  2. Bit width reduction filter
  3. BZIP2 filter (level=9)

* Blue

  1. Delta filter
  2. Bit width reduction filter
  3. BZIP2 filter (level=9)


The filter profile `none` does not set any default filters.

.. _TileDB: https://tiledb.io
