.. _writers.oci:

writers.oci
===========

.. note::

    The OCI writer is deprecated and will be removed in a future release.

The OCI writer is used to write data to `Oracle point cloud`_ databases.

.. plugin::

Example
-------


.. code-block:: json

  [
      {
          "type":"readers.las",
          "filename":"inputfile.las"
      },
      {
          "type":"writers.oci",
          "connection":"grid/grid@localhost/orcl",
          "block_table_name":"QFIT_BLOCKS",
          "base_table_name":"QFIT_CLOUD",
          "cloud_column_name":"CLOUD",
          "srid":"4269",
          "capacity":"5000"
      }
  ]

Options
-------

connection
  Oracle connection string to connect to database

is3d
  Should we use 3D objects (include the z dimension) for SDO_PC PC_EXTENT,
  BLK_EXTENT, and indexing [Default: false]

solid
  Define the point cloud's PC_EXTENT geometry gtype as (1,1007,3) instead
  of the normal (1,1003,3), and use gtype 3008/2008 vs 3003/2003 for
  BLK_EXTENT geometry values.  [Default: false]

overwrite
  Wipe the block table and recreate it before loading data [Default: false]

verbose
  Wipe the block table and recreate it before loading data  [Default: false]

srid
  The Oracle numerical SRID value to use for PC_EXTENT, BLK_EXTENT, and
  indexing  [Default: 0]

capacity
  The block capacity or maximum number of points a block can contain.
  [Default: 0]

stream_output_precision
  The number of digits past the decimal place for writing floats/doubles to
  streams. This is used for creating the SDO_PC object and adding the
  index entry to the USER_SDO_GEOM_METADATA for the block table.
  [Default: 8]

cloud_id
  The point cloud id that links the point cloud object to the entries in
  the block table.  [Default: -1]

block_table_name
  The table in which block data for the created SDO_PC will be placed.
  [Default: "output"]

block_table_partition_column
  The column name for which 'block_table_partition_value' will be placed
  in the 'block_table_name'.

block_table_partition_value
  Integer value to use to assing partition IDs in the block table. Used
  in conjunction with 'block_table_partition_column'  [Default: 0]

base_table_name
  The name of the table which will contain the SDO_PC object.
  [Default: "hobu"]

cloud_column_name
  The column name in 'base_table_name' that will hold the SDO_PC object.
  [Default: "CLOUD"]

base_table_aux_columns
  Quoted, comma-separated list of columns to add to the SQL that gets
  executed as part of the point cloud insertion into the 'base_table_name'
  table.

base_table_aux_values
  Quoted, comma-separated values that correspond to 'base_table_aux_columns',
  entries that will get inserted as part of the creation of the SDO_PC
  entry in the 'base_table_name' table.

base_table_boundary_column
  The SDO_GEOMETRY column in 'base_table_name' in which to insert the WKT in 'base_table_boundary_wkt' representing a boundary for the SDO_PC object. Note this is not the same as the 'base_table_bounds', which is just a bounding box that is placed on the SDO_PC object itself.

base_table_boundary_wkt
  WKT, in the form of a string or a file location, to insert into
  the SDO_GEOMTRY column defined by 'base_table_boundary_column'.

pre_block_sql
  SQL, in the form of a string or file location, that is executed after
  the SDO_PC object has been created but before the block data in
  'block_table_name' are inserted into the database.

pre_sql
  SQL, in the form of a string or file location, that is executed before
  the SDO_PC object is created.

post_block_sql
  SQL, in the form of a string or file location, that is executed after the block data in 'block_table_name' have been inserted

base_table_bounds
  A bounding box, given in the Oracle SRID specified in 'srid' to set on
  the PC_EXTENT object of the SDO_PC. If none is specified, the cumulated
  bounds of all of the block data are used.

pc_id
  Point Cloud id [Default: -1]

pack_ignored_fields
  Pack ignored dimensions out of the data buffer that is written
  [Default: true]

do_trace
  turn on server-side binds/waits tracing -- needs ALTER SESSION privs.
  [Default: false]

stream_chunks
  Stream block data chunk-wise by the DB's chunk size rather than as an
  entire blob. [Default: false]

blob_chunk_count
  When streaming, the number of chunks per write to use [Default: 16]

scale_x, scale_y, scale_z / offset_x, offset_y, offset_z
  If ANY of these options are specified the X, Y and Z dimensions are adjusted
  by subtracting the offset and then dividing the values by the specified
  scaling factor before being written as 32-bit integers (as opposed to double
  precision values).  If any of these options is specified, unspecified
  scale_<x,y,x> options are given the value of 1.0 and unspecified
  offset_<x,y,z> are given the value of 0.0.

output_dims
  If specified, limits the dimensions written for each point.  Dimensions
  are listed by name and separated by commas.

tolerance
  Oracle geometry tolerance. X, Y, and Z dimensions are all
  currently specified as a single value [Default: 0.05]

.. include:: writer_opts.rst

.. _Oracle point cloud: http://docs.oracle.com/cd/B28359_01/appdev.111/b28400/sdo_pc_pkg_ref.htm

