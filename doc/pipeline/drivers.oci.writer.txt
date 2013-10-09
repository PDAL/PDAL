.. _drivers.oci.writer:

drivers.oci.writer
==================

The OCI writer is used to write data to `Oracle point cloud`_ databases.


Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.oci.writer">
      <Option name="connection">
        lidar/lidar@oracle.hobu.biz/lidar
      </Option>
      <Option name="base_table_name">
        QFIT_CLOUD
      </Option>
      <Option name="block_table_name">
        QFIT_BLOCKS
      </Option>
      <Option name="cloud_column_name">
        CLOUD
      </Option>
      <Option name="is3d">
        false
      </Option>
      <Option name="solid">
        false
      </Option>
      <Option name="overwrite">
        false
      </Option>
      <Option name="srid">
        4269
      </Option>
      <Option name="capacity">
        5000
      </Option>
      <Option name="stream_output_precision">
        8
      </Option>
      <Filter type="filters.chipper">
        <Option name="capacity">
          5000
        </Option>
        <Filter type="filters.cache">
          <Option name="max_cache_blocks">
            1
          </Option>
          <Option name="cache_block_size">
            50000000
          </Option>
          <Reader type="drivers.las.reader">
            <Option name="filename">
              file.las
            </Option>
          </Reader>
        </Filter>
      </Filter>
    </Writer>
  </Pipeline>



Options
-------

connection
  Oracle connection string to connect to database

is3d
  Should we use 3D objects for SDO_PC PC_EXTENT, BLK_EXTENT, and indexing [Default: **false**]

solid
  Define the point cloud's PC_EXTENT geometry gtype as (1,1007,3) instead of the normal (1,1003,3), and use gtype 3008/2008 vs 3003/2003 for BLK_EXTENT geometry values.  [Default: **false**]

overwrite
  Wipe the block table and recreate it before loading data [Default: **false**]

verbose
  Wipe the block table and recreate it before loading data  [Default: **false**]
  
srid
  The Oracle numerical SRID value to use for PC_EXTENT, BLK_EXTENT, and indexing  [Default: **0**]
  
capacity
  The block capacity or maximum number of points a block can contain  [Default: **0**]

stream_output_precision
  The number of digits past the decimal place for outputting floats/doubles to streams. This is used for creating the SDO_PC object and adding the index entry to the USER_SDO_GEOM_METADATA for the block table  [Default: **8**]

cloud_id 
  The point cloud id that links the point cloud object to the entries in the block table.  [Default: **-1**]

block_table_name
  The table in which block data for the created SDO_PC will be placed  [Default: **output**]

block_table_partition_column
  The column name for which 'block_table_partition_value' will be placed in the 'block_table_name'
  
block_table_partition_value
  Integer value to use to assing partition IDs in the block table. Used in conjunction with 'block_table_partition_column'  [Default: **0**]
  
base_table_name
  The name of the table which will contain the SDO_PC object [Default: **hobu**]

cloud_column_name
 The column name in 'base_table_name' that will hold the SDO_PC object [Default: **CLOUD**]

base_table_aux_columns
  Quoted, comma-separated list of columns to add to the SQL that gets executed as part of the point cloud insertion into the 'base_table_name' table
  
base_table_aux_values
  Quoted, comma-separated values that correspond to 'base_table_aux_columns', entries that will get inserted as part of the creation of the SDO_PC entry in the 'base_table_name' table

base_table_boundary_column
  The SDO_GEOMETRY column in 'base_table_name' in which to insert the WKT in 'base_table_boundary_wkt' representing a boundary for the SDO_PC object. Note this is not the same as the 'base_table_bounds', which is just a bounding box that is placed on the SDO_PC object itself.
  
base_table_boundary_wkt
  WKT, in the form of a string or a file location, to insert into the SDO_GEOMTRY column defined by 'base_table_boundary_column'

pre_block_sql
  SQL, in the form of a string or file location, that is executed after the SDO_PC object has been created but before the block data in 'block_table_name' are inserted into the database

pre_sql
  SQL, in the form of a string or file location, that is executed before the SDO_PC object is created.

post_block_sql
  SQL, in the form of a string or file location, that is executed after the block data in 'block_table_name' have been inserted

base_table_bounds
  A bounding box, given in the Oracle SRID specified in 'srid' to set on the PC_EXTENT object of the SDO_PC. If none is specified, the cumulated bounds of all of the block data are used.

pc_id
  Point Cloud id [Default: **-1**]

pack_ignored_fields
  Pack ignored dimensions out of the data buffer that is written [Default: **true**]
  
do_trace
  turn on server-side binds/waits tracing -- needs ALTER SESSION privs [Default: **false**]
  
stream_chunks
  Stream block data chunk-wise by the DB's chunk size rather than as an entire blob" [Default: **false**]
  
blob_chunk_count
  When streaming, the number of chunks per write to use [Default: **16**]




.. _Oracle point cloud: http://docs.oracle.com/cd/B28359_01/appdev.111/b28400/sdo_pc_pkg_ref.htm

