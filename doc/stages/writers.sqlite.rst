.. _writers.sqlite:

writers.sqlite
====================

The `SQLite`_ driver outputs point cloud data into a PDAL-sepecific scheme
that matches the approach of :ref:`readers.pgpointcloud` and :ref:`readers.oci`.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.las",
          "filename":"inputfile.las"
        },
        {
          "type":"filters.chipper",
          "capacity":50
        }
        {
          "type":"writers.sqlite",
          "connection":"output.sqlite",
          "cloud_table_name":"SIMPLE_CLOUD",
          "pre_sql":"",
          "post_sql":"",
          "block_table_name":"SIMPLE_BLOCKS",
          "cloud_column_name":"CLOUD",
          "filename":"outputfile.pcd"
        }
      ]
    }



Options
-------

connection
  SQLite filename [Required]

cloud_table_name
  Name of table to store cloud (file) information [Required]

block_table_name
  Name of table to store patch information [Required]

cloud_column_name
  Name of column to store primary cloud_id key [Default: **cloud**]

compression
  Use https://github.com/verma/laz-perf compression technique to store patches

overwrite
  To drop the table before writing set to 'true'. To append to the table set to 'false'. [Default: **true**]

pre_sql
  Optional SQL to execute *before* running the translation. If the value references a file, the file is read and any SQL inside is executed. Otherwise the value is executed as SQL itself.

post_sql
  Optional SQL to execute *after* running the translation. If the value references a file, the file is read and any SQL inside is executed. Otherwise the value is executed as SQL itself.

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

.. _SQLite: http://sqlite.org
