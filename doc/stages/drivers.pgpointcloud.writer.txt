.. _drivers.pgpointcloud.writer:

drivers.pgpointcloud.writer
===========================

The **PostgreSQL Pointcloud Writer** allows you to write to PostgreSQL database that have the `PostgreSQL Pointcloud`_ extension enabled. The Pointcloud extension stores point cloud data in tables that contain rows of patches. Each patch in turn contains a large number of spatially nearby points.

While you can theoretically store the contents of a whole file of points in a single patch, it is more practical to store a table full of smaller patches, where the patches are under the PostgreSQL page size (8kb). For most LIDAR data, this practically means a patch size of between 400 and 600 points.

In order to create patches of the right size, the Pointcloud writer should be preceded in the pipeline file by :ref:`filters.chipper`.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.pgpointcloud.writer">
      <Option name="connection">
        host='localhost' dbname='lidar' user='pramsey'
      </Option>
      <Option name="table">example</Option>
      <Option name="compression">dimensional</Option>
      <Option name="srid">26916</Option>
      <Filter type="filters.chipper">
        <Option name="capacity">400</Option>
        <Reader type="drivers.las.reader">
            <Option name="filename">example.las</Option>
            <Option name="spatialreference">EPSG:26916</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

Options
-------

connection
  PostgreSQL connection string. In the form *"host=hostname dbname=database user=username password=pw port=5432"* [Required] 

table
  Database table to write to. [Required] 

schema
  Database schema to write to. [Default: **public**]
  
column
  Table column to put patches into. [Default: **pa**]
  
compression
  Patch compression type to use. [Default: **dimensional**]
  
  * **none** applies no compression
  * **dimensional** applies dynamic compression to each dimension separately
  * **ght** applies a "geohash tree" compression by sorting the points into a prefix tree
  
overwrite
  To drop the table before writing set to 'true'. To append to the table set to 'false'. [Default: **true**]
  
srid
  Spatial reference ID (relative to the `spatial_ref_sys` table in PostGIS) to store with the point cloud schema. [Default: **4326**]
  
pre_sql
  Optional SQL to execute *before* running the translation. If the value references a file, the file is read and any SQL inside is executed. Otherwise the value is executed as SQL itself.

post_sql
  Optional SQL to execute *after* running the translation. If the value references a file, the file is read and any SQL inside is executed. Otherwise the value is executed as SQL itself.
  

.. _PostgreSQL Pointcloud: http://github.com/pramsey/pointcloud
