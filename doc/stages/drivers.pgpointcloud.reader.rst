.. _drivers.pgpointcloud.reader:

drivers.pgpointcloud.reader
===========================

The **PostgreSQL Pointcloud Reader** allows you to read from a PostgreSQL database that the `PostgreSQL Pointcloud`_ extension enabled. The Pointcloud extension stores point cloud data in tables that contain rows of patches. Each patch in turn contains a large number of spatially nearby points.

The reader pulls patches from a table, potentially sub-setting the query on the way with a "where" clause.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.text.writer">
      <Option name="filename">output.txt</Option>
      <Option name="spatialreference">EPSG:26910</Option>
      <Reader type="drivers.pgpointcloud.reader">
        <Option name="connection">dbname='lidar' user='user'</Option>
        <Option name="table">lidar</Option>
        <Option name="column">pa</Option>
        <Option name="spatialreference">EPSG:26910</Option>
        <Option name="where">
          PC_Intersects(pa, ST_MakeEnvelope(560037.36, 5114846.45, 562667.31, 5118943.24, 26910))
        </Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

connection
  PostgreSQL connection string. In the form *"host=hostname dbname=database user=username password=pw port=5432"* [Required] 

table
  Database table to read from. [Required] 

schema
  Database schema to read from. [Default: **public**]

column
  Table column to read patches from. [Default: **pa**]

spatialreference
  The spatial reference to use for the points. Over-rides the value read from the database.


.. _PostgreSQL Pointcloud: https://github.com/pramsey/pointcloud
