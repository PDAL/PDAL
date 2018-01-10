.. _readers.pgpointcloud:

readers.pgpointcloud
====================

The **PostgreSQL Pointcloud Reader** allows you to read from a PostgreSQL
database that the `PostgreSQL Pointcloud`_ extension enabled. The Pointcloud
extension stores point cloud data in tables that contain rows of patches. Each
patch in turn contains a large number of spatially nearby points.

The reader pulls patches from a table, potentially sub-setting the query on the
way with a "where" clause.

.. plugin::

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.pgpointcloud",
          "connection":"dbname='lidar' user='user'",
          "table":"lidar",
          "column":"pa",
          "spatialreference":"EPSG:26910",
          "where":"PC_Intersects(pa, ST_MakeEnvelope(560037.36, 5114846.45, 562667.31, 5118943.24, 26910))",
        },
        {
          "type":"writers.text",
          "filename":"output.txt"
        }
      ]
    }


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
  Sets the spatial reference for the point data.  Overrides any spatial
  reference information read from the database.  Most text-based formats of
  SRS information are accepted, including WKT and proj.4.

count
  Maximum number of points to read [Optional]

.. _PostgreSQL Pointcloud: https://github.com/pramsey/pointcloud
