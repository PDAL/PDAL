.. _readers.sqlite:

readers.sqlite
====================

The `SQLite`_ point cloud reader allows you to
read data stored in a SQLite database using a scheme that
PDAL wrote using the :ref:`writers.sqlite` writer. Much like
the :ref:`writers.oci` and :ref:`writers.pgpointcloud`, the
SQLite driver stores data in tables that contain rows of
patches. Each patch contains a number of spatially contiguous points

.. plugin::

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.sqlite",
          "connection":"inputfile.sqlite",
          "query":"SELECT b.schema, l.cloud, l.block_id, l.num_points, l.bbox, l.extent, l.points, b.cloud\r\n                      FROM simple_blocks l, simple_cloud b\r\n                     WHERE l.cloud = b.cloud and l.cloud in (1)\r\n                    order by l.cloud"
        },
        {
          "type":"writers.las",
          "filename":"outputfile.las"
        }
      ]
    }



Options
-------

query
  SQL statement that selects a schema XML, cloud id, bbox, and extent [Required]

spatialreference
  The spatial reference to use for the points. Over-rides the value read from the database.

count
  Maximum number of points to read [Optional]

.. _SQLite: https://sqlite.org/
