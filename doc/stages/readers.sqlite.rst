.. _readers.sqlite:

readers.sqlite
====================

The **SQLite Reader** allows you to
read data stored in a `SQLite database`_ using a scheme that
PDAL wrote using the :ref:`writers.sqlite` writer.  The
SQLite driver stores data in tables that contain rows of
patches. Each patch contains a number of spatially contiguous points

.. plugin::

Example
-------

.. code-block:: json

  [
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


Options
-------

query
  SQL statement that selects a schema XML, cloud id, bbox, and extent [Required]

.. include:: reader_opts.rst

.. _SQLite database: https://sqlite.org/
