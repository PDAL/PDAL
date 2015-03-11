.. _readers.sqlite:

readers.sqlite
====================

The `SQLite`_ point cloud reader allows you to 
read data stored in a SQLite database using a scheme that 
PDAL wrote using the :ref:`writers.sqlite` writer. Much like 
the :ref:`writers.oci` and :ref:`writers.pgpointcloud`, the 
SQLite driver stores data in tables that contain rows of 
patches. Each patch contains a number of spatially contiguous points


Example
-------

.. code-block:: xml

    <?xml version="1.0" encoding="utf-8"?>
    <Pipeline version="1.0">
        <Writer type="writers.las">
            <Option name="filename">
                output.las
            </Option>
            <Reader type="readers.sqlite">
                <Option name="query">
                    SELECT b.schema, l.cloud, l.block_id, l.num_points, l.bbox, l.extent, l.points, b.cloud
                      FROM simple_blocks l, simple_cloud b
                     WHERE l.cloud = b.cloud and l.cloud in (1)
                    order by l.cloud
                </Option>
                <Option name="connection">
                    /Users/hobu/dev/git/pdal/test/data/thedata.sqlite
                </Option>
            </Reader>
        </Writer>
    </Pipeline>


Options
-------

query
  SQL statement that selects a schema XML, cloud id, bbox, and extent [Required] 

spatialreference
  The spatial reference to use for the points. Over-rides the value read from the database.


.. _SQLite: https://sqlite.org/
