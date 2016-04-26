.. _filters.splitter:

filters.splitter
===============================================================================

The splitter filter breaks a point cloud into square tiles of a size that
you choose.  The origin of the tiles is chosen arbitrarily unless specified
as an option.

The splitter takes a single PointView as its input and creates a PointView
for each tile as its output.

Splitting is usually applied to data read from files (which produce one large
stream of points) before the points are written to a database (which prefer
data segmented into smaller blocks).

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.splitter",
          "length":"100",
          "origin_x":"638900.0",
          "origin_y":"835500.0"
        },
        {
          "type":"writers.pgpointcloud",
          "connection":"dbname='lidar' user='user'"
        }
      ]
    }

Options
-------

length
  Length of the sides of the tiles that are created to hold points.
  [Default: 1000]

origin_x
  X Origin of the tiles.  [Default: none (chosen arbitarily)]

origin_y
  Y Origin of the tiles.  [Default: none (chosen arbitarily)]

