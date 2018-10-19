.. _filters.delaunay:

filters.delaunay
================

The Delaunay filter creates a triangulated mesh fulfilling the Delaunay
condition from a collection of points.

The filter is implemented using the `delaunator-cpp`_ library, a C++ port of
the JavaScript `Delaunator`_ library.

The filter currently only supports 2D Delaunay triangulation, using the x and y
dimensions of the point cloud.

.. _`delaunator-cpp`: https://github.com/delfrrr/delaunator-cpp
.. _`Delaunator`: https://github.com/mapbox/delaunator

.. embed::

Example
-------

.. code-block:: json

    {
      "pipeline": [
        "input.las",
        {
          "type": "filters.delaunay"
        },
        {
          "type": "writers.ply",
          "filename": "output.ply",
          "faces": true
        }
      ]
    }
